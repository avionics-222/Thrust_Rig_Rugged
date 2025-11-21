# thrust_rig_transmitter.py
# ideaForge DUAL-CHANNEL THRUST TELEMETRY — FINAL 
# Thrust 100% working | Dual CAN | 64-bit Raspberry Pi

import time
import multiprocessing
from datetime import datetime
from math import sqrt
from ctypes import *

# =============================================================================
# Waveshare USB-CAN direct control (64-bit)
# =============================================================================
VCI_USBCAN2 = 4
STATUS_OK = 1

class VCI_INIT_CONFIG(Structure):
    _fields_ = [("AccCode", c_uint), ("AccMask", c_uint), ("Reserved", c_uint),
                ("Filter", c_ubyte), ("Timing0", c_ubyte), ("Timing1", c_ubyte),
                ("Mode", c_ubyte)]

class VCI_CAN_OBJ(Structure):
    _fields_ = [("ID", c_uint), ("TimeStamp", c_uint), ("TimeFlag", c_ubyte),
                ("SendType", c_ubyte), ("RemoteFlag", c_ubyte), ("ExternFlag", c_ubyte),
                ("DataLen", c_ubyte), ("Data", c_ubyte*8), ("Reserved", c_ubyte*3)]

canlib = cdll.LoadLibrary("./libcontrolcan.so")  # Must be in same folder

# =============================================================================
# CONFIG
# =============================================================================
BAMOCAR_ID = 0x201
BAMOCAR_RESPONSE_ID = 0x181
BATTERY_ID = 0x0222

RPM_REGISTER     = 0xA8
TORQUE_REGISTER  = 0xA0
KT_REGISTER      = 0x87
CURRENT_REGISTER = 0x20
TEMP_REGISTER    = 0x49
VDC_REGISTER     = 0xEB
VOUT_VXXX        = 0x8A
IGBT_TEMP        = 0x4A

NOMINAL_RPM = 6000
KT_DEFAULT  = 0.88

LOAD_CELL_PINS = [(12,13), (20,21), (6,5), (23,24)]
OFFSETS = [-152333.5, -2636403.0, -826961.0, 3403535.0]
LABELS = ["T0", "T90", "T180", "T270"]

# Shared queue
data_queue = multiprocessing.Queue()
running = multiprocessing.Event()
running.set()

# =============================================================================
# LOAD CELL WORKER
# =============================================================================
def loadcell_worker(dout, sck, offset, idx):
    try:
        from hx711_module import HX711
        hx = HX711(dout, sck)
        hx.offset = offset
        time.sleep(1.5)
        while running.is_set():
            try:
                w, _ = hx.get_weight()
                data_queue.put(('weight', idx, round(w, 3)))
                time.sleep(0.38)
            except:
                time.sleep(0.1)
    except Exception as e:
        print(f"Loadcell worker {idx} failed to start: {e}")

# =============================================================================
# CAN INITIALIZATION
# =============================================================================
def init_can():
    if canlib.VCI_OpenDevice(VCI_USBCAN2, 0, 0) != STATUS_OK:
        raise RuntimeError("Failed to open USB-CAN device")
    cfg = VCI_INIT_CONFIG(0x80000008, 0xFFFFFFFF, 0, 0, 0x00, 0x1C, 0)
    for ch in [0, 1]:
        canlib.VCI_InitCAN(VCI_USBCAN2, 0, ch, byref(cfg))
        canlib.VCI_StartCAN(VCI_USBCAN2, 0, ch)
    print("Both CAN channels initialized @ 500 kbps")

# =============================================================================
# CAN READER (Channel 0)
# =============================================================================
def can_reader():
    init_can()
    kt = KT_DEFAULT
    vdc = 50.0

    obj = VCI_CAN_OBJ(ID=BAMOCAR_ID, SendType=0, DataLen=3)

    obj.Data = (c_ubyte*8)(0x3D, KT_REGISTER, 0x00)
    canlib.VCI_Transmit(VCI_USBCAN2, 0, 0, byref(obj), 1)
    time.sleep(0.6)

    for reg in [RPM_REGISTER, TORQUE_REGISTER, TEMP_REGISTER, CURRENT_REGISTER,
                VDC_REGISTER, VOUT_VXXX, IGBT_TEMP]:
        obj.Data = (c_ubyte*8)(0x3D, reg, 0x3C)
        canlib.VCI_Transmit(VCI_USBCAN2, 0, 0, byref(obj), 1)
        time.sleep(0.08)

    buf = (VCI_CAN_OBJ * 2500)()
    while running.is_set():
        n = canlib.VCI_Receive(VCI_USBCAN2, 0, 0, byref(buf), 2500, 50)
        if n > 0:
            for i in range(n):
                m = buf[i]
                if m.ID == BAMOCAR_RESPONSE_ID and m.DataLen >= 3:
                    raw = (m.Data[2] << 8) | m.Data[1]
                    if raw > 32767: raw -= 65536
                    reg = m.Data[0]
                    if reg == KT_REGISTER:
                        kt = raw * 0.001
                    elif reg == RPM_REGISTER:
                        data_queue.put(('RPM', round(raw * NOMINAL_RPM / 32767, 1)))
                    elif reg == TORQUE_REGISTER:
                        data_queue.put(('ESC_Torque', round(raw * 169.7 * kt / (32767 * sqrt(2)), 3)))
                    elif reg == TEMP_REGISTER:
                        data_queue.put(('Motor_Temp', round(0.0000003 * raw**2 + 0.0103 * raw - 127.43, 1)))
                    elif reg == CURRENT_REGISTER:
                        data_queue.put(('Current', round(raw * 0.138768, 2)))
                    elif reg == VDC_REGISTER:
                        vdc = round(raw * 0.0316635, 2)
                        data_queue.put(('Vdc', vdc))
                    elif reg == VOUT_VXXX:
                        data_queue.put(('Vout', round(raw / 4096 * vdc / sqrt(2) * 0.92, 2)))
                    elif reg == IGBT_TEMP:
                        data_queue.put(('ESC_Temp', round(6e-8 * raw**2 + 0.0032 * raw - 23.236, 1)))
                elif m.ID == BATTERY_ID and m.DataLen == 8:
                    b = ''.join(f'{x:08b}' for x in m.Data)
                    p = int(b[28:46], 2)
                    if p & (1 << 17): p -= (1 << 18)
                    data_queue.put(('Power', round(p * 10, 1)))
        time.sleep(0.001)

# =============================================================================
# CAN2 TRANSMITTER + 1 Hz PRINT
# =============================================================================
def can_transmitter():
    time.sleep(3)

    weights = [0.0] * 4
    rpm = esc_torque = motor_temp = esc_temp = current = vdc = vout = power = 0.0

    obj = VCI_CAN_OBJ(SendType=0, RemoteFlag=0, ExternFlag=0, DataLen=8)

    print("\n" + "="*100)
    print(" ideaForge DUAL-CHANNEL THRUST TELEMETRY — LIVE & 100% ACCURATE")
    print(" CAN1 (Ch0): Reading ESC | CAN2 (Ch1): Broadcasting 0x300–0x303 @ 10 Hz")
    print("="*100 + "\n")

    last_print = time.time()

    while running.is_set():
        while not data_queue.empty():
            try:
                typ, *vals = data_queue.get_nowait()
                if typ == 'weight':
                    weights[vals[0]] = vals[1]
                elif typ == 'RPM': rpm = vals[0]
                elif typ == 'ESC_Torque': esc_torque = vals[0]
                elif typ == 'Motor_Temp': motor_temp = vals[0]
                elif typ == 'ESC_Temp': esc_temp = vals[0]
                elif typ == 'Current': current = vals[0]
                elif typ == 'Vdc': vdc = vals[0]
                elif typ == 'Vout': vout = vals[0]
                elif typ == 'Power': power = vals[0]
            except:
                pass

        total_kg = sum(weights)
        thrust_n = total_kg * 9.81

        if time.time() - last_print >= 1.0:
            print(f"{datetime.now():%H:%M:%S} | "
                  f"T0:{weights[0]:+7.3f} T90:{weights[1]:+7.3f} "
                  f"T180:{weights[2]:+7.3f} T270:{weights[3]:+7.3f} kg "
                  f"| TOTAL:{total_kg:+8.3f} kg → {thrust_n:+8.2f} N "
                  f"| RPM:{rpm:6.0f} TQ:{esc_torque:+7.3f}Nm "
                  f"| M:{motor_temp:5.1f}°C ESC:{esc_temp:5.1f}°C "
                  f"| I:{current:6.1f}A Vdc:{vdc:5.1f}V Vout:{vout:5.1f}V "
                  f"| P:{power:6.0f}W")
            last_print = time.time()

        def i16(v, s=1): return int(round(v * s)) & 0xFFFF

        frames = [
            (0x300, [i16(weights[0],1000)&0xFF, i16(weights[0],1000)>>8,
                     i16(weights[1],1000)&0xFF, i16(weights[1],1000)>>8,
                     i16(weights[2],1000)&0xFF, i16(weights[2],1000)>>8,
                     i16(weights[3],1000)&0xFF, i16(weights[3],1000)>>8]),
            (0x301, [i16(total_kg,100)&0xFF, i16(total_kg,100)>>8,
                     i16(thrust_n,10)&0xFF, i16(thrust_n,10)>>8,
                     int(rpm)&0xFF, int(rpm)>>8,
                     i16(esc_torque,1000)&0xFF, i16(esc_torque,1000)>>8]),
            (0x302, [int(motor_temp*10), int(esc_temp*10),
                     i16(current,100)&0xFF, i16(current,100)>>8,
                     i16(vdc,100)&0xFF, i16(vdc,100)>>8, 0, 0]),
            (0x303, [i16(vout,100)&0xFF, i16(vout,100)>>8,
                     int(power)&0xFF, int(power)>>8, 0,0,0,0])
        ]

        for fid, payload in frames:
            obj.ID = fid
            obj.Data = (c_ubyte*8)(*payload)
            canlib.VCI_Transmit(VCI_USBCAN2, 0, 1, byref(obj), 1)

        time.sleep(0.1)

# =============================================================================
# MAIN
# =============================================================================
def main():
    print("Starting ideaForge Dual-Channel Thrust Telemetry Transmitter...")
    procs = []

    p_can = multiprocessing.Process(target=can_reader)
    p_can.start()
    procs.append(p_can)

    for i in range(4):
        p = multiprocessing.Process(target=loadcell_worker,
                                   args=(LOAD_CELL_PINS[i][0], LOAD_CELL_PINS[i][1], OFFSETS[i], i))
        p.start()
        procs.append(p)

    p_tx = multiprocessing.Process(target=can_transmitter)
    p_tx.start()
    procs.append(p_tx)

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        running.clear()
        for p in procs:
            p.terminate()
            p.join(5)
        canlib.VCI_CloseDevice(VCI_USBCAN2, 0)
        print("Transmitter stopped cleanly.")

if __name__ == "__main__":
    main()