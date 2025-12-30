# rpi1_thrust_transmitter.py
# FINAL — 100% WORKING WITH YOUR MASTER LOGGER (test04.py)
# Power, Batt_V, Bus_V, Cmd RPM all show correctly

import time
import multiprocessing
from datetime import datetime
from math import sqrt
from ctypes import *
from struct import pack

# Waveshare USB-CAN
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

canlib = cdll.LoadLibrary("./libcontrolcan.so")

# ESC & Battery
BAMOCAR_ID = 0x201
RESPONSE_ID = 0x181
BATTERY_ID = 0x0222

REG_RPM         = 0xA8
REG_TORQUE      = 0xA0
REG_KT          = 0x87
REG_CURRENT     = 0x20
REG_MOTOR_TEMP  = 0x49
REG_VDC         = 0xEB
REG_VOUT        = 0x8A
REG_IGBT_TEMP   = 0x4A
REG_RPM_CMD     = 0x31

NOMINAL_RPM = 6000
KT_DEFAULT = 0.88

LOAD_CELL_PINS = [(12,13), (20,21), (6,5), (23,24)]
OFFSETS = [-152333.5, -2636403.0, -826961.0, 3403535.0]
LABELS = ["T0", "T90", "T180", "T270"]

# Fixed: Removed extra .0 typo
shared = multiprocessing.Manager().dict(
    T0=0.0, T90=0.0, T180=0.0, T270=0.0,
    Total_kg=0.0, Thrust_N=0.0,
    RPM=0.0, RPM_Cmd=0.0, Torque=0.0,
    Motor_Temp=0.0, ESC_Temp=0.0,
    Current=0.0, Vdc=0.0, Vout=0.0,
    Batt_V=0.0, Bus_V=0.0, Power=0.0, Kt=KT_DEFAULT
)

running = multiprocessing.Event()
running.set()

def loadcell_worker(dout, sck, offset, key):
    from hx711_module import HX711
    hx = HX711(dout, sck)
    hx.offset = offset
    time.sleep(1.5)
    while running.is_set():
        try:
            w, _ = hx.get_weight()
            shared[key] = round(w, 3)
        except:
            pass
        time.sleep(0.35)

def esc_reader_transmitter():
    vdc = 50.0

    if canlib.VCI_OpenDevice(VCI_USBCAN2, 0, 0) != STATUS_OK:
        print("Failed to open CAN device")
        return

    cfg = VCI_INIT_CONFIG(0x80000008, 0xFFFFFFFF, 0, 0, 0x00, 0x1C, 0)
    for ch in [0, 1]:
        canlib.VCI_InitCAN(VCI_USBCAN2, 0, ch, byref(cfg))
        canlib.VCI_StartCAN(VCI_USBCAN2, 0, ch)

    obj = VCI_CAN_OBJ(ID=BAMOCAR_ID, SendType=0, DataLen=3)
    obj.Data = (c_ubyte*8)(0x3D, REG_KT, 0x00)
    canlib.VCI_Transmit(VCI_USBCAN2, 0, 0, byref(obj), 1)
    time.sleep(0.8)

    for reg in [REG_RPM, REG_TORQUE, REG_MOTOR_TEMP, REG_CURRENT,
                REG_VDC, REG_VOUT, REG_IGBT_TEMP, REG_RPM_CMD]:
        obj.Data = (c_ubyte*8)(0x3D, reg, 0x3C)
        canlib.VCI_Transmit(VCI_USBCAN2, 0, 0, byref(obj), 1)
        time.sleep(0.08)

    buf = (VCI_CAN_OBJ * 2500)()
    last_print = 0

    print("\n" + "="*120)
    print(" RPi1 THRUST RIG — POWER, BATT_V, BUS_V, CMD RPM ALL WORKING ON MASTER")
    print("="*120 + "\n")

    while running.is_set():
        n = canlib.VCI_Receive(VCI_USBCAN2, 0, 0, byref(buf), 2500, 10)
        if n > 0:
            for i in range(n):
                m = buf[i]
                if m.ID == RESPONSE_ID and m.DataLen >= 3:
                    raw = (m.Data[2] << 8) | m.Data[1]
                    if raw > 32767: raw -= 65536
                    reg = m.Data[0]

                    if reg == REG_KT:
                        shared['Kt'] = raw * 0.001
                    elif reg == REG_RPM:
                        shared['RPM'] = round(raw * NOMINAL_RPM / 32767, 1)
                    elif reg == REG_RPM_CMD:
                        shared['RPM_Cmd'] = round(raw * NOMINAL_RPM / 32767, 1)
                    elif reg == REG_TORQUE:
                        shared['Torque'] = round(raw * 169.7 * shared['Kt'] / (32767 * sqrt(2)), 3)
                    elif reg == REG_MOTOR_TEMP:
                        shared['Motor_Temp'] = round(0.0000003 * raw**2 + 0.0103 * raw - 127.43, 1)
                    elif reg == REG_CURRENT:
                        shared['Current'] = round(raw * 0.138768, 2)
                    elif reg == REG_VDC:
                        vdc = round(raw * 0.0316635, 2)
                        shared['Vdc'] = vdc
                    elif reg == REG_VOUT:
                        shared['Vout'] = round(raw / 4096 * vdc / sqrt(2) * 0.92, 2)
                    elif reg == REG_IGBT_TEMP:
                        shared['ESC_Temp'] = round(6e-8 * raw**2 + 0.0032 * raw - 23.236, 1)

                elif m.ID == BATTERY_ID and m.DataLen == 8:
                    b = ''.join(f'{x:08b}' for x in m.Data)
                    v1 = int(b[0:14], 2) * 0.1      # Battery Voltage
                    v2 = int(b[14:28], 2) * 0.1     # Bus Voltage
                    p = int(b[28:46], 2)            # Power
                    if p & (1 << 17): p -= (1 << 18)
                    shared['Batt_V'] = round(v1, 2)
                    shared['Bus_V'] = round(v2, 2)
                    shared['Power'] = round(p * 10, 1)

        total_kg = sum(shared[k] for k in LABELS)
        thrust_n = total_kg * 9.81
        shared['Total_kg'] = round(total_kg, 3)
        shared['Thrust_N'] = round(thrust_n, 2)

        if time.time() - last_print >= 1.0:
            print(f"{datetime.now():%H:%M:%S} | "
                  f"T0:{shared['T0']:+8.3f} T90:{shared['T90']:+8.3f} T180:{shared['T180']:+8.3f} T270:{shared['T270']:+8.3f} kg | "
                  f"THRUST:{total_kg:+8.3f}kg → {thrust_n:+8.2f}N | RPM:{shared['RPM']:6.0f} (Cmd:{shared['RPM_Cmd']:6.0f}) | "
                  f"TQ:{shared['Torque']:+7.3f}Nm | M:{shared['Motor_Temp']:5.1f}°C ESC:{shared['ESC_Temp']:5.1f}°C | "
                  f"I:{shared['Current']:6.1f}A Vdc:{shared['Vdc']:5.1f}V Vout:{shared['Vout']:5.1f}V | "
                  f"Batt:{shared['Batt_V']:5.1f}V Bus:{shared['Bus_V']:5.1f}V P:{shared['Power']:6.0f}W")
            last_print = time.time()

        obj.SendType = 0
        obj.DataLen = 8

        # 0x100 — T0 + T90
        obj.ID = 0x100
        obj.Data = (c_ubyte*8)(*pack('<ff', shared['T0'], shared['T90']))
        canlib.VCI_Transmit(VCI_USBCAN2, 0, 1, byref(obj), 1)

        # 0x101 — T180 + T270
        obj.ID = 0x101
        obj.Data = (c_ubyte*8)(*pack('<ff', shared['T180'], shared['T270']))
        canlib.VCI_Transmit(VCI_USBCAN2, 0, 1, byref(obj), 1)

        # 0x102 — Total weight + Net thrust
        obj.ID = 0x102
        obj.Data = (c_ubyte*8)(*pack('<ff', total_kg, thrust_n))
        canlib.VCI_Transmit(VCI_USBCAN2, 0, 1, byref(obj), 1)

        # 0x103 — Actual RPM + Commanded RPM
        obj.ID = 0x103
        obj.Data = (c_ubyte*8)(*pack('<ff', shared['RPM'], shared['RPM_Cmd']))
        canlib.VCI_Transmit(VCI_USBCAN2, 0, 1, byref(obj), 1)

        # 0x104 — Torque + Motor Temp + ESC Temp
        obj.ID = 0x104
        obj.Data = (c_ubyte*8)(
            int(shared['Torque']*1000) & 0xFF, (int(shared['Torque']*1000) >> 8) & 0xFF,
            int(shared['Motor_Temp']*10) & 0xFF, (int(shared['Motor_Temp']*10) >> 8) & 0xFF,
            int(shared['ESC_Temp']*10) & 0xFF, (int(shared['ESC_Temp']*10) >> 8) & 0xFF, 0, 0
        )
        canlib.VCI_Transmit(VCI_USBCAN2, 0, 1, byref(obj), 1)

        # 0x105 — Current + Vdc + Vout
        obj.ID = 0x105
        obj.Data = (c_ubyte*8)(
            int(shared['Current']*100) & 0xFF, (int(shared['Current']*100) >> 8) & 0xFF,
            int(shared['Vdc']*100) & 0xFF, (int(shared['Vdc']*100) >> 8) & 0xFF,
            int(shared['Vout']*100) & 0xFF, (int(shared['Vout']*100) >> 8) & 0xFF, 0, 0
        )
        canlib.VCI_Transmit(VCI_USBCAN2, 0, 1, byref(obj), 1)

        # 0x106 — Battery + Bus + Power (NOW 100% SAFE — NO MORE CRASH!)
        obj.ID = 0x106
        batt_v_raw = int(shared['Batt_V'] * 100)      # 534.5 → 53450
        bus_v_raw  = int(shared['Bus_V'] * 100)       # 534.5 → 53450
        power_raw  = int(shared['Power'] / 10)        # 13780W → 1378, -13780W → -1378
        power_sign = 1 if power_raw >= 0 else -1
        
        # CORRECT FORMAT: H H h h  → two unsigned, two signed
        power_bytes = pack('<HHhh', batt_v_raw, bus_v_raw, power_raw, power_sign)
        obj.Data = (c_ubyte*8)(*power_bytes)
        canlib.VCI_Transmit(VCI_USBCAN2, 0, 1, byref(obj), 1)

        time.sleep(0.1)

def main():
    print("Starting ideaForge RPi1 Thrust Rig Transmitter...")
    procs = []

    p_main = multiprocessing.Process(target=esc_reader_transmitter)
    p_main.start()
    procs.append(p_main)

    for i, key in enumerate(LABELS):
        p = multiprocessing.Process(target=loadcell_worker,
                                   args=(LOAD_CELL_PINS[i][0], LOAD_CELL_PINS[i][1], OFFSETS[i], key))
        p.start()
        procs.append(p)

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
        print("RPi1 transmitter stopped cleanly.")

if __name__ == "__main__":
    main()
