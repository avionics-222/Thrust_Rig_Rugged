import time
import csv
import threading
import os
from datetime import datetime
import logging
from pymodbus.client import ModbusSerialClient
from gpiozero import Button
import qwiic_kx13x
import sys

# ---- RS485 & Sensor Config ----
RS485_PORT = '/dev/ttyAMA0'
BAUD_RATE = 9600

FLOW_SENSOR_PINS = [23, 24]
FLOW_FACTORS = [9.9, 9.9]

pulse_counts = [0, 0]
flow_rates = [0.0, 0.0]
temperatures = [0.0, 0.0]
accel_data = [0.0, 0.0, 0.0]  # X, Y, Z in g

# Temperature offsets
off_t2 = 9.2  # Outlet (ID-1 RS485)
off_t1 = 6.7  # Inlet  (ID-3 RS485)

data_lock = threading.Lock()
modbus_lock = threading.Lock()

# ---- Logging config ----
logging.basicConfig(level=logging.ERROR)
logging.getLogger("pymodbus").setLevel(logging.ERROR)
logging.getLogger("pymodbus.logging").setLevel(logging.ERROR)
logging.getLogger("serial").setLevel(logging.ERROR)

# ---- Flow sensor ----
def pulse_inc_0():
    pulse_counts[0] += 1

def pulse_inc_1():
    pulse_counts[1] += 1

flow_sensors = [
    Button(FLOW_SENSOR_PINS[0], pull_up=True),
    Button(FLOW_SENSOR_PINS[1], pull_up=True)
]
flow_sensors[0].when_pressed = pulse_inc_0
flow_sensors[1].when_pressed = pulse_inc_1

def calc_flow(pulse_count, factor):
    return round(pulse_count / factor, 2)

def flow_thread():
    while True:
        time.sleep(1)
        with data_lock:
            flow_rates[0] = calc_flow(pulse_counts[0], FLOW_FACTORS[0])
            flow_rates[1] = calc_flow(pulse_counts[1], FLOW_FACTORS[1])
            pulse_counts[0] = 0
            pulse_counts[1] = 0

# ---- RS485 Temp thread ----
def rs485_temp_thread():
    client = None
    try:
        client = ModbusSerialClient(
            port=RS485_PORT, baudrate=BAUD_RATE,
            parity='N', stopbits=1, bytesize=8, timeout=3
        )
        if not client.connect():
            print("Could not connect to RS485")
            return
        while True:
            for idx, dev_id in enumerate([1, 3]):
                with modbus_lock:
                    if hasattr(client, "socket") and hasattr(client.socket, "reset_input_buffer"):
                        try:
                            client.socket.reset_input_buffer()
                        except:
                            pass
                    rr = client.read_holding_registers(address=0, count=1, device_id=dev_id)
                    temp = 0.0
                    if not rr.isError():
                        raw = rr.registers[0]
                        temp = (raw - 65536)/10 if (raw & 0x8000) else raw/10
                    with data_lock:
                        temperatures[idx] = temp
                    time.sleep(0.3)
    except Exception as e:
        print(f"RS485 temp thread error: {e}")
    finally:
        if client:
            client.close()

# ---- Accelerometer thread ----
def accel_thread(kx):
    try:
        if not kx.connected:
            print("KX13X Accelerometer not connected. Please check your connection", file=sys.stderr)
            return
        if not kx.begin():
            print("Failed to initialize KX13X. Check if using correct model (KX134/KX132).")
            return
        kx.software_reset()
        kx.enable_accel(False)
        kx.set_output_data_rate(0x0A)  # 50Hz
        kx.set_range(kx.KX134_RANGE32G)  # For KX134, use KX132_RANGE16G for KX132
        kx.enable_data_engine()
        kx.enable_accel()
        while True:
            if kx.data_ready():
                kx.get_accel_data()
                with data_lock:
                    accel_data[0] = kx.kx134_accel.x
                    accel_data[1] = kx.kx134_accel.y
                    accel_data[2] = kx.kx134_accel.z
            time.sleep(0.02)  # 50Hz
    except Exception as e:
        print(f"Accelerometer thread error: {e}")

# ---- Main loop ----
def main():
    loc = input("Enter sensor location (default: Clog): ").strip()
    if not loc:
        loc = "Clog"

    now = datetime.now()
    timestamp_suffix = now.strftime("%Y%m%d_%H%M%S")
    log_folder = "logs"
    os.makedirs(log_folder, exist_ok=True)
    filename = os.path.join(log_folder, f"{loc}_{timestamp_suffix}.csv")

    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["Timestamp", "Flow1_Lpm", "Temp1_C", "Flow2_Lpm", "Temp2_C", "X_g", "Y_g", "Z_g"])

    # Initialize Accelerometer
    kx = qwiic_kx13x.QwiicKX134()  # Use QwiicKX132() for KX132

    # Start threads
    threading.Thread(target=flow_thread, daemon=True).start()
    threading.Thread(target=rs485_temp_thread, daemon=True).start()
    threading.Thread(target=accel_thread, args=(kx,), daemon=True).start()

    with open(filename, 'a', newline='') as f:
        writer = csv.writer(f)
        last_print = time.time()
        try:
            while True:
                with data_lock:
                    f1, f2 = flow_rates
                    t2 = temperatures[0] - off_t2
                    t1 = temperatures[1] - off_t1
                    x, y, z = accel_data
                ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                # Log at 50Hz
                writer.writerow([ts, f1, t1, f2, t2, x, y, z])
                f.flush()
                # Print at 1Hz
                if time.time() - last_print >= 1.0:
                    print(f"| F1: {f1:.2f} Lpm | T1: {t1:.2f} °C | F2: {f2:.2f} Lpm | T2: {t2:.2f} °C | "
                          f"X: {x:.2f} g | Y: {y:.2f} g | Z: {z:.2f} g")
                    last_print = time.time()
                time.sleep(0.02)  # 50Hz logging
        except KeyboardInterrupt:
            print("\nTerminated by User")
        except Exception as e:
            print(f"Main loop error: {e}")

if __name__ == "__main__":
    main()