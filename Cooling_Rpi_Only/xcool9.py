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
SDA_PIN = 2
SCL_PIN = 3
FLOW_SENSOR_PINS = [23]  # Motor inlet flow sensor only
FLOW_FACTORS = [9.9]
pulse_counts = [0]
flow_rates = [0.0]  # flow_motor_in
temperatures = [0.0, 0.0]  # [raw_fin_inlet, raw_fin_outlet]
accel_data = [0.0, 0.0, 0.0]  # X, Y, Z in g

# Temperature offsets
off_fin_inlet = 9.2 - 2.9  # Fin inlet (ID-5 RS485)
off_fin_outlet = 6.7 + 0.1 # Fin outlet (ID-6 RS485)

data_lock = threading.Lock()
modbus_lock = threading.Lock()
i2c_lock = threading.Lock()  # Protects all I2C access (now used for accel)

# ---- Logging config — SUPPRESSED: No more pymodbus errors in terminal ----
logging.basicConfig(level=logging.CRITICAL)  # Global to CRITICAL (ignores ERROR)
logging.getLogger("pymodbus").setLevel(logging.CRITICAL)
logging.getLogger("pymodbus.logging").setLevel(logging.CRITICAL)
logging.getLogger("serial").setLevel(logging.CRITICAL)

# ---- Flow sensor ----
def pulse_inc():
    pulse_counts[0] += 1

flow_sensors = [
    Button(FLOW_SENSOR_PINS[0], pull_up=True)
]
flow_sensors[0].when_pressed = pulse_inc

def calc_flow(pulse_count, factor):
    return round(pulse_count / factor, 2)

def flow_thread():
    while True:
        time.sleep(1)
        with data_lock:
            flow_rates[0] = calc_flow(pulse_counts[0], FLOW_FACTORS[0])  # flow_motor_in
            pulse_counts[0] = 0

# ---- RS485 Temp thread — FIXED: Skip update on error (no temp=0.0 reset) ----
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
            for idx, dev_id in enumerate([5, 6]):  # ID5 = Fin inlet, ID6 = Fin outlet
                with modbus_lock:
                    if hasattr(client, "socket") and hasattr(client.socket, "reset_input_buffer"):
                        try:
                            client.socket.reset_input_buffer()
                        except:
                            pass
                    rr = client.read_holding_registers(address=0, count=1, device_id=dev_id)
                    if not rr.isError() and rr.registers:  # ONLY update if SUCCESS
                        raw = rr.registers[0]
                        temp = (raw - 65536)/10 if (raw & 0x8000) else raw/10
                        with data_lock:
                            temperatures[idx] = temp
                    # On error (wrong ID, etc.): SKIP — keep last good temp (no reset to 0.0)
                    time.sleep(0.3)
    except Exception as e:
        print(f"RS485 temp thread error: {e}")
    finally:
        if client:
            client.close()

# ---- Accelerometer thread — NEW: 50Hz sampling ----
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
        kx.set_output_data_rate(0x0A) # 50Hz
        kx.set_range(kx.KX134_RANGE32G) # For KX134, use KX132_RANGE16G for KX132
        kx.enable_data_engine()
        kx.enable_accel()
        while True:
            if kx.data_ready():
                kx.get_accel_data()
                with i2c_lock:  # Use existing I2C lock for safety
                    with data_lock:
                        accel_data[0] = kx.kx134_accel.x
                        accel_data[1] = kx.kx134_accel.y
                        accel_data[2] = kx.kx134_accel.z
            time.sleep(0.02) # 50Hz
    except Exception as e:
        print(f"Accelerometer thread error: {e}")

# ---- Main loop — UPDATED: 50Hz logging, 1Hz print ----
def main():
    loc = input("Enter sensor location (default: Clog): ").strip()
    if not loc:
        loc = "Clog"
    now = datetime.now()
    timestamp_suffix = now.strftime("%Y%m%d_%H%M%S")
    log_folder = "logs"
    os.makedirs(log_folder, exist_ok=True)
    filename = os.path.join(log_folder, f"{loc}_{timestamp_suffix}.csv")

    # CSV Header with proper labels (added accel columns)
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            "Timestamp",
            "Flow_Motor_In_Lpm",
            "Fin_Inlet_Temp_C",
            "Fin_Outlet_Temp_C",
            "X_g",
            "Y_g",
            "Z_g"
        ])

    # Initialize Accelerometer
    kx = qwiic_kx13x.QwiicKX134() # Use QwiicKX132() for KX132

    # Start threads
    threading.Thread(target=flow_thread, daemon=True).start()
    threading.Thread(target=rs485_temp_thread, daemon=True).start()
    threading.Thread(target=accel_thread, args=(kx,), daemon=True).start()

    # Logging loop — 50Hz CSV logging, 1Hz console print
    with open(filename, 'a', newline='') as f:
        writer = csv.writer(f)
        last_print = time.time()
        try:
            while True:
                with data_lock:
                    flow_motor_in = flow_rates[0]
                    fin_inlet = temperatures[0] - off_fin_inlet  # Fin inlet temp
                    fin_outlet = temperatures[1] - off_fin_outlet  # Fin outlet temp
                    x, y, z = accel_data
                ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                # CSV row at 50Hz (low-rate data repeated)
                writer.writerow([
                    ts,
                    round(flow_motor_in, 2),
                    round(fin_inlet, 2),
                    round(fin_outlet, 2),
                    round(x, 2),
                    round(y, 2),
                    round(z, 2)
                ])
                f.flush()
                # Console print at 1Hz
                if time.time() - last_print >= 1.0:
                    print(f"| Flow Motor In: {flow_motor_in:5.2f} Lpm | "
                          f"Fin Inlet Temp: {fin_inlet:5.2f} °C | "
                          f"Fin Outlet Temp: {fin_outlet:5.2f} °C | "
                          f"X: {x:5.2f} g | Y: {y:5.2f} g | Z: {z:5.2f} g")
                    last_print = time.time()
                time.sleep(0.02)  # 50Hz loop
        except KeyboardInterrupt:
            print("\nTerminated by User")

if __name__ == "__main__":
    main()