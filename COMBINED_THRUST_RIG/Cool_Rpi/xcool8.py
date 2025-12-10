import time
import csv
import threading
import os
from datetime import datetime
import logging
from pymodbus.client import ModbusSerialClient
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from gpiozero import Button
# ---- RS485 & Sensor Config ----
RS485_PORT = '/dev/ttyAMA0'
BAUD_RATE = 9600 
REF = 5.2 # ADS1115 supply voltage
SDA_PIN = 2
SCL_PIN = 3
FLOW_SENSOR_PINS = [23]
FLOW_FACTORS = [9.9]
pulse_counts = [0]
flow_rates = [0.0] # f1_esc_in
pressure_bars = [0.0] # only p1_esc_in
temperatures = [0.0, 0.0] # [raw_t2_esc_in, raw_t1_motor_in]
# Temperature offsets
off_t2 = 9.2 - 1.5 - 1.8 - 0.6 # ESC inlet (ID-1 RS485)
off_t1 = 6.7 - 2.8 + 0.9 - 0.3 # Motor inlet (ID-3 RS485)
data_lock = threading.Lock()
modbus_lock = threading.Lock()
i2c_lock = threading.Lock() # Protects all I2C access
# ---- Logging config — SUPPRESSED: No more pymodbus errors in terminal ----
logging.basicConfig(level=logging.CRITICAL) # Global to CRITICAL (ignores ERROR)
logging.getLogger("pymodbus").setLevel(logging.CRITICAL)
logging.getLogger("pymodbus.logging").setLevel(logging.CRITICAL)
logging.getLogger("serial").setLevel(logging.CRITICAL)
# ---- Flow sensor ----
def pulse_inc_0():
    pulse_counts[0] += 1
flow_sensors = [
    Button(FLOW_SENSOR_PINS[0], pull_up=True)
]
flow_sensors[0].when_pressed = pulse_inc_0
def calc_flow(pulse_count, factor):
    return round(pulse_count / factor, 2)
def flow_thread():
    while True:
        time.sleep(1)
        with data_lock:
            flow_rates[0] = calc_flow(pulse_counts[0], FLOW_FACTORS[0]) # f1_esc_in
            pulse_counts[0] = 0
# ---- Pressure sensor thread (only ESC inlet pressure on A0) ----
def pressure_thread(ads):
    try:
        chan = AnalogIn(ads, ADS.P0)
        while True:
            with i2c_lock:
                voltage = chan.voltage
                if voltage < 0.5:
                    psi = 0.0
                elif voltage > (REF - 0.8):
                    psi = 100.0
                else:
                    psi = ((voltage - 0.5) / (REF - 0.5)) * 100.0
                bar = psi * 0.0689
                bar = 0 if (0.82 * bar - 0.017) < 0 else (0.82 * bar - 0.017)
            with data_lock:
                pressure_bars[0] = bar # p1_esc_in
            time.sleep(0.1)
    except Exception as e:
        print(f"Pressure thread error: {e}")
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
            for idx, dev_id in enumerate([1, 3]): # ID1 = ESC inlet, ID3 = Motor inlet
                with modbus_lock:
                    if hasattr(client, "socket") and hasattr(client.socket, "reset_input_buffer"):
                        try:
                            client.socket.reset_input_buffer()
                        except:
                            pass
                    rr = client.read_holding_registers(address=0, count=1, device_id=dev_id)
                    if not rr.isError() and rr.registers: # ONLY update if SUCCESS
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
    # CSV Header with proper labels
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            "Timestamp",
            "ESC_Inlet_Pressure_Bar",
            "ESC_Inlet_Flow_Lpm",
            "Motor_Inlet_Temp_C",
            "ESC_Inlet_Temp_C"
        ])
    # Start threads
    i2c = busio.I2C(SCL_PIN, SDA_PIN)
    ads = ADS.ADS1115(i2c)
    ads.gain = 1
    threading.Thread(target=pressure_thread, args=(ads,), daemon=True).start()
    threading.Thread(target=flow_thread, daemon=True).start()
    threading.Thread(target=rs485_temp_thread, daemon=True).start()
    # Logging loop
    with open(filename, 'a', newline='') as f:
        writer = csv.writer(f)
        try:
            while True:
                with data_lock:
                    p1_esc_in = pressure_bars[0]
                    f1_esc_in = flow_rates[0]
                    t2_esc_in = temperatures[0] - off_t2 # ESC inlet temp
                    t1_motor_in = temperatures[1] - off_t1 # Motor inlet temp
                ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                # Console print with clear labels
                print(f"| ESC Inlet Pressure: {p1_esc_in:6.3f} Bar | "
                      f"ESC Inlet Flow: {f1_esc_in:5.2f} Lpm | "
                      f"Motor Inlet Temp: {t1_motor_in:5.2f} °C | "
                      f"ESC Inlet Temp: {t2_esc_in:5.2f} °C")
                # CSV row (same order as header)
                writer.writerow([
                    ts,
                    round(p1_esc_in, 4),
                    round(f1_esc_in, 2),
                    round(t1_motor_in, 2),
                    round(t2_esc_in, 2)
                ])
                f.flush()
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nTerminated by User")
if __name__ == "__main__":
    main()