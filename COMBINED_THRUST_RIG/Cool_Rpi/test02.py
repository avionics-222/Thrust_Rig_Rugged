#!/usr/bin/env python3
# cooling_system_monitor.py
# RPI2 - Cooling System Monitor with CAN Telemetry

import time
import csv
import threading
import os
from datetime import datetime
import logging
from struct import pack
from pymodbus.client import ModbusSerialClient
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from gpiozero import Button

from can_usb import setup_can, close_can, send_data_message

# ---- CAN Message IDs ----
TX_COOLING_PRESSURE_FLOW = 0x120
TX_COOLING_TEMPS = 0x121

# ---- Sensor Config ----
RS485_PORT = '/dev/ttyAMA0'
BAUD_RATE = 9600
REF = 5.2
SDA_PIN = 2
SCL_PIN = 3
FLOW_SENSOR_PINS = [23]
FLOW_FACTORS = [9.9]

pulse_counts = [0]
flow_rates = [0.0]
pressure_bars = [0.0]
temperatures = [0.0, 0.0]

off_t2 = 9.2 - 1.5 - 1.8 - 0.6
off_t1 = 6.7 - 2.8 + 0.9 - 0.3

data_lock = threading.Lock()
modbus_lock = threading.Lock()
i2c_lock = threading.Lock()
can_initialized = threading.Event()

logging.basicConfig(level=logging.CRITICAL)
logging.getLogger("pymodbus").setLevel(logging.CRITICAL)

# ---- Flow sensor ----
def pulse_inc_0():
    pulse_counts[0] += 1

flow_sensors = [Button(FLOW_SENSOR_PINS[0], pull_up=True)]
flow_sensors[0].when_pressed = pulse_inc_0

def calc_flow(pulse_count, factor):
    return round(pulse_count / factor, 2)

def flow_thread():
    while True:
        time.sleep(1)
        with data_lock:
            flow_rates[0] = calc_flow(pulse_counts[0], FLOW_FACTORS[0])
            pulse_counts[0] = 0

# ---- Pressure sensor thread ----
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
                pressure_bars[0] = bar
            time.sleep(0.1)
    except Exception as e:
        print(f"Pressure thread error: {e}")

# ---- RS485 Temp thread ----
def rs485_temp_thread():
    client = None
    try:
        client = ModbusSerialClient(
            port=RS485_PORT, baudrate=BAUD_RATE,
            parity='N', stopbits=1, bytesize=8, timeout=3
        )
        if not client.connect():
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
                    if not rr.isError() and rr.registers:
                        raw = rr.registers[0]
                        temp = (raw - 65536)/10 if (raw & 0x8000) else raw/10
                        with data_lock:
                            temperatures[idx] = temp
                    time.sleep(0.3)
    except:
        pass
    finally:
        if client:
            client.close()

# ---- CAN Transmit Thread ----
def can_transmit_thread():
    if not can_initialized.wait(timeout=10):
        return
    
    tx_success_count = 0
    tx_fail_count = 0
    last_print = time.time()
    tx_period = 0.0182
    last_tx_time = time.time()
    cycle_count = 0
    start_time = time.time()
    
    while True:
        current_time = time.time()
        if current_time - last_tx_time >= tx_period:
            last_tx_time = current_time
            cycle_count += 1
            
            with data_lock:
                p1_esc_in = pressure_bars[0]
                f1_esc_in = flow_rates[0]
                t2_esc_in = temperatures[0] - off_t2
                t1_motor_in = temperatures[1] - off_t1
            
            tx_batch_success = 0
            try:
                if send_data_message(TX_COOLING_PRESSURE_FLOW, pack('<ff', p1_esc_in, f1_esc_in)):
                    tx_batch_success += 1
                if send_data_message(TX_COOLING_TEMPS, pack('<ff', t1_motor_in, t2_esc_in)):
                    tx_batch_success += 1
                
                if tx_batch_success == 2:
                    tx_success_count += 1
                else:
                    tx_fail_count += 1
            except:
                tx_fail_count += 1
            
            # Print every 5 seconds
            if current_time - last_print > 5.0:
                elapsed_time = current_time - start_time
                actual_rate = cycle_count / elapsed_time if elapsed_time > 0 else 0
                print(f"Pressure: {p1_esc_in:5.3f}bar | Flow: {f1_esc_in:4.1f}lpm | " +
                      f"Motor T: {t1_motor_in:4.1f}°C | ESC T: {t2_esc_in:4.1f}°C | " +
                      f"CAN TX: {tx_success_count}/{tx_fail_count} | Rate: {actual_rate:.1f}Hz")
                last_print = current_time
        
        time.sleep(0.0001)

# ---- Main loop ----
def main():
    print("=" * 70)
    print("RPI2 - Cooling System Monitor with CAN Telemetry")
    print("=" * 70)
    
    # Initialize CAN
    if not setup_can():
        return
    can_initialized.set()
    
    # Setup CSV
    os.makedirs("logs", exist_ok=True)
    filename = f"logs/cooling_{datetime.now():%Y%m%d_%H%M%S}.csv"
    
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["Timestamp", "ESC_Inlet_Pressure_Bar", "ESC_Inlet_Flow_Lpm",
                        "Motor_Inlet_Temp_C", "ESC_Inlet_Temp_C"])
    
    # Start I2C and ADS1115
    i2c = busio.I2C(SCL_PIN, SDA_PIN)
    ads = ADS.ADS1115(i2c)
    ads.gain = 1
    
    # Start threads
    threading.Thread(target=pressure_thread, args=(ads,), daemon=True).start()
    threading.Thread(target=flow_thread, daemon=True).start()
    threading.Thread(target=rs485_temp_thread, daemon=True).start()
    threading.Thread(target=can_transmit_thread, daemon=True).start()
    
    print(f"Logging to: {filename}\n")
    
    # Logging loop
    with open(filename, 'a', newline='') as f:
        writer = csv.writer(f)
        try:
            while True:
                with data_lock:
                    p1_esc_in = pressure_bars[0]
                    f1_esc_in = flow_rates[0]
                    t2_esc_in = temperatures[0] - off_t2
                    t1_motor_in = temperatures[1] - off_t1
                
                ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                writer.writerow([ts, round(p1_esc_in, 4), round(f1_esc_in, 2),
                               round(t1_motor_in, 2), round(t2_esc_in, 2)])
                f.flush()
                time.sleep(1)
                
        except KeyboardInterrupt:
            pass
    
    close_can()
    print("\n\nRPI2 Cooling System Monitor stopped.\n")

if __name__ == "__main__":
    main()
