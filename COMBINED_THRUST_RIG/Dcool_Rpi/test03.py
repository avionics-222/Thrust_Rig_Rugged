#!/usr/bin/env python3
# accel_cooling_monitor.py
# RPI3 - Accelerometer & Cooling Fin Monitor with CAN Telemetry

import time
import csv
import threading
import os
from datetime import datetime
import logging
from struct import pack
from pymodbus.client import ModbusSerialClient
from gpiozero import Button
import qwiic_kx13x
import sys

from can_usb import setup_can, close_can, send_data_message

# ---- CAN Message IDs ----
TX_ACCEL_DATA = 0x130
TX_ACCEL_Z = 0x131
TX_FIN_TEMPS = 0x132

# ---- Sensor Config ----
RS485_PORT = '/dev/ttyAMA0'
BAUD_RATE = 9600
SDA_PIN = 2
SCL_PIN = 3
FLOW_SENSOR_PINS = [23]
FLOW_FACTORS = [9.9]

pulse_counts = [0]
flow_rates = [0.0]
temperatures = [0.0, 0.0]
accel_data = [0.0, 0.0, 0.0]

off_fin_inlet = 6.7 + 0.1 - 2.9 + 1.0
off_fin_outlet = 9.2 - 2.9 - 2.9 - 0.5 + 0.5

data_lock = threading.Lock()
modbus_lock = threading.Lock()
i2c_lock = threading.Lock()
can_initialized = threading.Event()

logging.basicConfig(level=logging.CRITICAL)

# ---- Flow sensor ----
def pulse_inc():
    pulse_counts[0] += 1

flow_sensors = [Button(FLOW_SENSOR_PINS[0], pull_up=True)]
flow_sensors[0].when_pressed = pulse_inc

def calc_flow(pulse_count, factor):
    return round(pulse_count / factor, 2)

def flow_thread():
    while True:
        time.sleep(1)
        with data_lock:
            flow_rates[0] = calc_flow(pulse_counts[0], FLOW_FACTORS[0])
            pulse_counts[0] = 0

# ---- RS485 Temperature thread ----
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
            for idx, dev_id in enumerate([5, 6]):
                with modbus_lock:
                    if hasattr(client, "socket") and hasattr(client.socket, "reset_input_buffer"):
                        try:
                            client.socket.reset_input_buffer()
                        except:
                            pass
                    rr = client.read_holding_registers(address=0, count=1, device_id=dev_id)
                    if not rr.isError() and rr.registers:
                        raw = rr.registers[0]
                        temp = (raw - 65536) / 10 if (raw & 0x8000) else raw / 10
                        with data_lock:
                            temperatures[idx] = temp
                    time.sleep(0.3)
    except:
        pass
    finally:
        if client:
            client.close()

# ---- Accelerometer thread ----
def accel_thread(kx):
    try:
        if not kx.connected or not kx.begin():
            return
        
        kx.software_reset()
        kx.enable_accel(False)
        kx.set_output_data_rate(0x0A)
        kx.set_range(kx.KX134_RANGE32G)
        kx.enable_data_engine()
        kx.enable_accel()
        
        while True:
            if kx.data_ready():
                kx.get_accel_data()
                with i2c_lock:
                    with data_lock:
                        accel_data[0] = kx.kx134_accel.x
                        accel_data[1] = kx.kx134_accel.y
                        accel_data[2] = kx.kx134_accel.z
            time.sleep(0.02)
    except:
        pass

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
                flow_motor_in = flow_rates[0]
                fin_outlet_temp = temperatures[0] - off_fin_outlet
                fin_inlet_temp = temperatures[1] - off_fin_inlet
                x, y, z = accel_data
            
            tx_batch_success = 0
            try:
                if send_data_message(TX_ACCEL_DATA, pack('<ff', x, y)):
                    tx_batch_success += 1
                if send_data_message(TX_ACCEL_Z, pack('<ff', z, flow_motor_in)):
                    tx_batch_success += 1
                if send_data_message(TX_FIN_TEMPS, pack('<ff', fin_inlet_temp, fin_outlet_temp)):
                    tx_batch_success += 1
                
                if tx_batch_success == 3:
                    tx_success_count += 1
                else:
                    tx_fail_count += 1
            except:
                tx_fail_count += 1
            
            # Print every 5 seconds
            if current_time - last_print > 5.0:
                elapsed_time = current_time - start_time
                actual_rate = cycle_count / elapsed_time if elapsed_time > 0 else 0
                print(f"Accel: X={x:5.2f}g Y={y:5.2f}g Z={z:5.2f}g | Flow: {flow_motor_in:4.1f}lpm | " +
                      f"Fin In: {fin_inlet_temp:4.1f}°C Out: {fin_outlet_temp:4.1f}°C | " +
                      f"CAN TX: {tx_success_count}/{tx_fail_count} | Rate: {actual_rate:.1f}Hz")
                last_print = current_time
        
        time.sleep(0.0001)

# ---- Main loop ----
def main():
    print("=" * 70)
    print("RPI3 - Accelerometer & Cooling Fin Monitor with CAN Telemetry")
    print("=" * 70)
    
    # Initialize CAN
    if not setup_can():
        return
    can_initialized.set()
    
    # Setup CSV
    os.makedirs("logs", exist_ok=True)
    filename = f"logs/accel_fins_{datetime.now():%Y%m%d_%H%M%S}.csv"
    
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["Timestamp", "Flow_Motor_In_Lpm", "Fin_Outlet_Temp_C",
                        "Fin_Inlet_Temp_C", "X_g", "Y_g", "Z_g"])
    
    # Initialize Accelerometer
    kx = qwiic_kx13x.QwiicKX134()
    
    # Start threads
    threading.Thread(target=flow_thread, daemon=True).start()
    threading.Thread(target=rs485_temp_thread, daemon=True).start()
    threading.Thread(target=accel_thread, args=(kx,), daemon=True).start()
    threading.Thread(target=can_transmit_thread, daemon=True).start()
    
    print(f"Logging to: {filename}\n")
    
    # Logging loop
    with open(filename, 'a', newline='') as f:
        writer = csv.writer(f)
        try:
            while True:
                with data_lock:
                    flow_motor_in = flow_rates[0]
                    fin_outlet_temp = temperatures[0] - off_fin_outlet
                    fin_inlet_temp = temperatures[1] - off_fin_inlet
                    x, y, z = accel_data
                
                ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                writer.writerow([ts, round(flow_motor_in, 2), round(fin_outlet_temp, 2),
                               round(fin_inlet_temp, 2), round(x, 2), round(y, 2), round(z, 2)])
                f.flush()
                time.sleep(0.02)
                
        except KeyboardInterrupt:
            pass
    
    close_can()
    print("\n\nRPI3 Accel & Fins Monitor stopped.\n")

if __name__ == "__main__":
    main()
