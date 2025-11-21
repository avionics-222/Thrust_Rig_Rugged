# thrust_rig_monitor.py
# ideaForge Thrust Rig Monitoring System — FINAL CLEAN VERSION

import curses
import time
import multiprocessing
import os
import csv
from datetime import datetime
from math import sqrt

from hx711_module import HX711
from can_usb import setup_can, close_can, send_message, receive_messages, clear_can_buffer

# === CONFIG ===
BAMOCAR_ID = 0x201
BAMOCAR_RESPONSE_ID = 0x181
BATTERY_ID = 0x0222

RPM_REGISTER = 0xA8
TORQUE_REGISTER = 0xA0
KT_REGISTER = 0x87
CURRENT_REGISTER = 0x20
TEMP_REGISTER = 0x49
VDC_REGISTER = 0xEB
VOUT_VXXX = 0x8A
IGBT_TEMP = 0x4A

NOMINAL_RPM = 6000
CAN_CYCLIC_RATE = 0x3C
KT_DEFAULT = 0.88

LOAD_CELL_PINS = [(12,13), (20,21), (6,5), (23,24)]
OFFSETS = [-152333.5, -2636403.0, -826961.0, 3403535.0]
LABELS = ["Thrust_0Deg", "Thrust_90Deg", "Thrust_180Deg", "Thrust_270Deg"]

data_queue = multiprocessing.Queue()
running = multiprocessing.Event()
running.set()

# === WORKERS ===
def loadcell_worker(dout, sck, offset, idx):
    try:
        hx = HX711(dout, sck)
        hx.offset = offset
        while running.is_set():
            try:
                w, _ = hx.get_weight()
                data_queue.put(('weight', idx, round(w, 3)))
                time.sleep(0.4)
            except:
                time.sleep(0.1)
    except:
        pass

def can_worker():
    kt = KT_DEFAULT
    vdc = 50.0
    setup_can()
    time.sleep(0.6)
    send_message(BAMOCAR_ID, [0x3D, KT_REGISTER, 0x00])
    time.sleep(0.6)
    for reg in [RPM_REGISTER, TORQUE_REGISTER, TEMP_REGISTER, CURRENT_REGISTER,
                VDC_REGISTER, VOUT_VXXX, IGBT_TEMP]:
        send_message(BAMOCAR_ID, [0x3D, reg, CAN_CYCLIC_RATE])
        time.sleep(0.1)

    while running.is_set():
        try:
            msgs = receive_messages(10)
            for m in msgs:
                aid = m["arbitration_id"]
                d = m["data"]
                if aid == BAMOCAR_RESPONSE_ID and len(d) >= 3:
                    reg = d[0]
                    raw = (d[2] << 8) | d[1]
                    if raw > 32767: raw -= 65536
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
                        data_queue.put(('ESC_Temp', round(6e-8 * raw**2 + 0.0032 * raw - 23.236)))
                elif aid == BATTERY_ID and len(d) == 8:
                    b = ''.join(f'{x:08b}' for x in d)
                    v1 = int(b[0:14], 2) * 0.1
                    v2 = int(b[14:28], 2) * 0.1
                    p = int(b[28:46], 2)
                    if p & (1 << 17): p -= (1 << 18)
                    data_queue.put(('BattV', round(v1, 2)))
                    data_queue.put(('BusV', round(v2, 2)))
                    data_queue.put(('Power', round(p * 10, 1)))
            time.sleep(0.001)
        except:
            continue

# === CURSES DISPLAY ===
def curses_main(stdscr):
    curses.curs_set(0)
    stdscr.timeout(100)
    curses.start_color()
    curses.init_pair(1, curses.COLOR_CYAN, curses.COLOR_BLACK)
    curses.init_pair(2, curses.COLOR_GREEN, curses.COLOR_BLACK)   # Green for header & thrust
    curses.init_pair(3, curses.COLOR_YELLOW, curses.COLOR_BLACK)
    curses.init_pair(4, curses.COLOR_RED, curses.COLOR_BLACK)
    curses.init_pair(5, curses.COLOR_WHITE, curses.COLOR_BLUE)

    # Data
    weights = [0.0] * 4
    rpm = esc_torque = motor_temp = current = vdc = vout = esc_temp = 0.0
    batt_v = bus_v = power = 0.0

    # CSV
    os.makedirs("logs", exist_ok=True)
    csvfile = open(f"logs/thrust_rig_{datetime.now():%Y%m%d_%H%M%S}.csv", "w", newline="")
    writer = csv.writer(csvfile)
    writer.writerow(["Time","T0","T90","T180","T270","Total_kg","Thrust_N",
                     "RPM","ESC_Torque_Nm","Motor_Temp_C","ESC_Temp_C","Current_A","Vdc","Vout",
                     "Batt_V","Bus_V","Power_W"])

    while running.is_set():
        stdscr.erase()
        h, w = stdscr.getmaxyx()

        if h < 30 or w < 100:
            stdscr.addstr(0, 0, "Terminal too small! Resize to at least 100x30", curses.A_BOLD)
            stdscr.refresh()
            time.sleep(0.5)
            continue

        # GREEN HEADER
        title = " ideaForge Thrust Rig Monitoring System "
        safe_addstr(stdscr, 0, 0, title.center(w), curses.A_BOLD | curses.color_pair(2) | curses.A_REVERSE)

        subtitle = f"Live • {datetime.now():%Y-%m-%d %H:%M:%S} • Press 'q' to quit"
        safe_addstr(stdscr, 1, 0, subtitle.center(w), curses.color_pair(2))

        safe_addstr(stdscr, 3, 0, "─" * w, curses.color_pair(1))

        # Thrust
        total_kg = sum(weights)
        thrust_n = total_kg * 9.81
        safe_addstr(stdscr, 5, 4, "THRUST MEASUREMENT", curses.A_BOLD | curses.color_pair(2))
        for i, label in enumerate(LABELS):
            safe_addstr(stdscr, 7+i, 6, f"{label:<12}: {weights[i]:8.3f} kg")
        safe_addstr(stdscr, 11, 6, "─" * 35)
        safe_addstr(stdscr, 12, 6, f"TOTAL WEIGHT : {total_kg:8.3f} kg", curses.A_BOLD)
        safe_addstr(stdscr, 13, 6, f"NET THRUST : {thrust_n:8.2f} N", curses.A_BOLD | curses.color_pair(2))

        # ESC & MOTOR STATUS — ESC Temp UNDER Motor Temp
        safe_addstr(stdscr, 5, 52, "ESC & MOTOR STATUS", curses.A_BOLD | curses.color_pair(2))
        safe_addstr(stdscr, 7, 54, f"RPM          : {rpm:8.1f}")
        safe_addstr(stdscr, 8, 54, f"ESC_Torque   : {esc_torque:8.3f} Nm")
        safe_addstr(stdscr, 9, 54, f"Motor Temp   : {motor_temp:6.1f} °C")
        safe_addstr(stdscr, 10, 54, f"ESC Temp     : {esc_temp:6.1f} °C")   # ← Now on next line
        safe_addstr(stdscr, 11, 54, f"Current      : {current:8.2f} A")
        safe_addstr(stdscr, 12, 54, f"Vdc          : {vdc:8.2f} V")
        safe_addstr(stdscr, 13, 54, f"Vout (Phase) : {vout:8.2f} V")

        # Battery
        safe_addstr(stdscr, 16, 52, "BATTERY & POWER", curses.A_BOLD | curses.color_pair(2))
        safe_addstr(stdscr, 18, 54, f"Battery Volt : {batt_v:8.2f} V")
        safe_addstr(stdscr, 19, 54, f"Bus Voltage  : {bus_v:8.2f} V")
        safe_addstr(stdscr, 20, 54, f"Power Output : {power:8.1f} W")

        # Status bar
        status = "RUNNING" if rpm > 100 else "STANDBY"
        safe_addstr(stdscr, h-3, 0, " " * w, curses.A_REVERSE)
        safe_addstr(stdscr, h-3, 4, f" STATUS: {status} ", curses.color_pair(2) | curses.A_BOLD)

        footer = "ideaForge Internal • Thrust Rig v10 • USB-CAN-B • HX711 ×4"
        safe_addstr(stdscr, h-1, 0, footer.center(w), curses.A_DIM)

        stdscr.refresh()

        # Update data
        while not data_queue.empty():
            try:
                typ, *vals = data_queue.get_nowait()
                if typ == 'weight':
                    weights[vals[0]] = vals[1]
                elif typ == 'RPM': rpm = vals[0]
                elif typ == 'ESC_Torque': esc_torque = vals[0]
                elif typ == 'Motor_Temp': motor_temp = vals[0]
                elif typ == 'Current': current = vals[0]
                elif typ == 'Vdc': vdc = vals[0]
                elif typ == 'Vout': vout = vals[0]
                elif typ == 'ESC_Temp': esc_temp = vals[0]
                elif typ == 'BattV': batt_v = vals[0]
                elif typ == 'BusV': bus_v = vals[0]
                elif typ == 'Power': power = vals[0]
            except:
                pass

        # CSV — Motor_Temp and ESC_Temp still next to each other in columns
        row = [datetime.now().strftime("%H:%M:%S.%f")[:-3]] + weights + \
              [round(total_kg,3), round(thrust_n,2), rpm, esc_torque, motor_temp, esc_temp,
               current, vdc, vout, batt_v, bus_v, power]
        writer.writerow(row)
        csvfile.flush()

        ch = stdscr.getch()
        if ch in [ord('q'), ord('Q')]:
            break

    csvfile.close()

# Safe addstr
def safe_addstr(win, y, x, text, attr=0):
    try:
        h, w = win.getmaxyx()
        if y < h and x < w:
            win.addstr(y, x, text[:w-x-1], attr)
    except:
        pass

# === MAIN ===
def main():
    print("Starting ideaForge Thrust Rig Monitoring System...")
    p1 = multiprocessing.Process(target=can_worker)
    p1.start()
    procs = [p1]
    for i in range(4):
        p = multiprocessing.Process(target=loadcell_worker,
                                   args=(LOAD_CELL_PINS[i][0], LOAD_CELL_PINS[i][1], OFFSETS[i], i))
        p.start()
        procs.append(p)

    try:
        curses.wrapper(curses_main)
    except:
        pass
    finally:
        running.clear()
        time.sleep(0.6)
        for p in procs:
            p.terminate()
            p.join(2)
        clear_can_buffer()
        close_can()
        print("\nideaForge Thrust Rig Monitor stopped cleanly.")
        print("All data saved to logs/ folder.\n")

if __name__ == "__main__":
    main()

