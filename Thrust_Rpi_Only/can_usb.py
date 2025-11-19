# can_usb.py
# Waveshare USB-CAN-B Driver (NO SocketCAN, NO can0)
# Works 100% reliably with libcontrolcan.so 

import ctypes
import time
import logging
from ctypes import *

VCI_USBCAN2 = 4
STATUS_OK = 1

class VCI_INIT_CONFIG(Structure):
    _fields_ = [
        ("AccCode", c_uint), ("AccMask", c_uint), ("Reserved", c_uint),
        ("Filter", c_ubyte), ("Timing0", c_ubyte), ("Timing1", c_ubyte), ("Mode", c_ubyte)
    ]

class VCI_CAN_OBJ(Structure):
    _fields_ = [
        ("ID", c_uint), ("TimeStamp", c_uint), ("TimeFlag", c_ubyte),
        ("SendType", c_ubyte), ("RemoteFlag", c_ubyte), ("ExternFlag", c_ubyte),
        ("DataLen", c_ubyte), ("Data", c_ubyte * 8), ("Reserved", c_ubyte * 3)
    ]

# Try multiple possible library locations
lib = None
for path in ["/home/user/Sensor/Loadcell/libcontrolcan.so"]:
    try:
        lib = cdll.LoadLibrary(path)
        logging.info(f"Loaded libcontrolcan.so from {path}")
        break
    except:
        continue

if lib is None:
    raise RuntimeError("libcontrolcan.so not found! Install Waveshare USB-CAN driver.")

# CAN IDs
BAMOCAR_ID = 0x201
BAMOCAR_RESPONSE_ID = 0x181
BATTERY_ID = 0x0222

# Global state
_initialized = False

def setup_can():
    global _initialized
    if _initialized:
        return True

    if lib.VCI_OpenDevice(VCI_USBCAN2, 0, 0) != STATUS_OK:
        raise RuntimeError("VCI_OpenDevice failed - Is USB-CAN-B plugged in and driver installed?")

    config = VCI_INIT_CONFIG()
    config.AccCode = 0
    config.AccMask = 0xFFFFFFFF
    config.Filter = 0
    config.Timing0 = 0x00  # 500kbps
    config.Timing1 = 0x1C  # Standard for 500k
    config.Mode = 0        # Normal mode

    if lib.VCI_InitCAN(VCI_USBCAN2, 0, 0, byref(config)) != STATUS_OK:
        lib.VCI_CloseDevice(VCI_USBCAN2, 0)
        raise RuntimeError("VCI_InitCAN failed")

    if lib.VCI_StartCAN(VCI_USBCAN2, 0, 0) != STATUS_OK:
        lib.VCI_CloseDevice(VCI_USBCAN2, 0)
        raise RuntimeError("VCI_StartCAN failed")

    _initialized = True
    logging.info("USB-CAN-B initialized successfully (500kbps)")
    return True

def close_can():
    global _initialized
    if _initialized:
        lib.VCI_CloseDevice(VCI_USBCAN2, 0)
        _initialized = False
        logging.info("USB-CAN-B closed")

def send_message(arbitration_id, data):
    if not _initialized:
        return False
    obj = VCI_CAN_OBJ()
    obj.ID = arbitration_id
    obj.SendType = 0
    obj.RemoteFlag = 0
    obj.ExternFlag = 0
    obj.DataLen = len(data)
    for i in range(min(len(data), 8)):
        obj.Data[i] = data[i]
    return lib.VCI_Transmit(VCI_USBCAN2, 0, 0, byref(obj), 1) > 0

def receive_messages(timeout_ms=5):
    if not _initialized:
        return []
    count = lib.VCI_GetReceiveNum(VCI_USBCAN2, 0, 0)
    if count == 0:
        return []
    objs = (VCI_CAN_OBJ * count)()
    received = lib.VCI_Receive(VCI_USBCAN2, 0, 0, byref(objs), count, timeout_ms)
    result = []
    for i in range(received):
        obj = objs[i]
        data = bytes(obj.Data[:obj.DataLen])
        result.append({"arbitration_id": obj.ID, "data": data})
    return result

def clear_can_buffer():
    logging.info("Clearing USB-CAN buffer...")
    start = time.time()
    while time.time() - start < 2.0:
        count = lib.VCI_GetReceiveNum(VCI_USBCAN2, 0, 0)
        if count == 0:
            break
        lib.VCI_Receive(VCI_USBCAN2, 0, 0, None, 1000, 1)
    logging.info("USB-CAN buffer cleared")