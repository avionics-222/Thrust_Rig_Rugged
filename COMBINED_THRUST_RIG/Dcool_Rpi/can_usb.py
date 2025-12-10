# can_usb.py
# CAN support for RPI2 - Only CAN2 TX (no CAN1 needed)

import ctypes
import os

# Load the Waveshare library
try:
    lib_path = os.path.join(os.path.dirname(__file__), 'libcontrolcan.so')
    lib = ctypes.cdll.LoadLibrary(lib_path)
except Exception as e:
    print(f"ERROR: Cannot load libcontrolcan.so: {e}")
    import sys
    sys.exit(1)

VCI_USBCAN2 = 4

class VCI_CAN_OBJ(ctypes.Structure):
    _fields_ = [
        ("ID", ctypes.c_uint),
        ("TimeStamp", ctypes.c_uint),
        ("TimeFlag", ctypes.c_byte),
        ("SendType", ctypes.c_byte),
        ("RemoteFlag", ctypes.c_byte),
        ("ExternFlag", ctypes.c_byte),
        ("DataLen", ctypes.c_byte),
        ("Data", ctypes.c_byte * 8),
        ("Reserved", ctypes.c_byte * 3)
    ]

class VCI_INIT_CONFIG(ctypes.Structure):
    _fields_ = [
        ("AccCode", ctypes.c_uint),
        ("AccMask", ctypes.c_uint),
        ("Reserved", ctypes.c_uint),
        ("Filter", ctypes.c_byte),
        ("Timing0", ctypes.c_byte),
        ("Timing1", ctypes.c_byte),
        ("Mode", ctypes.c_byte)
    ]

def setup_can():
    """Setup only CAN2 for transmitting (RPI2 doesn't need CAN1)"""
    try:
        # Open device
        result = lib.VCI_OpenDevice(VCI_USBCAN2, 0, 0)
        if result != 1:
            print(f"Failed to open USB CAN B device (result={result})")
            return False
        
        print("USB CAN B device opened")
        
        # Configure CAN2 (channel 1) - 500kbps - Telemetry broadcast
        init_config = VCI_INIT_CONFIG()
        init_config.AccCode = 0x00000000
        init_config.AccMask = 0xFFFFFFFF
        init_config.Filter = 1
        init_config.Timing0 = 0x00
        init_config.Timing1 = 0x1C  # 500kbps
        init_config.Mode = 0
        
        result = lib.VCI_InitCAN(VCI_USBCAN2, 0, 1, ctypes.byref(init_config))
        if result != 1:
            print(f"Failed to initialize CAN2 (result={result})")
            return False
        print("CAN2 initialized at 500kbps (Cooling System Telemetry)")
        
        # Start CAN2
        result = lib.VCI_StartCAN(VCI_USBCAN2, 0, 1)
        if result != 1:
            print(f"Failed to start CAN2 (result={result})")
            return False
        print("CAN2 started")
        
        print("✓ USB CAN B: CAN2 ready at 500kbps\n")
        return True
        
    except Exception as e:
        print(f"CAN setup error: {e}")
        return False

def send_data_message(arbitration_id, data):
    """Send message on CAN2 (channel 1)"""
    try:
        msg = VCI_CAN_OBJ()
        msg.ID = arbitration_id
        msg.SendType = 0
        msg.RemoteFlag = 0
        msg.ExternFlag = 0
        
        if isinstance(data, bytes):
            data = list(data)
        
        msg.DataLen = len(data)
        for i in range(len(data)):
            msg.Data[i] = data[i]
        
        result = lib.VCI_Transmit(VCI_USBCAN2, 0, 1, ctypes.byref(msg), 1)
        return result == 1
        
    except Exception as e:
        return False

def close_can():
    """Close CAN device"""
    try:
        lib.VCI_CloseDevice(VCI_USBCAN2, 0)
        print("USB CAN B closed")
    except Exception as e:
        print(f"Close error: {e}")
