# can_usb.py
# Dual CAN channel support for Waveshare USB CAN B

import ctypes
import os

# Load the Waveshare library
try:
    lib_path = os.path.join(os.path.dirname(__file__), 'libcontrolcan.so')
    lib = ctypes.cdll.LoadLibrary(lib_path)
except Exception as e:
    print(f"ERROR: Cannot load libcontrolcan.so: {e}")
    print("Make sure libcontrolcan.so is in your project directory")
    import sys
    sys.exit(1)

# Device type constant
VCI_USBCAN2 = 4  # USB-CAN-B device type

# CAN message structure
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

# Init config structure
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
    """Setup both CAN channels on Waveshare USB CAN B"""
    try:
        # Open device (DevType=4, DevIndex=0, Reserved=0)
        result = lib.VCI_OpenDevice(VCI_USBCAN2, 0, 0)
        if result != 1:
            print(f"Failed to open USB CAN B device (result={result})")
            return False
        
        print("USB CAN B device opened")
        
        # Configure CAN1 (channel 0) - 500kbps - ESC/Battery communication
        init_config = VCI_INIT_CONFIG()
        init_config.AccCode = 0x00000000
        init_config.AccMask = 0xFFFFFFFF
        init_config.Filter = 1  # Dual filter
        init_config.Timing0 = 0x00
        init_config.Timing1 = 0x1C  # 500kbps
        init_config.Mode = 0  # Normal mode
        
        result = lib.VCI_InitCAN(VCI_USBCAN2, 0, 0, ctypes.byref(init_config))
        if result != 1:
            print(f"Failed to initialize CAN1 (result={result})")
            return False
        print("CAN1 initialized at 500kbps (ESC/Battery)")
        
        # Configure CAN2 (channel 1) - 500kbps - Telemetry broadcast
        result = lib.VCI_InitCAN(VCI_USBCAN2, 0, 1, ctypes.byref(init_config))
        if result != 1:
            print(f"Failed to initialize CAN2 (result={result})")
            return False
        print("CAN2 initialized at 500kbps (Telemetry)")
        
        # Start CAN1
        result = lib.VCI_StartCAN(VCI_USBCAN2, 0, 0)
        if result != 1:
            print(f"Failed to start CAN1 (result={result})")
            return False
        print("CAN1 started")
        
        # Start CAN2
        result = lib.VCI_StartCAN(VCI_USBCAN2, 0, 1)
        if result != 1:
            print(f"Failed to start CAN2 (result={result})")
            return False
        print("CAN2 started")
        
        print("? USB CAN B: Both channels ready at 500kbps\n")
        return True
        
    except Exception as e:
        print(f"CAN setup error: {e}")
        return False

def send_message(arbitration_id, data):
    """Send message on CAN1 (channel 0) for ESC commands"""
    try:
        msg = VCI_CAN_OBJ()
        msg.ID = arbitration_id
        msg.SendType = 0  # Normal send
        msg.RemoteFlag = 0  # Data frame
        msg.ExternFlag = 0  # Standard frame
        msg.DataLen = len(data)
        
        for i in range(len(data)):
            msg.Data[i] = data[i]
        
        result = lib.VCI_Transmit(VCI_USBCAN2, 0, 0, ctypes.byref(msg), 1)
        return result == 1
        
    except Exception as e:
        return False

def send_data_message(arbitration_id, data):
    """Send message on CAN2 (channel 1) for telemetry broadcast"""
    try:
        msg = VCI_CAN_OBJ()
        msg.ID = arbitration_id
        msg.SendType = 0  # Normal send
        msg.RemoteFlag = 0  # Data frame
        msg.ExternFlag = 0  # Standard frame
        
        # Convert bytes to list if needed
        if isinstance(data, bytes):
            data = list(data)
        
        msg.DataLen = len(data)
        for i in range(len(data)):
            msg.Data[i] = data[i]
        
        result = lib.VCI_Transmit(VCI_USBCAN2, 0, 1, ctypes.byref(msg), 1)
        return result == 1
        
    except Exception as e:
        return False

def receive_messages(timeout_ms=10):
    """Receive messages from CAN1 (channel 0)"""
    messages = []
    
    try:
        # Allocate array for received messages
        msg_buffer = (VCI_CAN_OBJ * 100)()
        
        # VCI_Receive(DevType, DevIndex, CANIndex, pReceive, Len, WaitTime_ms)
        count = lib.VCI_Receive(VCI_USBCAN2, 0, 0, ctypes.byref(msg_buffer), 100, timeout_ms)
        
        if count > 0:
            for i in range(count):
                msg = msg_buffer[i]
                messages.append({
                    "arbitration_id": msg.ID,
                    "data": list(msg.Data[:msg.DataLen])
                })
    except Exception as e:
        pass
    
    return messages

def clear_can_buffer():
    """Clear receive buffer on CAN1"""
    try:
        lib.VCI_ClearBuffer(VCI_USBCAN2, 0, 0)
    except:
        pass

def close_can():
    """Close both CAN channels"""
    try:
        lib.VCI_CloseDevice(VCI_USBCAN2, 0)
        print("USB CAN B closed")
    except Exception as e:
        print(f"Close error: {e}")

def close_can_tx():
    """CAN TX channel is closed with close_can()"""
    pass
