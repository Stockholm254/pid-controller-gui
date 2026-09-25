import serial
import time

try:
    # Use the parameters found in your remotecontroller.py line 311
    ser = serial.Serial('COM44', 112500, timeout=1)
    print("--- Success: COM17 is open ---")
    
    # Basic hardware info
    print(f"Port: {ser.name}")
    print(f"Baudrate: {ser.baudrate}")
    
    # Optional: Send a 'read setpoint' request if your hardware uses 
    # the protocol defined in your script (REMOTECONTROLLER_MSG_SIZE = 9)
    # This sends a 'read' opcode (0) for 'setpoint' (bit 0b0100)
    test_request = bytearray([0x20, 0, 0, 0, 0, 0, 0, 0, 0]) 
    ser.write(test_request)
    
    time.sleep(0.1)
    if ser.in_waiting > 0:
        response = ser.read(ser.in_waiting)
        print(f"Received raw data from hardware: {response.hex()}")
    else:
        print("Port opened, but no data received back. Check if hardware is powered.")

    ser.close()
    print("--- Port closed safely ---")

except serial.SerialException as e:
    print(f"--- Error: Could not connect ---")
    print(f"Reason: {e}")