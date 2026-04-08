#!/usr/bin/env python3
"""
Test script for PT-IO-Mini2 JSON protocol
Usage: python test_protocol.py COM3
"""

import serial
import json
import time
import sys

def send_cmd(ser, cmd_dict):
    """Send a JSON command and print response"""
    cmd_json = json.dumps(cmd_dict)
    print(f"\n→ {cmd_json}")
    ser.write((cmd_json + '\n').encode())
    ser.flush()
    
    # Read response (could be multiple lines for telemetry)
    time.sleep(0.1)
    while ser.in_waiting:
        line = ser.readline().decode().strip()
        if line:
            try:
                resp = json.loads(line)
                print(f"← {json.dumps(resp, indent=2)}")
            except:
                print(f"← {line}")

def main():
    if len(sys.argv) < 2:
        print("Usage: python test_protocol.py COM3")
        sys.exit(1)
    
    port = sys.argv[1]
    
    print(f"Connecting to {port}...")
    ser = serial.Serial(port, 115200, timeout=1)
    time.sleep(2)  # Wait for connection
    
    # Read any initial messages (hello)
    print("\n=== Initial Messages ===")
    time.sleep(0.5)
    while ser.in_waiting:
        line = ser.readline().decode().strip()
        if line:
            try:
                msg = json.loads(line)
                print(f"← {json.dumps(msg, indent=2)}")
            except:
                print(f"← {line}")
    
    # Test getHello
    print("\n=== Test 1: getHello ===")
    send_cmd(ser, {"cmd": "getHello"})
    
    # Test getStatus
    print("\n=== Test 2: getStatus ===")
    send_cmd(ser, {"cmd": "getStatus"})
    
    # Test getConfig
    print("\n=== Test 3: getConfig ===")
    send_cmd(ser, {"cmd": "getConfig"})
    
    # Test setOutput
    print("\n=== Test 4: setOutput (ch 1 to 50%) ===")
    send_cmd(ser, {"cmd": "setOutput", "ch": 1, "duty": 50})
    
    # Test setOutput multiple channels
    print("\n=== Test 5: setOutput (ch 2 to 75%, ch 3 to 25%) ===")
    send_cmd(ser, {"cmd": "setOutput", "ch": 2, "duty": 75})
    send_cmd(ser, {"cmd": "setOutput", "ch": 3, "duty": 25})
    
    # Test setSerialOverride (mask mode)
    print("\n=== Test 6: setSerialOverride (mask 0x07 = channels 1-3) ===")
    send_cmd(ser, {"cmd": "setSerialOverride", "mask": 0x07})
    
    # Test setSerialOverride (per-channel mode)
    print("\n=== Test 7: setSerialOverride (ch 4 enabled) ===")
    send_cmd(ser, {"cmd": "setSerialOverride", "ch": 4, "enabled": True})
    
    # Test setDebounce
    print("\n=== Test 8: setDebounce (20ms) ===")
    send_cmd(ser, {"cmd": "setDebounce", "ms": 20})
    
    # Test subscribe to telemetry
    print("\n=== Test 9: Subscribe to telemetry (1 Hz) ===")
    send_cmd(ser, {"cmd": "subscribe", "stream": "telemetry", "interval": 1000})
    
    print("\n=== Receiving telemetry (10 seconds) ===")
    start = time.time()
    while time.time() - start < 10:
        if ser.in_waiting:
            line = ser.readline().decode().strip()
            if line:
                try:
                    msg = json.loads(line)
                    if msg.get("type") == "telemetry":
                        print(f"← Telemetry: AI={msg.get('analog_in', 'N/A')}, DI={msg.get('digital_in', 'N/A')}")
                except:
                    print(f"← {line}")
        time.sleep(0.1)
    
    # Test unsubscribe
    print("\n=== Test 10: Unsubscribe ===")
    send_cmd(ser, {"cmd": "unsubscribe"})
    
    # Test invalid command (error handling)
    print("\n=== Test 11: Invalid command (error test) ===")
    send_cmd(ser, {"cmd": "invalidCommand"})
    
    # Test text command compatibility
    print("\n=== Test 12: Text command compatibility ===")
    print("→ STATUS")
    ser.write(b"STATUS\n")
    time.sleep(0.2)
    while ser.in_waiting:
        print(f"← {ser.readline().decode().strip()}")
    
    print("\n=== Tests Complete ===")
    ser.close()

if __name__ == "__main__":
    main()
