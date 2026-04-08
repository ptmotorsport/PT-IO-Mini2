# PT-IO-Mini2 — GUI Communication Protocol

## Overview

The PT-IO-Mini2 device supports bidirectional communication over USB Serial (CDC) using either **text commands** (CLI) or **JSON messages** (GUI). Both protocols can coexist—the device auto-detects message format and responds accordingly.

**Transport:** USB CDC Serial, 115200 baud (default on RA4M1 USB CDC)  
**Framing:** Newline-delimited (`\n`)  
**Encoding:** UTF-8 text (JSON) or ASCII (text CLI)  
**Protocol Version:** 1  
**Firmware Version:** Reported in hello message as `fw` field

---

## Message Format

All messages are **newline-terminated** strings. The device uses `\n` as the frame delimiter.

### JSON Messages

JSON messages start with `{` and are auto-detected by the firmware.

**Structure:**
```json
{"cmd":"<command>", "param1":value1, ...}\n
```

**Response:**
```json
{"type":"response", "status":"ok"}\n
```

**Error:**
```json
{"type":"error", "msg":"error description"}\n
```

### Text Commands (CLI)

Text commands are uppercase strings (e.g., `STATUS`, `OUT 3 50`). See existing CLI documentation for details. Text commands remain fully supported for backward compatibility.

---

## Connection Handshake

When the device detects a serial connection (USB CDC enumeration), it automatically sends a **hello** message:

```json
{
  "type": "hello",
  "version": 1,
  "fw": 6,
  "canMode": 0,
  "canModeName": "PT_Default1",
  "capabilities": ["analog", "digital", "pwm", "can", "config"],
  "analogChannels": 8,
  "digitalIn": 8,
  "digitalOut": 8
}
```

**Fields:**
- `version`: Protocol version (currently 1)
- `fw`: Firmware version (hex, e.g., `0x06` for v0.0.6)
- `canMode`: Active CAN mode ID (0-15)
- `canModeName`: Human-readable CAN mode name
- `capabilities`: Array of supported features
- `analogChannels`: Number of analog input channels
- `digitalIn`: Number of digital input channels with freq/duty capture
- `digitalOut`: Number of PWM output channels

**Important:** If your app connects after the device has booted, or if you miss the initial hello message, use the `getHello` command to retrieve device information on-demand (see command reference below).

**Usage:** GUI should parse this message to auto-configure UI and validate compatibility.

---

## JSON Commands (GUI → Device)

### 1. Subscribe to Telemetry

**Request:**
```json
{"cmd":"subscribe", "stream":"telemetry", "interval":100}
```

**Parameters:**
- `stream`: Must be `"telemetry"` (only supported stream type)
- `interval`: Update interval in milliseconds (min: 10ms, default: 100ms)

**Response:**
```json
{"type":"response", "status":"ok"}
```

**Behavior:**
- Device begins streaming telemetry frames at the specified interval
- Telemetry frames continue until `unsubscribe` command is sent
- Overrides any text `MONITOR` command settings

**Bandwidth:** At 10Hz (100ms interval), telemetry uses ~2.5 KB/s (~1% of USB CDC capacity).

---

### 2. Unsubscribe from Telemetry

**Request:**
```json
{"cmd":"unsubscribe"}
```

**Response:**
```json
{"type":"response", "status":"ok"}
```

**Behavior:** Stops telemetry streaming.

---

### 3. Get Hello (Device Info)

**Request:**
```json
{"cmd":"getHello"}
```

**Response:** Same as connection handshake message:
```json
{
  "type": "hello",
  "version": 1,
  "fw": 6,
  "canMode": 0,
  "canModeName": "PT_Default1",
  "capabilities": ["analog", "digital", "pwm", "can", "config"],
  "analogChannels": 8,
  "digitalIn": 8,
  "digitalOut": 8
}
```

**Usage:** 
- Use this if your app connects after device boot and misses the initial hello
- Use to refresh device capabilities after configuration changes
- Good practice to call this on app startup before subscribing to telemetry

---

### 4. Set Output Duty Cycle

**Request:**
```json
{"cmd":"setOutput", "ch":3, "duty":75}
```

**Parameters:**
- `ch`: Output channel (1-8)
- `duty`: Duty cycle percentage (0-100)

**Response:**
```json
{"type":"response", "status":"ok"}
```

**Side effects:**
- Sets the override bit for this channel (enables serial control)
- Applies new duty immediately

---

### 5. Set Output Frequency

**Request:**
```json
{"cmd":"setOutputFreq", "ch":5, "freq":1000}
```

**Parameters:**
- `ch`: Output channel (1-8)
- `freq`: Frequency in Hz (>0)

**Response:**
```json
{"type":"response", "status":"ok"}
```

**Note:** Outputs are paired (DPO1-2, DPO3-4, DPO5-6, DPO7-8). Both channels in a pair share the same frequency. Setting frequency for one channel updates both.

**Side effects:**
- Saves to EEPROM
- Applies frequency change immediately
- May adjust GPT prescaler to maintain resolution

---

### 6. Set Safe State

**Request:**
```json
{"cmd":"setSafe", "ch":2, "value":true}
```

**Parameters:**
- `ch`: Output channel (1-8)
- `value`: `true` = HIGH in safe state, `false` = LOW in safe state

**Response:**
```json
{"type":"response", "status":"ok"}
```

**Side effects:** Saves to EEPROM, applies if currently in safe state.

---

### 7. Set Active Polarity

**Request:**
```json
{"cmd":"setActive", "ch":4, "activeHigh":false}
```

**Parameters:**
- `ch`: Output channel (1-8)
- `activeHigh`: `true` = active-HIGH, `false` = active-LOW

**Response:**
```json
{"type":"response", "status":"ok"}
```

**Side effects:** Saves to EEPROM, applies immediately.

---

### 8. Set Input Pullup

**Request:**
```json
{"cmd":"setInputPullup", "ch":6, "enabled":true}
```

**Parameters:**
- `ch`: Digital input channel (1-8)
- `enabled`: `true` = enable pullup, `false` = disable

**Response:**
```json
{"type":"response", "status":"ok"}
```

**Side effects:** Saves to EEPROM, reconfigures pin immediately.

---

### 9. Set Input Debounce

**Request:**
```json
{"cmd":"setDebounce", "ms":50}
```

**Parameters:**
- `ms`: Debounce time in milliseconds (0-100)

**Response:**
```json
{"type":"response", "status":"ok"}
```

**Usage:** Controls how long a digital input must remain stable before its state is updated. This filters out noise and contact bounce from mechanical switches.
- `0` ms = no debouncing (immediate state changes)
- `20` ms = default (good for most mechanical switches)
- `50-100` ms = high noise environments

**Side effects:** Saves to EEPROM, applies immediately to all digital inputs.

---

### 10. Get Configuration

**Request:**
```json
{"cmd":"getConfig"}
```

**Response:**
```json
{
  "type": "config",
  "canSpeed": 1000,
  "txBaseId": 1792,
  "rxBaseId": 1600,
  "txRate": 10,
  "rxTimeout": 2000,
  "canMode": 0,
  "safeMask": 0,
  "activeMask": 255,
  "inputPullupMask": 0,
  "diDebounceMs": 20,
  "outFreq": [300, 300, 300, 300, 300, 300, 300, 300]
}
```

**Fields:**
- `canSpeed`: CAN bus speed in kbps
- `txBaseId`: CAN TX base ID (11-bit)
- `rxBaseId`: CAN RX base ID (11-bit)
- `txRate`: CAN TX frame rate in Hz
- `rxTimeout`: CAN RX timeout in ms (triggers safe state)
- `canMode`: CAN mode ID (0-15)
- `safeMask`: Bitmask of safe state levels (bit=1 → HIGH in safe mode)
- `activeMask`: Bitmask of active polarity (bit=1 → active-HIGH)
- `inputPullupMask`: Bitmask of input pullups (bit=1 → pullup enabled)
- `diDebounceMs`: Digital input debounce time in ms (0-100)
- `outFreq`: Array of 8 output frequencies (Hz), pairs share values

---

### 11. Get Status (One-Shot Telemetry)

**Request:**
```json
{"cmd":"getStatus"}
```

**Response:** Single telemetry frame (see Telemetry Frame Format below).

---

### 12. Set CAN Configuration

**Request:**
```json
{
  "cmd": "setCanConfig",
  "canSpeed": 500,
  "txBaseId": 1792,
  "rxBaseId": 1600,
  "txRate": 20,
  "rxTimeout": 1000,
  "canMode": 1
}
```

**Parameters (all optional):**
- `canSpeed`: CAN speed in kbps (125, 250, 500, or 1000)
- `txBaseId`: CAN TX base ID (0x000-0x7FF)
- `rxBaseId`: CAN RX base ID (0x000-0x7FF)
- `txRate`: TX frame rate in Hz (>0)
- `rxTimeout`: RX timeout in ms (>=100)
- `canMode`: CAN mode (0-15)

**Response:**
```json
{"type":"response", "status":"ok"}
```

**Side effects:** Saves to EEPROM, applies changes immediately.

**Errors:**
- Invalid CAN speed → `{"type":"error", "msg":"CAN speed must be 125, 250, 500, or 1000"}`
- Invalid CAN ID → `{"type":"error", "msg":"CAN ID must be 0x000-0x7FF"}`

---

### 13. Reset to Defaults

**Request:**
```json
{"cmd":"resetDefaults"}
```

**Response:**
```json
{"type":"response", "status":"ok", "msg":"Defaults restored"}
```

**Side effects:**
- Restores all config to factory defaults
- Saves to EEPROM
- Applies output frequencies and states immediately

---

### 14. Set Serial Override

Control which output channels are under serial/app control vs CAN control. Channels with override enabled ignore CAN commands and are not affected by the CAN watchdog timeout.

**Method 1: Set entire mask (all channels at once)**

**Request:**
```json
{"cmd":"setSerialOverride", "mask":0x07}
```

**Parameters:**
- `mask`: 8-bit bitmask (0x00-0xFF)
  - Bit 0 = channel 1, bit 1 = channel 2, ... bit 7 = channel 8
  - `0x00` = all channels under CAN control
  - `0xFF` = all channels under serial/app control
  - `0x07` = channels 1-3 under app control, 4-8 under CAN control

**Method 2: Set individual channel**

**Request:**
```json
{"cmd":"setSerialOverride", "ch":3, "enabled":true}
```

**Parameters:**
- `ch`: Output channel (1-8)
- `enabled`: `true` = enable override (app control), `false` = disable (CAN control)

**Response:**
```json
{"type":"response", "status":"ok"}
```

**Behavior:**
- **Overridden channels (bit=1):**
  - Controlled by app via `setOutput` commands
  - Ignore CAN RX messages
  - Not affected by CAN watchdog timeout
  - Remain at commanded value even if CAN bus fails
  
- **Non-overridden channels (bit=0):**
  - Controlled by CAN messages
  - Go to safe state on CAN RX timeout
  - App cannot change these channels (CAN has priority)

**Use Cases:**
- **Mixed control:** CAN controls safety-critical outputs (e.g., fuel pump), app controls auxiliary outputs (e.g., LEDs)
- **Testing:** Override specific channels for diagnostics while CAN controls others
- **Failsafe:** Critical outputs remain under CAN watchdog protection

**Note:** Calling `setOutput` automatically sets the override bit for that channel, so manual override control is optional.

---

## Telemetry Frame Format (Device → GUI)

Sent automatically when subscribed, or as one-shot response to `getStatus`.

```json
{
  "type": "telemetry",
  "timestamp": 12345678,
  "analog": [2048, 1536, 4096, 0, 8192, 1024, 512, 16383],
  "digital": [
    {"state": true, "freq": "300.25", "duty": "45.2"},
    {"state": false, "freq": null, "duty": null},
    ...
  ],
  "outputs": [
    {"duty": "50.0", "freq": 300, "safe": false, "activeHigh": true},
    {"duty": "0.0", "freq": 300, "safe": false, "activeHigh": true},
    ...
  ],
  "can": {
    "init": true,
    "safeState": false,
    "rxCount": 1234,
    "txCount": 567,
    "txFail": 2,
    "mode": 0,
    "lastRxMs": 12345600,
    "rxTimeout": 2000
  }
}
```

**Fields:**

### `analog` (array of 8 integers)
14-bit ADC raw values (0-16383). To convert to voltage:
```
voltage = (analogValue / 16383.0) * 5.0
```

### `digital` (array of 8 objects)
- `state`: Boolean pin level (after debounce)
- `freq`: Frequency in Hz as string (2 decimal places), or `null` if no signal
- `duty`: Duty cycle in % as string (1 decimal place), or `null` if no signal

### `outputs` (array of 8 objects)
- `duty`: Current duty cycle in % as string (1 decimal place)
- `freq`: Current frequency in Hz (integer)
- `safe`: Safe state level (`true` = HIGH in safe mode, `false` = LOW)
- `activeHigh`: Active polarity (`true` = active-HIGH, `false` = active-LOW)

### `can` (object)
- `init`: CAN bus initialized successfully
- `safeState`: Outputs currently in safe state (CAN RX timeout or boot)
- `rxCount`: Total CAN frames received
- `txCount`: Total CAN frames transmitted
- `txFail`: CAN TX failures (buffer full)
- `mode`: Active CAN mode ID (0-15)
- `lastRxMs`: Timestamp of last CAN RX (millis)
- `rxTimeout`: Configured RX timeout in ms

---

## Error Handling

### Parse Errors

If JSON is malformed:
```json
{"type":"error", "msg":"JSON parse error: ..."}
```

### Validation Errors

If parameters are out of range:
```json
{"type":"error", "msg":"Channel must be 1-8"}
```

### Unknown Commands

If `cmd` field is not recognized:
```json
{"type":"error", "msg":"Unknown command: xyz"}
```

**Device behavior:** Device continues operating normally after errors. GUI should display error to user and allow retry.

---

## Example GUI Session

```
[Device connects, sends hello]
← {"type":"hello","version":1,"fw":6,"canMode":0,...}

[If GUI missed hello or connected late, request it:]
→ {"cmd":"getHello"}
← {"type":"hello","version":1,"fw":6,"canMode":0,...}

[GUI subscribes to telemetry at 100ms]
→ {"cmd":"subscribe","stream":"telemetry","interval":100}
← {"type":"response","status":"ok"}

[Device streams telemetry every 100ms]
← {"type":"telemetry","timestamp":1000,...}
← {"type":"telemetry","timestamp":1100,...}
← {"type":"telemetry","timestamp":1200,...}

[User sets output 3 to 75%]
→ {"cmd":"setOutput","ch":3,"duty":75}
← {"type":"response","status":"ok"}
[Note: Channel 3 override bit is automatically set]

[Next telemetry shows updated output]
← {"type":"telemetry",...,"outputs":[...{"duty":"75.0",...}...],...}

[User wants mixed control: app controls ch 1-3, CAN controls ch 4-8]
→ {"cmd":"setSerialOverride","mask":0x07}
← {"type":"response","status":"ok"}
[Now CAN messages will only affect channels 4-8]

[User adjusts debounce for noisy switch]
→ {"cmd":"setDebounce","ms":50}
← {"type":"response","status":"ok"}

[User requests config]
→ {"cmd":"getConfig"}
← {"type":"config","canSpeed":1000,"diDebounceMs":50,...}

[User unsubscribes]
→ {"cmd":"unsubscribe"}
← {"type":"response","status":"ok"}

[Telemetry stops]
```

---

## Implementation Notes

### Bandwidth

- **Telemetry frame size:** ~750 bytes (with all channels populated)
- **At 10 Hz (100ms):** 7.5 KB/s = 60 kbps (~3% of USB CDC bandwidth)
- **At 50 Hz (20ms):** 37.5 KB/s = 300 kbps (~15% of USB CDC bandwidth)
- **At 100 Hz (10ms):** 75 KB/s = 600 kbps (~30% of USB CDC bandwidth)

Recommended: **10 Hz (100ms)** for dashboard applications, **50 Hz (20ms)** for real-time monitoring.

### Parsing Overhead (Device Side)

- **JSON parsing time:** ~2ms per frame on 24 MHz RA4M1 (ArduinoJson)
- **CPU usage at 10 Hz:** < 5%
- **CPU usage at 50 Hz:** ~20%
- **RAM usage:** ~1.5 KB (static allocation, no heap fragmentation)

### GUI Libraries

**Desktop (C++/Qt):**
- `QSerialPort` for serial communication
- `QJsonDocument` for JSON parsing

**Desktop (Python):**
- `pyserial` for serial communication
- `json` module for parsing

**Web (JavaScript):**
- Web Serial API (`navigator.serial`)
- `JSON.parse()` / `JSON.stringify()`

**Desktop (C#/.NET):**
- `System.IO.Ports.SerialPort`
- `System.Text.Json` or `Newtonsoft.Json`

---

## Backward Compatibility

### Text CLI Commands

All existing text commands (`STATUS`, `OUT`, `CONFIG`, `MONITOR`, etc.) remain fully functional alongside JSON. The device auto-detects message format based on whether the line starts with `{`.

**Example:** You can mix JSON and text in the same session:
```
→ STATUS                           (text command)
← [text status output]

→ {"cmd":"subscribe",...}          (JSON command)
← {"type":"response","status":"ok"}

→ OUT 5 50                         (text command)
← OK: DPO5 = 50%
```

### Text MONITOR vs JSON Subscribe

Both mechanisms can coexist. If both are enabled:
- JSON subscription takes priority for timing
- Text `MONITOR` command will not send duplicate data if JSON subscription is active

**Recommendation:** GUIs should use JSON `subscribe` and avoid text `MONITOR`.

---

## Future Extensions

Potential additions (not yet implemented):
- Event-based telemetry (send only on change, not periodic)
- Bulk configuration (`setAllOutputs`, `setBulkConfig`)
- Firmware update protocol
- Data logging to onboard storage
- CAN bus passthrough / sniffer mode

---

## Testing Tools

### Command-Line Testing (PuTTY / Serial Monitor)

```
{"cmd":"getStatus"}
{"cmd":"subscribe","stream":"telemetry","interval":500}
{"cmd":"setOutput","ch":1,"duty":50}
{"cmd":"unsubscribe"}
```

### Python Test Script

```python
import serial
import json
import time

ser = serial.Serial('COM3', 115200, timeout=1)

# Read hello
hello = json.loads(ser.readline().decode())
print("Device:", hello)

# Subscribe
ser.write(b'{"cmd":"subscribe","stream":"telemetry","interval":100}\n')
response = json.loads(ser.readline().decode())
print("Subscribe:", response)

# Read telemetry for 5 seconds
start = time.time()
while time.time() - start < 5:
    line = ser.readline().decode()
    if line:
        data = json.loads(line)
        if data['type'] == 'telemetry':
            print(f"Analog[0]: {data['analog'][0]}")

# Unsubscribe
ser.write(b'{"cmd":"unsubscribe"}\n')
response = json.loads(ser.readline().decode())
print("Unsubscribe:", response)

ser.close()
```

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1       | 2026-04-08 | Initial JSON protocol implementation |
