# PT-IO-Mini2 — Development Log

## Project Overview

CAN-bus I/O expander for automotive use on Renesas RA4M1 (ARM Cortex-M4).  
8× analog inputs, 8× digital inputs (freq/duty capture), 8× digital outputs (PWM), CAN bus, USB serial CLI.

Prototype is based on **Arduino UNO R4 Minima** module on a custom carrier PCB.  
Rev 2 PCB will use the bare RA4M1 chip — see `PINOUT.md` for the new pinout.

---

## Current State (April 2026)

### What Works
- **8× Analog inputs** — 14-bit ADC, all channels working
- **8× Digital inputs** — GPT input capture, frequency + duty cycle measurement (overflow-aware), configurable debounce (0-100ms)
- **8× Digital outputs** — Hardware GPT PWM on GPT4/5/6/7 (all 8 channels), per-pair frequency configurable at runtime
- **CAN bus** — TX/RX with configurable IDs, rates, timeout-based safe state
- **USB CDC serial CLI** — Full command set (STATUS, DIAG, OUT, CONFIG, DEFAULTS, etc.)
- **USB CDC JSON protocol** — 14 commands for GUI applications, telemetry streaming, device auto-discovery
- **Per-channel serial override** — Individual outputs can be under app control while others remain under CAN control
- **EEPROM config** — Persistent settings with CRC validation
- **NeoPixel** — Status LED indicator
- **CAN watchdog** — Outputs go to safe state on RX timeout, with per-channel override support

### Known Issues (Prototype Only — Fixed in Rev 2 Pinout)
- **DI5 (P107/GPT0A)**: Intermittent capture — init ordering fix applied (initCaptureInputs runs last), may still need verification on new hardware
- **Output frequency pairs**: GPT4-7 A/B share frequency — 4 independent freq groups, not 8. Acceptable trade-off for hardware PWM resolution

### Fixed Bugs (March 2026)
- **DI1/DI6/DI7/DI8 reported 1659 Hz for a 300 Hz signal** — GPT2/GPT3 are 16-bit timers (overflow at 65 535 counts). At 24 MHz a 300 Hz period = 80 000 counts, which wraps once; the residual 14 464 counts gave 24 MHz/14 464 ≈ 1659 Hz. Fix: added a per-timer overflow counter incremented on `TIMER_EVENT_CYCLE_END` and used in the period calculation.
- **Output frequency locked at 300 Hz** — `initHardwarePwm()` always used the default 300 Hz constant and `applyOutputs()` never changed the timer period. Fix: `initHardwarePwm()` now reads from the loaded `outputFreq[]` values; `applyOutputs()` detects per-pair frequency changes and calls `reinitOutputTimer()`.
- **DI5 (GPT0A) NVIC slot collision** — `initCaptureInputs()` ran before `CAN.begin()`, which called IRQManager and could overwrite the IELSR slot allocated for GPT0 capture-A. Fix: `initCaptureInputs()` now runs last in `setup()`, after CAN and NeoPixel initialisation.
- **Serial welcome message lost on boot** — USB CDC `Serial.begin()` returns immediately; output sent before the host opens the port is discarded. Fix: welcome banner is deferred and sent in `loop()` on first `Serial` connect.

---

## Version History

### v0.1.1 (October 2026)
**Emtron AV9-16 from DI1-8**

- Emtron mode (Mode 8) now fills the EIC16M's previously-unsent AN Voltage
  9-16 frames (`base+2`/`base+3`) by synthesizing them from DI1-8 state:
  5000mV when a DI reads high, 0mV when low. Lets users wire a basic on/off
  switch to a spare DI and read it back as an AV channel in EMtune once
  they've run out of Mini2's 8 real analog inputs.
- Firmware version bumped to `v0.1.1` (`FW_VERSION = 0x0B`).

### v0.1.0 (October 2026)
**Haltech IO12 Polarity Fixes + Per-Parameter Overrides**

- Fixed Haltech IO12 CAN decode: Active State bit (byte1 bit5) was read
  inverted, causing commanded-on channels to drive 0 physical duty and vice
  versa. Confirmed against the official Haltech IO12 CAN spec and a real
  Haltech ECU bench test.
- Fixed `applyOutputs()` active-polarity handling: the activeHigh inversion
  direction was backwards, and this board's GPT-timer PWM output stage has
  its own unconditional hardware-level duty inversion (not present on the
  original AVR+MCP2515 board) that was never being compensated for. Enabled
  `OUTPUT_STAGE_INVERT_MASK` for all channels and corrected the inversion
  condition so Active LOW/HIGH now behaves exactly as it did on the original
  reference firmware. Confirmed on the bench against a real Haltech ECU
  (duty, frequency, safe state, and active polarity all verified correct in
  all three IO12 modes).
- Added per-parameter (duty/frequency/safe-state/active-polarity) app-vs-CAN
  override toggles — any output channel can now have individual parameters
  locked to app control while others remain CAN-controlled, instead of a
  single all-or-nothing override. Ported to Mini2's 8-channel, paired-timer
  output layout.
- Removed auto-lock-on-write: editing Hz/%/Safe/Active from the app no longer
  silently sets the override/lock bit; locking now only happens via an
  explicit setOverride command.
- Default CAN TX rate for Haltech IO12A/IO12B (mode 3) changed to 20 Hz.
- Telemetry now reports the actual safe-state-applied duty instead of the
  stale pre-safe-state commanded duty.
- Emtron mode: use the fixed EIC16M base ID (705/0x2C1) for the primary
  device and ignore `config.txBaseId`; use the app-configured, persisted
  output frequency (`config.outFreqHz`) for all DPOs instead of a hardcoded
  default, respecting the paired-timer layout.
- Added board identification field to the hello message, and build output is
  now named after board+firmware version instead of generic `firmware.bin`.
- Fixed `.gitignore`: concatenated entries and missing `releases/` ignore.
- Firmware version bumped to `v0.1.0` (`FW_VERSION = 0x0A`).

### v0.0.9 (October 2026)
**Real Emtron EIC16M CAN Mode (Mode 8), with EIC10 overflow device**

- Replaced the Emtron mode's prior PT-default copy/placeholder with a real
  implementation of Emtron's EIC16M input-expander protocol
  (https://docs.emtronaustralia.com.au/accessories/input-expander/index.html):
  - TX: AN Voltage 1-8 (`base+0`/`base+1`, 0.001 V/count) and Frequency 1-4
    from DI1-4 (`base+4`, 0.1 Hz/count), 8-byte little-endian frames,
    default base ID 705/0x2C1.
  - RX: EIC16M/EIC10 are transmit-only devices — no spec-defined RX side.
- Mini2 has 8 digital inputs but EIC16M only defines 4 Frequency channels:
  DI5-8 are emulated as a second Emtron device, the **EIC10** (EMtune
  picker: "73: Emtron EIC10 x1 Device", CAN PID 718-721), using a new
  configurable `eic10BaseId` (default 718, CLI `EIC10BASE`, JSON
  `setCanConfig`/`getConfig` field `eic10BaseId`). Only the Frequency frame
  (`eic10BaseId+3`) is sent, and only when DI5-8 actually have data —
  **unconfirmed against real EIC10 hardware/EmTune captures, bench-test
  before trusting this mapping** (see `TODO_EMTRON_CAN_MODE.md` §3).
- Added output control via EMtune's "Custom Tx Data Set" feature, the same
  approach already proven in the field by PT Motorsport's earlier
  Haltech-based IO expanders
  (https://www.ptmotorsport.com.au/emtron-can-information-for-use-with-pt-motorsport-io-expanders/):
  - One 8-byte frame at `rxBaseId` (4x `uint16` LE duty, 0-1000 =
    0.0-100.0%) drives DPO1-4; a second frame at `rxBaseId+1` drives DPO5-8
    (Mini2 has 8 physical outputs).
  - Deliberately does **not** replicate the reference firmware's legacy
    safe/active flag-bit packing — safe state remains entirely owned by the
    existing app-programmable `config.safeMask`/`config.activeMask` +
    CAN-RX-timeout watchdog, left dormant while Emtron frames are arriving.
- `CONFIG_VERSION` bumped 2 -> 3 for the new `eic10BaseId` field (existing
  EEPROM configs reset to defaults on first boot after upgrading, same as
  prior version bumps on this branch).
- Full design notes and the DTM2-vs-Mini2 channel-count differences are
  tracked in `TODO_EMTRON_CAN_MODE.md`.
- Not yet bench-tested against real Emtron hardware — byte layout/scaling
  follow the official datasheet and the proven reference firmware, but this
  still needs confirmation on the bench before being considered final.
- Firmware version bumped to `v0.0.9` (`FW_VERSION = 0x09`).

### v0.0.8 (May 2026)
**IO16 Finalization + MOTEC E888 Mode 7**

- Completed HALTECH IO16 mode updates:
  Multiplexed SPI transmission with duty cycle, corrected DI frame timing, fixed frequency scaling, and added DPI pull-up control.
- Added MOTEC E888 CAN mode support:
  PWM output RX handling and multiplexed input/diagnostic TX frames.
- Corrected MOTEC E888 mappings and scaling after hardware test:
  fixed mux index packing, analog field packing, PWM RX ID coverage (including `0xF3`), and DI frequency scaling.
- Firmware version bumped to `v0.0.8` (`FW_VERSION = 0x08`).

### v0.0.7 (April 2026)
**Rev 2 Pinout + Polarity + Monitor Alignment**

- Updated firmware pin mapping to match Rev 2 hardware pinout:
  DI: `P501, P104, P105, P106, P107, P113, P112, P111`
  DPO: `P408, P409, P410, P411, P304, P303, P302, P301`
  NeoPixel moved to `P400` (UNO R4 variant pin 26)
- Upgraded output path from mixed PWM/GPIO to full hardware PWM for all outputs:
  `DPO1-2 -> GPT5`, `DPO3-4 -> GPT6`, `DPO5-6 -> GPT7`, `DPO7-8 -> GPT4`
- Set output stage inversion default for current board wiring:
  `OUTPUT_STAGE_INVERT_MASK = 0x00`
- Fixed PWM endpoint polarity behavior (0%/100%) by correcting compare mapping used in `set_duty_cycle()` conversion.
- Fixed DI duty reporting polarity so reported duty reflects **active duty** (active-low by default):
  Added `DI_ACTIVE_LOW_MASK` and applied it to STATUS output, JSON telemetry, and CAN DI duty frames.
- Updated serial monitor/help/config text so displayed driver/GPT mapping matches the actual pinout and timer allocation.

Validation on hardware:
- Frequency loopback DPO -> DI verified.
- Duty polarity validated with LED + DI readback after fixes.

### v0.0.6 (April 2026)
**JSON Protocol & Per-Channel Override**

Added complete JSON protocol for GUI applications alongside existing text CLI:
- **14 JSON commands**: subscribe, unsubscribe, getHello, setOutput, setOutputFreq, setSafe, setActive, setInputPullup, setDebounce, getConfig, getStatus, setCanConfig, resetDefaults, setSerialOverride
- **Telemetry streaming**: Configurable interval (10-1000ms), full device state (analog, digital, outputs, CAN status)
- **Auto-detection**: Device sends hello message on USB connection, app can request it with getHello
- **Per-channel serial override**: Changed from global bool to 8-bit bitmask, allows mixed CAN/app control (e.g., CAN controls safety-critical outputs while app controls auxiliary LEDs)
- **Debounce configuration**: Added setDebounce command (0-100ms) with EEPROM persistence
- **Protocol documentation**: Complete PROTOCOL.md with all commands, telemetry format, error handling, and test scripts
- **Test utilities**: PowerShell and Python test scripts for protocol validation

Backward compatible: All existing text CLI commands unchanged. Device auto-detects JSON (lines starting with `{`) vs text commands.

### v0.0.5 and earlier
See Fixed Bugs section above for March 2026 fixes (timer overflow, frequency setting, NVIC collision, USB CDC serial timing).

---

## Hard-Won Lessons (RA4M1 / Arduino Renesas)

### 1. Clock Speeds — Not What You'd Expect
The UNO R4 Minima runs HOCO at **24 MHz**, not 48 MHz as some documentation suggests.
- `ICLK = 24 MHz` (DIV_1)
- `PCLKB = 12 MHz` (DIV_2) — AGT timers use this
- `PCLKD = 24 MHz` (DIV_1) — GPT timers use this
- Verify with: `R_FSP_SystemClockHzGet(FSP_PRIV_CLOCK_PCLKB)`

### 2. Pin Function Selection (PFS) — Write Protection
PFS registers are write-protected. You **must** bracket all PFS writes:
```cpp
R_BSP_PinAccessEnable();
// ... write PFS registers ...
R_BSP_PinAccessDisable();
```
Without this, pin mux changes silently fail.

### 3. GPIO Output — Use PCNTR3 for Atomic Set/Clear
Don't read-modify-write PODR. Instead use PCNTR3 register:
- `PCNTR3.POSR` (bits 15:0) = set pins HIGH
- `PCNTR3.PORR` (bits 31:16) = set pins LOW
- Both are atomic, no interrupt race conditions
- PORT registers are spaced 0x20 bytes apart from `R_PORT0`

### 4. Reading Actual Pin State
Use `PCNTR2.PIDR` (pin input data register), NOT `PFS.PODR`. PODR is the output latch, not the actual pin level.

### 5. ISR + USB CDC — CPU Budget is Tiny
At 24 MHz HOCO, a 100 kHz ISR consumes ~60% CPU and **kills USB CDC** (no serial, device not recognized by Windows). Even 20 kHz was too much.
- **Safe limit: 10 kHz** with a tight ISR (~150 cycles)
- ISR priority must be **lower** than USB (~12). We use priority 14.
- **Never** put `delay()` in init code before USB is enumerated — it blocks the USB task

### 6. AGT Timer (FspTimer) — Fragile Abstraction
FspTimer's AGT wrapper has issues:
- The float `begin()` overload behaves differently from the integer (RAW) overload
- The timer can appear to start but stop after ~3 ticks
- Root cause was never fully isolated in the abstraction layer

**Final solution: bare-metal AGT1** — bypass FspTimer entirely, write registers directly. See `initSwPwm()` in main.cpp.

### 7. IELSR Slot Allocation — IRQManager Collision
IRQManager allocates interrupt slots from **index 0 upward** using a private counter. If you manually allocate an IELSR slot, IRQManager will later overwrite it.
- **Solution**: Allocate manual IELSR slots from **index 31 downward**
- IRQManager fills 0→N, you fill 31→N — they never collide

### 8. Bare-Metal ISR on RA4M1 — Must Clear ICU IR Bit
When writing your own ISR (not using FSP's `agt_int_isr`), you **must** clear the ICU's Interrupt Request flag:
```cpp
IRQn_Type irq = R_FSP_CurrentIrqGet();
R_ICU->IELSR_b[irq].IR = 0U;
```
Without this, the interrupt re-fires endlessly, consuming 100% CPU and killing everything else. The FSP's built-in ISRs call `R_BSP_IrqStatusClear()` which does this — bare-metal ISRs must do it manually.

Also clear AGT status flags and add a `__DSB()` before return to ensure write completes.

### 9. GPT Input Capture — Channel Allocation
Each GPT timer has A and B capture channels. Both share the same counter but capture independently on different edges.
- PSEL = 0x03 for GPT on RA4M1 (`IOPORT_PERIPHERAL_GPT1`)
- `force_use_of_pwm_reserved_timer()` is needed to use GPT channels that Arduino reserves for PWM
- Capture ISR must handle both rising and falling edges (GTICASR/GTICBSR = 0xF00/0xF000)

### 10. EEPROM — Validate on Load
Uninitialized EEPROM contains arbitrary values. Always validate loaded config:
- Check magic number + CRC
- Range-validate individual fields (e.g., `outFreqHz` must be 50-10000)
- `DEFAULTS` command resets to known-good values

### 11. GPT Timer Widths
- GPT0, GPT1 = **32-bit** (good for input capture — no overflow at 24 MHz for ~179 seconds)
- GPT2-GPT7 = **16-bit** (overflow at 24 MHz every ~2.7 ms — fine for PWM output, tighter for low-freq capture)

### 12. CAN Bus
- `Arduino_CAN` library handles setup but `CAN.begin()` has side effects on timers
- CAN uses P103 (TX) and P102 (RX) — these are also GPT2A/GPT2B alternative pins, but PSEL selects CAN (0x09) so no conflict as long as you don't try to use GPT2 on those pins

---

## Rev 2 Hardware Changes

Rev 2 introduces a broader digital I/O remap (DI + DPO) and moves NeoPixel to `P400`.
See `PINOUT.md` for the full authoritative mapping.

| Signal | Rev 1 | Rev 2 | Why |
|--------|-------|-------|-----|
| DI1 | P107 | P501 | Updated Rev 2 routing |
| DI2 | P106 | P104 | Updated Rev 2 routing |
| DI3 | P105 | P105 | unchanged |
| DI4 | P104 | P106 | Updated Rev 2 routing |
| DI5 | P113 | P107 | Updated Rev 2 routing |
| DI6 | P501 | P113 | Updated Rev 2 routing |
| DI7 | P111 | P112 | Updated Rev 2 routing |
| DI8 | P112 | P111 | Updated Rev 2 routing |
| DPO1 | P302 | P408 | Updated Rev 2 routing |
| DPO2 | P301 | P409 | Updated Rev 2 routing |
| DPO3 | P408 | P410 | Updated Rev 2 routing |
| DPO4 | P409 | P411 | Updated Rev 2 routing |
| DPO5 | P410 | P304 | Updated Rev 2 routing |
| DPO6 | P411 | P303 | Updated Rev 2 routing |
| DPO7 | P303 | P302 | Updated Rev 2 routing |
| DPO8 | P304 | P301 | Updated Rev 2 routing |
| NeoPixel | P109 | P400 | Better routing / board fit |

---

## Rev 2 Firmware TODO

### Priority 1: Replace Software PWM with Hardware GPT PWM
- [ ] Remove AGT1 bare-metal timer ISR and all `swPwm*` infrastructure
- [ ] Implement GPT4-7 hardware PWM init (saw-wave mode, PCLKD source)
- [ ] Auto-select GPT prescaler to keep period ≤ 65535 for any configured frequency
- [ ] Update `applyOutputs()` to write GTCCRA/GTCCRB compare values directly
- [ ] Update pin config: PSEL=0x03 for all 8 output pins
- [ ] Update `OUTFREQ` command — now sets per-pair frequency (affects both A+B on same GPT)
- [ ] Update `Config.outFreqHz[4]` — 4 frequencies for 4 GPT pairs

### Priority 2: Fix DI5
- [ ] Investigate P107/GPT0A capture issue on new hardware
- [ ] May be a proto wiring issue — verify on Rev 2

### Priority 3: DAC Output (Future)
- [ ] Optional: firmware-switchable AV7 ↔ DAC mode on P014
- [ ] Requires op-amp with shutdown pin + 1 GPIO (see conversation notes)
- [ ] Not needed for initial Rev 2

### Priority 4: Polish
- [ ] Clean up diagnostic prints (remove `[SWPWM]` boot messages once stable)
- [ ] Consider adding firmware version to STATUS output
- [ ] Consider OTA firmware update via CAN (stretch goal)

---

## Serial CLI Commands Reference

| Command | Description |
|---------|-------------|
| `HELP` | List all commands |
| `STATUS` | Full system status dump |
| `MONITOR [ms]` | Periodic status output |
| `DIAG` | Detailed register dump (GPT, AGT, PFS, IELSR) |
| `DEFAULTS` | Reset EEPROM config to factory defaults |
| `OUT <1-8> <0-100>` | Set output duty cycle (%), enables `serialOverride` |
| `OUTFREQ <1-4> <Hz>` | Set output frequency for a pair |
| `CANSPEED <kbps>` | Set CAN bus speed |
| `TXBASE <hex>` | Set CAN TX base ID |
| `RXBASE <hex>` | Set CAN RX base ID |
| `TXRATE <Hz>` | Set CAN TX rate |
| `RXTIMEOUT <ms>` | Set CAN RX watchdog timeout |
| `CANMODE <0\|1>` | CAN mode |
| `CONFIG` | Show current EEPROM config |
| `SAFE` | Force safe state |
| `ACTIVE` | Force active state |

---

## Build Environment

- **PlatformIO** with `platform: renesas-ra`, `board: uno_r4_minima`
- `framework-arduinorenesas-uno @ 1.4.1`
- `toolchain-gccarmnoneeabi @ 1.70201.0 (7.2.1)`
- Flash: ~67 KB / 256 KB, RAM: ~5 KB / 32 KB
- Upload via DFU (USB)

---

## Key Files

| File | Purpose |
|------|---------|
| `src/main.cpp` | All firmware (~1500 lines) |
| `PINOUT.md` | Rev 2 MCU pinout with timer/ADC allocation |
| `DEVLOG.md` | This file — development notes and learnings |
| `platformio.ini` | Build configuration |
