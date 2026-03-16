# PT-IO-Mini2 — Development Log

## Project Overview

CAN-bus I/O expander for automotive use on Renesas RA4M1 (ARM Cortex-M4).  
8× analog inputs, 8× digital inputs (freq/duty capture), 8× digital outputs (PWM), CAN bus, USB serial CLI.

Prototype is based on **Arduino UNO R4 Minima** module on a custom carrier PCB.  
Rev 2 PCB will use the bare RA4M1 chip — see `PINOUT.md` for the new pinout.

---

## Current State (March 2026)

### What Works
- **8× Analog inputs** — 14-bit ADC, all channels working
- **7× Digital inputs** — GPT input capture, frequency + duty cycle measurement
- **8× Digital outputs** — Software PWM via bare-metal AGT1 timer ISR at 10 kHz
- **CAN bus** — TX/RX with configurable IDs, rates, timeout-based safe state
- **USB CDC serial CLI** — Full command set (STATUS, DIAG, OUT, CONFIG, DEFAULTS, etc.)
- **EEPROM config** — Persistent settings with CRC validation
- **NeoPixel** — Status LED indicator
- **CAN watchdog** — Outputs go to safe state on RX timeout, bypassed with `serialOverride`

### Known Issues (Prototype Only — Fixed in Rev 2 Pinout)
- **DI5 (P107/GPT0A)**: Intermittent capture — parked for investigation later
- **DPO7/DPO8 (P300/P108)**: SWD debug pins with no GPT peripheral — stuck on software PWM. Rev 2 moves to P303/P304 (GPT7)
- **Output frequency pairs**: GPT4-7 A/B share frequency — 4 independent freq groups, not 8. Acceptable trade-off for hardware PWM resolution
- **Software PWM resolution**: At 10 kHz tick, max output freq is 5 kHz at 50% only. Hardware PWM (Rev 2) solves this completely

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

Only 2 pins change from the prototype. See `PINOUT.md` for full details.

| Signal | Rev 1 | Rev 2 | Why |
|--------|-------|-------|-----|
| DPO7 | P300 (SWCLK) | P303 (GPT7B) | Gets hardware PWM, frees SWD |
| DPO8 | P108 (SWDIO) | P304 (GPT7A) | Gets hardware PWM, frees SWD |

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
