# Plan: IO-Mini2 as a native DingoConfig device

## TL;DR
dingoConfig's "FwDevice" model is already highly data-driven: device capabilities come from a JSON `FwDeviceDef` (counts of analog/digital in, outputs, etc. + feature flags) plus a JSON `CyclicSigsConfig` describing cyclic CAN frames, both loaded from files at runtime — no recompilation. The wire protocol (`dingoFW`'s `ParamProtocol`) is a simple generic index/subindex CAN read/write scheme, completely decoupled from ChibiOS-specific code. This means: **protocol compatibility can be added without porting Mini2 to dingoFW** — a new CAN operating mode inside the existing Arduino/RA4M1 firmware is sufficient, coexisting with Mini2's existing CAN-mode plugin system.

## Status
- **Tier 1 (this mode) implemented**: `CAN_MODE_DINGO_CONFIG` added, param protocol (Read/Write/ReadAll/WriteAll/Version/BurnSettings), cyclic telemetry frames, and a NeoPixel param block are all in firmware (`src/dingo_protocol.h/.cpp`). Build verified: RAM 17.3% (5664/32768 B), Flash 38.0% (99672/262144 B) on `uno_r4_minima`.
- **IOMini2Tool.App updated**: `CanModeCatalog` (shared mode name list), Config page dropdown/confirmation dialog + warning banner, Dashboard mode display/color — all via the existing USB/JSON `setCanConfig` command (no protocol changes needed, mode 0-15 already accepted generically). Build verified (`dotnet build`, 0 errors).
- **Not yet done**: dingoConfig-side JSON device definition + signals file (Phase 4), end-to-end hardware validation against real DingoConfigurator (Phase 5).

## Research findings

### dingoConfig device model (C#, Blazor/MudBlazor)
- `IDevice` (read-only signal consumption) and `IDeviceConfigurable : IDevice` (adds param read/write, burn, version, sleep/wakeup/bootloader) are the two integration surfaces.
- **`DbcDevice`**: fully generic — just point it at CAN IDs/signals (either via a `.dbc` file or ad-hoc `AddCustomSignal`). Zero device-specific C# code. Read-only (no config write, no burn/version/id-change). This is the "instant, zero-firmware-change" integration path (Tier 0 below).
- **`FwDevice`**: models dingoFW-protocol devices generically, parameterized entirely by:
  - `FwDeviceDef` (record): DeviceType id, TypeName, per-function counts (`NumDigitalInputs/Outputs/AnalogInputs/Outputs(Profet)/CanInputs/CanOutputs/VirtualInputs/Flashers/Counters/Conditions/Keypads`), feature flags (`HasWipers/HasStarterDisable/HasBattVoltSense/HasUsb/HasExtTempSensor/CanSleep/CanBootloader`), min FW version. Loaded from `Definitions/device-definitions.json` at startup.
  - `CyclicSigsConfig`/`CyclicMsgDef`/`CyclicSigDef` (per device, `Definitions/{signalsFile}.json`): declares each cyclic CAN frame's `idOffset` + signals (DBC-style start bit/length/byteOrder/factor/offset) and a `target` string resolved via `FwDevice.ResolveSetter()` against two hardcoded dictionaries in `FwDevice.RegisterAccessors()`: `_setters` (scalar) and `_indexedSetters` (per-instance).
  - **Already-registered setter targets directly reusable by Mini2, zero C# changes needed**: `AnalogInput.Millivolts`, `DigitalInput.State`, `Output.State`/`Output.DutyCycle`/`Output.Current` (PWM outputs — `Output` class already has `FixedDutyCycle`/`Frequency`/`VariableDutyCycle` params, a much better fit for Mini2's PWM outputs than the simpler on/off `DigitalOutput` class), `DeviceState`, `Heartbeat`, `PdmTypeCheck` (generic device-type-match check despite the legacy name — just a dictionary key, works for any device).
  - Param index ranges (all generic, table-driven): Device 0x0000, Outputs(Profet) 0x1000+, Digital Inputs 0x1200+, CAN Inputs 0x1300+, Virtual Inputs 0x1400+, Conditions 0x1500+, Counters 0x1600+, Flashers 0x1700+, Starter 0x1800, Wiper 0x1900, CAN Outputs 0x2000+, Digital Outputs 0x2100+, Analog Inputs 0x2200+, Keypads 0x3000+.
  - Adding a brand-new `FwDeviceDef` entry + matching signals JSON is **pure data** — no `AddDevice()` switch-statement changes needed since `"fwd"` device type already looks up `Def` by numeric `DeviceType` id.

### dingoFW wire protocol (CAN ParamProtocol) — what firmware must speak
- Frame is always 8 bytes: `[cmd(1), indexLo(1), indexHi(1), subIndex(1), value(4 bytes, LE)]`.
- Fixed CAN ID offsets from a device's single `BaseId`: `CONFIG_TX_OFFSET=0` (device→app), `CONFIG_RX_OFFSET=1` (app→device), `CYCLIC_TX_OFFSET=2` (cyclic status/telemetry frames start here, one ID per cyclic message, msg0=+2, msg1=+3, ...).
- `MsgCmd` enum: `Read=1, Write=2, ReadParamNotFound=5, ReadAll=10/ReadAllRsp=11/ReadAllComplete=12/ReadAllModified=13, WriteAll=20/WriteAllVal=21/WriteAllComplete=22/WriteAllModified=23/WriteAllParamNotFound=25/WriteAllOutOfRange=26, BurnSettings=30, Version=31, Sleep=32, Bootloader=33, CheckCrc=34/CheckCrcRsp=35, Invalid=0xFF`.
- `BurnSettings` just calls the device's existing `WriteConfig()` (flash/EEPROM save) and echoes success — directly maps to Mini2's existing `saveConfig()`.
- A static, flash-resident param registry (array of `{index, subIndex, get, set}`) is the whole read/write engine — completely reusable pattern, does not require ChibiOS.

## Feature mapping table (Mini2 → dingoFW/dingoConfig concept)

| Mini2 feature | dingoFW/dingoConfig equivalent | Fit |
|---|---|---|
| `config.canSpeedKbps` | `Config_Device.eCanSpeed` | Direct |
| `config.txBaseId` (reused as the single dingoFW BaseId while in this mode) | `Config_Device.nBaseId` | Direct (see architecture decision below) |
| `outputDuty[]`/`outputFreq[]`/`config.outFreqHz[]` | `Output` (Profet) params: `fixedDutyCycle`, `frequency` | Good fit (ignore unused current-sensing fields) |
| `diDebouncedState[]`, `config.inputPullupMask`, `getDiDebounceMs()`, DI freq/duty | Digital Input params: `debounceTime`, `pull` (+ Mini2-only extra subindices `state`/`freq`/`duty`) | Good fit + extension |
| `readAnalogRawAll()`/`adcBuf` (mV) | Analog Input param/signal `Millivolts` | Good fit |
| `config.canMode` (CAN-mode plugin selection) | No dingoFW equivalent | Mini2-specific, stays out of the param table |
| `saveConfig()`/EEPROM CRC | `BurnSettings` cmd / `WriteConfig()` | Direct |
| `FW_VERSION` | `Version` cmd response | Direct |
| NeoPixel aux mode + test HSL | No dingoFW equivalent | New Mini2-only param block (0x4000+) |
| — (none) | Virtual Inputs, Conditions, Counters, Flashers, Wiper, Starter Disable, Keypads, CAN Inputs/Outputs, battery-voltage sense | **Missing** — declared absent (0 counts / false flags) for Tier 1; only needed for Tier 2 |

## Integration tiers

### Tier 0 — DbcDevice (read-only, zero firmware changes)
Write a `.dbc` (or use `AddCustomSignal`) describing Mini2's existing `PT_Default1` CAN-mode TX frames. Add as a `DbcDevice` in dingoConfig. Gives dashboard/logging/monitoring today, no config write, no burn/version/device-type check. Not implemented as part of this work; remains available as a fallback.

### Tier 1 — FwDevice-compatible, via a new "DingoConfig" CAN mode (IMPLEMENTED)

**Architecture:**
- `CAN_MODE_DINGO_CONFIG = 9` added to `CanModeId` in `src/can_modes.h`, using the exact same plugin dispatch pattern (`canModeHandleRx`/`canModeBuildTx*Frame`) already used for Haltech/Motec/etc.
- **No new persisted CAN-ID field.** While `config.canMode == CAN_MODE_DINGO_CONFIG`, the existing `config.txBaseId` is reinterpreted as the single dingoFW `BaseId`. `rxBaseId` is unused in this mode. `main.cpp`'s `handleCanRx()` computes `effectiveRxBaseId = (canMode == CAN_MODE_DINGO_CONFIG) ? txBaseId : rxBaseId` before calling `canModeHandleRx()`, so no shared function signatures needed to change.
- While this mode is active, normal vehicle CAN-mode I/O (Haltech/Motec/etc.) is fully suspended — the Dingo dispatch cases do not call into any other mode's logic. Switching into/out of DingoConfig mode is just another `config.canMode` change (already supported at runtime via the `CANMODE`/`setCanConfig` CLI/JSON commands).
- **Outputs ARE writable via Dingo params** (`Output.fixedDutyCycle`/`Output.frequency` at index 0x1000+ch write directly into `outputDuty[]`/`outputFreq[]`). `handleCanRx()`'s existing generic post-processing (memcmp-based change detection) automatically calls `applyOutputs(false)` — no duplicated output-application code.
- **Extra provisions beyond what dingoConfig currently consumes** (implemented firmware-side now so no second firmware revision is needed later — see TODO list below): Digital Input `state`/`freq`/`duty` subindices (2/3/4 at 0x1200+ch) and a new NeoPixel param block (0x4000: `auxMode`/`testExtraPixels`/`testHue`/`testSat`/`testLight`).
- **Safety**: response/telemetry CAN sends use a short bounded retry (max ~1ms), never blocking. `ReadAll`/`ReadAllModified` sends a bounded ~80-frame burst (one CAN frame per param, ~50µs spacing) only in response to an explicit user "Read Config" action — never on the periodic cyclic path. `BurnSettings` calls `saveConfig()` synchronously (same pattern already used by several existing CLI commands, e.g. `OUTFREQ`).

**Cyclic telemetry (6 message slots, `txBaseId+2` through `txBaseId+7`, reusing the existing State/DiPair×4/Status TX call sites in `sendTxFrames()`):**
| Slot | Offset | Content |
|---|---|---|
| msg0 (state) | +2 | Digital input bitmask, DeviceState, heartbeat, FW version |
| msg1 (dipair1) | +3 | AI1-4 millivolts |
| msg2 (dipair2) | +4 | AI5-8 millivolts |
| msg3 (dipair3) | +5 | Output duty % (all 8 channels) |
| msg4 (dipair4) | +6 | Output pair frequency (Hz), one per GPT pair |
| msg5 (status) | +7 | NeoPixel: auxMode, testExtraPixels, testHue, testSat, testLight |

Offsets `+0`/`+1` (CONFIG_TX/RX) are reserved for the param-protocol channel; the analog-frame TX slot is suppressed (`len=0`) while in this mode to avoid colliding with them — its raw ADC readings are instead cached (`dingoCacheAnalogRaw()`) for the msg1/msg2 mV frames.

**Param table** (`src/dingo_protocol.cpp`):
| Index | SubIndex | Field | R/W |
|---|---|---|---|
| 0x0000 | 0/1/2 | BaseId / CAN speed / FW version | RO |
| 0x1000+ch (8) | 0/1 | Output fixedDutyCycle (%) / frequency (Hz) | RW |
| 0x1200+ch (8) | 0/1/2/3/4 | DI debounceTime(ms) / pull(0-1) / state / freq(Hz) / duty(%) | debounce+pull RW, rest RO |
| 0x2200+ch (8) | 0/1 | AI enabled / millivolts | RO |
| 0x4000 | 0/1/2/3/4 | NeoPixel auxMode / testExtraPixels / testHue / testSat / testLight | RW |

### TODO list — features implemented in firmware but not yet consumable by dingoConfig
1. **Digital Input `freq`/`duty` param subindices** — no equivalent field exists in dingoFW's stock `DigitalInput` C# class/param set today; needs a dingoConfig-side extension (new properties + signal targets) before the UI can show them.
2. **NeoPixel param block (0x4000+)** — entirely new to dingoFW; needs a new `IDeviceFunction`-style class + UI tab in dingoConfig before it's usable, analogous to how `Wiper`/`StarterDisable` were added.
3. **`config.canMode`** itself (selecting which vehicle CAN-mode plugin is active) has no dingoFW equivalent — out of scope for dingoConfig to control; remains CLI/JSON-protocol-only.
4. **Exact ack semantics for single `Write`/`WriteAllVal` failures** were implemented pragmatically (reusing `ReadParamNotFound` for single-`Write` failures, silent no-response for `WriteAllVal` failures) since the upstream dingoFW source wasn't fully available during this session — verify against real DingoConfigurator traffic in Phase 5 and adjust if needed.

**dingoConfig-side work (not yet done, Phase 4):**
- New `Definitions/device-definitions.json` entry: new `DeviceType` id, `TypeName: "PT-IO-Mini2"`, `NumAnalogInputs=8, NumDigitalInputs=8, NumOutputs=8` (Profet-style), `NumDigitalOutputs=0, NumCanInputs=0, NumCanOutputs=0, NumVirtualInputs=0, NumConditions=0, NumCounters=0, NumFlashers=0, NumKeypads=0, HasWipers=false, HasStarterDisable=false, HasBattVoltSense=false, HasUsb=true, HasExtTempSensor=false, CanSleep=false, CanBootloader=false`.
- New `Definitions/pt-iomini2-signals.json`: cyclic frame declarations using only already-registered setter targets (`AnalogInput.Millivolts`, `DigitalInput.State`, `Output.State`, `Output.DutyCycle`, `DeviceState`, `Heartbeat`, `PdmTypeCheck`).
- No `FwDevice.cs` code changes anticipated for the fields above — to be verified against real traffic.

### Tier 2 — optional, larger: native Conditions/Virtual Inputs/Counters/Flashers
dingoFW's `Condition`/`VirtualInput`/`Counter`/`Flasher` param-driven boolean-logic building blocks are a non-Turing-complete alternative to the scripting-engine plan (see `SCRIPTING-PLAN.md`) for "if this, then that" vehicle logic — simpler, already has first-class dingoConfig UI support, no bytecode VM needed, but far less expressive. Not started; a strategic alternative/complement to the scripting engine, to be decided once Tier 1 is validated on hardware.

## Files changed/added
- `src/can_modes.h` — `CAN_MODE_DINGO_CONFIG = 9` added to `CanModeId`.
- `src/can_modes.cpp` — name table entry, dispatch cases in `canModeHandleRx`/`canModeBuildTxAnalogFrames`/`canModeBuildTxStateFrame`/`canModeBuildTxDiPairFrame`/`canModeBuildTxStatusFrame`.
- `src/main.cpp` — `getFwVersion()`/`getDiActiveLowMask()` getters (needed because `const` globals have internal linkage in C++); `handleCanRx()` computes `effectiveRxBaseId` for Dingo mode.
- New: `src/dingo_protocol.h`/`src/dingo_protocol.cpp` — full param table, frame codec, command handling, cyclic frame builders.

## Verification
1. ✅ Firmware builds clean for `uno_r4_minima` (RAM 17.3%, Flash 38.0%).
2. ⬜ Bench test with a CAN sniffer: confirm exact byte-for-byte frame layout, `CANMODE 9` correctly suspends vehicle CAN-mode I/O and starts Dingo cyclic frames, mode switch back to a vehicle CAN mode cleanly resumes normal operation.
3. ⬜ Full read/write/burn/version/device-type-mismatch cycle against real DingoConfigurator once the Phase 4 JSON definitions exist.
4. ⬜ Regression: confirm all other CAN modes / CLI / JSON telemetry / existing DFU flashing workflow unaffected when NOT in `CAN_MODE_DINGO_CONFIG`.

## Decisions
- Protocol compatibility is added via a new CAN operating mode (`CAN_MODE_DINGO_CONFIG`), using the existing CAN-mode plugin architecture — no port to ChibiOS/dingoFW, no rewrite of the existing plugin dispatch.
- No new persisted CAN-ID field — `config.txBaseId` is reinterpreted as the single dingoFW `BaseId` while this mode is active; `rxBaseId` unused in this mode.
- Vehicle CAN-mode I/O is fully suspended while `CAN_MODE_DINGO_CONFIG` is selected.
- Outputs (duty + frequency) are writable via Dingo params, applied through the existing `applyOutputs()` path.
- Digital Input `state`/`freq`/`duty` and the NeoPixel param block are implemented firmware-side now even though dingoConfig can't yet consume them (TODO list above).
- Mini2's PWM outputs map to dingoFW's `Output` (Profet) param concept, not the simpler `DigitalOutput`.
