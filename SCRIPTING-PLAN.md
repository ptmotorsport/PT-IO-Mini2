# Plan: Deterministic scripting/rules engine for PT-IO-Mini2

> **Status: proposal only — not yet implemented.** `CAN_MODE_DINGO_CONFIG` now
> occupies slot 9 (see `DINGOCONFIG-PLAN.md`), so this plan's CAN mode should
> use slot 10 (`CAN_MODE_RESERVED_10`) instead of the originally-proposed 9
> when implementation starts.

## Architecture findings (existing firmware)
- Superloop, no RTOS: `setup()`/`loop()` in src/main.cpp. loop() order: adcService() -> handleSerial() -> processCan() (bounded to MAX_RX_PER_LOOP=12 frames) -> updateDigitalInDebounce() -> deferred saveConfig() (2s debounce) -> CAN-RX-timeout safe-state check -> DI stale-capture clear -> telemetry -> sendTxFrames() (config.txRateHz) -> updateStatusLed() (50ms).
- NO hardware watchdog today. Only software CAN-RX-timeout -> safe state (config.rxTimeoutMs).
- Config struct (main.cpp ~L188) persisted via EEPROM.put/get with magic+version+crc (data flash emulation). CONFIG_MAGIC=0x5049, CONFIG_VERSION=2.
- CAN mode plugin pattern: can_modes.h/.cpp, enum CanModeId (0-9 used as of the DingoConfig work, 10-15 CAN_MODE_RESERVED_*). Dispatch via canModeHandleRx()/canModeBuildTx*Frame() functions keyed by config.canMode.
- Outputs: outputDuty[8]/outputFreq[8] globals -> applyOutputs(bool useSafeState) is the SOLE place that writes hardware PWM (GPT4/5/6/7). Precedence today: safeMask (on CAN timeout) > serialOverrideMask (per-channel bit; CAN RX handler restores overridden channels to prior value) > CAN-driven duty.
- Inputs: analog via readAnalogRawAll()/adcBuf double-buffer; digital via diDebouncedState[]/diPeriodCounts[]/diHighCounts[] (GPT capture ISR + main-loop debounce).
- Protocol: text CLI (handleSerial, ~L1600-2100) and JSON (protocol.cpp, strcmp(cmd,...) dispatch chain, JSON_CMD_DOC_SIZE=512 bytes/msg).
- Platform: Arduino UNO R4 Minima (RA4M1 Cortex-M4 @ 24MHz HOCO), 32KB SRAM, 256KB code flash, small data flash (need to confirm exact size, likely 8KB) via EEPROM library.
- Hard-won lessons (DEVLOG.md): PFS write-protect bracketing, IELSR slot allocation must go 31-downward for manual/bare-metal IRQs to avoid IRQManager collisions, bare-metal ISRs must clear ICU IR bit or CPU locks at 100%.

## User decisions (confirmed via questions)
1. **Compilation happens on the PC app** (IOMini2Tool.App) — text Lua-like script -> compact bytecode. Firmware only validates+executes bytecode, no on-device parser.
2. **New CAN mode `CAN_MODE_CUSTOM_SCRIPT`** dedicated to script-driven outputs, using the existing CanModeId plugin architecture (slot 10, updated from the original proposal of 9 since DingoConfig now uses 9) — cleaner than a new arbitration bitmask; reuses existing serialOverrideMask/safe-state logic unchanged.
3. **Add a real MCU hardware watchdog (IWDT)** via RA4M1 FSP WDT driver, refreshed once per loop(), as the last-resort safety net for genuine hangs (not per-script fault handling — that's the instruction budget/verifier).
4. **Script runtime variables/timers reset on boot** — no persistence of runtime state across power cycles (only bytecode+signal table persist).

## Proposed architecture

### Bytecode VM (new `src/script_engine.h/.cpp`)
- Fixed opcode set: LOAD_SIGNAL, LOAD_CONST, CMP_GT/LT/GE/LE/EQ/NE, AND, OR, NOT, JUMP_IF_FALSE, JUMP (forward-only), SET_OUTPUT, START_TIMER/TIMER_ELAPSED, IS_STALE, HALT.
- **No loops/recursion in the language at all** (grammar excludes them) — compiler never emits backward jumps. Verifier (below) statically rejects any backward jump target as defense in depth, guaranteeing termination.
- Fixed-size static RAM buffers, zero dynamic allocation:
  - Bytecode buffer: ~512 bytes (128 x 4-byte instructions) — loaded once from data flash at boot/on upload.
  - Eval stack: 16 x int32/float (~64 bytes).
  - Variables/timers: 16 x float + 8 x uint32 timer-start (~160 bytes).
  - Signal table: up to 16 entries x ~12 bytes (source type + index/CAN id/offset/len/scale/offset) (~200 bytes).
  - Control struct: enabled flag, fault code, instruction counter, script version (~32 bytes).
  - **Total RAM estimate: ~1KB** (verify against actual free SRAM via build map — 32KB total; the DingoConfig work above uses only 5664/32768 bytes (17.3%), so there is ample headroom).
- Hard instruction budget per invocation (e.g. 200-500 steps) enforced by a counter; exceeding it aborts execution and sets a fault (should be unreachable given the verifier, but defense-in-depth per requirements).
- Upload-time verifier (runs in firmware before accepting new bytecode): validates magic/version/CRC, opcode set, in-range operand indices (signals/outputs/vars), forward-only jump targets, static stack-depth safety. Reject on any failure — device keeps previous script (or none) active and reports an error.
- Signals: script only reads via `LOAD_SIGNAL(index)` resolved through the signal table — never raw memory/registers. Signal sources:
  - Local: AI1-8 raw/voltage, DI1-8 state/freq/duty, DPO1-8 current commanded state.
  - CAN-derived: generic decode entries (CAN ID, start bit/byte offset, length, endianness, scale, offset) populated by the new CAN_MODE_CUSTOM_SCRIPT RX handler — needed for the EngineRPM/OilPressure/CoolantTemp examples, which are ECU CAN broadcasts, not local I/O.
  - `IS_STALE(signal)` opcode lets scripts detect a CAN signal hasn't updated recently (per-signal staleness timer), since CustomScript mode does not use the generic CAN-RX-timeout safe state.
- Outputs: `SET_OUTPUT(ch, value)` writes into a script-local shadow array; scriptEngineService() copies results into the existing `outputDuty[]`/`outputFreq[]` globals (respecting `serialOverrideMask` exactly like `handleCanRx()` does today) and calls `applyOutputs(false)` only when changed — reuses all existing PWM/safe-state code, zero duplication.

### Persistence (new `src/script_store.h/.cpp`)
- Separate CRC'd data-flash block (own magic/version), stored after the existing `Config` block via EEPROM.put/get at a fixed offset — same mechanism already used for Config, no new flash driver risk.
- Stores: bytecode buffer + signal table + header (magic/version/length/CRC). No source text stored on-device (source lives in the PC app project file only).
- **Must verify actual RA4M1 data-flash capacity** (likely ~8KB) before finalizing exact byte budget for bytecode+signal table (~1.5KB proposed) — flagged as an early validation task.
- Firmware boots with no script (or an invalid/rejected script) and operates normally in whatever CAN mode is configured — CustomScript mode with no valid script simply drives all outputs to their configured safe state (same code path as CAN-RX-timeout today).

### CAN mode integration (`can_modes.h/.cpp`)
- Add `CAN_MODE_CUSTOM_SCRIPT = 10` to `CanModeId` (slot 10, see status note above).
- `canModeHandleRx()` for this mode: does NOT map CAN frames to outputDuty[] directly (no hardcoded frame layout); instead feeds matching CAN IDs into the script engine's CAN-signal decode cache (value + last-seen timestamp) per the signal table's configured CAN entries. Returns handled=false for output-change purposes since only the script engine writes outputs in this mode.
- `canModeBuildTx*Frame()` for this mode: reuse the existing mode0 (PT_Default1) builders so telemetry/status frames keep transmitting even while in script mode (assumption to confirm with user later, easy to change).
- CAN-RX-timeout safe-state (`config.rxTimeoutMs`) is skipped while `config.canMode == CAN_MODE_CUSTOM_SCRIPT` (outputs aren't CAN-driven in this mode); staleness of individual CAN-sourced signals is instead the script author's responsibility via `IS_STALE()`.

### Scheduling (`main.cpp`)
- `scriptEngineService(now)` called once per `loop()` iteration, internally gated to ~100Hz (10ms) via the same `if (now - lastScriptMs >= 10)` pattern already used for the status LED (50ms) and TX rate.
- Runs only when `config.canMode == CAN_MODE_CUSTOM_SCRIPT` and a valid script is loaded; otherwise a no-op (near-zero overhead when unused).
- On any runtime fault (invalid opcode, stack over/underflow, instruction-budget exceeded): engine disables itself, sets a fault code, and holds/forces safe outputs (reuse `applyOutputs(true)` semantics) until re-enabled or a new script is uploaded — never blocks the rest of `loop()`.

### Hardware watchdog (IWDT)
- Use RA4M1 FSP WDT driver (`r_wdt`), configured for register-start (not OFS auto-start) with **reset action** (avoid extra NMI/ISR complexity and IELSR slot pressure).
- `R_WDT_Open()`/`R_WDT_Start()`-equivalent in `setup()` after existing init (order matters given past IELSR collisions — add last, like `initCaptureInputs()`).
- Refresh (`R_WDT_Refresh()`-equivalent) once near the top of `loop()`, with a timeout margin generous vs. worst-case loop cycle time (e.g. 500ms-1s) — this catches genuine firmware/script-engine hangs, independent of and in addition to the script instruction budget.

### Protocol additions (`protocol.h/.cpp`, JSON only — CLI optional stretch)
- Chunked upload (JSON_CMD_DOC_SIZE is only 512 bytes/message): `scriptUploadBegin` (total size, CRC) / `scriptUploadChunk` (offset, base64/hex data) / `scriptUploadEnd` (finalize, triggers verifier) — mirrors chunked patterns likely already used by DfuUtilService on the App side.
- `getScriptStatus`: enabled flag, fault code, script version/CRC, last-run instruction count — surfaced in STATUS/telemetry similar to existing `adcScanFailCount`/`canInitOk` fields.
- `enableScript` / `disableScript` commands.

### PC App (IOMini2Tool.App) — separate follow-up
- New Lua-like lexer/parser/codegen producing the bytecode+signal-table format above, plus an upload service (reusing `DeviceCommunicationService` patterns) and a script editor view.
- This is a sizable, mostly-independent sub-project; recommend planning it as its own follow-up session once the firmware-side bytecode format is finalized (format is the contract between the two).

## Steps / Phases
1. **Phase 0 — Validation spikes** (*no shared dependency, can start immediately*): confirm actual RA4M1 data-flash capacity (EEPROM.length()), confirm `r_wdt` FSP driver is available/usable in this Arduino-Renesas core, confirm current free-SRAM headroom via build map.
2. **Phase 1 — IWDT integration** (*depends on Phase 0 WDT spike*): add watchdog open/start in setup(), refresh in loop(), verify via intentional-hang test that it resets the board.
3. **Phase 2 — Script VM core** (`script_engine.h/.cpp`, *no hardware dependency, can be developed/tested against a hand-written bytecode fixture in parallel with Phase 1*): opcode interpreter, verifier, fixed-size buffers, fault handling.
4. **Phase 3 — Persistence** (`script_store.h/.cpp`, *depends on Phase 0 flash-size spike and Phase 2 bytecode format*): CRC'd data-flash block, load-at-boot, upload/validate entrypoints.
5. **Phase 4 — CAN mode + main.cpp integration** (*depends on Phases 2 & 3*): `CAN_MODE_CUSTOM_SCRIPT` (slot 10) in can_modes.h/.cpp, CAN-signal decode cache, `scriptEngineService()` scheduling in loop(), output-array wiring through existing `applyOutputs()`/`serialOverrideMask` path.
6. **Phase 5 — Protocol commands** (*depends on Phase 3*): chunked upload commands, status/enable/disable commands, STATUS/telemetry fault fields.
7. **Phase 6 — PC App compiler/editor** (*separate follow-up plan, depends on final bytecode format from Phase 2*).

## Relevant files
- `PT-IO-Mini2/src/main.cpp` — setup()/loop() integration points, Config struct, applyOutputs(), globals.
- `PT-IO-Mini2/src/can_modes.h` / `can_modes.cpp` — CanModeId enum, dispatch functions to extend with CAN_MODE_CUSTOM_SCRIPT (slot 10).
- `PT-IO-Mini2/src/protocol.h` / `protocol.cpp` — JSON command dispatch chain to extend.
- New: `PT-IO-Mini2/src/script_engine.h/.cpp`, `PT-IO-Mini2/src/script_store.h/.cpp`.
- `PT-IO-Mini2-App/src/IOMini2Tool.App/Services/DeviceCommunicationService.cs` — pattern to reuse for future upload service (Phase 6).

## Verification
1. Phase 0: report actual EEPROM.length()/data-flash size and confirm r_wdt driver presence before Phase 1/3 proceed.
2. Phase 1: deliberately hang loop() (e.g. `while(true){}` behind a debug flag) and confirm MCU resets within expected watchdog window; confirm normal operation still refreshes correctly under heavy CAN load.
3. Phase 2: host-side/unit-style test harness (or on-device DIAG command) feeding hand-crafted bytecode fixtures covering: normal if/elseif/else, malformed opcode, out-of-range signal index, backward jump (must be rejected by verifier), instruction-budget overrun.
4. Phase 4: bench test with CAN_MODE_CUSTOM_SCRIPT active — verify outputs respond to local AI/DI signals per a simple uploaded script, verify serialOverrideMask still lets the app override a channel while script controls others, verify no-script-installed state drives configured safe outputs.
5. Full regression: confirm all other CAN modes / CLI / JSON telemetry / existing DFU flashing workflow unaffected.

## Decisions
- Compiler lives on PC app; firmware never parses text.
- Script control is a dedicated CAN mode (CAN_MODE_CUSTOM_SCRIPT, slot 10), not a new global bitmask — reuses existing serialOverrideMask/safe-state precedence unchanged.
- Real IWDT added as a general firmware safety net, kicked once per loop().
- Script variables/timers are RAM-only, reset on boot.
- No on-device script source storage — bytecode + signal table only.

## Further Considerations
1. Exact data-flash capacity of the RA4M1 part used is unconfirmed — this bounds the max bytecode+signal-table size and must be validated in Phase 0 before locking the 1.5KB budget.
2. Whether CAN_MODE_CUSTOM_SCRIPT should still transmit standard telemetry frames (assumed yes, reusing mode0 builders) — confirm this is desired vs. a script-defined custom TX frame (bigger scope, deferred).
3. Tier 2 of DINGOCONFIG-PLAN.md (native Conditions/VirtualInputs) is a simpler, less-expressive alternative to this entire plan for "if this, then that" logic — worth a joint decision before starting Phase 1 here.
