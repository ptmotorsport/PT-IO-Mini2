# TODO — Emtron (Mode 8) CAN Mode Implementation

Tracks the work to give `CAN_MODE_EMTRON` a real implementation (it currently
just forwards to `modeStub*`, which is a thin wrapper around the PT
(`mode0*`) functions — see [src/can_modes.cpp](src/can_modes.cpp) lines
653-721). This file coordinates changes across **three repositories/branches**
that must stay in sync:

| Component | Location | Branch |
|---|---|---|
| Mini2 firmware | `PT-IO-Mini2` | `main` |
| DTM2 firmware  | `PT-IO-Mini2` | `PT-IO-DTM2` |
| App            | `PT-IO-Mini2-APP` (separate repo, linked locally, gitignored) | `main` |

Reference: [Emtron Input Expander (EIC16M) CAN protocol docs](https://docs.emtronaustralia.com.au/accessories/input-expander/index.html)

---

## 1. Protocol summary (device being emulated: EIC16M)

We are emulating an Emtron EIC16M so a real Emtron ECU can decode our data
with zero extra configuration. Key facts from the datasheet:

- Factory **CAN Base Address = 705 (0x2C1)**, standard 11-bit ID, transmits
  sequentially on the next 5 IDs (705-709 / 0x2C1-0x2C5). Base address must be
  configurable (matches our existing `txBaseId` pattern).
- All frames are 8 bytes, little-endian `uint16`, Tx rate adjustable
  (50/100/200/500 Hz — we already support configurable CAN rates).
- Frame layout:

  | ID (base + n) | Bytes 0-1 | Bytes 2-3 | Bytes 4-5 | Bytes 6-7 | Scale |
  |---|---|---|---|---|---|
  | +0 (0x2C1) | AN Voltage 1 | AN Voltage 2 | AN Voltage 3 | AN Voltage 4 | 0.001 V/count |
  | +1 (0x2C2) | AN Voltage 5 | AN Voltage 6 | AN Voltage 7 | AN Voltage 8 | 0.001 V/count |
  | +2 (0x2C3) | AN Voltage 9 | AN Voltage 10 | AN Voltage 11 | AN Voltage 12 | 0.001 V/count |
  | +3 (0x2C4) | AN Voltage 13 | AN Voltage 14 | AN Voltage 15 | AN Voltage 16 | 0.001 V/count |
  | +4 (0x2C5) | Frequency 1 | Frequency 2 | Frequency 3 | Frequency 4 | 0.1 Hz/count |
  | +5 (0x2C6, 2 bytes, 20 Hz fixed) | 5V Sensor Supply | — | — | — | 0.001 V/count |

- AN Voltage 1-12: 0.0-5.0 V range. AN Voltage 13-16 are shared with
  Frequency 1-4 (same physical pins, either mode).
- There is **no output channel** in the Emtron protocol — the EIC16M is
  input-only (its only "output" is the 5V sensor supply rail reading).

This is a **transmit-only emulation**: unlike our other CAN modes, there is no
meaningful RX side to decode (the real EIC16M doesn't receive control frames
from the ECU, just gets auto-detected). `emtronHandleRx` can be a no-op stub.

---

## 2. Shared / common work (do first, in `can_modes.cpp` on each branch) — DONE (PT-IO-DTM2 branch)

- [x] Added dedicated `emtron*` static functions (mirror the pattern used for
  `motecE888*` / `ecuMasterCanswb*`), replacing the `modeStub*` passthrough
  for `CAN_MODE_EMTRON` in [src/can_modes.cpp](src/can_modes.cpp).
- [x] `ModeTxFrame frame0/frame1` only carries 2 frames per call; the
  Frequency 1-4 frame (base+4) is accumulated across two `canModeBuildTxDiPairFrame`
  calls (DI0-1, DI2-3) using a static accumulator, following the `motecE888`
  precedent, and emitted on the second call.
- [x] Raw-ADC → millivolt conversion reuses the existing `scaleAnalog14ToMv5000`
  helper (already used by ECU Master mode, identical 0-5.0 V / 14-bit scale
  Emtron needs — no new helper required).
- [x] `emtronHandleRx` = no-op stub (protocol has no RX side to decode).
- [ ] Unit/self-check: one small test verifying a known raw ADC value maps to
  the expected mV count in the Emtron frame (ponytail: non-trivial scaling
  logic needs a runnable check) — **still outstanding**.
- [ ] AN Voltage 9-16 (frames +2/+3) and the 5V Sensor Supply frame (+5) are
  not sent at all yet (see §3/§4 — need confirmation a real Emtron ECU
  tolerates missing frames in the 705-710 sequence, and §4 needs a real 5V
  rail ADC source before that frame can be sent truthfully).

## 3. Mini2-specific (`main` branch)

Mini2 has 8× analog in, 8× digital in (freq/duty), 8× digital out.

- [x] Map AV1-8 → Emtron AN Voltage 1-8 (frames +0, +1) directly — ported
  from the DTM2 implementation (§4), same code shared via `can_modes.cpp`.
- [x] **Resolved:** Emtron EIC16M only has 4 frequency channels, but Mini2
  has 8 digital inputs. DI1-4 → EIC16M Frequency 1-4 (frame base+4), same
  as DTM2. DI5-8 → emulate a **second Emtron device, the EIC10**, on its own
  CAN base ID so the ECU can decode it as a distinct device rather than us
  inventing a non-spec extension to the EIC16M frames.
  - No public Emtron datasheet for the EIC10 (checked
    [docs.emtronaustralia.com.au/accessories](https://docs.emtronaustralia.com.au/accessories/index.html)
    — EIC16M is the only Input Expander documented there).
  - Confirmed from Emtune's ECU CAN Dataset picker: `73: Emtron EIC10 x1
    Device (CAN PID 718-721)` — a fixed 4-ID range (vs. EIC16M's 5-6).
  - **Assumption (unverified against real hardware/Emtune captures):** same
    per-value encoding as EIC16M (8-byte frames, 4× `uint16` LE per frame,
    analog 0.001 V/count, frequency 0.1 Hz/count). Only the base ID differs
    (718 instead of 705).
  - **Open question:** which of the 4 IDs (718-721) is the Frequency frame
    vs. spare/analog frames — EIC16M puts Frequency last (base+4) after 3
    analog frames (base+0..+2), but EIC10 only has 4 IDs total. Best guess:
    718 = AN Voltage 1-4, 719 = AN Voltage 5-8, 720 = AN Voltage 9-10 (2
    channels, rest padding), 721 = Frequency 1-4 — i.e. same frame *shapes*
    as EIC16M, just fewer of them and renumbered from 718. Since Mini2 only
    has spare DI5-8 data (no spare analog channels) to put on this second
    device, we'd only ever populate 721 (Frequency) and leave 718-720 as
    zero/unsent.
  - **Implemented:** added `eic10BaseId` field to `Config` (default 718),
    `CONFIG_VERSION` bumped 2→3, CLI `EIC10BASE <hex>` command, JSON
    `eic10BaseId` field in `getConfig`/`setCanConfig`. Only the Frequency
    frame (`eic10BaseId + 3` = 721) is ever populated, and only when DI5-8
    actually show real capture data — this makes the frame a no-op on
    hardware without DI5-8 data by construction.
  - Still **must bench-verify** against Emtune/a real Emtron ECU before
    trusting the 718-721 frame-shape mapping — not yet bench-tested.
- [ ] AN Voltage 9-16 (frames +2/+3 of the primary EIC16M ID): Mini2 has no
  extra analog inputs to fill these — left unsent (frames +2/+3 never
  built/transmitted), same approach as DTM2 (confirm whether a real Emtron
  ECU tolerates missing frames from the sequence).

## 4. DTM2-specific (`PT-IO-DTM2` branch) — input mapping DONE

DTM2 has 8× analog in (AV1-8, see [PINOUT.md](PINOUT.md)) and only 4×
digital in (DI1-4) — this is a **cleaner fit** than Mini2: DI1-4 map exactly
onto Emtron Frequency 1-4 with nothing left over.

- [x] Map AV1-8 → Emtron AN Voltage 1-8 (frames +0, +1), same as Mini2.
- [x] Map DI1-4 → Frequency 1-4 (frame +4) — 1:1, no leftover channels.
  (DI5-8 calls in `sendTxFrames()` don't exist on DTM2 since `NUM_DIGITAL_IN == 4`,
  so the 3rd/4th `packFreqDuty` calls just see `hasPeriod == false` and the
  Emtron dispatcher's "call index 2/3" branch sends nothing, as intended.)
- [x] AN Voltage 9-16 — left unsent (frames +2/+3 never built/transmitted).
- [ ] DTM2's ADS1115 output-voltage sensing has no Emtron-spec home either;
  park under "outputs" work (§5).
- [ ] Not yet bench-tested against a real Emtron ECU/display — verify byte
  layout and scaling against actual hardware before calling this done.

## 5. Output channel handling

Emtron's own EIC16M protocol has zero output channels, but PT's IO expanders
do (Mini2: 8, DTM2: 4 + ADS1115 sense). **This has been solved before** —
PT Motorsport's prior Haltech-based boards (4 AI/4 DI/4 DO hardware, not
today's 8-channel Mini2/DTM2) already did this for real customers. Found two
primary sources documenting it:

- Write-up: [Emtron CAN information for use with PT Motorsport IO expanders](https://www.ptmotorsport.com.au/emtron-can-information-for-use-with-pt-motorsport-io-expanders/)
- Reference firmware: [`PTMotorsportAU_CANIO_DTM_EMTRON_AUG25`](https://github.com/ptmotorsport/IObox-emulator-haltech/blob/patch-1/PTMotorsportAU_CANIO_DTM_EMTRON/PTMotorsportAU_CANIO_DTM_EMTRON_AUG25)
  (public repo, MIT-style licence, explicitly says "the DTM IO and the Mini IO
  both use the same firmware")

### How it actually works (confirmed from the reference firmware)

The Emtron ECU doesn't natively know about our outputs — EMtune lets the user
build a **Custom Tx Data Set** on the ECU (parameters: `User 1-4 Duty Cycle`)
and transmit it on a CAN ID of their choosing. The article has the user set
that ID to **722 (0x2D2)**, and the reference firmware hardcodes exactly that
as `DPOrxCANAddress1 = 0x2D2`. This is RX, not TX — the IO expander *receives*
duty commands from the ECU, same direction as our existing PT/Haltech output
RX paths.

Confirmed byte layout (`canRead()` in the reference firmware), one 8-byte
frame carries all 4 duty values, little-endian `uint16`, 0-1000 = 0.0-100.0%
(this is the exact same 0.1%-resolution convention already used internally by
our `mapDuty1000To255` / Haltech IO16 duty handling — no new scale needed):

| Bytes | Value |
|---|---|
| 0-1 | DPO1 duty, 0-1000 |
| 2-3 | DPO2 duty, 0-1000 |
| 4-5 | DPO3 duty, 0-1000 |
| 6-7 | DPO4 duty, 0-1000 |

**Decided (per user):** the reference firmware packs ad-hoc "safe state"/
"active state" flag bits into bits 4/5 of each duty value's *high byte*
(`bitRead(rxBuf[1], 4/5)` etc.) — a legacy hack, not something Emtron's
protocol actually defines. We are **not** replicating it. Instead:

- The app's own safe-state programming (`config.safeMask`/`config.activeMask`,
  set via the existing USB JSON protocol and persisted to EEPROM) stays
  **stored but dormant** the whole time Emtron CAN frames are arriving on
  time — while Emtron is "calling the shots", outputs just track the live
  commanded duty from the 722 frame, full stop.
- That stored safe state only becomes live through the firmware's existing,
  mode-agnostic CAN-RX-timeout watchdog (`rxTimeoutMs` / `applyOutputs(true)`
  in `main.cpp`) — the same mechanism every other CAN mode already relies on.
  No Emtron-specific safe-state code is needed at all.
- This is already how `canModeHandleRx`'s `CAN_MODE_EMTRON` case was wired
  (it never touches `safeMask`/`activeMask`, always reports `maskChanged =
  false`) — implemented and build-verified.

Also confirmed from the same firmware: the EMtune UI lets the user manually
pick each input's byte offset/endianness per channel (see "Input Setup" in the
article) — so our analog/frequency TX byte order doesn't have to blindly match
the datasheet to work in practice, the end user can remap it in EMtune. Our
current implementation already follows the datasheet's little-endian
convention, which the reference firmware's analog TX (`SendAnalogValues()`)
also does — so no change needed there, just noting it as the reason the
EIC10 question (previous message) is likely fine even without a datasheet.

### Tasks

- [x] Decide on the safe/active flag-bit question above before writing code.
- [x] Implement Emtron output RX: decode the 4x `uint16` LE duty frame at
  `rxBaseId` (configure to 722 for Emtron mode — no new config field needed,
  reuses the existing, previously-unused-by-Emtron `config.rxBaseId`) into
  `outputDuty[0..3]` via `mapDuty1000To255`. Done in `can_modes.cpp`
  (`emtronHandleRx`), DTM2 branch, build-verified.
- [x] Mini2's 8 physical outputs vs EMtune's 4-channel Custom Tx Data Set:
  solved by adding a **second** Custom Tx Data Set in EMtune at `rxBaseId+1`
  (723/0x2D3 by convention) for DPO5-8 — same "second device" pattern as the
  EIC10 addendum in §3. Since Custom Tx is user-defined in EMtune, there's no
  fixed spec to violate here, just a mapping to keep in sync on both ends.
  Implemented in `emtronHandleRx`/`emtronDecodeDutyFrame`, build-verified.
  DTM2 (4 outputs) simply never sees/needs the second frame.
- [ ] Confirm this doesn't collide with the Emtron CAN ID range (705-710 for
  EIC16M #1, 718-721 for EIC10 #1) — 722 sits just outside both, matches the
  article's recommendation.
- [ ] Not yet bench-tested by us against a real Emtron ECU — verify byte
  layout/scaling behaviour before calling this done, per user request.
- [ ] Document the decision in [PROTOCOL.md](PROTOCOL.md) once settled.

## 6. App integration (`PT-IO-Mini2-APP`)

- [ ] Add "Emtron" as a selectable CAN mode in device configuration UI (base
  ID field, default 705/0x2C1).
- [ ] Decode/display AN Voltage 1-16 + Frequency 1-4 + 5V supply for live
  monitoring, matching the Emtron scale factors above.
- [ ] Surface the Mini2-vs-DTM2 channel-count difference in the UI (don't
  show AV9-16/DI5-8 controls that don't apply to the connected device's
  Emtron mapping).
- [ ] Once §5 is settled, add whatever UI is needed to control outputs
  out-of-band from the Emtron emulation frames.

## 7. Cross-repo sync checklist

- [ ] Keep the Emtron scaling/frame-layout logic in `can_modes.cpp`
  consistent between `main` (Mini2) and `PT-IO-DTM2` — port fixes both ways,
  they will not be identical (different channel counts) but should share the
  same constants/scale factors.
- [ ] Update `canModeName()` / docs in both branches when mode 8 moves from
  stub to real.
- [ ] Update [PROTOCOL.md](PROTOCOL.md) with the finalized Mini2 and DTM2
  Emtron channel maps.
- [ ] Bump app's device-compatibility check if the JSON/status protocol
  changes as part of this work.
