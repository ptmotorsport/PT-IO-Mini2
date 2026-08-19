#pragma once

#include <Arduino.h>
#include <Arduino_CAN.h>
#include "can_modes.h"

// DingoConfigurator/dingoFW-compatible CAN param protocol.
// Active only while config.canMode == CAN_MODE_DINGO_CONFIG. Reuses
// config.txBaseId as the single dingoFW "BaseId": frames at
// txBaseId+0 = config responses (device->app), txBaseId+1 = config
// commands (app->device), txBaseId+2..+7 = cyclic telemetry (dingoFW
// CYCLIC_TX_OFFSET msg0-msg5).
//
// See PT-IO-Mini2/DINGOCONFIG-PLAN.md for the full param table layout.

// Handles one incoming CAN frame. Returns true if the frame was addressed to
// this device's DingoConfig command channel (txBaseId + 1) and was processed;
// false otherwise (caller should treat the frame as not-for-us).
bool dingoHandleRx(const CanMsg &msg,
                   uint16_t baseId,
                   uint16_t outputFreq[8],
                   uint8_t outputDuty[8]);

// Caches this TX cycle's raw 14-bit analog readings so the DiPair cyclic
// slots (repurposed as analog mV frames in DingoConfig mode) can report them.
void dingoCacheAnalogRaw(const uint16_t analogRaw14[8]);

// Cyclic telemetry frame builders — one call site per CYCLIC_TX_OFFSET+n slot,
// invoked from can_modes.cpp's existing dispatch (see canModeBuildTx*Frame).
void dingoBuildStateFrame(uint16_t txBaseId, uint8_t digitalInMask, uint8_t fwVersion, ModeTxFrame &frame);
void dingoBuildDiPairFrame(uint32_t baseId, ModeTxFrame &frame);
void dingoBuildStatusFrame(uint16_t txBaseId, ModeTxFrame &frame);
