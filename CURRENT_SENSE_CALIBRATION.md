# Current Sense Calibration Table

Date: __________
Board/Throttle: __________
Supply Voltage: __________
Ambient Temp: __________

This template assumes current-sense channels are on AV1-AV4.
From firmware mapping:
- AV1 -> P000 (AN000)
- AV2 -> P001 (AN001)
- AV3 -> P002 (AN002)
- AV4 -> P003 (AN003)

Use STATUS output to read ADC raw values.
ADC conversion (14-bit, 5.0 V reference):
- V_pin = raw * 5.0 / 16383

Linear calibration model per channel:
- I_est = gain_A_per_V * V_pin + offset_A

If your sensor is ratiometric around a zero-current midpoint V_zero:
- I_est = gain_A_per_V * (V_pin - V_zero)

## 1) Zero-Current Offset Capture

Take at least 50 samples per channel with no motor current.

| Channel | Signal Name | MCU Pin | Mean Raw Zero | StdDev Raw Zero | Mean V_zero (V) | Notes |
|---|---|---|---:|---:|---:|---|
| 1 | CS1 | P000 / AV1 |  |  |  |  |
| 2 | CS2 | P001 / AV2 |  |  |  |  |
| 3 | CS3 | P002 / AV3 |  |  |  |  |
| 4 | CS4 | P003 / AV4 |  |  |  |  |

## 2) Loaded Points (Per Channel)

Record at least 5 points spanning expected operating current.
Recommended points: 0 A, 1 A, 2 A, 4 A, 6 A (or your real range).

| Channel | Ref Current (A) | Mean Raw | Mean V_pin (V) | Delta V from Zero (V) | Computed Current (A) | Error vs Ref (A) |
|---|---:|---:|---:|---:|---:|---:|
| CS1 | 0.0 |  |  |  |  |  |
| CS1 | 1.0 |  |  |  |  |  |
| CS1 | 2.0 |  |  |  |  |  |
| CS1 | 4.0 |  |  |  |  |  |
| CS1 | 6.0 |  |  |  |  |  |
| CS2 | 0.0 |  |  |  |  |  |
| CS2 | 1.0 |  |  |  |  |  |
| CS2 | 2.0 |  |  |  |  |  |
| CS2 | 4.0 |  |  |  |  |  |
| CS2 | 6.0 |  |  |  |  |  |
| CS3 | 0.0 |  |  |  |  |  |
| CS3 | 1.0 |  |  |  |  |  |
| CS3 | 2.0 |  |  |  |  |  |
| CS3 | 4.0 |  |  |  |  |  |
| CS3 | 6.0 |  |  |  |  |  |
| CS4 | 0.0 |  |  |  |  |  |
| CS4 | 1.0 |  |  |  |  |  |
| CS4 | 2.0 |  |  |  |  |  |
| CS4 | 4.0 |  |  |  |  |  |
| CS4 | 6.0 |  |  |  |  |  |

## 3) Fit Results (Per Channel)

Fit a straight line to each channel.

| Channel | Gain (A/V) | Offset (A) | R^2 | Max Abs Error (A) | Max Abs Error (%) | Pass/Fail |
|---|---:|---:|---:|---:|---:|---|
| CS1 |  |  |  |  |  |  |
| CS2 |  |  |  |  |  |  |
| CS3 |  |  |  |  |  |  |
| CS4 |  |  |  |  |  |  |

Suggested pass criteria:
- R^2 >= 0.995
- Max Abs Error <= 0.15 A (or <= 3% of reading)
- Zero-current drift <= 0.05 A equivalent over 5 minutes

## 4) Runtime Calibration Constants

Populate these values for firmware integration.

| Channel | V_zero (V) | Gain (A/V) | Offset (A) | Clamp Min (A) | Clamp Max (A) |
|---|---:|---:|---:|---:|---:|
| CS1 |  |  |  |  |  |
| CS2 |  |  |  |  |  |
| CS3 |  |  |  |  |  |
| CS4 |  |  |  |  |  |

## 5) Sanity/Noise Checks

| Check | Result | Notes |
|---|---|---|
| Motor disabled noise floor stable |  |  |
| Noise under PWM load acceptable |  |  |
| No clipping near max expected current |  |  |
| Channel-to-channel agreement acceptable |  |  |

## 6) Signoff

Technician: __________
Date: __________
Revision: __________
