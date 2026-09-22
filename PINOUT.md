# PT-IO-DTM2 — MCU Pinout

Renesas RA4M1 (R7FA4M1AB3CFM) — 48-pin LQFP  
DTM2 branch derived from PT-IO-Mini2 main, with 4 physical PWM outputs, 4 digital inputs, switchable analog pull-ups, switchable CAN termination, and an added ADS1115 for output voltage sensing.

---

## Arduino Pin Map

| Arduino Pin | MCU Pin | DTM2 Function |
|-------------|---------|---------------|
| D0          | P301    | Pull-up Resistor Switch 4 |
| D1          | P302    | OUT3 |
| D2          | P105    | DI3 |
| D3          | P104    | DI2 |
| D4          | P103    | CAN TX |
| D5          | P102    | CAN RX |
| D6          | P106    | DI4 |
| D7          | P107    | NC |
| D8          | P304    | OUT4 |
| D9          | P303    | Pull-up Resistor Switch 3 |
| D10         | P112    | NeoPixel Output |
| D11         | P109    | NC |
| D12         | P110    | NC |
| D13         | P111    | Built-in LED |
| A0          | P014    | AVI3 |
| A1          | P000    | AVI8 |
| A2          | P001    | AVI7 |
| A3          | P002    | AVI6 |
| A4          | P101    | I2C SDA |
| A5          | P100    | I2C SCL |

## Analog Inputs (8) — ADC Unit 0

Firmware keeps the logical AV1-AV8 names, but DTM2 routes them differently from the older Mini2 board.

| Signal | MCU Pin | ADC Channel | Notes |
|--------|---------|-------------|-------|
| AV1    | P004    | AN004       | External pull-up switchable |
| AV2    | P011    | AN006       | External pull-up switchable |
| AV3    | P014    | AN009       | External pull-up switchable |
| AV4    | P015    | AN010       | External pull-up switchable |
| AV5    | P003    | AN003       |       |
| AV6    | P002    | AN002       |       |
| AV7    | P001    | AN001       |       |
| AV8    | P000    | AN000       |       |

## Digital Inputs (4) — GPT0-2 Input Capture

Only DI1-DI4 are populated on DTM2.

| Signal | MCU Pin | GPT | Channel | Notes |
|--------|---------|-----|---------|-------|
| DI1    | P501    | GPT2 | B | Input capture |
| DI2    | P104    | GPT1 | B | Input capture |
| DI3    | P105    | GPT1 | A | Input capture |
| DI4    | P106    | GPT0 | B | Input capture |

## PWM Outputs (4) — GPT4-7 Hardware PWM

Each output has its own GPT frequency on DTM2.

| Signal | MCU Pin | GPT | Channel | Notes |
|--------|---------|-----|---------|-------|
| OUT1   | P410    | GPT6 | B | Hardware PWM |
| OUT2   | P408    | GPT5 | B | Hardware PWM |
| OUT3   | P302    | GPT4 | A | Hardware PWM |
| OUT4   | P304    | GPT7 | A | Hardware PWM |

### Output PWM Specs

GPT uses the same 16-bit timers and prescaler scheme as PT-IO-Mini2.  
Firmware auto-selects the smallest divider that keeps period within 65,535 counts.

| Output Freq | Prescaler | Eff. Clock | Period (counts) | Duty Resolution |
|-------------|-----------|-----------|-----------------|-----------------|
| 50 Hz       | /8        | 3 MHz     | 60,000          | 0.002%          |
| 100 Hz      | /4        | 6 MHz     | 60,000          | 0.002%          |
| 300 Hz      | /2        | 12 MHz    | 40,000          | 0.003%          |
| 1 kHz       | /1        | 24 MHz    | 24,000          | 0.004%          |
| 5 kHz       | /1        | 24 MHz    | 4,800           | 0.02%           |
| 10 kHz      | /1        | 24 MHz    | 2,400           | 0.04%           |
| 20 kHz      | /1        | 24 MHz    | 1,200           | 0.08%           |

Min frequency (divider /1024): 24 MHz / 1024 / 65536 ≈ **0.36 Hz**  
Max frequency: limited by useful duty resolution (≥2 counts).

## Auxiliary Switch Outputs

These are plain GPIO outputs, stored in EEPROM so they persist across reboot.

| Function | MCU Pin | CLI Control | Notes |
|----------|---------|-------------|-------|
| Analog Pull-up Switch 1 | P205 | `AIPULLUP 1 0|1` | Adds external pull-up to AV1 |
| Analog Pull-up Switch 2 | P204 | `AIPULLUP 2 0|1` | Adds external pull-up to AV2 |
| Analog Pull-up Switch 3 | P303 | `AIPULLUP 3 0|1` | Adds external pull-up to AV3 |
| Analog Pull-up Switch 4 | P301 | `AIPULLUP 4 0|1` | Adds external pull-up to AV4 |
| CAN Termination Switch | P409 | `CANTERM 0|1` | Enables CAN terminating resistor |

## ADS1115 Output Voltage Sense

Added device: ADS1115IDGS on I2C address `0x90` write / `0x48` 7-bit.

| ADS Channel | Measured Signal | Front End |
|-------------|-----------------|-----------|
| AIN0        | OUT4 Voltage    | 100k / 20k divider |
| AIN1        | OUT3 Voltage    | 100k / 20k divider |
| AIN2        | OUT2 Voltage    | 100k / 20k divider |
| AIN3        | OUT1 Voltage    | 100k / 20k divider |

CLI exposure: `OUTVOLT` and `STATUS`.

## Peripherals

| Function | Pin(s) | Notes |
|----------|--------|-------|
| CAN TX   | P103   | CAN0 |
| CAN RX   | P102   | CAN0 |
| NeoPixel | P112   | GPIO bit-bang output |
| I2C SDA  | P101   | ADS1115 connection |
| I2C SCL  | P100   | ADS1115 connection |
| USB CDC  | Dedicated | USB FS device pins |
| LED      | P111   | Built-in LED |

## Clocks

| Clock | Source | Frequency | Divider |
|-------|--------|-----------|---------|
| HOCO  | —      | 24 MHz    | —       |
| ICLK  | HOCO   | 24 MHz    | DIV_1   |
| PCLKB | HOCO   | 12 MHz    | DIV_2   |
| PCLKD | HOCO   | 24 MHz    | DIV_1   |

## GPT Timer Allocation Summary

| GPT | Width | DTM2 Use | Pin A | Pin B |
|-----|-------|----------|-------|-------|
| 0   | 32-bit | DI4 capture | P107 unused | P106 |
| 1   | 32-bit | DI3 / DI2 capture | P105 | P104 |
| 2   | 16-bit | DI1 capture | P205 unused | P501 |
| 3   | 16-bit | Unused | P111 | P112 |
| 4   | 16-bit | OUT3 PWM | P302 | P301 pull-up switch 4 |
| 5   | 16-bit | OUT2 PWM | P101 SDA alt | P408 |
| 6   | 16-bit | OUT1 PWM | P112 NeoPixel alt | P410 |
| 7   | 16-bit | OUT4 PWM | P304 | P303 pull-up switch 3 |

## Spare / Unused Board Pins

| MCU Pin | Notes |
|---------|-------|
| P107    | Routed to D7, not used by firmware |
| P109    | Routed to D11, not used by firmware |
| P110    | Routed to D12, not used by firmware |
| P500    | Spare MCU pin |
| P502    | Spare MCU pin |
| P010    | Spare analog-capable MCU pin |
| P012    | Uno R4 TX LED pad |
| P013    | Uno R4 RX LED pad |
| P212    | EXTAL, if crystal used |
| P213    | XTAL, if crystal used |
