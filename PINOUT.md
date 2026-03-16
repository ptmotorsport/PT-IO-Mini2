# PT-IO-Mini2 — MCU Pinout (Rev 2)

Renesas RA4M1 (R7FA4M1AB3CFM) — 48-pin LQFP  
Custom PCB, all MCU pins routable.

## Changes from Rev 1 (Prototype)

| Signal | Rev 1 Pin | Rev 2 Pin | Reason |
|--------|-----------|-----------|--------|
| DPO7   | P300 (SWCLK) | **P303** (GPT7B) | No GPT on P300; frees SWD debug |
| DPO8   | P108 (SWDIO) | **P304** (GPT7A) | No GPT on P108; frees SWD debug |

All other pins unchanged.

---

## Analog Inputs (8) — ADC Unit 0

| Signal | MCU Pin | ADC Channel | Notes |
|--------|---------|-------------|-------|
| AV1    | P000    | AN000       |       |
| AV2    | P001    | AN001       |       |
| AV3    | P002    | AN002       |       |
| AV4    | P003    | AN003       |       |
| AV5    | P004    | AN004       |       |
| AV6    | P011    | AN006       |       |
| AV7    | P014    | AN009       | DAC capable |
| AV8    | P015    | AN010       |       |

## Digital Inputs (8) — GPT0-3 Input Capture

Each GPT timer captures two inputs (A + B channels).  
All 32-bit counters on GPT0-1; 16-bit on GPT2-3.

| Signal | MCU Pin | GPT  | Channel | Timer Group |
|--------|---------|------|---------|-------------|
| DI1    | P107    | GPT0 | A       | GPT0: DI1+DI2 |
| DI2    | P106    | GPT0 | B       | GPT0: DI1+DI2 |
| DI3    | P105    | GPT1 | A       | GPT1: DI3+DI4 |
| DI4    | P104    | GPT1 | B       | GPT1: DI3+DI4 |
| DI5    | P113    | GPT2 | A       | GPT2: DI5+DI6 |
| DI6    | P501    | GPT2 | B       | GPT2: DI5+DI6 |
| DI7    | P111    | GPT3 | A       | GPT3: DI7+DI8 |
| DI8    | P112    | GPT3 | B       | GPT3: DI7+DI8 |

## Digital Outputs (8) — GPT4-7 Hardware PWM

Each GPT timer drives two outputs (A + B channels).  
Outputs in the same pair share frequency but have independent duty cycles.  
All 16-bit timers (GPT4-7), clocked at PCLKD = 24 MHz.

| Signal | MCU Pin | GPT  | Channel | Freq Group |
|--------|---------|------|---------|------------|
| DPO1   | P302    | GPT4 | A       | Freq A     |
| DPO2   | P301    | GPT4 | B       | Freq A     |
| DPO3   | P408    | GPT5 | B       | Freq B     |
| DPO4   | P409    | GPT5 | A       | Freq B     |
| DPO5   | P410    | GPT6 | B       | Freq C     |
| DPO6   | P411    | GPT6 | A       | Freq C     |
| DPO7   | P303    | GPT7 | B       | Freq D     |
| DPO8   | P304    | GPT7 | A       | Freq D     |

### Output PWM Specs (GPT4-7, 16-bit, PCLKD = 24 MHz)

GPT has built-in prescalers (/1, /2, /4, /8, /16, /32, /64, /256, /1024).  
Firmware auto-selects the smallest divider that keeps period ≤ 65,535.

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
Max frequency: limited by useful duty resolution (≥2 counts → 12 MHz)

> At 5 kHz with 1% duty resolution: need ≥100 counts → 4,800 counts available. ✔

## Peripherals

| Function   | Pin(s)     | Notes |
|------------|------------|-------|
| CAN TX     | P103       | CAN0 (PSEL=0x09) |
| CAN RX     | P102       | CAN0 (PSEL=0x09) |
| NeoPixel   | P109       | GPIO bit-bang |
| USB CDC    | Dedicated  | USB FS device pins |
| SWD SWDIO  | P108       | Debug — freed from DPO8 |
| SWD SWCLK  | P300       | Debug — freed from DPO7 |

## Clocks

| Clock  | Source | Frequency | Divider |
|--------|--------|-----------|---------|
| HOCO   | —      | 24 MHz    | —       |
| ICLK   | HOCO   | 24 MHz    | DIV_1   |
| PCLKB  | HOCO   | 12 MHz    | DIV_2   |
| PCLKD  | HOCO   | 24 MHz    | DIV_1   |

## GPT Timer Allocation Summary

| GPT | Width  | Function        | Pin A | Pin B |
|-----|--------|-----------------|-------|-------|
| 0   | 32-bit | DI capture 1+2  | P107  | P106  |
| 1   | 32-bit | DI capture 3+4  | P105  | P104  |
| 2   | 16-bit | DI capture 5+6  | P113  | P501  |
| 3   | 16-bit | DI capture 7+8  | P111  | P112  |
| 4   | 16-bit | PWM out 1+2     | P302  | P301  |
| 5   | 16-bit | PWM out 3+4     | P409  | P408  |
| 6   | 16-bit | PWM out 5+6     | P411  | P410  |
| 7   | 16-bit | PWM out 7+8     | P304  | P303  |

## Spare Pins (available on 48-pin LQFP)

| MCU Pin | Capabilities | Notes |
|---------|-------------|-------|
| P100    | GPT5B alt, AN022, I2C SCL | |
| P101    | GPT5A alt, AN021, I2C SDA | |
| P110    | GPT1B alt, SPI MISO | |
| P400    | GPT6A alt, I2C SCL0 | |
| P500    | GPT2A alt, AN016 | |
| P502    | GPT3B alt, AN018 | |
| P010    | AN005 | |
| P012    | AN007 | TX LED on UNO R4 |
| P013    | AN008 | RX LED on UNO R4 |
| P212    | EXTAL | Crystal osc (if used) |
| P213    | XTAL  | Crystal osc (if used) |
