#include <Arduino.h>
#include <Arduino_CAN.h>
#include <EEPROM.h>
#include <Wire.h>
#include "bsp_api.h"
#include "r_ioport.h"
#include "r_adc.h"
#include "FspTimer.h"
#include "can_modes.h"
#include "protocol.h"

// NeoPixel Configuration (raw BSP pin, not Arduino pin index)
#define NEOPIXEL_BSP_PIN BSP_IO_PORT_01_PIN_12  // P112 (DTM2 D10)
#define NEOPIXEL_COUNT 1
constexpr uint8_t NEOPIXEL_BRIGHTNESS = 26; // ~10% of 255

// WS2812 timing (800 kHz) using DWT cycle counter on Cortex-M4.
#define ARM_DEMCR (*((volatile uint32_t *)0xE000EDFCUL))
#define ARM_DEMCR_TRCENA (1UL << 24)
#define ARM_DWT_CTRL (*((volatile uint32_t *)0xE0001000UL))
#define ARM_DWT_CTRL_CYCCNTENA (1UL << 0)
#define ARM_DWT_CYCCNT (*((volatile uint32_t *)0xE0001004UL))

static volatile uint16_t *neoSetReg = nullptr;
static volatile uint16_t *neoClrReg = nullptr;
static uint16_t neoMask = 0;

static uint8_t scaleNeo(uint8_t value) {
  return static_cast<uint8_t>(((uint16_t)value * NEOPIXEL_BRIGHTNESS + 127U) / 255U);
}

static uint32_t makeNeoColor(uint8_t r, uint8_t g, uint8_t b) {
  return (static_cast<uint32_t>(r) << 16) |
         (static_cast<uint32_t>(g) << 8) |
         static_cast<uint32_t>(b);
}

static void neopixelInitRaw() {
  constexpr uint32_t IOPORT_CFG_OUTPUT_LOW =
    IOPORT_CFG_PORT_DIRECTION_OUTPUT | IOPORT_CFG_PORT_OUTPUT_LOW;

  R_IOPORT_PinCfg(&g_ioport_ctrl, NEOPIXEL_BSP_PIN, IOPORT_CFG_OUTPUT_LOW);

  // Port registers are spaced evenly by device definition; derive POSR/PORR from pin.
  R_PORT0_Type *port = (R_PORT0_Type *)(R_PORT0 + ((uint32_t)(R_PORT1 - R_PORT0) * (NEOPIXEL_BSP_PIN >> 8U)));
  neoSetReg = &port->POSR;
  neoClrReg = &port->PORR;
  neoMask = static_cast<uint16_t>(1U << (NEOPIXEL_BSP_PIN & 0xFFU));
}

static void neopixelShowRaw(uint8_t r, uint8_t g, uint8_t b) {
  if (neoSetReg == nullptr || neoClrReg == nullptr) {
    return;
  }

  // WS2812 expects GRB byte order.
  uint8_t bytes[3] = {scaleNeo(g), scaleNeo(r), scaleNeo(b)};

  constexpr uint32_t fCpu = 48000000UL;
  constexpr uint32_t cyclesT0H = fCpu / 4000000UL;  // ~0.25 us
  constexpr uint32_t cyclesT1H = fCpu / 1250000UL;  // ~0.80 us
  constexpr uint32_t cyclesBit = fCpu / 800000UL;   // 1.25 us

  noInterrupts();
  ARM_DEMCR |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;

  uint32_t cyc = ARM_DWT_CYCCNT + cyclesBit;
  for (uint8_t i = 0; i < 3; i++) {
    uint8_t pix = bytes[i];
    for (uint8_t mask = 0x80; mask != 0; mask >>= 1) {
      while ((ARM_DWT_CYCCNT - cyc) < cyclesBit) {
      }
      cyc = ARM_DWT_CYCCNT;
      *neoSetReg = neoMask;
      if (pix & mask) {
        while ((ARM_DWT_CYCCNT - cyc) < cyclesT1H) {
        }
      } else {
        while ((ARM_DWT_CYCCNT - cyc) < cyclesT0H) {
        }
      }
      *neoClrReg = neoMask;
    }
  }
  while ((ARM_DWT_CYCCNT - cyc) < cyclesBit) {
  }

  interrupts();
  delayMicroseconds(60);  // Latch time
}

static void neopixelShowPacked(uint32_t color) {
  uint8_t r = static_cast<uint8_t>((color >> 16) & 0xFFU);
  uint8_t g = static_cast<uint8_t>((color >> 8) & 0xFFU);
  uint8_t b = static_cast<uint8_t>(color & 0xFFU);
  neopixelShowRaw(r, g, b);
}

// Analog Input Pin Definitions (using FSP)
// DTM2 keeps the existing AV1-AV8 logical ordering, but the PCB routes those
// signals onto a different set of MCU analog pins.
const bsp_io_port_pin_t ANALOG_PINS[] = {
  BSP_IO_PORT_00_PIN_04,  // P004 - AV1
  BSP_IO_PORT_00_PIN_11,  // P011 - AV2
  BSP_IO_PORT_00_PIN_14,  // P014 - AV3
  BSP_IO_PORT_00_PIN_15,  // P015 - AV4
  BSP_IO_PORT_00_PIN_03,  // P003 - AV5
  BSP_IO_PORT_00_PIN_02,  // P002 - AV6
  BSP_IO_PORT_00_PIN_01,  // P001 - AV7
  BSP_IO_PORT_00_PIN_00   // P000 - AV8
};
const char* ANALOG_NAMES[] = {"AV1", "AV2", "AV3", "AV4", "AV5", "AV6", "AV7", "AV8"};
const char* ANALOG_PORT_NAMES[] = {"P004", "P011", "P014", "P015", "P003", "P002", "P001", "P000"};
const int NUM_ANALOG = 8;

const uint8_t ANALOG_CHANNELS[] = {4, 6, 9, 10, 3, 2, 1, 0};

// Digital Input Pin Definitions (using FSP)
const bsp_io_port_pin_t DIGITAL_IN_PINS[] = {
  BSP_IO_PORT_05_PIN_01,  // P501 - DI1 (GTIOC2B)
  BSP_IO_PORT_01_PIN_04,  // P104 - DI2 (GTIOC1B)
  BSP_IO_PORT_01_PIN_05,  // P105 - DI3 (GTIOC1A)
  BSP_IO_PORT_01_PIN_06   // P106 - DI4 (GTIOC0B)
};
const char* DIGITAL_IN_PORT_NAMES[] = {"P501", "P104", "P105", "P106"};
const int NUM_DIGITAL_IN = 4;
const int DI_SLOT_COUNT = 8;

// Digital Output Pin Definitions (using FSP)
const bsp_io_port_pin_t DIGITAL_OUT_PINS[] = {
  BSP_IO_PORT_04_PIN_10,  // P410 - OUT1 (GTIOC6B)
  BSP_IO_PORT_04_PIN_08,  // P408 - OUT2 (GTIOC5B)
  BSP_IO_PORT_03_PIN_02,  // P302 - OUT3 (GTIOC4A)
  BSP_IO_PORT_03_PIN_04   // P304 - OUT4 (GTIOC7A)
};
const char* DIGITAL_OUT_PORT_NAMES[] = {"P410", "P408", "P302", "P304"};
const int NUM_DIGITAL_OUT = 4;
const int OUTPUT_SLOT_COUNT = 8;

const bsp_io_port_pin_t ANALOG_PULLUP_SWITCH_PINS[] = {
  BSP_IO_PORT_02_PIN_05,  // P205 - AV1 pull-up switch
  BSP_IO_PORT_02_PIN_04,  // P204 - AV2 pull-up switch
  BSP_IO_PORT_03_PIN_03,  // P303 - AV3 pull-up switch
  BSP_IO_PORT_03_PIN_01   // P301 - AV4 pull-up switch
};
const char* ANALOG_PULLUP_SWITCH_PORT_NAMES[] = {"P205", "P204", "P303", "P301"};
const int NUM_ANALOG_PULLUP_SWITCHES = 4;

const bsp_io_port_pin_t CAN_TERM_SWITCH_PIN = BSP_IO_PORT_04_PIN_09;  // P409
const char* CAN_TERM_SWITCH_PORT_NAME = "P409";

// ADC Configuration
const int ADC_RESOLUTION = 14;  // 14-bit ADC
const float AREF = 5.0;         // Reference voltage
const int ADC_MAX = (1 << ADC_RESOLUTION) - 1;  // 16383 for 14-bit

// Defaults
const uint16_t DEFAULT_CAN_SPEED_KBPS = 1000;
const uint16_t DEFAULT_TX_RATE_HZ = 10;
const uint16_t DEFAULT_TX_BASE_ID = 0x700;
const uint16_t DEFAULT_RX_BASE_ID = 0x640;
const uint16_t DEFAULT_RX_TIMEOUT_MS = 2000;
const uint16_t DEFAULT_PWM_FREQ_HZ = 300;
const uint8_t DEFAULT_DI_DEBOUNCE_MS = 20;
const uint8_t FW_VERSION = 0x08;

const uint8_t AUX_SWITCH_ANALOG_PULLUP_1 = 1U << 0;
const uint8_t AUX_SWITCH_ANALOG_PULLUP_2 = 1U << 1;
const uint8_t AUX_SWITCH_ANALOG_PULLUP_3 = 1U << 2;
const uint8_t AUX_SWITCH_ANALOG_PULLUP_4 = 1U << 3;
const uint8_t AUX_SWITCH_CAN_TERM = 1U << 4;
const uint8_t AUX_SWITCH_MASK_ALL = AUX_SWITCH_ANALOG_PULLUP_1 |
                                    AUX_SWITCH_ANALOG_PULLUP_2 |
                                    AUX_SWITCH_ANALOG_PULLUP_3 |
                                    AUX_SWITCH_ANALOG_PULLUP_4 |
                                    AUX_SWITCH_CAN_TERM;

const uint8_t OUTPUT_SENSE_COUNT = 4;
const uint8_t OUTPUT_SENSE_I2C_ADDRESS = 0x48;  // ADS1115 7-bit address (datasheet write addr 0x90)
const uint8_t OUTPUT_SENSE_CHANNELS[OUTPUT_SENSE_COUNT] = {3, 2, 1, 0};
const float OUTPUT_SENSE_FULL_SCALE_VOLTS = 6.144f;
const float OUTPUT_SENSE_DIVIDER_RATIO = 6.0f;

// Additional hardware polarity stage (e.g. external MOSFET inverter).
// Bit=1 means invert the post-activeMask duty before writing to the pin.
// Set to 0xFF to invert all outputs, or set bits per-channel as needed.
const uint8_t OUTPUT_STAGE_INVERT_MASK = 0x00;

// Digital input duty polarity.
// Bit=1 means DI duty is reported as low-time (active LOW).
const uint8_t DI_ACTIVE_LOW_MASK = 0xFF;

// Config persistence
const uint16_t CONFIG_MAGIC = 0x5049; // "PI"
const uint8_t CONFIG_VERSION = 3;

struct Config {
  uint16_t magic;
  uint8_t version;
  uint16_t canSpeedKbps;
  uint16_t txBaseId;
  uint16_t rxBaseId;
  uint16_t txRateHz;
  uint16_t rxTimeoutMs;
  uint16_t outFreqHz[4];
  uint8_t safeMask;
  uint8_t activeMask;
  uint8_t canMode;
  uint8_t inputPullupMask;
  uint8_t reserved[2];
  uint16_t crc;
};

Config config;

// ADC driver
adc_instance_ctrl_t adc_ctrl;
adc_cfg_t adc_cfg;
adc_channel_cfg_t adc_channel_cfg;
adc_extended_cfg_t adc_ext_cfg;
bool adc_ready = false;
uint16_t adcBuf[2][NUM_ANALOG];
volatile uint8_t adcActiveBuf = 0;
bool adcScanRunning = false;
uint32_t adcScanFailCount = 0;
bool outputSenseReady = false;

// Output state
uint8_t outputDuty[OUTPUT_SLOT_COUNT];
uint16_t outputFreq[OUTPUT_SLOT_COUNT];
uint32_t lastAppliedPwmCounts[NUM_DIGITAL_OUT] = {
  0xFFFFFFFFUL, 0xFFFFFFFFUL, 0xFFFFFFFFUL, 0xFFFFFFFFUL
};

uint32_t diTimerFreq[DI_SLOT_COUNT];

// Input capture stats
volatile uint32_t diLastRise[DI_SLOT_COUNT];       // timer count at last rising edge (0 = no rise yet)
volatile uint32_t diLastRiseOverflow[DI_SLOT_COUNT]; // per-channel overflow count at last rising edge
volatile bool     diHasFirstRise[DI_SLOT_COUNT];   // true once first rising edge has been seen
volatile uint32_t diPeriodCounts[DI_SLOT_COUNT];
volatile uint32_t diHighCounts[DI_SLOT_COUNT];
volatile bool diHasPeriod[DI_SLOT_COUNT];
volatile bool diHasHigh[DI_SLOT_COUNT];
volatile uint32_t diCapSeq[DI_SLOT_COUNT];  // incremented on each capture event

// Stale capture detection (main loop)
uint32_t diCapSeqLast[DI_SLOT_COUNT];
uint32_t diCapTsLast[DI_SLOT_COUNT];
const uint32_t DI_STALE_TIMEOUT_MS = 500;

// Debounced DI state used for reported state (CAN state frame + STATUS)
bool diDebouncedState[DI_SLOT_COUNT];
bool diRawPrevState[DI_SLOT_COUNT];
uint32_t diRawStableMs[DI_SLOT_COUNT];

// Serial connection tracking
bool serialWelcomeSent = false;

// JSON telemetry subscription
TelemetrySubscription telemetrySub = {false, TELEMETRY_FORMAT_TEXT, 1000, 0};

// Runtime status
uint32_t lastCanRxMs = 0;
uint32_t lastTxMs = 0;
uint32_t lastLedMs = 0;
bool outputsInSafeState = false;
bool monitorEnabled = false;
uint32_t monitorIntervalMs = 1000;
uint32_t lastMonitorMs = 0;
bool canInitOk = false;
uint16_t canRxCount = 0;
uint16_t canTxCount = 0;
uint16_t canTxFail = 0;
uint8_t serialOverrideMask = 0x00;  // bitmask: bit set = channel under serial/app control, not CAN
bool rxDebug = false;         // when true, log RX duties, applyOutputs triggers and TX failures

// Timers - input capture
FspTimer gpt0; // DI4 on GTIOC0B
FspTimer gpt1; // DI2/DI3 on GTIOC1B/A
FspTimer gpt2; // DI1 on GTIOC2B
FspTimer gpt3; // unused on DTM2

// Hardware PWM output timers (one active channel per GPT on DTM2)
FspTimer gpt4_out;  // OUT3 (GTIOC4A)
FspTimer gpt5_out;  // OUT2 (GTIOC5B)
FspTimer gpt6_out;  // OUT1 (GTIOC6B)
FspTimer gpt7_out;  // OUT4 (GTIOC7A)
uint32_t gpt4PeriodCounts = 0;
uint32_t gpt5PeriodCounts = 0;
uint32_t gpt6PeriodCounts = 0;
uint32_t gpt7PeriodCounts = 0;
// Last applied output frequency per hardware output.
// Initialised to 0 so any non-zero loaded frequency triggers an update on first applyOutputs()
uint16_t lastAppliedOutputFreqHz[4] = {0, 0, 0, 0};

struct CaptureCtx {
  uint8_t idxA;
  uint8_t idxB;
  uint32_t maxCounts;
  uint32_t timerFreqHz;
  volatile uint32_t overflowCount;  // incremented by TIMER_EVENT_CYCLE_END in captureCallback
  int16_t overflowIrq;              // NVIC IRQ number of the cycle-end ISR, or -1 if none
};

CaptureCtx gpt0Ctx;
CaptureCtx gpt1Ctx;
CaptureCtx gpt2Ctx;
CaptureCtx gpt3Ctx;

static uint32_t diffCounts(uint32_t now, uint32_t prev, uint32_t maxCounts) {
  if (now >= prev) {
    return now - prev;
  }
  return (maxCounts - prev) + now + 1;
}

static uint32_t diActiveCounts(uint8_t index, uint32_t periodCounts, uint32_t highCounts) {
  if (periodCounts == 0U) {
    return 0U;
  }
  if (highCounts > periodCounts) {
    highCounts = periodCounts;
  }
  bool activeLow = ((DI_ACTIVE_LOW_MASK >> index) & 0x01U) != 0U;
  return activeLow ? (periodCounts - highCounts) : highCounts;
}

static bool readDigitalIn(uint8_t index) {
  if (index >= NUM_DIGITAL_IN) {
    return false;
  }
  uint8_t port = DIGITAL_IN_PINS[index] >> 8;
  uint8_t pin = DIGITAL_IN_PINS[index] & 0xFF;
  return R_PFS->PORT[port].PIN[pin].PmnPFS_b.PIDR ? true : false;
}

uint8_t getDiDebounceMs() {
  return config.reserved[0];
}

void setDiDebounceMs(uint8_t ms) {
  config.reserved[0] = ms;
}

static uint8_t getAuxSwitchMask() {
  return static_cast<uint8_t>(config.reserved[1] & AUX_SWITCH_MASK_ALL);
}

static void setAuxSwitchMask(uint8_t mask) {
  config.reserved[1] = static_cast<uint8_t>((config.reserved[1] & ~AUX_SWITCH_MASK_ALL) | (mask & AUX_SWITCH_MASK_ALL));
}

static bool getAnalogPullupEnabled(uint8_t index) {
  if (index >= NUM_ANALOG_PULLUP_SWITCHES) {
    return false;
  }
  return ((getAuxSwitchMask() >> index) & 0x01U) != 0U;
}

static bool getCanTerminationEnabled() {
  return (getAuxSwitchMask() & AUX_SWITCH_CAN_TERM) != 0U;
}

static void updateDigitalInDebounce(uint32_t nowMs) {
  uint8_t debounceMs = getDiDebounceMs();
  for (int i = 0; i < NUM_DIGITAL_IN; i++) {
    bool raw = readDigitalIn(i);
    if (debounceMs == 0U) {
      diRawPrevState[i] = raw;
      diDebouncedState[i] = raw;
      diRawStableMs[i] = nowMs;
      continue;
    }
    if (raw != diRawPrevState[i]) {
      diRawPrevState[i] = raw;
      diRawStableMs[i] = nowMs;
    } else if (raw != diDebouncedState[i] && (uint32_t)(nowMs - diRawStableMs[i]) >= debounceMs) {
      diDebouncedState[i] = raw;
    }
  }
}

static bool readDigitalOut(uint8_t index) {
  if (index >= NUM_DIGITAL_OUT) {
    return false;
  }
  uint8_t port = DIGITAL_OUT_PINS[index] >> 8;
  uint8_t pin = DIGITAL_OUT_PINS[index] & 0xFF;
  // Read actual pin level via PCNTR2.PIDR (not PFS PODR which may be stale)
  R_PORT0_Type *portReg = (R_PORT0_Type *)((uintptr_t)R_PORT0 + port * 0x20u);
  return (portReg->PCNTR2 & (1u << pin)) ? true : false;
}

static void configureGpioInput(bsp_io_port_pin_t pin) {
  uint8_t port = pin >> 8;
  uint8_t bit = pin & 0xFF;
  R_BSP_PinAccessEnable();
  R_PFS->PORT[port].PIN[bit].PmnPFS_b.PMR = 0;
  R_PFS->PORT[port].PIN[bit].PmnPFS_b.PSEL = 0;
  R_PFS->PORT[port].PIN[bit].PmnPFS_b.PDR = 0;
  R_BSP_PinAccessDisable();
}

static void configureGpioOutput(bsp_io_port_pin_t pin, bool level) {
  uint8_t port = pin >> 8;
  uint8_t bit = pin & 0xFF;
  R_BSP_PinAccessEnable();
  R_PFS->PORT[port].PIN[bit].PmnPFS_b.PMR = 0;
  R_PFS->PORT[port].PIN[bit].PmnPFS_b.PSEL = 0;
  R_PFS->PORT[port].PIN[bit].PmnPFS_b.PDR = 1;
  R_PFS->PORT[port].PIN[bit].PmnPFS_b.PODR = level ? 1 : 0;
  R_BSP_PinAccessDisable();
}

static void writeGpioOutput(bsp_io_port_pin_t pin, bool level) {
  uint8_t port = pin >> 8;
  uint8_t bit = pin & 0xFF;
  R_BSP_PinAccessEnable();
  R_PFS->PORT[port].PIN[bit].PmnPFS_b.PODR = level ? 1 : 0;
  R_BSP_PinAccessDisable();
}

static void configureGptPeripheral(bsp_io_port_pin_t pin) {
  pinPeripheral(pin, (uint32_t)(IOPORT_CFG_PERIPHERAL_PIN | IOPORT_PERIPHERAL_GPT1));
}

static void configureGptPeripheralForChannel(bsp_io_port_pin_t pin, uint8_t channel) {
  (void)channel;
  R_IOPORT_PinCfg(&g_ioport_ctrl, pin,
                  (uint32_t)(IOPORT_CFG_PERIPHERAL_PIN | IOPORT_CFG_PIM_TTL | IOPORT_PERIPHERAL_GPT1));
}

void applyInputPullups() {
  for (int i = 0; i < NUM_DIGITAL_IN; i++) {
    uint32_t cfg = (uint32_t)(IOPORT_CFG_PERIPHERAL_PIN | IOPORT_CFG_PIM_TTL | IOPORT_PERIPHERAL_GPT1);
    if (((config.inputPullupMask >> i) & 0x01U) != 0U) {
      cfg |= IOPORT_CFG_PULLUP_ENABLE;
    }
    R_IOPORT_PinCfg(&g_ioport_ctrl, DIGITAL_IN_PINS[i], cfg);
  }
}

static void initAuxSwitchOutputs() {
  for (int i = 0; i < NUM_ANALOG_PULLUP_SWITCHES; i++) {
    configureGpioOutput(ANALOG_PULLUP_SWITCH_PINS[i], false);
  }
  configureGpioOutput(CAN_TERM_SWITCH_PIN, false);
}

static void applyAuxSwitchStates() {
  for (int i = 0; i < NUM_ANALOG_PULLUP_SWITCHES; i++) {
    writeGpioOutput(ANALOG_PULLUP_SWITCH_PINS[i], getAnalogPullupEnabled(static_cast<uint8_t>(i)));
  }
  writeGpioOutput(CAN_TERM_SWITCH_PIN, getCanTerminationEnabled());
}

static bool adsWriteRegister(uint8_t reg, uint16_t value) {
  Wire.beginTransmission(OUTPUT_SENSE_I2C_ADDRESS);
  Wire.write(reg);
  Wire.write(static_cast<uint8_t>((value >> 8) & 0xFFU));
  Wire.write(static_cast<uint8_t>(value & 0xFFU));
  return Wire.endTransmission() == 0;
}

static bool adsReadRegister(uint8_t reg, uint16_t &value) {
  Wire.beginTransmission(OUTPUT_SENSE_I2C_ADDRESS);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(static_cast<int>(OUTPUT_SENSE_I2C_ADDRESS), 2) != 2) {
    return false;
  }
  value = static_cast<uint16_t>((Wire.read() << 8) | Wire.read());
  return true;
}

static void initOutputSenseAdc() {
  Wire.begin();
  Wire.setClock(400000);

  uint16_t configReg = 0;
  outputSenseReady = adsReadRegister(0x01, configReg);
}

static bool readOutputSenseRaw(uint8_t channel, int16_t &raw) {
  if (!outputSenseReady || channel >= OUTPUT_SENSE_COUNT) {
    raw = 0;
    return false;
  }

  uint16_t configWord = static_cast<uint16_t>(0x8000U |
                                              ((0x4U + channel) << 12) |
                                              0x0100U |
                                              0x00E0U |
                                              0x0003U);
  if (!adsWriteRegister(0x01, configWord)) {
    outputSenseReady = false;
    raw = 0;
    return false;
  }

  for (int attempt = 0; attempt < 8; attempt++) {
    delay(2);
    uint16_t statusWord = 0;
    if (!adsReadRegister(0x01, statusWord)) {
      outputSenseReady = false;
      raw = 0;
      return false;
    }
    if ((statusWord & 0x8000U) != 0U) {
      uint16_t conversion = 0;
      if (!adsReadRegister(0x00, conversion)) {
        outputSenseReady = false;
        raw = 0;
        return false;
      }
      raw = static_cast<int16_t>(conversion);
      return true;
    }
  }

  raw = 0;
  return false;
}

static bool readOutputSenseVoltages(float outVoltages[OUTPUT_SENSE_COUNT], int16_t outRaw[OUTPUT_SENSE_COUNT]) {
  bool allOk = outputSenseReady;
  for (int i = 0; i < OUTPUT_SENSE_COUNT; i++) {
    int16_t raw = 0;
    bool ok = readOutputSenseRaw(OUTPUT_SENSE_CHANNELS[i], raw);
    outRaw[i] = raw;
    if (ok) {
      float adcVolts = (static_cast<float>(raw) * OUTPUT_SENSE_FULL_SCALE_VOLTS) / 32767.0f;
      outVoltages[i] = adcVolts * OUTPUT_SENSE_DIVIDER_RATIO;
    } else {
      outVoltages[i] = 0.0f;
      allOk = false;
    }
  }
  return allOk;
}

static void printOutputSenseVoltages() {
  Serial.println("--- Output Voltage Sense (ADS1115) ---");
  if (!outputSenseReady) {
    Serial.println("ADS1115 not detected");
    return;
  }

  float voltages[OUTPUT_SENSE_COUNT];
  int16_t raw[OUTPUT_SENSE_COUNT];
  bool ok = readOutputSenseVoltages(voltages, raw);
  for (int i = 0; i < OUTPUT_SENSE_COUNT; i++) {
    Serial.print("OUT");
    Serial.print(i + 1);
    Serial.print(": ADC=");
    Serial.print(raw[i]);
    Serial.print(" | V=");
    Serial.print(voltages[i], 3);
    Serial.println(" V");
  }
  if (!ok) {
    Serial.println("WARN: One or more ADS1115 conversions failed");
  }
}

static void initAdc() {
  for (int i = 0; i < NUM_ANALOG; i++) {
    pinPeripheral(ANALOG_PINS[i], (uint32_t)IOPORT_CFG_ANALOG_ENABLE);
  }

  memset(&adc_ctrl, 0, sizeof(adc_ctrl));
  memset(&adc_cfg, 0, sizeof(adc_cfg));
  memset(&adc_channel_cfg, 0, sizeof(adc_channel_cfg));
  memset(&adc_ext_cfg, 0, sizeof(adc_ext_cfg));

  adc_cfg.unit = 0;
  adc_cfg.mode = ADC_MODE_SINGLE_SCAN;
  adc_cfg.resolution = ADC_RESOLUTION_14_BIT;
  adc_cfg.alignment = ADC_ALIGNMENT_RIGHT;
  adc_cfg.trigger = ADC_TRIGGER_SOFTWARE;
  adc_cfg.scan_end_irq = FSP_INVALID_VECTOR;
  adc_cfg.scan_end_b_irq = FSP_INVALID_VECTOR;
  adc_cfg.scan_end_ipl = BSP_IRQ_DISABLED;
  adc_cfg.scan_end_b_ipl = BSP_IRQ_DISABLED;
  adc_cfg.p_callback = nullptr;
  adc_cfg.p_context = nullptr;
  adc_cfg.p_extend = &adc_ext_cfg;

  adc_ext_cfg.add_average_count = ADC_ADD_OFF;
  adc_ext_cfg.clearing = ADC_CLEAR_AFTER_READ_OFF;
  adc_ext_cfg.trigger_group_b = ADC_TRIGGER_SOFTWARE;
  adc_ext_cfg.double_trigger_mode = ADC_DOUBLE_TRIGGER_DISABLED;
  adc_ext_cfg.adc_vref_control = ADC_VREF_CONTROL_AVCC0_AVSS0;
  adc_ext_cfg.enable_adbuf = 0;
  adc_ext_cfg.window_a_irq = FSP_INVALID_VECTOR;
  adc_ext_cfg.window_b_irq = FSP_INVALID_VECTOR;
  adc_ext_cfg.window_a_ipl = BSP_IRQ_DISABLED;
  adc_ext_cfg.window_b_ipl = BSP_IRQ_DISABLED;

  uint32_t mask = 0;
  for (int i = 0; i < NUM_ANALOG; i++) {
    mask |= (1UL << ANALOG_CHANNELS[i]);
  }
  adc_channel_cfg.scan_mask = mask;
  adc_channel_cfg.scan_mask_group_b = 0;
  adc_channel_cfg.add_mask = 0;
  adc_channel_cfg.p_window_cfg = nullptr;
  adc_channel_cfg.priority_group_a = ADC_GROUP_A_PRIORITY_OFF;
  adc_channel_cfg.sample_hold_mask = 0;
  adc_channel_cfg.sample_hold_states = ADC_SAMPLE_STATE_HOLD_COUNT_DEFAULT;

  if (R_ADC_Open(&adc_ctrl, &adc_cfg) != FSP_SUCCESS) {
    adc_ready = false;
    return;
  }
  if (R_ADC_ScanCfg(&adc_ctrl, &adc_channel_cfg) != FSP_SUCCESS) {
    adc_ready = false;
    return;
  }
  adc_ready = true;
}

static void adcService() {
  if (!adc_ready) {
    return;
  }

  if (adcScanRunning) {
    adc_status_t status;
    R_ADC_StatusGet(&adc_ctrl, &status);
    if (status.state == ADC_STATE_SCAN_IN_PROGRESS) {
      return;
    }

    uint8_t writeBuf = adcActiveBuf ^ 1U;
    for (int i = 0; i < NUM_ANALOG; i++) {
      R_ADC_Read(&adc_ctrl, static_cast<adc_channel_t>(ANALOG_CHANNELS[i]), &adcBuf[writeBuf][i]);
    }
    adcActiveBuf = writeBuf;
    adcScanRunning = false;
  }

  if (R_ADC_ScanStart(&adc_ctrl) != FSP_SUCCESS) {
    if (adcScanFailCount != 0xFFFFFFFFUL) {
      adcScanFailCount++;
    }
  } else {
    adcScanRunning = true;
  }
}

static bool readAnalogRawAll(uint16_t outValues[NUM_ANALOG]) {
  if (!adc_ready) {
    for (int i = 0; i < NUM_ANALOG; i++) {
      outValues[i] = 0;
    }
    return false;
  }

  memcpy(outValues, adcBuf[adcActiveBuf], NUM_ANALOG * sizeof(uint16_t));
  return true;
}

static uint16_t computeCrc(const Config &cfg) {
  const uint8_t *ptr = reinterpret_cast<const uint8_t*>(&cfg);
  uint16_t crc = 0;
  for (size_t i = 0; i < sizeof(Config) - sizeof(cfg.crc); i++) {
    crc = static_cast<uint16_t>(crc + ptr[i]);
  }
  return crc;
}

void setDefaults(Config &cfg) {
  cfg.magic = CONFIG_MAGIC;
  cfg.version = CONFIG_VERSION;
  cfg.canSpeedKbps = DEFAULT_CAN_SPEED_KBPS;
  cfg.txBaseId = DEFAULT_TX_BASE_ID;
  cfg.rxBaseId = DEFAULT_RX_BASE_ID;
  cfg.txRateHz = DEFAULT_TX_RATE_HZ;
  cfg.rxTimeoutMs = DEFAULT_RX_TIMEOUT_MS;
  cfg.outFreqHz[0] = DEFAULT_PWM_FREQ_HZ;
  cfg.outFreqHz[1] = DEFAULT_PWM_FREQ_HZ;
  cfg.outFreqHz[2] = DEFAULT_PWM_FREQ_HZ;
  cfg.outFreqHz[3] = DEFAULT_PWM_FREQ_HZ;
  cfg.safeMask = 0x00;   // all outputs safe OFF
  cfg.activeMask = 0x0F; // DTM2 outputs are active HIGH by default
  cfg.canMode = 0;
  cfg.inputPullupMask = 0x00; // all DI pull-ups OFF by default
  memset(cfg.reserved, 0, sizeof(cfg.reserved));
  cfg.reserved[0] = DEFAULT_DI_DEBOUNCE_MS;
  cfg.crc = computeCrc(cfg);
}

void saveConfig() {
  config.crc = computeCrc(config);
  EEPROM.put(0, config);
  Serial.println("[EEPROM] config written");
}

static void loadConfig() {
  EEPROM.get(0, config);
  bool migrated = false;
  if (config.magic == CONFIG_MAGIC && config.version == 2) {
    config.version = CONFIG_VERSION;
    config.activeMask = 0x0F;
    config.safeMask &= 0x0F;
    config.inputPullupMask &= 0x0F;
    setAuxSwitchMask(getAuxSwitchMask() & AUX_SWITCH_MASK_ALL);
    migrated = true;
  }

  bool valid = (config.magic == CONFIG_MAGIC) && (config.version == CONFIG_VERSION);
  if (valid) {
    uint16_t crc = computeCrc(config);
    valid = (crc == config.crc);
  }
  if (!valid) {
    setDefaults(config);
    saveConfig();
  }
  // Validate frequency values â€” corrupted EEPROM can yield garbage
  for (int p = 0; p < 4; p++) {
    if (config.outFreqHz[p] < 50 || config.outFreqHz[p] > 10000) {
      config.outFreqHz[p] = DEFAULT_PWM_FREQ_HZ;
    }
  }
  setAuxSwitchMask(getAuxSwitchMask());
  if (getDiDebounceMs() > 100U) {
    setDiDebounceMs(DEFAULT_DI_DEBOUNCE_MS);
  }
  if (migrated) {
    saveConfig();
  }
}

static uint32_t modeColor(uint8_t mode) {
  switch (mode) {
    case 0: return makeNeoColor(0, 150, 0);      // Green
    case 1: return makeNeoColor(150, 150, 0);    // Yellow
    case 2: return makeNeoColor(150, 150, 0);    // Yellow
    case 3: return makeNeoColor(150, 150, 0);    // Yellow
    case 4: return makeNeoColor(80, 120, 150);   // BluishYellow
    case 5: return makeNeoColor(80, 120, 150);   // BluishYellow
    case 6: return makeNeoColor(150, 80, 0);     // Amber
    case 7: return makeNeoColor(150, 0, 0);      // Red
    case 8: return makeNeoColor(0, 0, 150);      // Blue
    default: return makeNeoColor(150, 0, 150);   // Violet
  }
}

static void updateStatusLed(uint32_t nowMs) {
  bool canSilent = (lastCanRxMs == 0) ? (nowMs > config.rxTimeoutMs) : (nowMs - lastCanRxMs > config.rxTimeoutMs);
  if (canSilent) {
    bool on = ((nowMs / 250) % 2) == 0;
    neopixelShowPacked(on ? makeNeoColor(0, 0, 150) : makeNeoColor(0, 0, 0));
  } else {
    neopixelShowPacked(modeColor(config.canMode));
  }
}

static void captureCallback(timer_callback_args_t *p_args) {
  CaptureCtx *ctx = static_cast<CaptureCtx *>(const_cast<void *>(p_args->p_context));
  if (!ctx) return;

  // Count timer overflows so that periods spanning multiple wrap-arounds are
  // measured correctly.  GPT2/GPT3 are 16-bit timers: at 24 MHz they overflow
  // every ~2.73 ms, which is shorter than one period of a 300 Hz signal.
  if (p_args->event == TIMER_EVENT_CYCLE_END) {
    ctx->overflowCount++;
    return;
  }

  uint8_t idx = 0xFF;
  if (p_args->event == TIMER_EVENT_CAPTURE_A) {
    idx = ctx->idxA;
  } else if (p_args->event == TIMER_EVENT_CAPTURE_B) {
    idx = ctx->idxB;
  }
  if (idx >= NUM_DIGITAL_IN) return;

  // Determine the overflow count that was current when this capture occurred.
  // Problem: on Cortex-M, same-priority IRQs cannot preempt each other. If the
  // overflow (CYCLE_END) IRQ fires at the same moment as this capture IRQ, the
  // overflow ISR is queued but hasn't run yet — so ctx->overflowCount is still
  // the pre-overflow value even though the hardware counter has already wrapped.
  // We detect this via NVIC_GetPendingIRQ: if the overflow IRQ is pending AND
  // the captured count is already small (< half of max, i.e. counter wrapped),
  // the overflow happened before this capture. Add 1 to compensate.
  uint32_t nowOv = ctx->overflowCount;
  uint32_t captured = p_args->capture;
  if (ctx->overflowIrq >= 0 &&
      NVIC_GetPendingIRQ(static_cast<IRQn_Type>(ctx->overflowIrq)) &&
      captured < (ctx->maxCounts >> 1U)) {
    nowOv++;
  }

  bool level = readDigitalIn(idx);

  diCapSeq[idx]++;

  if (level) {
    // Rising edge: compute period from previous rising edge using overflow count
    if (diHasFirstRise[idx]) {
      uint32_t prevOv  = diLastRiseOverflow[idx];
      uint32_t prevCap = diLastRise[idx];
      uint32_t ovSpan  = nowOv - prevOv;  // wraps correctly for uint32_t

      uint32_t period;
      if (ovSpan == 0) {
        period = (captured >= prevCap) ? (captured - prevCap)
                                       : diffCounts(captured, prevCap, ctx->maxCounts);
      } else {
        // Period spans ovSpan full overflow cycles (ovSpan >= 1 here) plus
        // the fractional parts before/after the first/last overflow boundary:
        //   (maxCounts - prevCap + 1)  counts to the end of the first overflow cycle
        //   (ovSpan - 1) * (maxCounts + 1)  counts for any complete middle cycles
        //   captured                    counts into the current (final) cycle
        uint64_t total = (uint64_t)(ctx->maxCounts - prevCap + 1U)
                       + (uint64_t)(ovSpan - 1U) * ((uint64_t)ctx->maxCounts + 1U)
                       + (uint64_t)captured;
        period = (total > 0xFFFFFFFFULL) ? 0xFFFFFFFFUL : (uint32_t)total;
      }

      diPeriodCounts[idx] = period;
      diHasPeriod[idx]    = (period > 0);
    }
    diHasFirstRise[idx]     = true;
    diLastRise[idx]         = captured;
    diLastRiseOverflow[idx] = nowOv;
  } else {
    if (diHasFirstRise[idx]) {
      uint32_t prevOv  = diLastRiseOverflow[idx];
      uint32_t prevCap = diLastRise[idx];
      uint32_t ovSpan  = nowOv - prevOv;  // wraps correctly for uint32_t

      uint32_t highCounts;
      if (ovSpan == 0) {
        highCounts = (captured >= prevCap) ? (captured - prevCap)
                                           : diffCounts(captured, prevCap, ctx->maxCounts);
      } else {
        uint64_t total = (uint64_t)(ctx->maxCounts - prevCap + 1U)
                       + (uint64_t)(ovSpan - 1U) * ((uint64_t)ctx->maxCounts + 1U)
                       + (uint64_t)captured;
        highCounts = (total > 0xFFFFFFFFULL) ? 0xFFFFFFFFUL : (uint32_t)total;
      }

      diHighCounts[idx] = highCounts;
      diHasHigh[idx] = (highCounts > 0U);
    }
  }
}

static void initCaptureTimer(FspTimer &timer, CaptureCtx &ctx, uint8_t gptChannel, uint8_t idxA, uint8_t idxB) {
  // Pre-initialize context so callback has valid data as soon as timer starts
  ctx.idxA = idxA;
  ctx.idxB = idxB;
  ctx.maxCounts = 0xFFFFFFFF;
  ctx.timerFreqHz = 0;
  ctx.overflowCount = 0;
  ctx.overflowIrq = -1;

  // Arduino variant marks GPT0-3 as TIMER_PWM for default PWM pins.
  // force_use_of_pwm_reserved_timer() allows begin() to claim these channels
  // for input capture instead. The flag resets after each begin() call.
  FspTimer::force_use_of_pwm_reserved_timer();
  timer.begin(TIMER_MODE_PERIODIC, GPT_TIMER, gptChannel, 0xFFFFFFFF, 1, TIMER_SOURCE_DIV_1, captureCallback, &ctx);
  timer.set_source_capture_a((gpt_source_t)(
    GPT_SOURCE_GTIOCA_RISING_WHILE_GTIOCB_LOW |
    GPT_SOURCE_GTIOCA_RISING_WHILE_GTIOCB_HIGH |
    GPT_SOURCE_GTIOCA_FALLING_WHILE_GTIOCB_LOW |
    GPT_SOURCE_GTIOCA_FALLING_WHILE_GTIOCB_HIGH));
  timer.set_source_capture_b((gpt_source_t)(
    GPT_SOURCE_GTIOCB_RISING_WHILE_GTIOCA_LOW |
    GPT_SOURCE_GTIOCB_RISING_WHILE_GTIOCA_HIGH |
    GPT_SOURCE_GTIOCB_FALLING_WHILE_GTIOCA_LOW |
    GPT_SOURCE_GTIOCB_FALLING_WHILE_GTIOCA_HIGH));
  if (timer.get_cfg() != nullptr && timer.get_cfg()->p_extend != nullptr) {
    gpt_extended_cfg_t *ext = static_cast<gpt_extended_cfg_t *>(const_cast<void *>(timer.get_cfg()->p_extend));
    ext->capture_filter_gtioca = GPT_CAPTURE_FILTER_PCLKD_DIV_64;
    ext->capture_filter_gtiocb = GPT_CAPTURE_FILTER_PCLKD_DIV_64;
  }
  bool irqA = timer.setup_capture_a_irq(12, nullptr);
  bool irqB = timer.setup_capture_b_irq(12, nullptr);
  // GPT2/GPT3 are 16-bit and need cycle-end IRQs to count overflows for
  // low-frequency capture periods.
  bool irqOvf = false;
  if (gptChannel == 2U || gptChannel == 3U) {
    irqOvf = timer.setup_overflow_irq(12, nullptr);
  }
  (void)irqA;
  (void)irqB;
  (void)irqOvf;
  timer.open();
  timer.start();

  // Update context with actual timer parameters now that it's running
  ctx.maxCounts = timer.get_period_raw();
  ctx.timerFreqHz = timer.get_freq_hz();

  // Store the cycle-end IRQ number so captureCallback can check NVIC_GetPendingIRQ
  // to detect an overflow that fired simultaneously with a capture but hasn't been
  // serviced yet (same-priority IRQs can't preempt each other on Cortex-M).
  if ((gptChannel == 2U || gptChannel == 3U) &&
      irqOvf &&
      timer.get_cfg() != nullptr &&
      timer.get_cfg()->cycle_end_irq >= 0) {
    ctx.overflowIrq = static_cast<int16_t>(timer.get_cfg()->cycle_end_irq);
  }

  diTimerFreq[idxA] = ctx.timerFreqHz;
  diTimerFreq[idxB] = ctx.timerFreqHz;
}

bool setCanBitrate(uint16_t kbps) {
  CanBitRate rate;
  switch (kbps) {
    case 125: rate = CanBitRate::BR_125k; break;
    case 250: rate = CanBitRate::BR_250k; break;
    case 500: rate = CanBitRate::BR_500k; break;
    case 1000: rate = CanBitRate::BR_1000k; break;
    default: return false;
  }
  CAN.end();
  canInitOk = CAN.begin(rate);
  return canInitOk;
}

// swPwmIsr removed - using hardware PWM instead

// Re-initialise a single output GPT timer at a new frequency, then invalidate
// the cached duty-cycle counts so that applyOutputs() re-applies the duty.
static void reinitOutputTimer(FspTimer &timer,
                              uint8_t gptChannel,
                              TimerPWMChannel_t pwmChannel,
                              uint16_t hz,
                              uint32_t &periodCounts,
                              uint32_t &cachedCounts) {
  timer.end();
  FspTimer::force_use_of_pwm_reserved_timer();
  timer.begin(TIMER_MODE_PWM, GPT_TIMER, gptChannel, hz, 50.0f);
  timer.add_pwm_extended_cfg();
  timer.enable_pwm_channel(pwmChannel);
  timer.open();
  timer.set_duty_cycle(0, pwmChannel);
  timer.start();
  periodCounts  = timer.get_period_raw();
  cachedCounts = 0xFFFFFFFFUL;
}

// Apply output values using hardware PWM (one channel per GPT on DTM2)
void applyOutputs(bool useSafeState) {
  uint8_t newDuty[NUM_DIGITAL_OUT];

  for (int i = 0; i < NUM_DIGITAL_OUT; i++) {
    uint8_t duty = outputDuty[i];
    // Apply safe state only to channels NOT under serial/app override
    bool isOverridden = (serialOverrideMask >> i) & 0x01;
    if (useSafeState && !isOverridden) {
      bool safeOn = (config.safeMask >> i) & 0x01;
      duty = safeOn ? 255 : 0;
    }
    bool activeHigh = (config.activeMask >> i) & 0x01;
    if (!activeHigh) duty = 255 - duty;
    bool stageInverted = ((OUTPUT_STAGE_INVERT_MASK >> i) & 0x01U) != 0U;
    if (stageInverted) duty = 255 - duty;
    newDuty[i] = duty;
  }

  if (gpt6_out.is_opened() && outputFreq[0] != lastAppliedOutputFreqHz[0]) {
    if (rxDebug) {
      Serial.print("[FREQ OUT1 "); Serial.print(lastAppliedOutputFreqHz[0]);
      Serial.print("->"); Serial.print(outputFreq[0]); Serial.println("Hz]");
    }
    reinitOutputTimer(gpt6_out, 6, CHANNEL_B, outputFreq[0], gpt6PeriodCounts, lastAppliedPwmCounts[0]);
    lastAppliedOutputFreqHz[0] = outputFreq[0];
  }
  if (gpt5_out.is_opened() && outputFreq[1] != lastAppliedOutputFreqHz[1]) {
    if (rxDebug) {
      Serial.print("[FREQ OUT2 "); Serial.print(lastAppliedOutputFreqHz[1]);
      Serial.print("->"); Serial.print(outputFreq[1]); Serial.println("Hz]");
    }
    reinitOutputTimer(gpt5_out, 5, CHANNEL_B, outputFreq[1], gpt5PeriodCounts, lastAppliedPwmCounts[1]);
    lastAppliedOutputFreqHz[1] = outputFreq[1];
  }
  if (gpt4_out.is_opened() && outputFreq[2] != lastAppliedOutputFreqHz[2]) {
    if (rxDebug) {
      Serial.print("[FREQ OUT3 "); Serial.print(lastAppliedOutputFreqHz[2]);
      Serial.print("->"); Serial.print(outputFreq[2]); Serial.println("Hz]");
    }
    reinitOutputTimer(gpt4_out, 4, CHANNEL_A, outputFreq[2], gpt4PeriodCounts, lastAppliedPwmCounts[2]);
    lastAppliedOutputFreqHz[2] = outputFreq[2];
  }
  if (gpt7_out.is_opened() && outputFreq[3] != lastAppliedOutputFreqHz[3]) {
    if (rxDebug) {
      Serial.print("[FREQ OUT4 "); Serial.print(lastAppliedOutputFreqHz[3]);
      Serial.print("->"); Serial.print(outputFreq[3]); Serial.println("Hz]");
    }
    reinitOutputTimer(gpt7_out, 7, CHANNEL_A, outputFreq[3], gpt7PeriodCounts, lastAppliedPwmCounts[3]);
    lastAppliedOutputFreqHz[3] = outputFreq[3];
  }

  // Update hardware PWM using set_duty_cycle.
  // On this RA4M1 GPT setup, the compare value maps to low-time, so convert
  // logical duty to compare counts with (255 - duty).
  // Cap max to period-1 to ensure compare logic works properly.
  
  // OUT1: GPT6B
  if (gpt6_out.is_opened()) {
    uint32_t period = gpt6PeriodCounts;
    if (period < 2U) {
      period = gpt6_out.get_period_raw();
      gpt6PeriodCounts = period;
    }
    if (period < 2U) {
      period = 2U;
    }
    uint32_t counts = (period * (uint32_t)(255U - newDuty[0])) / 255U;
    if (counts >= period) counts = period - 1;
    if (lastAppliedPwmCounts[0] != counts) {
      if (rxDebug) { Serial.print("[PWM OUT1 "); Serial.print(lastAppliedPwmCounts[0]); Serial.print("->"); Serial.print(counts); Serial.println("]"); }
      gpt6_out.set_duty_cycle(counts, CHANNEL_B);
      lastAppliedPwmCounts[0] = counts;
    }
  }

  // OUT2: GPT5B
  if (gpt5_out.is_opened()) {
    uint32_t period = gpt5PeriodCounts;
    if (period < 2U) {
      period = gpt5_out.get_period_raw();
      gpt5PeriodCounts = period;
    }
    if (period < 2U) {
      period = 2U;
    }
    uint32_t counts = (period * (uint32_t)(255U - newDuty[1])) / 255U;
    if (counts >= period) counts = period - 1;
    if (lastAppliedPwmCounts[1] != counts) {
      if (rxDebug) { Serial.print("[PWM OUT2 "); Serial.print(lastAppliedPwmCounts[1]); Serial.print("->"); Serial.print(counts); Serial.println("]"); }
      gpt5_out.set_duty_cycle(counts, CHANNEL_B);
      lastAppliedPwmCounts[1] = counts;
    }
  }

  // OUT3: GPT4A
  if (gpt4_out.is_opened()) {
    uint32_t period = gpt4PeriodCounts;
    if (period < 2U) {
      period = gpt4_out.get_period_raw();
      gpt4PeriodCounts = period;
    }
    if (period < 2U) {
      period = 2U;
    }
    uint32_t counts = (period * (uint32_t)(255U - newDuty[2])) / 255U;
    if (counts >= period) counts = period - 1;
    if (lastAppliedPwmCounts[2] != counts) {
      if (rxDebug) { Serial.print("[PWM OUT3 "); Serial.print(lastAppliedPwmCounts[2]); Serial.print("->"); Serial.print(counts); Serial.println("]"); }
      gpt4_out.set_duty_cycle(counts, CHANNEL_A);
      lastAppliedPwmCounts[2] = counts;
    }
  }

  // OUT4: GPT7A
  if (gpt7_out.is_opened()) {
    uint32_t period = gpt7PeriodCounts;
    if (period < 2U) {
      period = gpt7_out.get_period_raw();
      gpt7PeriodCounts = period;
    }
    if (period < 2U) {
      period = 2U;
    }
    uint32_t counts = (period * (uint32_t)(255U - newDuty[3])) / 255U;
    if (counts >= period) counts = period - 1;
    if (lastAppliedPwmCounts[3] != counts) {
      if (rxDebug) { Serial.print("[PWM OUT4 "); Serial.print(lastAppliedPwmCounts[3]); Serial.print("->"); Serial.print(counts); Serial.println("]"); }
      gpt7_out.set_duty_cycle(counts, CHANNEL_A);
      lastAppliedPwmCounts[3] = counts;
    }
  }
}

// Pending config save flag â€” deferred so EEPROM writes never happen inside
// the CAN drain loop (flash writes can block for ms and stall the SW PWM ISR).
static bool pendingConfigSave = false;

static void handleCanRx(const CanMsg &msg) {
  // Snapshot duty/freq so we can detect whether outputs actually changed
  uint8_t  prevDuty[OUTPUT_SLOT_COUNT];
  uint16_t prevFreq[OUTPUT_SLOT_COUNT];
  memcpy(prevDuty, outputDuty, sizeof(prevDuty));
  memcpy(prevFreq, outputFreq, sizeof(prevFreq));

  bool changed = false;
  bool pullupChanged = false;
  bool handled = canModeHandleRx(config.canMode,
                                 msg,
                                 config.rxBaseId,
                                 DEFAULT_PWM_FREQ_HZ,
                                 outputFreq,
                                 outputDuty,
                                 config.safeMask,
                                 config.activeMask,
                                 changed,
                                 config.inputPullupMask,
                                 pullupChanged);

  if (!handled) {
    return;
  }

  // Restore any serial-overridden channels to their pre-CAN values
  // (CAN should not control channels under serial/app override)
  for (int i = 0; i < OUTPUT_SLOT_COUNT; i++) {
    if ((serialOverrideMask >> i) & 0x01) {
      outputDuty[i] = prevDuty[i];
      outputFreq[i] = prevFreq[i];
    }
  }

  // Defer EEPROM writes â€” never call saveConfig() from within the CAN drain
  // loop; schedule it so it runs once after the loop exits.
  if (changed || pullupChanged) {
    pendingConfigSave = true;
  }

  if (pullupChanged) {
    applyInputPullups();
  }

  // Only re-apply PWM when output behavior can change:
  //  - duty/freq changed
  //  - safe/active mask changed
  //  - we were in safe state and need to restore commanded outputs
  bool outputsChanged = (memcmp(prevDuty, outputDuty, sizeof(prevDuty)) != 0) ||
                        (memcmp(prevFreq, outputFreq, sizeof(prevFreq)) != 0);
  if (pullupChanged && !outputsChanged && !changed) {
    return;
  }

  if (outputsChanged || changed || outputsInSafeState) {
    if (rxDebug) {
      Serial.print("[OUT id=0x"); Serial.print(msg.id, HEX);
      Serial.print(" trig=");
      if (outputsInSafeState) Serial.print("safe");
      else if (changed)       Serial.print("mask");
      else                    Serial.print("duty");
      Serial.print(" d=");
      for (int _i = 0; _i < OUTPUT_SLOT_COUNT; _i++) {
        if (_i) Serial.print(',');
        Serial.print(outputDuty[_i]);
      }
      Serial.println("]");
    }
    outputsInSafeState = false;
    applyOutputs(false);
  }
}

static void processCan() {
  // Drain a bounded number of frames per loop iteration to prevent starvation
  // of serial/USB and other foreground work under heavy CAN traffic.
  // Any remaining frames will be picked up on the next loop().
  const uint8_t MAX_RX_PER_LOOP = 12;
  uint8_t count = 0;
  while (CAN.available() && count < MAX_RX_PER_LOOP) {
    CanMsg msg = CAN.read();
    lastCanRxMs = millis();
    if (canRxCount != 0xFFFF) {
      canRxCount++;
    }
    handleCanRx(msg);
    count++;
  }
}

static void sendCanFrame(uint32_t id, const uint8_t *data, uint8_t len) {
  if (len > 0 && data == nullptr) {
    return;
  }
  // Zero-init the whole struct â€” CanMsg has fields beyond id/data_length/data
  // (e.g. rtr, extended flags) that the RA4M1 CAN controller reads.
  // Without this, those fields are garbage on calls 2-N and the controller
  // rejects or mangles the frame.
  CanMsg tx;
  memset(&tx, 0, sizeof(tx));
  tx.id = id;
  tx.data_length = len;
  if (len > 0) {
    memcpy(tx.data, data, len);
  }

  // Bounded retry so burst TX frames are not dropped on a busy mailbox,
  // but without long blocking delays that starve foreground work.
  const uint32_t txTimeoutUs = 1200UL;
  uint32_t startUs = micros();
  bool ok = false;
  do {
    ok = CAN.write(tx);
    if (!ok) {
      delayMicroseconds(20);
    }
  } while (!ok && (micros() - startUs < txTimeoutUs));

  if (ok) {
    delayMicroseconds(150);  // one frame-time at 1Mbps; ensures mailbox is free for next send
    if (canTxCount != 0xFFFF) {
      canTxCount++;
    }
  } else {
    if (rxDebug) {
      Serial.print("[TX FAIL 0x"); Serial.print(tx.id, HEX); Serial.println("]");
    }
    if (canTxFail != 0xFFFF) {
      canTxFail++;
    }
  }
}

static void sendTxFrames() {
  uint16_t analogRaw[NUM_ANALOG];
  readAnalogRawAll(analogRaw);

  uint8_t digitalInMask = 0;
  for (int i = 0; i < NUM_DIGITAL_IN; i++) {
    if (diDebouncedState[i]) {
      digitalInMask |= (1 << i);
    }
  }

  uint8_t analogStateMask = 0;
  for (int i = 0; i < NUM_ANALOG; i++) {
    if (analogRaw[i] > (ADC_MAX / 2)) {
      analogStateMask |= (1 << i);
    }
  }

  uint8_t digitalOutMask = 0;
  for (int i = 0; i < NUM_DIGITAL_OUT; i++) {
    if (readDigitalOut(i)) {
      digitalOutMask |= (1 << i);
    }
  }

  ModeTxFrame frame;
  ModeTxFrame frame1;

  canModeBuildTxAnalogFrames(config.canMode, config.txBaseId, analogRaw, frame, frame1);
  if (frame.len > 0) {
    sendCanFrame(frame.id, frame.data, frame.len);
  }
  if (frame1.len > 0) {
    sendCanFrame(frame1.id, frame1.data, frame1.len);
  }

  canModeBuildTxStateFrame(config.canMode,
                           config.txBaseId,
                           digitalInMask,
                           analogStateMask,
                           digitalOutMask,
                           config.safeMask,
                           config.activeMask,
                           FW_VERSION,
                           frame);
  if (frame.len > 0) {
    sendCanFrame(frame.id, frame.data, frame.len);
  }

  auto packFreqDuty = [&](uint8_t baseIndex, uint32_t baseId) {
    uint32_t period0, high0;
    bool has0;
    uint32_t period1, high1;
    bool has1;

    noInterrupts();
    period0 = diPeriodCounts[baseIndex];
    high0 = diHighCounts[baseIndex];
    has0 = diHasPeriod[baseIndex];
    period1 = diPeriodCounts[baseIndex + 1];
    high1 = diHighCounts[baseIndex + 1];
    has1 = diHasPeriod[baseIndex + 1];
    interrupts();

    uint32_t timerFreq0 = diTimerFreq[baseIndex];
    uint32_t timerFreq1 = diTimerFreq[baseIndex + 1];

    uint32_t dutyCounts0 = diActiveCounts(baseIndex, period0, high0);
    uint32_t dutyCounts1 = diActiveCounts(static_cast<uint8_t>(baseIndex + 1U), period1, high1);

    canModeBuildTxDiPairFrame(config.canMode,
                              baseId,
                              timerFreq0,
                              period0,
                  dutyCounts0,
                              has0,
                              timerFreq1,
                              period1,
                  dutyCounts1,
                              has1,
                              frame);
    if (frame.len > 0) {
      sendCanFrame(frame.id, frame.data, frame.len);
    }
  };

  if (config.canMode == CAN_MODE_PT_DEFAULT1) {
    packFreqDuty(0, config.txBaseId + 3U);
    packFreqDuty(2, config.txBaseId + 4U);

    float outputVoltages[OUTPUT_SENSE_COUNT];
    int16_t outputSenseRaw[OUTPUT_SENSE_COUNT];
    readOutputSenseVoltages(outputVoltages, outputSenseRaw);
    uint16_t outputVoltageMv[OUTPUT_SENSE_COUNT];
    for (uint8_t i = 0; i < OUTPUT_SENSE_COUNT; i++) {
      float millivolts = outputVoltages[i] * 1000.0f;
      if (millivolts <= 0.0f) {
        outputVoltageMv[i] = 0U;
      } else if (millivolts >= 80018.0f) {
        outputVoltageMv[i] = 65535U;
      } else {
        outputVoltageMv[i] = static_cast<uint16_t>((millivolts * 65535.0f / 80018.0f) + 0.5f);
      }
    }
    mode0BuildTxOutputVoltageFrame(config.txBaseId, outputVoltageMv, frame);
    sendCanFrame(frame.id, frame.data, frame.len);
  } else if (config.canMode != CAN_MODE_ECUMASTER_CANSWB_V3) {
    packFreqDuty(0, config.txBaseId + 3);
    packFreqDuty(2, config.txBaseId + 4);
    packFreqDuty(4, config.txBaseId + 5);
    packFreqDuty(6, config.txBaseId + 6);

    Mode0Status modeStatus;
    modeStatus.canInitOk = canInitOk;
    modeStatus.outputsInSafeState = outputsInSafeState;
    modeStatus.canRxCount = canRxCount;
    modeStatus.canTxCount = canTxCount;
    modeStatus.canTxFail = canTxFail;
    modeStatus.canMode = config.canMode;
    modeStatus.lastCanRxMs = lastCanRxMs;
    modeStatus.rxTimeoutMs = config.rxTimeoutMs;
    modeStatus.nowMs = millis();

    canModeBuildTxStatusFrame(config.canMode, config.txBaseId, modeStatus, frame);
    if (frame.len > 0) {
      sendCanFrame(frame.id, frame.data, frame.len);
    }
  }
}

static void printDiag() {
  Serial.println("\n========== DIAG ==========");

  Serial.print("ADC: ready="); Serial.print(adc_ready ? "YES" : "NO");
  Serial.print(" scanRunning="); Serial.print(adcScanRunning ? "YES" : "NO");
  Serial.print(" activeBuf="); Serial.print(adcActiveBuf);
  Serial.print(" scanFail="); Serial.println(adcScanFailCount);

  // GPT capture timer diagnostics
  struct { FspTimer *timer; CaptureCtx *ctx; uint8_t ch; const char *label; } timers[] = {
    {&gpt0, &gpt0Ctx, 0, "GPT0 (DI4-B)"},
    {&gpt1, &gpt1Ctx, 1, "GPT1 (DI3-A, DI2-B)"},
    {&gpt2, &gpt2Ctx, 2, "GPT2 (DI1-B)"},
    {&gpt3, &gpt3Ctx, 3, "GPT3 (unused)"},
  };
  for (auto &t : timers) {
    const timer_cfg_t *tmrCfg = t.timer->get_cfg();
    if (tmrCfg == nullptr || tmrCfg->p_extend == nullptr) {
      Serial.print(t.label);
      Serial.println(": cfg unavailable");
      continue;
    }
    gpt_extended_cfg_t *ext = static_cast<gpt_extended_cfg_t *>(const_cast<void *>(tmrCfg->p_extend));
    uint32_t baseAddr = (uint32_t)R_GPT0 + (t.ch * ((uint32_t)R_GPT1 - (uint32_t)R_GPT0));
    R_GPT0_Type *reg = (R_GPT0_Type *)baseAddr;

    Serial.print(t.label);
    Serial.print(": capA_irq="); Serial.print((int)ext->capture_a_irq);
    Serial.print(" capB_irq="); Serial.print((int)ext->capture_b_irq);
    Serial.print(" GTICASR=0x"); Serial.print(reg->GTICASR, HEX);
    Serial.print(" GTICBSR=0x"); Serial.print(reg->GTICBSR, HEX);
    Serial.print(" GTIOR=0x"); Serial.print(reg->GTIOR, HEX);
    Serial.print(" freq="); Serial.print(t.ctx->timerFreqHz);
    Serial.println();

    // IELSR, NVIC, and GTST details
    int irqA = (int)ext->capture_a_irq;
    int irqB = (int)ext->capture_b_irq;
    Serial.print("  IELSR["); Serial.print(irqA); Serial.print("]=0x");
    Serial.print(irqA >= 0 ? R_ICU->IELSR[irqA] : 0, HEX);
    Serial.print(" IELSR["); Serial.print(irqB); Serial.print("]=0x");
    Serial.print(irqB >= 0 ? R_ICU->IELSR[irqB] : 0, HEX);
    // NVIC enabled?
    Serial.print(" NVIC_EN=");
    Serial.print((irqA >= 0 && NVIC_GetEnableIRQ((IRQn_Type)irqA)) ? "A" : "-");
    Serial.print((irqB >= 0 && NVIC_GetEnableIRQ((IRQn_Type)irqB)) ? "B" : "-");
    // NVIC pending?
    Serial.print(" NVIC_PEND=");
    Serial.print((irqA >= 0 && NVIC_GetPendingIRQ((IRQn_Type)irqA)) ? "A" : "-");
    Serial.print((irqB >= 0 && NVIC_GetPendingIRQ((IRQn_Type)irqB)) ? "B" : "-");
    // GTST (status register): bit 0=TCFA(cap A flag), bit 1=TCFB(cap B flag)
    Serial.print(" GTST=0x"); Serial.print(reg->GTST, HEX);
    // GTCNT
    Serial.print(" GTCNT="); Serial.print(reg->GTCNT);
    // GTCCRA and GTCCRB (last captured values)
    Serial.print(" CCRA="); Serial.print(reg->GTCCR[0]);
    Serial.print(" CCRB="); Serial.print(reg->GTCCR[1]);
    Serial.println();
  }

  // Hardware PWM diagnostics
  Serial.println("--- HW PWM (GPT4/5/6/7) ---");
  Serial.print("GPT4 open="); Serial.print(gpt4_out.is_opened() ? "YES" : "NO");
  Serial.print(" period="); Serial.print(gpt4_out.is_opened() ? gpt4_out.get_period_raw() : 0);
  Serial.print("  GPT5 open="); Serial.print(gpt5_out.is_opened() ? "YES" : "NO");
  Serial.print(" period="); Serial.print(gpt5_out.is_opened() ? gpt5_out.get_period_raw() : 0);
  Serial.print("  GPT6 open="); Serial.print(gpt6_out.is_opened() ? "YES" : "NO");
  Serial.print(" period="); Serial.print(gpt6_out.is_opened() ? gpt6_out.get_period_raw() : 0);
  Serial.print("  GPT7 open="); Serial.print(gpt7_out.is_opened() ? "YES" : "NO");
  Serial.print(" period="); Serial.println(gpt7_out.is_opened() ? gpt7_out.get_period_raw() : 0);
  for (int i = 0; i < NUM_DIGITAL_OUT; i++) {
    uint8_t port = DIGITAL_OUT_PINS[i] >> 8;
    uint8_t pin = DIGITAL_OUT_PINS[i] & 0xFF;
    uint32_t pfs = R_PFS->PORT[port].PIN[pin].PmnPFS;
    Serial.print("OUT"); Serial.print(i + 1);
    Serial.print(" ("); Serial.print(DIGITAL_OUT_PORT_NAMES[i]);
    Serial.print("): PFS=0x"); Serial.print(pfs, HEX);
    Serial.print(" PDR="); Serial.print((pfs >> 2) & 1);
    Serial.print(" PMR="); Serial.print((pfs >> 16) & 1);
    Serial.print(" period=--; high=--");  // Hardware PWM
    Serial.println();
  }

  // Pin PFS registers
  Serial.println("--- DI Pin PFS ---");
  for (int i = 0; i < NUM_DIGITAL_IN; i++) {
    uint8_t port = DIGITAL_IN_PINS[i] >> 8;
    uint8_t pin = DIGITAL_IN_PINS[i] & 0xFF;
    uint32_t pfs = R_PFS->PORT[port].PIN[pin].PmnPFS;
    Serial.print("DI"); Serial.print(i + 1);
    Serial.print(" ("); Serial.print(DIGITAL_IN_PORT_NAMES[i]);
    Serial.print("): PFS=0x"); Serial.print(pfs, HEX);
    Serial.print(" PMR="); Serial.print((pfs >> 16) & 1);
    Serial.print(" PSEL=0x"); Serial.print((pfs >> 24) & 0x1F, HEX);
    Serial.print(" [cap="); Serial.print(diCapSeq[i]); Serial.print("]");
    Serial.println();
  }

  // Full IELSR dump for all allocated slots
  Serial.println("--- IELSR (first 20) ---");
  for (int i = 0; i < 20; i++) {
    Serial.print("IELSR["); Serial.print(i); Serial.print("]=0x");
    Serial.print(R_ICU->IELSR[i], HEX);
    if ((i % 4) == 3) Serial.println(); else Serial.print("  ");
  }
  Serial.println();
  Serial.println("============================");
}

static void printHelp() {
  Serial.println("\nCommands:");
  Serial.println("  HELP                  - Show this help");
  Serial.println("  STATUS                - Print one-time status report");
  Serial.println("  MONITOR ON|OFF|<ms>    - Enable/disable status monitor or set interval");
  Serial.println("  RXDBG ON|OFF           - Log RX duties, PWM changes and TX failures");
  Serial.println("  DIAG                  - Print capture timer diagnostics");
  Serial.println("  CANSPEED <kbps>        - Set CAN speed (125/250/500/1000)");
  Serial.println("  TXBASE <hex>           - Set CAN TX base ID");
  Serial.println("  RXBASE <hex>           - Set CAN RX base ID");
  Serial.println("  TXRATE <hz>            - Set CAN TX rate in Hz");
  Serial.println("  RXTIMEOUT <ms>         - Set RX timeout in ms");
  Serial.println("  CANMODE <0-15>          - Set CAN mode");
  Serial.println("  OUT <ch> <duty%>        - Set output duty 0-100% (1-4)");
  Serial.println("  OUTFREQ <ch> <hz>       - Set output PWM freq (1-4)");
  Serial.println("  CONFIG                 - Print stored configuration");
  Serial.println("  SERIALOVERRIDE <mask>  - Set serial override mask (0x00-0xFF)");
  Serial.println("  SAFE <ch> <0|1>         - Set safe state for output (1-4)");
  Serial.println("  ACTIVE <ch> <LOW|HIGH>  - Set active state for output (1-4)");
  Serial.println("  DIPULLUP <ch> <0|1>      - Set DI internal pull-up (1-4)");
  Serial.println("  DIPULLUPMASK <hex>       - Set DI pull-up bitmask (bit0=DI1..bit3=DI4)");
  Serial.println("  AIPULLUP <ch> <0|1>      - Set external analog pull-up (AV1-AV4)");
  Serial.println("  AIPULLUPMASK <hex>       - Set external analog pull-up bitmask");
  Serial.println("  CANTERM <0|1>            - Enable or disable CAN termination");
  Serial.println("  OUTVOLT                 - Print output voltage sense readings");
  Serial.println("  DIDEBOUNCE <ms>          - Set DI state debounce (0-100 ms)");
  Serial.println("  DEFAULTS                - Reset all config to factory defaults");
  Serial.println();
}

static void printStatusOnce() {
  Serial.println("\n========== STATUS ==========");
  Serial.print("CAN Mode: ");
  Serial.print(config.canMode);
  Serial.print(" (");
  Serial.print(canModeName(config.canMode));
  Serial.println(")");
  Serial.print("CAN: ");
  Serial.print(config.canSpeedKbps);
  Serial.print(" kbps  TX Base=0x");
  Serial.print(config.txBaseId, HEX);
  Serial.print("  RX Base=0x");
  Serial.print(config.rxBaseId, HEX);
  Serial.print("  TX Rate=");
  Serial.print(config.txRateHz);
  Serial.println(" Hz");
  Serial.print("RX Timeout: ");
  Serial.print(config.rxTimeoutMs);
  Serial.print(" ms  SafeState=");
  Serial.print(outputsInSafeState ? "YES" : "NO");
  Serial.print("  SerialOverride=0x");
  Serial.println(serialOverrideMask, HEX);
  Serial.print("ADC: ");
  Serial.print(ADC_RESOLUTION);
  Serial.print("-bit (0-");
  Serial.print(ADC_MAX);
  Serial.println(")");
  Serial.println();

  Serial.println("--- Analog Inputs ---");
  if (!adc_ready) {
    Serial.println("ADC not initialized");
  }
  uint16_t analogRaw[NUM_ANALOG];
  readAnalogRawAll(analogRaw);
  for (int i = 0; i < NUM_ANALOG; i++) {
    uint16_t adcValue = analogRaw[i];
    float voltage = (adcValue * AREF) / ADC_MAX;
    Serial.print(ANALOG_NAMES[i]);
    Serial.print(" (");
    Serial.print(ANALOG_PORT_NAMES[i]);
    Serial.print("): ADC=");
    Serial.print(adcValue);
    Serial.print(" | V=");
    Serial.print(voltage, 3);
    Serial.println(" V");
  }

  Serial.println();
  Serial.println("--- Digital Inputs ---");
  for (int i = 0; i < NUM_DIGITAL_IN; i++) {
    bool state = diDebouncedState[i];
    uint32_t period;
    uint32_t high;
    bool hasP;
    bool hasH;
    noInterrupts();
    period = diPeriodCounts[i];
    high = diHighCounts[i];
    hasP = diHasPeriod[i];
    hasH = diHasHigh[i];
    interrupts();

    uint32_t timerFreq = diTimerFreq[i];

    Serial.print("DI");
    Serial.print(i + 1);
    Serial.print(" (");
    Serial.print(DIGITAL_IN_PORT_NAMES[i]);
    Serial.print("): ");
    Serial.print(state ? "HIGH" : "LOW ");
    Serial.print(" | ");

    if (hasP && period > 0 && timerFreq > 0) {
      float freq = static_cast<float>(timerFreq) / static_cast<float>(period);
      Serial.print(freq, 1);
      Serial.print(" Hz  duty=");
      if (hasH) {
        uint32_t activeCounts = diActiveCounts(static_cast<uint8_t>(i), period, high);
        float duty = (static_cast<float>(activeCounts) / static_cast<float>(period)) * 100.0f;
        Serial.print(duty, 1);
        Serial.print("%");
      } else {
        Serial.print("--");
      }
    } else {
      Serial.print("-- Hz  duty=--");
    }
    Serial.print(" [cap=");
    Serial.print(diCapSeq[i]);
    Serial.print("]");
    Serial.println();
  }

  Serial.println();
  Serial.println("--- Digital Outputs (Hardware PWM) ---");
  Serial.print("HW_PWM=");
  Serial.print((gpt4_out.is_opened() && gpt5_out.is_opened() && gpt6_out.is_opened() && gpt7_out.is_opened()) ? "OK" : "ERR");
  Serial.print("  lastRxMs=");
  Serial.print(lastCanRxMs);
  Serial.print("  now=");
  Serial.print(millis());
  Serial.print("  timeout=");
  Serial.println(config.rxTimeoutMs);
  for (int i = 0; i < NUM_DIGITAL_OUT; i++) {
    bool pinState  = readDigitalOut(i);
    bool safeOn    = (config.safeMask   >> i) & 0x01;
    bool activeHigh= (config.activeMask >> i) & 0x01;
    Serial.print("OUT");
    Serial.print(i + 1);
    Serial.print(" (");
    Serial.print(DIGITAL_OUT_PORT_NAMES[i]);
    Serial.print("): pin=");
    Serial.print(pinState ? "H" : "L");
    Serial.print("  duty=");
    Serial.print(outputDuty[i]);
    Serial.print("/255  freq=");
    Serial.print(outputFreq[i]);
    Serial.print(" Hz  active=");
    Serial.print(activeHigh ? "HIGH" : "LOW ");
    Serial.print("  safe=");
    Serial.print(safeOn ? "ON " : "OFF");
    Serial.print("  drv=");
    switch (i) {
      case 0: Serial.print("GPT6B"); break;  // OUT1 -> P410
      case 1: Serial.print("GPT5B"); break;  // OUT2 -> P408
      case 2: Serial.print("GPT4A"); break;  // OUT3 -> P302
      case 3: Serial.print("GPT7A"); break;  // OUT4 -> P304
      default: Serial.print("--"); break;
    }
    Serial.println();
  }

  Serial.println();
  Serial.println("--- Auxiliary Switches ---");
  for (int i = 0; i < NUM_ANALOG_PULLUP_SWITCHES; i++) {
    Serial.print("AV");
    Serial.print(i + 1);
    Serial.print(" Pull-up (");
    Serial.print(ANALOG_PULLUP_SWITCH_PORT_NAMES[i]);
    Serial.print("): ");
    Serial.println(getAnalogPullupEnabled(static_cast<uint8_t>(i)) ? "ON" : "OFF");
  }
  Serial.print("CAN Term (");
  Serial.print(CAN_TERM_SWITCH_PORT_NAME);
  Serial.print("): ");
  Serial.println(getCanTerminationEnabled() ? "ON" : "OFF");

  Serial.println();
  printOutputSenseVoltages();

  Serial.println("\n============================");
}

static void printConfig() {
  Serial.println("\n========== CONFIG ==========");
  Serial.print("CAN Speed: ");
  Serial.print(config.canSpeedKbps);
  Serial.println(" kbps");
  Serial.print("TX Base ID: 0x");
  Serial.println(config.txBaseId, HEX);
  Serial.print("RX Base ID: 0x");
  Serial.println(config.rxBaseId, HEX);
  Serial.print("TX Rate: ");
  Serial.print(config.txRateHz);
  Serial.println(" Hz");
  Serial.print("RX Timeout: ");
  Serial.print(config.rxTimeoutMs);
  Serial.println(" ms");
  Serial.print("CAN Mode: ");
  Serial.print(config.canMode);
  Serial.print(" (");
  Serial.print(canModeName(config.canMode));
  Serial.println(")");
  Serial.print("Serial Override: 0x");
  Serial.println(serialOverrideMask, HEX);
  Serial.print("Safe Mask: 0x");
  Serial.println(config.safeMask, HEX);
  Serial.print("Active Mask: 0x");
  Serial.println(config.activeMask, HEX);
  Serial.print("DI Pullup Mask: 0x");
  Serial.println(config.inputPullupMask, HEX);
  Serial.print("AI Pullup Mask: 0x");
  Serial.println(getAuxSwitchMask() & 0x0F, HEX);
  Serial.print("CAN Termination: ");
  Serial.println(getCanTerminationEnabled() ? "ON" : "OFF");
  Serial.print("DI Debounce: ");
  Serial.print(getDiDebounceMs());
  Serial.println(" ms");
  Serial.print("OUT1 Freq: ");
  Serial.print(config.outFreqHz[0]);
  Serial.println(" Hz");
  Serial.print("OUT2 Freq: ");
  Serial.print(config.outFreqHz[1]);
  Serial.println(" Hz");
  Serial.print("OUT3 Freq: ");
  Serial.print(config.outFreqHz[2]);
  Serial.println(" Hz");
  Serial.print("OUT4 Freq: ");
  Serial.print(config.outFreqHz[3]);
  Serial.println(" Hz");
  Serial.print("CRC: 0x");
  Serial.println(config.crc, HEX);
  Serial.println("============================");
}

static String readLine() {
  static String buffer;
  while (Serial.available()) {
    char c = static_cast<char>(Serial.read());
    if (c == '\r') continue;
    if (c == '\n') {
      String line = buffer;
      buffer = "";
      line.trim();
      return line;
    }
    buffer += c;
    if (buffer.length() > 160) {
      buffer = "";
      return "";
    }
  }
  return "";
}

static void handleSerial() {
  String line = readLine();
  if (line.length() == 0) return;
  line.trim();
  
  // Detect JSON commands (start with '{')
  if (line.startsWith("{")) {
    bool configChanged = false;
    bool outputsChanged = false;
    bool pullupChanged = false;
    
    JsonCmdResult result = handleJsonCommand(
      line, telemetrySub, outputDuty, outputFreq,
      config.safeMask, config.activeMask, config.inputPullupMask,
      configChanged, outputsChanged, pullupChanged, serialOverrideMask
    );
    
    if (!result.success) {
      sendJsonError(result.errorMsg.c_str());
      return;
    }
    
    // Handle special command markers
    if (result.errorMsg == "getConfig") {
      // Build device state and send config
      uint8_t readBuf = (adcActiveBuf == 0) ? 1 : 0;
      DeviceState state = {
        adcBuf[readBuf], NUM_ANALOG,
        diDebouncedState, diTimerFreq, diPeriodCounts, diHighCounts, diHasPeriod, diHasHigh, NUM_DIGITAL_IN, DI_ACTIVE_LOW_MASK,
        outputDuty, outputFreq, config.safeMask, config.activeMask, NUM_DIGITAL_OUT,
        canInitOk, outputsInSafeState, canRxCount, canTxCount, canTxFail, config.canMode, lastCanRxMs,
        config.canSpeedKbps, config.txBaseId, config.rxBaseId, config.txRateHz, config.rxTimeoutMs,
        config.inputPullupMask, getDiDebounceMs(), FW_VERSION, adcScanFailCount, millis()
      };
      sendJsonConfig(state);
      return;
    }
    
    if (result.errorMsg == "getHello") {
      // Send hello message
      sendJsonHello(FW_VERSION, config.canMode);
      return;
    }
    
    if (result.errorMsg == "getStatus") {
      // Build and send telemetry
      uint8_t readBuf = (adcActiveBuf == 0) ? 1 : 0;
      DeviceState state = {
        adcBuf[readBuf], NUM_ANALOG,
        diDebouncedState, diTimerFreq, diPeriodCounts, diHighCounts, diHasPeriod, diHasHigh, NUM_DIGITAL_IN, DI_ACTIVE_LOW_MASK,
        outputDuty, outputFreq, config.safeMask, config.activeMask, NUM_DIGITAL_OUT,
        canInitOk, outputsInSafeState, canRxCount, canTxCount, canTxFail, config.canMode, lastCanRxMs,
        config.canSpeedKbps, config.txBaseId, config.rxBaseId, config.txRateHz, config.rxTimeoutMs,
        config.inputPullupMask, getDiDebounceMs(), FW_VERSION, adcScanFailCount, millis()
      };
      sendJsonTelemetry(state);
      return;
    }
    
    if (result.errorMsg == "resetDefaults") {
      setDefaults(config);
      saveConfig();
      for (int p = 0; p < 4; p++) {
        outputFreq[p] = config.outFreqHz[p];
      }
      applyAuxSwitchStates();
      applyOutputs(outputsInSafeState);
      sendJsonResponse(true, "Defaults restored");
      return;
    }
    
    // Apply changes
    if (configChanged) {
      saveConfig();
    }
    if (outputsChanged) {
      applyOutputs(outputsInSafeState);
    }
    if (pullupChanged) {
      applyInputPullups();
    }
    
    sendJsonResponse(true);
    return;
  }
  
  // Text command processing
  String cmd = line;
  cmd.toUpperCase();

  if (cmd == "HELP") {
    printHelp();
    return;
  }
  if (cmd == "STATUS") {
    printStatusOnce();
    return;
  }
  if (cmd == "DEFAULTS") {
    setDefaults(config);
    saveConfig();
    for (int p = 0; p < 4; p++) {
      outputFreq[p] = config.outFreqHz[p];
    }
    applyAuxSwitchStates();
    applyOutputs(outputsInSafeState);
    Serial.println("OK: Config reset to defaults");
    return;
  }
  if (cmd == "CONFIG") {
    printConfig();
    return;
  }
  if (cmd == "DIAG") {
    printDiag();
    return;
  }
  if (cmd.startsWith("MONITOR")) {
    int spaceIdx = cmd.indexOf(' ');
    if (spaceIdx < 0) {
      Serial.println("ERR: MONITOR ON|OFF|<ms>");
      return;
    }
    String arg = cmd.substring(spaceIdx + 1);
    arg.trim();
    if (arg == "ON") {
      monitorEnabled = true;
      Serial.println("OK: Monitor ON");
      return;
    }
    if (arg == "OFF") {
      monitorEnabled = false;
      Serial.println("OK: Monitor OFF");
      return;
    }
    uint32_t ms = static_cast<uint32_t>(arg.toInt());
    if (ms >= 100) {
      monitorIntervalMs = ms;
      monitorEnabled = true;
      Serial.print("OK: Monitor interval ");
      Serial.print(monitorIntervalMs);
      Serial.println(" ms");
    } else {
      Serial.println("ERR: MONITOR <ms> (>=100)");
    }
    return;
  }

  auto getArg = [&](int index) -> String {
    int start = 0;
    for (int i = 0; i < index; i++) {
      start = cmd.indexOf(' ', start);
      if (start < 0) return "";
      start++;
    }
    int end = cmd.indexOf(' ', start);
    if (end < 0) end = cmd.length();
    return cmd.substring(start, end);
  };

  if (cmd.startsWith("SERIALOVERRIDE")) {
    String arg = getArg(1);
    arg.trim();
    if (arg == "ON") {
      serialOverrideMask = 0xFF;  // All channels overridden
      Serial.println("OK: Serial override ALL channels");
      return;
    }
    if (arg == "OFF") {
      serialOverrideMask = 0x00;  // No channels overridden
      Serial.println("OK: Serial override OFF");
      return;
    }
    // Parse as hex mask
    uint8_t mask = static_cast<uint8_t>(strtol(arg.c_str(), nullptr, 0));
    serialOverrideMask = mask;
    Serial.print("OK: Serial override mask=0x");
    Serial.println(serialOverrideMask, HEX);
    return;
  }

  if (cmd.startsWith("CANSPEED")) {
    uint16_t kbps = static_cast<uint16_t>(getArg(1).toInt());
    if (!setCanBitrate(kbps)) {
      Serial.println("ERR: CANSPEED 125|250|500|1000");
      return;
    }
    config.canSpeedKbps = kbps;
    saveConfig();
    Serial.println("OK");
    return;
  }

  if (cmd.startsWith("DIDEBOUNCE")) {
    int ms = getArg(1).toInt();
    if (ms < 0 || ms > 100) {
      Serial.println("ERR: DIDEBOUNCE <ms> (0-100)");
      return;
    }
    setDiDebounceMs(static_cast<uint8_t>(ms));
    saveConfig();
    Serial.print("OK: DI debounce ");
    Serial.print(ms);
    Serial.println(" ms");
    return;
  }

  if (cmd.startsWith("TXBASE")) {
    String arg = getArg(1);
    uint16_t id = static_cast<uint16_t>(strtol(arg.c_str(), nullptr, 0));
    if (id > 0x7FFU) {
      Serial.println("ERR: CAN ID must be 0x000-0x7FF (11-bit)");
      return;
    }
    config.txBaseId = id;
    saveConfig();
    Serial.println("OK");
    return;
  }

  if (cmd.startsWith("RXBASE")) {
    String arg = getArg(1);
    uint16_t id = static_cast<uint16_t>(strtol(arg.c_str(), nullptr, 0));
    if (id > 0x7FFU) {
      Serial.println("ERR: CAN ID must be 0x000-0x7FF (11-bit)");
      return;
    }
    config.rxBaseId = id;
    saveConfig();
    Serial.println("OK");
    return;
  }

  if (cmd.startsWith("TXRATE")) {
    uint16_t hz = static_cast<uint16_t>(getArg(1).toInt());
    if (hz == 0) {
      Serial.println("ERR: TXRATE <hz>");
      return;
    }
    config.txRateHz = hz;
    saveConfig();
    Serial.println("OK");
    return;
  }

  if (cmd.startsWith("RXTIMEOUT")) {
    uint16_t ms = static_cast<uint16_t>(getArg(1).toInt());
    if (ms < 100) {
      Serial.println("ERR: RXTIMEOUT <ms> (>=100)");
      return;
    }
    config.rxTimeoutMs = ms;
    saveConfig();
    Serial.println("OK");
    return;
  }

  if (cmd.startsWith("CANMODE")) {
    uint8_t mode = static_cast<uint8_t>(getArg(1).toInt());
    if (mode >= CAN_MODE_COUNT) {
      Serial.println("ERR: CANMODE <0-15>");
      return;
    }
    config.canMode = mode;
    saveConfig();
    Serial.println("OK");
    return;
  }

  if (cmd.startsWith("OUTFREQ")) {
    uint8_t ch = static_cast<uint8_t>(getArg(1).toInt());
    uint16_t hz = static_cast<uint16_t>(getArg(2).toInt());
    if (ch < 1 || ch > 4 || hz == 0) {
      Serial.println("ERR: OUTFREQ <ch 1-4> <hz>");
      return;
    }
    config.outFreqHz[ch - 1] = hz;
    saveConfig();
    outputFreq[ch - 1] = hz;
    applyOutputs(outputsInSafeState);
    Serial.print("OK: OUT");
    Serial.print(ch);
    Serial.print(" freq=");
    Serial.print(hz);
    Serial.println(" Hz");
    return;
  }

  if (cmd.startsWith("SAFE")) {
    uint8_t ch = static_cast<uint8_t>(getArg(1).toInt());
    uint8_t val = static_cast<uint8_t>(getArg(2).toInt());
    if (ch < 1 || ch > 4) {
      Serial.println("ERR: SAFE <1-4> <0|1>");
      return;
    }
    if (val > 1) {
      Serial.println("ERR: SAFE <1-4> <0|1>");
      return;
    }
    uint8_t mask = 1 << (ch - 1);
    config.safeMask = (config.safeMask & ~mask) | (val ? mask : 0);
    saveConfig();
    applyOutputs(outputsInSafeState);
    Serial.println("OK");
    return;
  }

  if (cmd.startsWith("DIPULLUPMASK")) {
    String arg = getArg(1);
    if (arg.length() == 0) {
      Serial.println("ERR: DIPULLUPMASK <hex>");
      return;
    }
    uint8_t mask = static_cast<uint8_t>(strtol(arg.c_str(), nullptr, 0) & 0x0F);
    config.inputPullupMask = mask;
    applyInputPullups();
    saveConfig();
    Serial.print("OK: DI pull-up mask=0x");
    Serial.println(config.inputPullupMask, HEX);
    return;
  }

  if (cmd.startsWith("DIPULLUP")) {
    uint8_t ch = static_cast<uint8_t>(getArg(1).toInt());
    uint8_t val = static_cast<uint8_t>(getArg(2).toInt());
    if (ch < 1 || ch > 4 || val > 1) {
      Serial.println("ERR: DIPULLUP <1-4> <0|1>");
      return;
    }
    uint8_t mask = static_cast<uint8_t>(1U << (ch - 1U));
    config.inputPullupMask = static_cast<uint8_t>((config.inputPullupMask & ~mask) | (val ? mask : 0U));
    applyInputPullups();
    saveConfig();
    Serial.print("OK: DI");
    Serial.print(ch);
    Serial.print(" pull-up ");
    Serial.println(val ? "ON" : "OFF");
    return;
  }

  if (cmd.startsWith("AIPULLUPMASK")) {
    String arg = getArg(1);
    if (arg.length() == 0) {
      Serial.println("ERR: AIPULLUPMASK <hex>");
      return;
    }
    uint8_t auxMask = static_cast<uint8_t>(strtol(arg.c_str(), nullptr, 0) & 0x0F);
    setAuxSwitchMask(static_cast<uint8_t>((getAuxSwitchMask() & AUX_SWITCH_CAN_TERM) | auxMask));
    applyAuxSwitchStates();
    saveConfig();
    Serial.print("OK: AI pull-up mask=0x");
    Serial.println(getAuxSwitchMask() & 0x0F, HEX);
    return;
  }

  if (cmd.startsWith("AIPULLUP")) {
    uint8_t ch = static_cast<uint8_t>(getArg(1).toInt());
    uint8_t val = static_cast<uint8_t>(getArg(2).toInt());
    if (ch < 1 || ch > 4 || val > 1) {
      Serial.println("ERR: AIPULLUP <1-4> <0|1>");
      return;
    }
    uint8_t mask = static_cast<uint8_t>(1U << (ch - 1U));
    uint8_t newMask = getAuxSwitchMask();
    newMask = static_cast<uint8_t>((newMask & ~mask) | (val ? mask : 0U));
    setAuxSwitchMask(newMask);
    applyAuxSwitchStates();
    saveConfig();
    Serial.print("OK: AV");
    Serial.print(ch);
    Serial.print(" pull-up ");
    Serial.println(val ? "ON" : "OFF");
    return;
  }

  if (cmd.startsWith("CANTERM")) {
    uint8_t val = static_cast<uint8_t>(getArg(1).toInt());
    if (val > 1) {
      Serial.println("ERR: CANTERM <0|1>");
      return;
    }
    uint8_t newMask = getAuxSwitchMask();
    if (val != 0U) {
      newMask |= AUX_SWITCH_CAN_TERM;
    } else {
      newMask &= static_cast<uint8_t>(~AUX_SWITCH_CAN_TERM);
    }
    setAuxSwitchMask(newMask);
    applyAuxSwitchStates();
    saveConfig();
    Serial.print("OK: CAN termination ");
    Serial.println(val ? "ON" : "OFF");
    return;
  }

  if (cmd == "OUTVOLT") {
    printOutputSenseVoltages();
    return;
  }

  if (cmd.startsWith("OUT ") && !cmd.startsWith("OUTFREQ")) {
    uint8_t ch = static_cast<uint8_t>(getArg(1).toInt());
    int pct = getArg(2).toInt();
    if (ch < 1 || ch > 4 || pct < 0 || pct > 100) {
      Serial.println("ERR: OUT <1-4> <0-100>");
      return;
    }
    outputDuty[ch - 1] = static_cast<uint8_t>((pct * 255 + 50) / 100);
    // Set this channel's override bit (enable serial control)
    serialOverrideMask |= (1 << (ch - 1));
    outputsInSafeState = false;
    lastCanRxMs = millis();
    applyOutputs(false);
    Serial.print("OK: OUT"); Serial.print(ch);
    Serial.print(" = "); Serial.print(pct); Serial.println("%");
    Serial.print("INFO: Serial override enabled for channel "); Serial.println(ch);
    return;
  }

  if (cmd == "RXDBG ON" || cmd == "RXDBG OFF") {
    rxDebug = (cmd == "RXDBG ON");
    Serial.print("OK: RXDBG "); Serial.println(rxDebug ? "ON" : "OFF");
    if (rxDebug) {
      Serial.println("  [OUT id=...] logged on each applyOutputs trigger");
      Serial.println("  [PWM DPOn old->new] logged on each set_duty_cycle call");
      Serial.println("  [TX FAIL 0xID] logged on each TX frame drop");
    }
    return;
  }

  if (cmd.startsWith("ACTIVE")) {
    uint8_t ch = static_cast<uint8_t>(getArg(1).toInt());
    String val = getArg(2);
    if (ch < 1 || ch > 4) {
      Serial.println("ERR: ACTIVE <1-4> <LOW|HIGH>");
      return;
    }
    if (val != "LOW" && val != "HIGH") {
      Serial.println("ERR: ACTIVE <1-4> <LOW|HIGH>");
      return;
    }
    uint8_t mask = 1 << (ch - 1);
    if (val == "HIGH") {
      config.activeMask |= mask;
    } else {
      config.activeMask &= ~mask;
    }
    saveConfig();
    applyOutputs(outputsInSafeState);
    Serial.println("OK");
    return;
  }

  Serial.println("ERR: Unknown command. Type HELP");
}

// Initialize hardware PWM using one channel on GPT4-GPT7 for OUT1-OUT4
static void initHardwarePwm() {
  uint32_t freqOut1 = outputFreq[0];
  uint32_t freqOut2 = outputFreq[1];
  uint32_t freqOut3 = outputFreq[2];
  uint32_t freqOut4 = outputFreq[3];

  // OUT1: GPT6B on P410
  configureGptPeripheral(DIGITAL_OUT_PINS[0]);
  FspTimer::force_use_of_pwm_reserved_timer();
  gpt6_out.begin(TIMER_MODE_PWM, GPT_TIMER, 6, freqOut1, 50.0f);
  gpt6_out.add_pwm_extended_cfg();
  gpt6_out.enable_pwm_channel(CHANNEL_B);
  gpt6_out.open();
  gpt6_out.set_duty_cycle(0, CHANNEL_B);
  gpt6_out.start();
  gpt6PeriodCounts = gpt6_out.get_period_raw();
  lastAppliedOutputFreqHz[0] = static_cast<uint16_t>(freqOut1);

  // OUT2: GPT5B on P408
  configureGptPeripheral(DIGITAL_OUT_PINS[1]);
  FspTimer::force_use_of_pwm_reserved_timer();
  gpt5_out.begin(TIMER_MODE_PWM, GPT_TIMER, 5, freqOut2, 50.0f);
  gpt5_out.add_pwm_extended_cfg();
  gpt5_out.enable_pwm_channel(CHANNEL_B);
  gpt5_out.open();
  gpt5_out.set_duty_cycle(0, CHANNEL_B);
  gpt5_out.start();
  gpt5PeriodCounts = gpt5_out.get_period_raw();
  lastAppliedOutputFreqHz[1] = static_cast<uint16_t>(freqOut2);

  // OUT3: GPT4A on P302
  configureGptPeripheral(DIGITAL_OUT_PINS[2]);
  FspTimer::force_use_of_pwm_reserved_timer();
  gpt4_out.begin(TIMER_MODE_PWM, GPT_TIMER, 4, freqOut3, 50.0f);
  gpt4_out.add_pwm_extended_cfg();
  gpt4_out.enable_pwm_channel(CHANNEL_A);
  gpt4_out.open();
  gpt4_out.set_duty_cycle(0, CHANNEL_A);
  gpt4_out.start();
  gpt4PeriodCounts = gpt4_out.get_period_raw();
  lastAppliedOutputFreqHz[2] = static_cast<uint16_t>(freqOut3);

  // OUT4: GPT7A on P304
  configureGptPeripheral(DIGITAL_OUT_PINS[3]);
  FspTimer::force_use_of_pwm_reserved_timer();
  gpt7_out.begin(TIMER_MODE_PWM, GPT_TIMER, 7, freqOut4, 50.0f);
  gpt7_out.add_pwm_extended_cfg();
  gpt7_out.enable_pwm_channel(CHANNEL_A);
  gpt7_out.open();
  gpt7_out.set_duty_cycle(0, CHANNEL_A);
  gpt7_out.start();
  gpt7PeriodCounts = gpt7_out.get_period_raw();
  lastAppliedOutputFreqHz[3] = static_cast<uint16_t>(freqOut4);

  Serial.print("[HW PWM] OUT1=");
  Serial.print(freqOut1);
  Serial.print("Hz OUT2=");
  Serial.print(freqOut2);
  Serial.print("Hz OUT3=");
  Serial.print(freqOut3);
  Serial.print("Hz OUT4=");
  Serial.print(freqOut4);
  Serial.println("Hz");
}

static void initCaptureInputs() {
  // Configure pins for GPT input capture and current pull-up mask
  applyInputPullups();

  initCaptureTimer(gpt0, gpt0Ctx, 0, 4, 3); // GPT0: B=DI4, A unused
  initCaptureTimer(gpt1, gpt1Ctx, 1, 2, 1); // GPT1: A=DI3, B=DI2
  initCaptureTimer(gpt2, gpt2Ctx, 2, 5, 0); // GPT2: B=DI1, A unused

  // Force-enable NVIC for all capture IRQs and cycle-end IRQs.
  // IRQManager::addTimerCompareCaptureA/B allocates the IELSR slot and ISR
  // vector but does NOT call NVIC_EnableIRQ.  R_GPT_Open is supposed to
  // enable them via r_gpt_enable_irq, but GPT0 capture-A (the very first
  // slot) ends up with NVIC disabled in practice.  Explicitly enabling
  // all capture and cycle-end IRQs here is a safe no-op for already-enabled
  // ones and fixes the GPT0-A case.
  FspTimer *capTimers[] = {&gpt0, &gpt1, &gpt2};
  for (auto *t : capTimers) {
    auto *ext = static_cast<gpt_extended_cfg_t *>(const_cast<void *>(t->get_cfg()->p_extend));
    if (ext->capture_a_irq >= 0) NVIC_EnableIRQ((IRQn_Type)ext->capture_a_irq);
    if (ext->capture_b_irq >= 0) NVIC_EnableIRQ((IRQn_Type)ext->capture_b_irq);
    // Also enable the cycle-end (overflow) IRQ so captureCallback receives
    // TIMER_EVENT_CYCLE_END for multi-overflow period measurement.
    if (t->get_cfg()->cycle_end_irq >= 0) NVIC_EnableIRQ((IRQn_Type)t->get_cfg()->cycle_end_irq);
  }
}

void setup() {
  Serial.begin(115200);

  analogReadResolution(ADC_RESOLUTION);

  loadConfig();
  initAuxSwitchOutputs();
  applyAuxSwitchStates();
  initOutputSenseAdc();

  initAdc();

  for (int i = 0; i < DI_SLOT_COUNT; i++) {
    diLastRise[i]         = 0;
    diLastRiseOverflow[i] = 0;
    diHasFirstRise[i]     = false;
    diPeriodCounts[i]     = 0;
    diHighCounts[i]       = 0;
    diHasPeriod[i]        = false;
    diHasHigh[i]          = false;
    diTimerFreq[i]        = 0;
    diCapSeq[i]           = 0;
    diCapSeqLast[i]       = 0;
    diCapTsLast[i]        = 0;
  }

  // Initialize output state arrays
  for (int i = 0; i < OUTPUT_SLOT_COUNT; i++) {
    outputDuty[i] = 0;
    outputFreq[i] = DEFAULT_PWM_FREQ_HZ;
  }
  // Load per-output frequencies from stored config
  for (int p = 0; p < 4; p++) {
    outputFreq[p] = config.outFreqHz[p];
  }

  initHardwarePwm();

  // Initialize NeoPixel (raw P400 driver)
  neopixelInitRaw();
  neopixelShowPacked(makeNeoColor(0, 0, 0));

  // Initialize CAN before capture inputs so that CAN.begin()'s IRQManager
  // allocations don't overwrite IELSR slots that the GPT capture timers need.
  if (!setCanBitrate(config.canSpeedKbps)) {
    Serial.println("[CAN] init FAILED");
  } else {
    Serial.println("[CAN] init OK");
  }

  // Capture inputs are initialized last so their IELSR slots are not
  // overwritten by subsequent IRQManager allocations (CAN, NeoPixel, etc.).
  initCaptureInputs();

  // Seed debounce state from current raw pin levels
  uint32_t nowMs = millis();
  for (int i = 0; i < NUM_DIGITAL_IN; i++) {
    bool raw = readDigitalIn(i);
    diRawPrevState[i] = raw;
    diDebouncedState[i] = raw;
    diRawStableMs[i] = nowMs;
  }

  applyOutputs(true);
  outputsInSafeState = true;

  // Welcome banner is deferred to loop() so it is sent after USB CDC
  // enumerates.  Serial.begin() returns immediately on RA4M1 USB CDC;
  // the host won't see any output until the port is opened.
}

void loop() {
  uint32_t now = millis();

  // Print welcome banner the first time the serial port is opened.
  // On RA4M1 USB CDC, Serial.begin() completes immediately but the host
  // won't receive data until the port is opened by a terminal application.
  if (!serialWelcomeSent && Serial) {
    serialWelcomeSent = true;
    
    // Send JSON hello message first for GUI auto-discovery
    sendJsonHello(FW_VERSION, config.canMode);
    
    // Then text banner for CLI users
    Serial.println();
    Serial.println("============================================");
    Serial.println("      PT-IO-DTM2  CAN I/O Expander");
    Serial.print  ("      FW v");
    Serial.print  (FW_VERSION, HEX);
    Serial.print  ("  CAN Mode: ");
    Serial.print  (config.canMode);
    Serial.print  (" (");
    Serial.print  (canModeName(config.canMode));
    Serial.println(")");
    Serial.println("============================================");
    Serial.println("Type HELP for available commands.");
    Serial.println();
  }

  adcService();

  handleSerial();
  processCan();
  // Refresh now after processCan() so that time-delta checks below (safe-state
  // timeout, TX rate, LED, monitor) are never older than lastCanRxMs.  If the
  // stale pre-processCan() value were used and processCan() updated lastCanRxMs
  // to a newer millis(), unsigned subtraction (now - lastCanRxMs) would wrap to
  // ~4 billion and trigger the RX-timeout safe-state every single loop.
  now = millis();

  // Keep debounced DI state updated independently of CAN TX cadence.
  updateDigitalInDebounce(now);

  // Deferred config save: EEPROM writes are never issued inside the CAN drain
  // loop.  The pending flag is set by handleCanRx() and flushed here, with a
  // 2-second quiet-period debounce so rapid CAN config packets only cause one
  // EEPROM write.
  static uint32_t configDirtyMs = 0;
  if (pendingConfigSave) {
    if (configDirtyMs == 0) {
      configDirtyMs = now;
    } else if (now - configDirtyMs >= 2000UL) {
      saveConfig();
      pendingConfigSave = false;
      configDirtyMs     = 0;
    }
  } else {
    configDirtyMs = 0;
  }

  // CAN watchdog: put non-overridden channels into safe state on timeout
  // Serial-overridden channels remain under app control
  if (!outputsInSafeState && (now - lastCanRxMs > config.rxTimeoutMs)) {
    applyOutputs(true);  // applyOutputs respects serialOverrideMask
    outputsInSafeState = true;
  }

  // Clear stale capture data when signal is removed
  for (int i = 0; i < NUM_DIGITAL_IN; i++) {
    uint32_t seq = diCapSeq[i];
    if (seq != diCapSeqLast[i]) {
      diCapSeqLast[i] = seq;
      diCapTsLast[i] = now;
    } else if (diHasPeriod[i] && (now - diCapTsLast[i] > DI_STALE_TIMEOUT_MS)) {
      noInterrupts();
      diHasPeriod[i]        = false;
      diHasHigh[i]          = false;
      diPeriodCounts[i]     = 0;
      diHighCounts[i]       = 0;
      diLastRise[i]         = 0;
      diLastRiseOverflow[i] = 0;
      diHasFirstRise[i]     = false;
      interrupts();
    }
  }

  // Handle telemetry subscription (JSON or text monitor)
  if (telemetrySub.enabled && (now - telemetrySub.lastSendMs >= telemetrySub.intervalMs)) {
    telemetrySub.lastSendMs = now;
    
    if (telemetrySub.format == TELEMETRY_FORMAT_JSON) {
      // Send JSON telemetry
      uint8_t readBuf = (adcActiveBuf == 0) ? 1 : 0;
      DeviceState state = {
        adcBuf[readBuf], NUM_ANALOG,
        diDebouncedState, diTimerFreq, diPeriodCounts, diHighCounts, diHasPeriod, diHasHigh, NUM_DIGITAL_IN, DI_ACTIVE_LOW_MASK,
        outputDuty, outputFreq, config.safeMask, config.activeMask, NUM_DIGITAL_OUT,
        canInitOk, outputsInSafeState, canRxCount, canTxCount, canTxFail, config.canMode, lastCanRxMs,
        config.canSpeedKbps, config.txBaseId, config.rxBaseId, config.txRateHz, config.rxTimeoutMs,
        config.inputPullupMask, getDiDebounceMs(), FW_VERSION, adcScanFailCount, now
      };
      sendJsonTelemetry(state);
    } else {
      // Text format (existing monitor)
      printStatusOnce();
    }
  }
  
  // Legacy text MONITOR command compatibility
  if (monitorEnabled && (now - lastMonitorMs >= monitorIntervalMs)) {
    lastMonitorMs = now;
    if (!telemetrySub.enabled) {  // Don't double-send if JSON subscription is active
      printStatusOnce();
    }
  }

  if (config.txRateHz > 0) {
    uint32_t intervalMs = 1000UL / config.txRateHz;
    if (now - lastTxMs >= intervalMs) {
      lastTxMs = now;
      sendTxFrames();
    }
  }

  if (now - lastLedMs >= 50) {
    lastLedMs = now;
    updateStatusLed(now);
  }
}
