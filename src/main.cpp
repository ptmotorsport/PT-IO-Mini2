#include <Arduino.h>
#include <Arduino_CAN.h>
#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>
#include "bsp_api.h"
#include "r_ioport.h"
#include "r_adc.h"
#include "FspTimer.h"

// NeoPixel Configuration
#define NEOPIXEL_PIN 11  // P109 = Arduino D11
#define NEOPIXEL_COUNT 1
Adafruit_NeoPixel neopixel(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// Analog Input Pin Definitions (using FSP)
const bsp_io_port_pin_t ANALOG_PINS[] = {
  BSP_IO_PORT_00_PIN_00,  // P000 - AV1
  BSP_IO_PORT_00_PIN_01,  // P001 - AV2
  BSP_IO_PORT_00_PIN_02,  // P002 - AV3
  BSP_IO_PORT_00_PIN_03,  // P003 - AV4
  BSP_IO_PORT_00_PIN_04,  // P004 - AV5
  BSP_IO_PORT_00_PIN_11,  // P011 - AV6
  BSP_IO_PORT_00_PIN_14,  // P014 - AV7
  BSP_IO_PORT_00_PIN_15   // P015 - AV8
};
const char* ANALOG_NAMES[] = {"AV1", "AV2", "AV3", "AV4", "AV5", "AV6", "AV7", "AV8"};
const char* ANALOG_PORT_NAMES[] = {"P000", "P001", "P002", "P003", "P004", "P011", "P014", "P015"};
const int NUM_ANALOG = 8;

const uint8_t ANALOG_CHANNELS[] = {0, 1, 2, 3, 4, 6, 9, 10};

// Digital Input Pin Definitions (using FSP)
const bsp_io_port_pin_t DIGITAL_IN_PINS[] = {
  BSP_IO_PORT_05_PIN_01,  // P501 - DI1 (GTIOC2B)
  BSP_IO_PORT_01_PIN_04,  // P104 - DI2 (GTIOC1B)
  BSP_IO_PORT_01_PIN_05,  // P105 - DI3 (GTIOC1A)
  BSP_IO_PORT_01_PIN_06,  // P106 - DI4 (GTIOC0B)
  BSP_IO_PORT_01_PIN_07,  // P107 - DI5 (GTIOC0A)
  BSP_IO_PORT_01_PIN_13,  // P113 - DI6 (GTIOC2A)
  BSP_IO_PORT_01_PIN_12,  // P112 - DI7 (GTIOC3B)
  BSP_IO_PORT_01_PIN_11   // P111 - DI8 (GTIOC3A)
};
const char* DIGITAL_IN_PORT_NAMES[] = {"P501", "P104", "P105", "P106", "P107", "P113", "P112", "P111"};
const int NUM_DIGITAL_IN = 8;

// Digital Output Pin Definitions (using FSP)
const bsp_io_port_pin_t DIGITAL_OUT_PINS[] = {
  BSP_IO_PORT_04_PIN_08,  // P408 - DPO1 (GTIOC5B)
  BSP_IO_PORT_04_PIN_09,  // P409 - DPO2 (GTIOC5A)
  BSP_IO_PORT_04_PIN_10,  // P410 - DPO3 (GTIOC6B)
  BSP_IO_PORT_04_PIN_11,  // P411 - DPO4 (GTIOC6A)
  BSP_IO_PORT_03_PIN_02,  // P302 - DPO5 (GTIOC4A)
  BSP_IO_PORT_03_PIN_01,  // P301 - DPO6 (GTIOC4B)
  BSP_IO_PORT_03_PIN_00,  // P300 - DPO7 (SWCLK, GPIO only)
  BSP_IO_PORT_01_PIN_08   // P108 - DPO8 (SWDIO, GPIO only)
};
const char* DIGITAL_OUT_PORT_NAMES[] = {"P408", "P409", "P410", "P411", "P302", "P301", "P300", "P108"};
const int NUM_DIGITAL_OUT = 8;

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
const uint8_t FW_VERSION = 0x01;

// NeoPixel
const uint8_t BRIGHTNESS = 26; // 10% of 255

// Config persistence
const uint16_t CONFIG_MAGIC = 0x5049; // "PI"
const uint8_t CONFIG_VERSION = 1;

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
  uint8_t reserved[3];
  uint16_t crc;
};

Config config;

// ADC driver
adc_instance_ctrl_t adc_ctrl;
adc_cfg_t adc_cfg;
adc_channel_cfg_t adc_channel_cfg;
adc_extended_cfg_t adc_ext_cfg;
bool adc_ready = false;

// Output state
uint8_t outputDuty[NUM_DIGITAL_OUT];
uint16_t outputFreq[NUM_DIGITAL_OUT];

uint32_t diTimerFreq[NUM_DIGITAL_IN];

// Input capture stats
volatile uint32_t diLastRise[NUM_DIGITAL_IN];
volatile uint32_t diPeriodCounts[NUM_DIGITAL_IN];
volatile uint32_t diHighCounts[NUM_DIGITAL_IN];
volatile bool diHasPeriod[NUM_DIGITAL_IN];
volatile bool diHasHigh[NUM_DIGITAL_IN];

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

// Timers
FspTimer gpt0; // DI4/DI5
FspTimer gpt1; // DI2/DI3
FspTimer gpt2; // DI1/DI6
FspTimer gpt3; // DI7/DI8

FspTimer gpt4; // DPO5/6 PWM
FspTimer gpt5; // DPO1/2 PWM
FspTimer gpt6; // DPO3/4 PWM

struct CaptureCtx {
  uint8_t idxA;
  uint8_t idxB;
  uint32_t maxCounts;
  uint32_t timerFreqHz;
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

static bool readDigitalIn(uint8_t index) {
  uint8_t port = DIGITAL_IN_PINS[index] >> 8;
  uint8_t pin = DIGITAL_IN_PINS[index] & 0xFF;
  return R_PFS->PORT[port].PIN[pin].PmnPFS_b.PIDR ? true : false;
}

static bool readDigitalOut(uint8_t index) {
  uint8_t port = DIGITAL_OUT_PINS[index] >> 8;
  uint8_t pin = DIGITAL_OUT_PINS[index] & 0xFF;
  return R_PFS->PORT[port].PIN[pin].PmnPFS_b.PODR ? true : false;
}

static void configureGpioInput(bsp_io_port_pin_t pin) {
  uint8_t port = pin >> 8;
  uint8_t bit = pin & 0xFF;
  R_PFS->PORT[port].PIN[bit].PmnPFS_b.PMR = 0;
  R_PFS->PORT[port].PIN[bit].PmnPFS_b.PSEL = 0;
  R_PFS->PORT[port].PIN[bit].PmnPFS_b.PDR = 0;
}

static void configureGpioOutput(bsp_io_port_pin_t pin, bool level) {
  uint8_t port = pin >> 8;
  uint8_t bit = pin & 0xFF;
  R_PFS->PORT[port].PIN[bit].PmnPFS_b.PMR = 0;
  R_PFS->PORT[port].PIN[bit].PmnPFS_b.PSEL = 0;
  R_PFS->PORT[port].PIN[bit].PmnPFS_b.PDR = 1;
  R_PFS->PORT[port].PIN[bit].PmnPFS_b.PODR = level ? 1 : 0;
}

static void configureGptPeripheral(bsp_io_port_pin_t pin) {
  pinPeripheral(pin, (uint32_t)(IOPORT_CFG_PERIPHERAL_PIN | IOPORT_PERIPHERAL_GPT1));
}

static void configureGptPeripheralForChannel(bsp_io_port_pin_t pin, uint8_t channel) {
  (void)channel;
  R_IOPORT_PinCfg(&g_ioport_ctrl, pin,
                  (uint32_t)(IOPORT_CFG_PERIPHERAL_PIN | IOPORT_CFG_PIM_TTL | IOPORT_PERIPHERAL_GPT1));
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

static bool readAnalogRawAll(uint16_t outValues[NUM_ANALOG]) {
  if (!adc_ready) {
    for (int i = 0; i < NUM_ANALOG; i++) {
      outValues[i] = 0;
    }
    return false;
  }

  if (R_ADC_ScanStart(&adc_ctrl) != FSP_SUCCESS) {
    for (int i = 0; i < NUM_ANALOG; i++) {
      outValues[i] = 0;
    }
    return false;
  }

  adc_status_t status;
  status.state = ADC_STATE_SCAN_IN_PROGRESS;
  while (status.state == ADC_STATE_SCAN_IN_PROGRESS) {
    R_ADC_StatusGet(&adc_ctrl, &status);
  }

  for (int i = 0; i < NUM_ANALOG; i++) {
    uint16_t value = 0;
    R_ADC_Read(&adc_ctrl, static_cast<adc_channel_t>(ANALOG_CHANNELS[i]), &value);
    outValues[i] = value;
  }
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

static void setDefaults(Config &cfg) {
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
  cfg.activeMask = 0x00; // active LOW by default
  cfg.canMode = 0;
  memset(cfg.reserved, 0, sizeof(cfg.reserved));
  cfg.crc = computeCrc(cfg);
}

static void saveConfig() {
  config.crc = computeCrc(config);
  EEPROM.put(0, config);
}

static void loadConfig() {
  EEPROM.get(0, config);
  bool valid = (config.magic == CONFIG_MAGIC) && (config.version == CONFIG_VERSION);
  if (valid) {
    uint16_t crc = computeCrc(config);
    valid = (crc == config.crc);
  }
  if (!valid) {
    setDefaults(config);
    saveConfig();
  }
}

static uint32_t modeColor(uint8_t mode) {
  switch (mode) {
    case 0: return neopixel.Color(0, 150, 0);      // Green
    case 1: return neopixel.Color(150, 150, 0);    // Yellow
    case 2: return neopixel.Color(150, 150, 0);    // Yellow
    case 3: return neopixel.Color(150, 150, 0);    // Yellow
    case 4: return neopixel.Color(80, 120, 150);   // BluishYellow
    case 5: return neopixel.Color(80, 120, 150);   // BluishYellow
    case 6: return neopixel.Color(150, 80, 0);     // Amber
    case 7: return neopixel.Color(150, 0, 0);      // Red
    case 8: return neopixel.Color(0, 0, 150);      // Blue
    default: return neopixel.Color(150, 0, 150);   // Violet
  }
}

static void updateStatusLed(uint32_t nowMs) {
  bool canSilent = (lastCanRxMs == 0) ? (nowMs > config.rxTimeoutMs) : (nowMs - lastCanRxMs > config.rxTimeoutMs);
  if (canSilent) {
    bool on = ((nowMs / 250) % 2) == 0;
    neopixel.setPixelColor(0, on ? neopixel.Color(0, 0, 150) : neopixel.Color(0, 0, 0));
  } else {
    neopixel.setPixelColor(0, modeColor(config.canMode));
  }
  neopixel.show();
}

static void captureCallback(timer_callback_args_t *p_args) {
  CaptureCtx *ctx = static_cast<CaptureCtx *>(const_cast<void *>(p_args->p_context));
  if (!ctx) return;

  uint8_t idx = 0xFF;
  if (p_args->event == TIMER_EVENT_CAPTURE_A) {
    idx = ctx->idxA;
  } else if (p_args->event == TIMER_EVENT_CAPTURE_B) {
    idx = ctx->idxB;
  }
  if (idx >= NUM_DIGITAL_IN) return;

  bool level = readDigitalIn(idx);
  uint32_t captured = p_args->capture;

  if (level) {
    if (diLastRise[idx] != 0) {
      diPeriodCounts[idx] = diffCounts(captured, diLastRise[idx], ctx->maxCounts);
      diHasPeriod[idx] = (diPeriodCounts[idx] > 0);
    }
    diLastRise[idx] = captured;
  } else {
    if (diLastRise[idx] != 0) {
      diHighCounts[idx] = diffCounts(captured, diLastRise[idx], ctx->maxCounts);
      diHasHigh[idx] = true;
    }
  }
}

static void initCaptureTimer(FspTimer &timer, CaptureCtx &ctx, uint8_t gptChannel, uint8_t idxA, uint8_t idxB) {
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
  timer.setup_capture_a_irq(12, nullptr);
  timer.setup_capture_b_irq(12, nullptr);
  timer.open();
  timer.start();

  ctx.idxA = idxA;
  ctx.idxB = idxB;
  ctx.maxCounts = timer.get_period_raw();
  ctx.timerFreqHz = timer.get_freq_hz();

  diTimerFreq[idxA] = ctx.timerFreqHz;
  diTimerFreq[idxB] = ctx.timerFreqHz;
}

static bool setCanBitrate(uint16_t kbps) {
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

static void setOutputGpio(uint8_t index, bool on) {
  bool activeHigh = (config.activeMask >> index) & 0x01;
  bool level = activeHigh ? on : !on;
  configureGpioOutput(DIGITAL_OUT_PINS[index], level);
}

static void setPwmDutyPair(FspTimer &timer, uint16_t freqHz, float dutyA, float dutyB) {
  if (freqHz == 0) {
    freqHz = DEFAULT_PWM_FREQ_HZ;
  }
  double periodUs = 1000000.0 / static_cast<double>(freqHz);
  timer.set_period_us(periodUs);
  timer.set_pulse_us(periodUs * dutyA / 100.0, CHANNEL_A);
  timer.set_pulse_us(periodUs * dutyB / 100.0, CHANNEL_B);
}

static void applyOutputs(bool useSafeState) {
  float dutyPercent[NUM_DIGITAL_OUT];
  for (int i = 0; i < NUM_DIGITAL_OUT; i++) {
    uint8_t duty = outputDuty[i];
    if (useSafeState) {
      bool safeOn = (config.safeMask >> i) & 0x01;
      duty = safeOn ? 255 : 0;
    }
    dutyPercent[i] = (static_cast<float>(duty) * 100.0f) / 255.0f;
  }

  // DPO1/2 -> GPT5
  float duty1 = ((config.activeMask & 0x01) ? dutyPercent[0] : (100.0f - dutyPercent[0]));
  float duty2 = ((config.activeMask & 0x02) ? dutyPercent[1] : (100.0f - dutyPercent[1]));
  setPwmDutyPair(gpt5, outputFreq[0], duty2, duty1); // GPT5: A=P409 (DPO2), B=P408 (DPO1)

  // DPO3/4 -> GPT6
  float duty3 = ((config.activeMask & 0x04) ? dutyPercent[2] : (100.0f - dutyPercent[2]));
  float duty4 = ((config.activeMask & 0x08) ? dutyPercent[3] : (100.0f - dutyPercent[3]));
  setPwmDutyPair(gpt6, outputFreq[2], duty4, duty3); // GPT6: A=P411 (DPO4), B=P410 (DPO3)

  // DPO5/6 -> GPT4
  float duty5 = ((config.activeMask & 0x10) ? dutyPercent[4] : (100.0f - dutyPercent[4]));
  float duty6 = ((config.activeMask & 0x20) ? dutyPercent[5] : (100.0f - dutyPercent[5]));
  setPwmDutyPair(gpt4, outputFreq[4], duty5, duty6); // GPT4: A=P302 (DPO5), B=P301 (DPO6)

  // DPO7/8 GPIO only
  bool on7 = dutyPercent[6] > 0.1f;
  bool on8 = dutyPercent[7] > 0.1f;
  setOutputGpio(6, on7);
  setOutputGpio(7, on8);
}

static void handleCanRx(const CanMsg &msg) {
  uint32_t id = msg.id;
  if (id < config.rxBaseId || id > (config.rxBaseId + 3)) {
    return;
  }

  uint8_t idxPair = static_cast<uint8_t>(id - config.rxBaseId); // 0..3
  uint8_t outA = idxPair * 2;
  uint8_t outB = outA + 1;

  if (msg.data_length < 8) {
    return;
  }

  uint16_t freqA = static_cast<uint16_t>(msg.data[0] | (msg.data[1] << 8));
  uint8_t dutyA = msg.data[2];
  uint8_t flagsA = msg.data[3];

  uint16_t freqB = static_cast<uint16_t>(msg.data[4] | (msg.data[5] << 8));
  uint8_t dutyB = msg.data[6];
  uint8_t flagsB = msg.data[7];

  uint16_t pairFreq = freqA ? freqA : freqB;
  outputFreq[outA] = pairFreq;
  outputFreq[outB] = pairFreq;
  outputDuty[outA] = dutyA;
  outputDuty[outB] = dutyB;

  bool safeA = flagsA & 0x01;
  bool activeA = flagsA & 0x02;
  bool safeB = flagsB & 0x01;
  bool activeB = flagsB & 0x02;

  uint8_t newSafeMask = config.safeMask;
  uint8_t newActiveMask = config.activeMask;

  newSafeMask = (newSafeMask & ~(1 << outA)) | (safeA ? (1 << outA) : 0);
  newSafeMask = (newSafeMask & ~(1 << outB)) | (safeB ? (1 << outB) : 0);
  newActiveMask = (newActiveMask & ~(1 << outA)) | (activeA ? (1 << outA) : 0);
  newActiveMask = (newActiveMask & ~(1 << outB)) | (activeB ? (1 << outB) : 0);

  bool changed = (newSafeMask != config.safeMask) || (newActiveMask != config.activeMask);
  config.safeMask = newSafeMask;
  config.activeMask = newActiveMask;

  if (changed) {
    saveConfig();
  }

  outputsInSafeState = false;
  applyOutputs(false);
}

static void processCan() {
  while (CAN.available()) {
    CanMsg msg = CAN.read();
    lastCanRxMs = millis();
    if (canRxCount != 0xFFFF) {
      canRxCount++;
    }
    handleCanRx(msg);
  }
}

static void sendCanFrame(uint32_t id, const uint8_t *data, uint8_t len) {
  CanMsg tx;
  tx.id = id;
  tx.data_length = len;
  memcpy(tx.data, data, len);
  if (CAN.write(tx)) {
    if (canTxCount != 0xFFFF) {
      canTxCount++;
    }
  } else {
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
    if (readDigitalIn(i)) {
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

  // TX BaseID: AV1-AV4
  uint8_t tx0[8];
  memset(tx0, 0, sizeof(tx0));
  for (int i = 0; i < 4; i++) {
    tx0[i * 2] = static_cast<uint8_t>(analogRaw[i] & 0xFF);
    tx0[i * 2 + 1] = static_cast<uint8_t>((analogRaw[i] >> 8) & 0xFF);
  }
  sendCanFrame(config.txBaseId, tx0, 8);

  // TX BaseID+1: AV5-AV8
  uint8_t tx1[8];
  memset(tx1, 0, sizeof(tx1));
  for (int i = 0; i < 4; i++) {
    uint16_t val = analogRaw[i + 4];
    tx1[i * 2] = static_cast<uint8_t>(val & 0xFF);
    tx1[i * 2 + 1] = static_cast<uint8_t>((val >> 8) & 0xFF);
  }
  sendCanFrame(config.txBaseId + 1, tx1, 8);

  // TX BaseID+2: state masks + FW version
  uint8_t tx2[8] = {0};
  tx2[0] = digitalInMask;
  tx2[1] = analogStateMask;
  tx2[2] = digitalOutMask;
  tx2[3] = config.safeMask;
  tx2[4] = config.activeMask;
  tx2[7] = FW_VERSION;
  sendCanFrame(config.txBaseId + 2, tx2, 8);

  auto packFreqDuty = [&](uint8_t baseIndex, uint32_t baseId) {
    uint8_t tx[8] = {0};

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

    uint16_t freq0 = 0;
    uint16_t freq1 = 0;
    uint8_t duty0 = 0;
    uint8_t duty1 = 0;

    uint32_t timerFreq0 = diTimerFreq[baseIndex];
    uint32_t timerFreq1 = diTimerFreq[baseIndex + 1];

    if (has0 && period0 > 0 && timerFreq0 > 0) {
      freq0 = static_cast<uint16_t>(min<uint32_t>(65535, timerFreq0 / period0));
      if (high0 > 0) {
        duty0 = static_cast<uint8_t>(min<uint32_t>(255, (high0 * 255UL) / period0));
      }
    }
    if (has1 && period1 > 0 && timerFreq1 > 0) {
      freq1 = static_cast<uint16_t>(min<uint32_t>(65535, timerFreq1 / period1));
      if (high1 > 0) {
        duty1 = static_cast<uint8_t>(min<uint32_t>(255, (high1 * 255UL) / period1));
      }
    }

    tx[0] = static_cast<uint8_t>(freq0 & 0xFF);
    tx[1] = static_cast<uint8_t>((freq0 >> 8) & 0xFF);
    tx[2] = duty0;
    tx[4] = static_cast<uint8_t>(freq1 & 0xFF);
    tx[5] = static_cast<uint8_t>((freq1 >> 8) & 0xFF);
    tx[6] = duty1;

    sendCanFrame(baseId, tx, 8);
  };

  packFreqDuty(0, config.txBaseId + 3);
  packFreqDuty(2, config.txBaseId + 4);
  packFreqDuty(4, config.txBaseId + 5);
  packFreqDuty(6, config.txBaseId + 6);

  uint8_t txStatus[8] = {0};
  bool canSilent = (lastCanRxMs == 0) ? (millis() > config.rxTimeoutMs) : (millis() - lastCanRxMs > config.rxTimeoutMs);
  txStatus[0] = (canInitOk ? 0x01 : 0x00) |
                (canSilent ? 0x02 : 0x00) |
                (outputsInSafeState ? 0x04 : 0x00);
  txStatus[1] = static_cast<uint8_t>(canRxCount & 0xFF);
  txStatus[2] = static_cast<uint8_t>((canRxCount >> 8) & 0xFF);
  txStatus[3] = static_cast<uint8_t>(canTxCount & 0xFF);
  txStatus[4] = static_cast<uint8_t>((canTxCount >> 8) & 0xFF);
  txStatus[5] = static_cast<uint8_t>(canTxFail & 0xFF);
  txStatus[6] = static_cast<uint8_t>((canTxFail >> 8) & 0xFF);
  txStatus[7] = config.canMode;
  sendCanFrame(config.txBaseId + 7, txStatus, 8);
}

static void printHelp() {
  Serial.println("\nCommands:");
  Serial.println("  HELP                  - Show this help");
  Serial.println("  STATUS                - Print one-time status report");
  Serial.println("  MONITOR ON|OFF|<ms>    - Enable/disable status monitor or set interval");
  Serial.println("  CANSPEED <kbps>        - Set CAN speed (125/250/500/1000)");
  Serial.println("  TXBASE <hex>           - Set CAN TX base ID");
  Serial.println("  RXBASE <hex>           - Set CAN RX base ID");
  Serial.println("  TXRATE <hz>            - Set CAN TX rate in Hz");
  Serial.println("  RXTIMEOUT <ms>         - Set RX timeout in ms");
  Serial.println("  CANMODE <0-15>          - Set CAN mode (NeoPixel color)");
  Serial.println("  OUTFREQ <pair> <hz>     - Set output pair PWM frequency (1-4)");
  Serial.println("  CONFIG                 - Print stored configuration");
  Serial.println("  SAFE <ch> <0|1>         - Set safe state for output (1-8)");
  Serial.println("  ACTIVE <ch> <LOW|HIGH>  - Set active state for output (1-8)");
  Serial.println();
}

static void printStatusOnce() {
  Serial.println("\n========== STATUS ==========");
  Serial.print("ADC Resolution: ");
  Serial.print(ADC_RESOLUTION);
  Serial.print("-bit (0-");
  Serial.print(ADC_MAX);
  Serial.println(")");
  Serial.print("CAN: ");
  Serial.print(config.canSpeedKbps);
  Serial.print(" kbps, TX Base=0x");
  Serial.print(config.txBaseId, HEX);
  Serial.print(", RX Base=0x");
  Serial.print(config.rxBaseId, HEX);
  Serial.print(", TX Rate=");
  Serial.print(config.txRateHz);
  Serial.println(" Hz");
  Serial.print("RX Timeout: ");
  Serial.print(config.rxTimeoutMs);
  Serial.println(" ms");
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
    bool state = readDigitalIn(i);
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
    Serial.print(state ? "HIGH" : "LOW");
    Serial.print(" | ");

    if (hasP && period > 0 && timerFreq > 0) {
      float freq = static_cast<float>(timerFreq) / static_cast<float>(period);
      Serial.print(freq, 2);
      Serial.print(" Hz, duty=");
      if (hasH) {
        float duty = (static_cast<float>(high) / static_cast<float>(period)) * 100.0f;
        Serial.print(duty, 1);
        Serial.print("%");
      } else {
        Serial.print("--");
      }
    } else {
      Serial.print("-- Hz, duty=--");
    }
    Serial.println();
  }

  Serial.println();
  Serial.println("--- Digital Outputs ---");
  for (int i = 0; i < NUM_DIGITAL_OUT; i++) {
    bool state = readDigitalOut(i);
    Serial.print("DPO");
    Serial.print(i + 1);
    Serial.print(" (");
    Serial.print(DIGITAL_OUT_PORT_NAMES[i]);
    Serial.print("): ");
    Serial.print(state ? "ON" : "OFF");
    Serial.print(" | duty=");
    Serial.print(outputDuty[i]);
    Serial.print("/255, freq=");
    Serial.print(outputFreq[i]);
    Serial.println(" Hz");
  }

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
  Serial.println(config.canMode);
  Serial.print("Safe Mask: 0x");
  Serial.println(config.safeMask, HEX);
  Serial.print("Active Mask: 0x");
  Serial.println(config.activeMask, HEX);
  Serial.print("Out Freq Pair 1: ");
  Serial.print(config.outFreqHz[0]);
  Serial.println(" Hz");
  Serial.print("Out Freq Pair 2: ");
  Serial.print(config.outFreqHz[1]);
  Serial.println(" Hz");
  Serial.print("Out Freq Pair 3: ");
  Serial.print(config.outFreqHz[2]);
  Serial.println(" Hz");
  Serial.print("Out Freq Pair 4: ");
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
  if (cmd == "CONFIG") {
    printConfig();
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

  if (cmd.startsWith("TXBASE")) {
    String arg = getArg(1);
    uint16_t id = static_cast<uint16_t>(strtol(arg.c_str(), nullptr, 0));
    config.txBaseId = id;
    saveConfig();
    Serial.println("OK");
    return;
  }

  if (cmd.startsWith("RXBASE")) {
    String arg = getArg(1);
    uint16_t id = static_cast<uint16_t>(strtol(arg.c_str(), nullptr, 0));
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
    config.canMode = mode;
    saveConfig();
    Serial.println("OK");
    return;
  }

  if (cmd.startsWith("OUTFREQ")) {
    uint8_t pair = static_cast<uint8_t>(getArg(1).toInt());
    uint16_t hz = static_cast<uint16_t>(getArg(2).toInt());
    if (pair < 1 || pair > 4 || hz == 0) {
      Serial.println("ERR: OUTFREQ <pair 1-4> <hz>");
      return;
    }
    config.outFreqHz[pair - 1] = hz;
    saveConfig();
    uint8_t baseIdx = (pair - 1) * 2;
    outputFreq[baseIdx] = hz;
    outputFreq[baseIdx + 1] = hz;
    applyOutputs(outputsInSafeState);
    Serial.println("OK");
    return;
  }

  if (cmd.startsWith("SAFE")) {
    uint8_t ch = static_cast<uint8_t>(getArg(1).toInt());
    uint8_t val = static_cast<uint8_t>(getArg(2).toInt());
    if (ch < 1 || ch > 8) {
      Serial.println("ERR: SAFE <1-8> <0|1>");
      return;
    }
    if (val > 1) {
      Serial.println("ERR: SAFE <1-8> <0|1>");
      return;
    }
    uint8_t mask = 1 << (ch - 1);
    config.safeMask = (config.safeMask & ~mask) | (val ? mask : 0);
    saveConfig();
    applyOutputs(outputsInSafeState);
    Serial.println("OK");
    return;
  }

  if (cmd.startsWith("ACTIVE")) {
    uint8_t ch = static_cast<uint8_t>(getArg(1).toInt());
    String val = getArg(2);
    if (ch < 1 || ch > 8) {
      Serial.println("ERR: ACTIVE <1-8> <LOW|HIGH>");
      return;
    }
    if (val != "LOW" && val != "HIGH") {
      Serial.println("ERR: ACTIVE <1-8> <LOW|HIGH>");
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

static void initPwmOutputs() {
  // Configure pins for PWM peripheral
  configureGptPeripheral(DIGITAL_OUT_PINS[0]); // DPO1
  configureGptPeripheral(DIGITAL_OUT_PINS[1]); // DPO2
  configureGptPeripheral(DIGITAL_OUT_PINS[2]); // DPO3
  configureGptPeripheral(DIGITAL_OUT_PINS[3]); // DPO4
  configureGptPeripheral(DIGITAL_OUT_PINS[4]); // DPO5
  configureGptPeripheral(DIGITAL_OUT_PINS[5]); // DPO6

  gpt5.begin(TIMER_MODE_PWM, GPT_TIMER, 5, config.outFreqHz[0], 50.0f);
  gpt5.add_pwm_extended_cfg();
  gpt5.enable_pwm_channel(CHANNEL_A);
  gpt5.enable_pwm_channel(CHANNEL_B);
  gpt5.open();
  gpt5.start();

  gpt6.begin(TIMER_MODE_PWM, GPT_TIMER, 6, config.outFreqHz[1], 50.0f);
  gpt6.add_pwm_extended_cfg();
  gpt6.enable_pwm_channel(CHANNEL_A);
  gpt6.enable_pwm_channel(CHANNEL_B);
  gpt6.open();
  gpt6.start();

  gpt4.begin(TIMER_MODE_PWM, GPT_TIMER, 4, config.outFreqHz[2], 50.0f);
  gpt4.add_pwm_extended_cfg();
  gpt4.enable_pwm_channel(CHANNEL_A);
  gpt4.enable_pwm_channel(CHANNEL_B);
  gpt4.open();
  gpt4.start();
}

static void initCaptureInputs() {
  // Configure pins for GPT input capture
  configureGptPeripheralForChannel(DIGITAL_IN_PINS[0], 2); // DI1 GPT2B
  configureGptPeripheralForChannel(DIGITAL_IN_PINS[1], 1); // DI2 GPT1B
  configureGptPeripheralForChannel(DIGITAL_IN_PINS[2], 1); // DI3 GPT1A
  configureGptPeripheralForChannel(DIGITAL_IN_PINS[3], 0); // DI4 GPT0B
  configureGptPeripheralForChannel(DIGITAL_IN_PINS[4], 0); // DI5 GPT0A
  configureGptPeripheralForChannel(DIGITAL_IN_PINS[5], 2); // DI6 GPT2A
  configureGptPeripheralForChannel(DIGITAL_IN_PINS[6], 3); // DI7 GPT3B
  configureGptPeripheralForChannel(DIGITAL_IN_PINS[7], 3); // DI8 GPT3A

  initCaptureTimer(gpt0, gpt0Ctx, 0, 4, 3); // GPT0: A=DI5, B=DI4
  initCaptureTimer(gpt1, gpt1Ctx, 1, 2, 1); // GPT1: A=DI3, B=DI2
  initCaptureTimer(gpt2, gpt2Ctx, 2, 5, 0); // GPT2: A=DI6, B=DI1
  initCaptureTimer(gpt3, gpt3Ctx, 3, 7, 6); // GPT3: A=DI8, B=DI7
}

void setup() {
  Serial.begin(115200);

  analogReadResolution(ADC_RESOLUTION);

  loadConfig();

  initAdc();

  for (int i = 0; i < NUM_DIGITAL_IN; i++) {
    diLastRise[i] = 0;
    diPeriodCounts[i] = 0;
    diHighCounts[i] = 0;
    diHasPeriod[i] = false;
    diHasHigh[i] = false;
    diTimerFreq[i] = 0;
  }

  // Setup GPIO-only outputs for DPO7/8
  configureGpioOutput(DIGITAL_OUT_PINS[6], false);
  configureGpioOutput(DIGITAL_OUT_PINS[7], false);

  // Initialize output state arrays
  for (int i = 0; i < NUM_DIGITAL_OUT; i++) {
    outputDuty[i] = 0;
    outputFreq[i] = DEFAULT_PWM_FREQ_HZ;
  }
  outputFreq[0] = config.outFreqHz[0];
  outputFreq[1] = config.outFreqHz[0];
  outputFreq[2] = config.outFreqHz[1];
  outputFreq[3] = config.outFreqHz[1];
  outputFreq[4] = config.outFreqHz[2];
  outputFreq[5] = config.outFreqHz[2];
  outputFreq[6] = config.outFreqHz[3];
  outputFreq[7] = config.outFreqHz[3];

  initPwmOutputs();
  initCaptureInputs();

  // Initialize NeoPixel
  neopixel.begin();
  neopixel.setBrightness(BRIGHTNESS);
  neopixel.show();

  // Initialize CAN
  if (!setCanBitrate(config.canSpeedKbps)) {
    Serial.println("CAN init FAILED");
  } else {
    Serial.println("CAN init OK");
  }

  Serial.println("====================================");
  Serial.println("   PT-IO-Mini2-1 CAN IO Expander");
  Serial.println("====================================");
  printHelp();

  applyOutputs(true);
  outputsInSafeState = true;
}

void loop() {
  uint32_t now = millis();

  handleSerial();
  processCan();

  if (!outputsInSafeState && (now - lastCanRxMs > config.rxTimeoutMs)) {
    applyOutputs(true);
    outputsInSafeState = true;
  }

  if (monitorEnabled && (now - lastMonitorMs >= monitorIntervalMs)) {
    lastMonitorMs = now;
    printStatusOnce();
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
