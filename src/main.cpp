#include <Arduino.h>
#include <Arduino_CAN.h>
#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>
#include "bsp_api.h"
#include "r_ioport.h"
#include "r_adc.h"
#include "FspTimer.h"
#include "can_modes.h"

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
const uint8_t FW_VERSION = 0x03;

// Additional hardware polarity stage (e.g. external MOSFET inverter).
// Bit=1 means invert the post-activeMask duty before writing to the pin.
// Set to 0xFF to invert all outputs, or set bits per-channel as needed.
const uint8_t OUTPUT_STAGE_INVERT_MASK = 0xFF;

// NeoPixel
const uint8_t BRIGHTNESS = 26; // 10% of 255

// Config persistence
const uint16_t CONFIG_MAGIC = 0x5049; // "PI"
const uint8_t CONFIG_VERSION = 2;

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

// Output state
uint8_t outputDuty[NUM_DIGITAL_OUT];
uint16_t outputFreq[NUM_DIGITAL_OUT];
uint32_t lastAppliedPwmCounts[6] = {0xFFFFFFFFUL, 0xFFFFFFFFUL, 0xFFFFFFFFUL,
                                    0xFFFFFFFFUL, 0xFFFFFFFFUL, 0xFFFFFFFFUL};
uint8_t lastAppliedGpioLevel[2] = {0xFFU, 0xFFU};

uint32_t diTimerFreq[NUM_DIGITAL_IN];

// Input capture stats
volatile uint32_t diLastRise[NUM_DIGITAL_IN];       // timer count at last rising edge (0 = no rise yet)
volatile uint32_t diLastRiseOverflow[NUM_DIGITAL_IN]; // per-channel overflow count at last rising edge
volatile bool     diHasFirstRise[NUM_DIGITAL_IN];   // true once first rising edge has been seen
volatile uint32_t diPeriodCounts[NUM_DIGITAL_IN];
volatile uint32_t diHighCounts[NUM_DIGITAL_IN];
volatile bool diHasPeriod[NUM_DIGITAL_IN];
volatile bool diHasHigh[NUM_DIGITAL_IN];
volatile uint32_t diCapSeq[NUM_DIGITAL_IN];  // incremented on each capture event

// Stale capture detection (main loop)
uint32_t diCapSeqLast[NUM_DIGITAL_IN];
uint32_t diCapTsLast[NUM_DIGITAL_IN];
const uint32_t DI_STALE_TIMEOUT_MS = 500;

// Serial connection tracking
bool serialWelcomeSent = false;

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
bool serialOverride = false;  // when true, CAN watchdog is bypassed for serial testing
bool rxDebug = false;         // when true, log RX duties, applyOutputs triggers and TX failures

// Timers - input capture
FspTimer gpt0; // DI4/DI5
FspTimer gpt1; // DI2/DI3
FspTimer gpt2; // DI1/DI6
FspTimer gpt3; // DI7/DI8

// Hardware PWM output timers (GPT4, GPT5, GPT6)
FspTimer gpt4_out;  // DPO5-6  (GTIOC4A/4B)
FspTimer gpt5_out;  // DPO1-2  (GTIOC5A/5B)
FspTimer gpt6_out;  // DPO3-4  (GTIOC6A/6B)
uint32_t gpt4PeriodCounts = 0;
uint32_t gpt5PeriodCounts = 0;
uint32_t gpt6PeriodCounts = 0;
// Last applied output frequency per GPT pair (0=GPT5/DPO1-2, 1=GPT6/DPO3-4, 2=GPT4/DPO5-6)
// Initialised to 0 so any non-zero loaded frequency triggers an update on first applyOutputs()
uint16_t lastAppliedOutputFreqHz[3] = {0, 0, 0};

struct CaptureCtx {
  uint8_t idxA;
  uint8_t idxB;
  uint32_t maxCounts;
  uint32_t timerFreqHz;
  volatile uint32_t overflowCount;  // incremented by TIMER_EVENT_CYCLE_END in captureCallback
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

static void applyInputPullups() {
  for (int i = 0; i < NUM_DIGITAL_IN; i++) {
    uint32_t cfg = (uint32_t)(IOPORT_CFG_PERIPHERAL_PIN | IOPORT_CFG_PIM_TTL | IOPORT_PERIPHERAL_GPT1);
    if (((config.inputPullupMask >> i) & 0x01U) != 0U) {
      cfg |= IOPORT_CFG_PULLUP_ENABLE;
    }
    R_IOPORT_PinCfg(&g_ioport_ctrl, DIGITAL_IN_PINS[i], cfg);
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
  uint32_t adcStartMs = millis();
  while (status.state == ADC_STATE_SCAN_IN_PROGRESS) {
    if (millis() - adcStartMs > 10UL) {
      // ADC hardware failed to complete within 10 ms; mark as not ready
      // so subsequent calls return zeros rather than blocking forever.
      adc_ready = false;
      for (int i = 0; i < NUM_ANALOG; i++) {
        outValues[i] = 0;
      }
      return false;
    }
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
  cfg.inputPullupMask = 0x00; // all DI pull-ups OFF by default
  memset(cfg.reserved, 0, sizeof(cfg.reserved));
  cfg.crc = computeCrc(cfg);
}

static void saveConfig() {
  config.crc = computeCrc(config);
  EEPROM.put(0, config);
  Serial.println("[EEPROM] config written");
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
  // Validate frequency values â€” corrupted EEPROM can yield garbage
  for (int p = 0; p < 4; p++) {
    if (config.outFreqHz[p] < 50 || config.outFreqHz[p] > 10000) {
      config.outFreqHz[p] = DEFAULT_PWM_FREQ_HZ;
    }
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

  // Sample overflow counter around captured count read so we don't mis-attribute
  // an overflow that lands between the two reads.
  uint32_t ovf1 = ctx->overflowCount;
  uint32_t captured = p_args->capture;
  uint32_t ovf2 = ctx->overflowCount;
  uint32_t nowOv = (ovf2 != ovf1 && captured < (ctx->maxCounts >> 1U)) ? ovf2 : ovf1;

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

// swPwmIsr removed - using hardware PWM instead

// Re-initialise a single output GPT timer at a new frequency, then invalidate
// the cached duty-cycle counts so that applyOutputs() re-applies the duty.
static void reinitOutputTimer(FspTimer &timer, uint8_t gptChannel,
                              uint32_t &periodCounts,
                              uint32_t &cachedCounts0, uint32_t &cachedCounts1) {
  timer.end();
  FspTimer::force_use_of_pwm_reserved_timer();
  // Reload from the matching outputFreq[] slot (caller responsible for selecting correct Hz).
  // We pass 50.0f initial duty just to open the timer; actual duty is applied below.
  uint16_t hz = outputFreq[(gptChannel == 5) ? 0 : (gptChannel == 6) ? 2 : 4];
  timer.begin(TIMER_MODE_PWM, GPT_TIMER, gptChannel, hz, 50.0f);
  timer.add_pwm_extended_cfg();
  timer.enable_pwm_channel(CHANNEL_B);
  timer.enable_pwm_channel(CHANNEL_A);
  timer.open();
  timer.set_duty_cycle(0, CHANNEL_B);
  timer.set_duty_cycle(0, CHANNEL_A);
  timer.start();
  periodCounts  = timer.get_period_raw();
  // Invalidate cached counts so the duty section below re-applies them.
  cachedCounts0 = 0xFFFFFFFFUL;
  cachedCounts1 = 0xFFFFFFFFUL;
}

// Apply output values using hardware PWM (GPT4, GPT5, GPT6) and GPIO
static void applyOutputs(bool useSafeState) {
  uint8_t newDuty[NUM_DIGITAL_OUT];

  for (int i = 0; i < NUM_DIGITAL_OUT; i++) {
    uint8_t duty = outputDuty[i];
    if (useSafeState) {
      bool safeOn = (config.safeMask >> i) & 0x01;
      duty = safeOn ? 255 : 0;
    }
    bool activeHigh = (config.activeMask >> i) & 0x01;
    if (!activeHigh) duty = 255 - duty;
    bool stageInverted = ((OUTPUT_STAGE_INVERT_MASK >> i) & 0x01U) != 0U;
    if (stageInverted) duty = 255 - duty;
    newDuty[i] = duty;
  }

  // Update hardware PWM frequency per GPT pair when it has changed.
  // Each GPT timer drives two outputs that share the same period register.
  // Use the even-indexed channel of each pair as the authoritative frequency.
  //   GPT5 → DPO1-2 (outputFreq[0]), GPT6 → DPO3-4 (outputFreq[2]), GPT4 → DPO5-6 (outputFreq[4])
  if (gpt5_out.is_opened() && outputFreq[0] != lastAppliedOutputFreqHz[0]) {
    if (rxDebug) {
      Serial.print("[FREQ GPT5 "); Serial.print(lastAppliedOutputFreqHz[0]);
      Serial.print("->"); Serial.print(outputFreq[0]); Serial.println("Hz]");
    }
    reinitOutputTimer(gpt5_out, 5, gpt5PeriodCounts, lastAppliedPwmCounts[0], lastAppliedPwmCounts[1]);
    lastAppliedOutputFreqHz[0] = outputFreq[0];
  }
  if (gpt6_out.is_opened() && outputFreq[2] != lastAppliedOutputFreqHz[1]) {
    if (rxDebug) {
      Serial.print("[FREQ GPT6 "); Serial.print(lastAppliedOutputFreqHz[1]);
      Serial.print("->"); Serial.print(outputFreq[2]); Serial.println("Hz]");
    }
    reinitOutputTimer(gpt6_out, 6, gpt6PeriodCounts, lastAppliedPwmCounts[2], lastAppliedPwmCounts[3]);
    lastAppliedOutputFreqHz[1] = outputFreq[2];
  }
  if (gpt4_out.is_opened() && outputFreq[4] != lastAppliedOutputFreqHz[2]) {
    if (rxDebug) {
      Serial.print("[FREQ GPT4 "); Serial.print(lastAppliedOutputFreqHz[2]);
      Serial.print("->"); Serial.print(outputFreq[4]); Serial.println("Hz]");
    }
    reinitOutputTimer(gpt4_out, 4, gpt4PeriodCounts, lastAppliedPwmCounts[4], lastAppliedPwmCounts[5]);
    lastAppliedOutputFreqHz[2] = outputFreq[4];
  }

  // Update hardware PWM using set_duty_cycle
  // set_duty_cycle expects counts 0..period, so scale from 0-255
  // Cap max to period-1 to ensure compare logic works properly
  
  // Channels 0-1: GPT5 (DPO1-2, GTIOC5B/5A)
  if (gpt5_out.is_opened()) {
    uint32_t period = gpt5PeriodCounts;
    if (period < 2U) {
      period = gpt5_out.get_period_raw();
      gpt5PeriodCounts = period;
    }
    if (period < 2U) {
      period = 2U;
    }
    uint32_t counts0 = (period * (uint32_t)newDuty[0]) / 255;
    uint32_t counts1 = (period * (uint32_t)newDuty[1]) / 255;
    if (counts0 >= period) counts0 = period - 1;
    if (counts1 >= period) counts1 = period - 1;
      if (lastAppliedPwmCounts[0] != counts0) {
      if (rxDebug) { Serial.print("[PWM DPO1 "); Serial.print(lastAppliedPwmCounts[0]); Serial.print("->"); Serial.print(counts0); Serial.println("]"); }
      gpt5_out.set_duty_cycle(counts0, CHANNEL_B);  // DPO1
      lastAppliedPwmCounts[0] = counts0;
    }
    if (lastAppliedPwmCounts[1] != counts1) {
      if (rxDebug) { Serial.print("[PWM DPO2 "); Serial.print(lastAppliedPwmCounts[1]); Serial.print("->"); Serial.print(counts1); Serial.println("]"); }
      gpt5_out.set_duty_cycle(counts1, CHANNEL_A);  // DPO2
      lastAppliedPwmCounts[1] = counts1;
    }
  }

  // Channels 2-3: GPT6 (DPO3-4, GTIOC6B/6A)
  if (gpt6_out.is_opened()) {
    uint32_t period = gpt6PeriodCounts;
    if (period < 2U) {
      period = gpt6_out.get_period_raw();
      gpt6PeriodCounts = period;
    }
    if (period < 2U) {
      period = 2U;
    }
    uint32_t counts2 = (period * (uint32_t)newDuty[2]) / 255;
    uint32_t counts3 = (period * (uint32_t)newDuty[3]) / 255;
    if (counts2 >= period) counts2 = period - 1;
    if (counts3 >= period) counts3 = period - 1;
    if (lastAppliedPwmCounts[2] != counts2) {
      if (rxDebug) { Serial.print("[PWM DPO3 "); Serial.print(lastAppliedPwmCounts[2]); Serial.print("->"); Serial.print(counts2); Serial.println("]"); }
      gpt6_out.set_duty_cycle(counts2, CHANNEL_B);  // DPO3
      lastAppliedPwmCounts[2] = counts2;
    }
    if (lastAppliedPwmCounts[3] != counts3) {
      if (rxDebug) { Serial.print("[PWM DPO4 "); Serial.print(lastAppliedPwmCounts[3]); Serial.print("->"); Serial.print(counts3); Serial.println("]"); }
      gpt6_out.set_duty_cycle(counts3, CHANNEL_A);  // DPO4
      lastAppliedPwmCounts[3] = counts3;
    }
  }

  // Channels 4-5: GPT4 (DPO5-6, GTIOC4A/4B)
  if (gpt4_out.is_opened()) {
    uint32_t period = gpt4PeriodCounts;
    if (period < 2U) {
      period = gpt4_out.get_period_raw();
      gpt4PeriodCounts = period;
    }
    if (period < 2U) {
      period = 2U;
    }
    uint32_t counts4 = (period * (uint32_t)newDuty[4]) / 255;
    uint32_t counts5 = (period * (uint32_t)newDuty[5]) / 255;
    if (counts4 >= period) counts4 = period - 1;
    if (counts5 >= period) counts5 = period - 1;
    if (lastAppliedPwmCounts[4] != counts4) {
      if (rxDebug) { Serial.print("[PWM DPO5 "); Serial.print(lastAppliedPwmCounts[4]); Serial.print("->"); Serial.print(counts4); Serial.println("]"); }
      gpt4_out.set_duty_cycle(counts4, CHANNEL_A);  // DPO5
      lastAppliedPwmCounts[4] = counts4;
    }
    if (lastAppliedPwmCounts[5] != counts5) {
      if (rxDebug) { Serial.print("[PWM DPO6 "); Serial.print(lastAppliedPwmCounts[5]); Serial.print("->"); Serial.print(counts5); Serial.println("]"); }
      gpt4_out.set_duty_cycle(counts5, CHANNEL_B);  // DPO6
      lastAppliedPwmCounts[5] = counts5;
    }
  }

  // Channels 6-7: GPIO (DPO7-8, P300/P108)
  // These outputs are active-low at the pin, so logical ON drives LOW.
  uint8_t gpio6Level = (newDuty[6] > 127) ? 0U : 1U;
  uint8_t gpio7Level = (newDuty[7] > 127) ? 0U : 1U;
  if (lastAppliedGpioLevel[0] != gpio6Level) {
    writeGpioOutput(DIGITAL_OUT_PINS[6], gpio6Level != 0U);
    lastAppliedGpioLevel[0] = gpio6Level;
  }
  if (lastAppliedGpioLevel[1] != gpio7Level) {
    writeGpioOutput(DIGITAL_OUT_PINS[7], gpio7Level != 0U);
    lastAppliedGpioLevel[1] = gpio7Level;
  }
}

// Pending config save flag â€” deferred so EEPROM writes never happen inside
// the CAN drain loop (flash writes can block for ms and stall the SW PWM ISR).
static volatile bool pendingConfigSave = false;

static void handleCanRx(const CanMsg &msg) {
  if (serialOverride) {
    return;
  }

  // Snapshot duty/freq so we can detect whether outputs actually changed
  uint8_t  prevDuty[NUM_DIGITAL_OUT];
  uint16_t prevFreq[NUM_DIGITAL_OUT];
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
      for (int _i = 0; _i < NUM_DIGITAL_OUT; _i++) {
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

    canModeBuildTxDiPairFrame(config.canMode,
                              baseId,
                              timerFreq0,
                              period0,
                              high0,
                              has0,
                              timerFreq1,
                              period1,
                              high1,
                              has1,
                              frame);
    if (frame.len > 0) {
      sendCanFrame(frame.id, frame.data, frame.len);
    }
  };

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

static void printDiag() {
  Serial.println("\n========== DIAG ==========");

  // GPT capture timer diagnostics
  struct { FspTimer *timer; CaptureCtx *ctx; uint8_t ch; const char *label; } timers[] = {
    {&gpt0, &gpt0Ctx, 0, "GPT0 (DI5-A, DI4-B)"},
    {&gpt1, &gpt1Ctx, 1, "GPT1 (DI3-A, DI2-B)"},
    {&gpt2, &gpt2Ctx, 2, "GPT2 (DI6-A, DI1-B)"},
    {&gpt3, &gpt3Ctx, 3, "GPT3 (DI8-A, DI7-B)"},
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
  Serial.println("--- HW PWM (GPT4/5/6) ---");
  Serial.print("GPT4 open="); Serial.print(gpt4_out.is_opened() ? "YES" : "NO");
  Serial.print(" period="); Serial.print(gpt4_out.is_opened() ? gpt4_out.get_period_raw() : 0);
  Serial.print("  GPT5 open="); Serial.print(gpt5_out.is_opened() ? "YES" : "NO");
  Serial.print(" period="); Serial.print(gpt5_out.is_opened() ? gpt5_out.get_period_raw() : 0);
  Serial.print("  GPT6 open="); Serial.print(gpt6_out.is_opened() ? "YES" : "NO");
  Serial.print(" period="); Serial.println(gpt6_out.is_opened() ? gpt6_out.get_period_raw() : 0);
  for (int i = 0; i < NUM_DIGITAL_OUT; i++) {
    uint8_t port = DIGITAL_OUT_PINS[i] >> 8;
    uint8_t pin = DIGITAL_OUT_PINS[i] & 0xFF;
    uint32_t pfs = R_PFS->PORT[port].PIN[pin].PmnPFS;
    Serial.print("DPO"); Serial.print(i + 1);
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
  Serial.println("  OUT <ch> <duty%>        - Set output duty 0-100% (1-8)");
  Serial.println("  OUTFREQ <ch> <hz>       - Set output channel PWM frequency (1-8)");
  Serial.println("  CONFIG                 - Print stored configuration");
  Serial.println("  SERIALOVERRIDE ON|OFF  - Enable/disable serial test override");
  Serial.println("  SAFE <ch> <0|1>         - Set safe state for output (1-8)");
  Serial.println("  ACTIVE <ch> <LOW|HIGH>  - Set active state for output (1-8)");
  Serial.println("  DIPULLUP <ch> <0|1>      - Set DI internal pull-up (1-8)");
  Serial.println("  DIPULLUPMASK <hex>       - Set DI pull-up bitmask (bit0=DI1..bit7=DI8)");
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
  Serial.print("  SerialOverride=");
  Serial.println(serialOverride ? "ON" : "OFF");
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
    Serial.print(state ? "HIGH" : "LOW ");
    Serial.print(" | ");

    if (hasP && period > 0 && timerFreq > 0) {
      float freq = static_cast<float>(timerFreq) / static_cast<float>(period);
      Serial.print(freq, 1);
      Serial.print(" Hz  duty=");
      if (hasH) {
        float duty = (static_cast<float>(high) / static_cast<float>(period)) * 100.0f;
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
  Serial.print((gpt4_out.is_opened() && gpt5_out.is_opened() && gpt6_out.is_opened()) ? "OK" : "ERR");
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
    Serial.print("DPO");
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
    if (i < 2) {
      Serial.print("GPT5");
    } else if (i < 4) {
      Serial.print("GPT6");
    } else if (i < 6) {
      Serial.print("GPT4");
    } else {
      Serial.print("GPIO");
    }
    Serial.println();
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
  Serial.print(config.canMode);
  Serial.print(" (");
  Serial.print(canModeName(config.canMode));
  Serial.println(")");
  Serial.print("Serial Override: ");
  Serial.println(serialOverride ? "ON" : "OFF");
  Serial.print("Safe Mask: 0x");
  Serial.println(config.safeMask, HEX);
  Serial.print("Active Mask: 0x");
  Serial.println(config.activeMask, HEX);
  Serial.print("DI Pullup Mask: 0x");
  Serial.println(config.inputPullupMask, HEX);
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
  if (cmd == "DEFAULTS") {
    setDefaults(config);
    saveConfig();
    // Reload output frequencies from fresh defaults
    for (int p = 0; p < 4; p++) {
      outputFreq[p * 2]     = config.outFreqHz[p];
      outputFreq[p * 2 + 1] = config.outFreqHz[p];
    }
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
      serialOverride = true;
      Serial.println("OK: Serial override ON");
      return;
    }
    if (arg == "OFF") {
      serialOverride = false;
      Serial.println("OK: Serial override OFF");
      return;
    }
    Serial.println("ERR: SERIALOVERRIDE ON|OFF");
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
    if (ch < 1 || ch > 8 || hz == 0) {
      Serial.println("ERR: OUTFREQ <ch 1-8> <hz>");
      return;
    }
    // Persist as pair frequency (pairs share EEPROM slot and a single GPT timer).
    // Update both channels of the pair so applyOutputs() picks up the change
    // regardless of which channel in the pair was specified.
    uint8_t pair = (ch - 1) / 2;
    config.outFreqHz[pair] = hz;
    saveConfig();
    outputFreq[pair * 2]     = hz;
    outputFreq[pair * 2 + 1] = hz;
    applyOutputs(outputsInSafeState);
    Serial.print("OK: DPO");
    Serial.print(pair * 2 + 1);
    Serial.print("-");
    Serial.print(pair * 2 + 2);
    Serial.print(" freq=");
    Serial.print(hz);
    Serial.println(" Hz");
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

  if (cmd.startsWith("DIPULLUPMASK")) {
    String arg = getArg(1);
    if (arg.length() == 0) {
      Serial.println("ERR: DIPULLUPMASK <hex>");
      return;
    }
    uint8_t mask = static_cast<uint8_t>(strtol(arg.c_str(), nullptr, 0));
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
    if (ch < 1 || ch > 8 || val > 1) {
      Serial.println("ERR: DIPULLUP <1-8> <0|1>");
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

  if (cmd.startsWith("OUT ") && !cmd.startsWith("OUTFREQ")) {
    uint8_t ch = static_cast<uint8_t>(getArg(1).toInt());
    int pct = getArg(2).toInt();
    if (ch < 1 || ch > 8 || pct < 0 || pct > 100) {
      Serial.println("ERR: OUT <1-8> <0-100>");
      return;
    }
    outputDuty[ch - 1] = static_cast<uint8_t>((pct * 255 + 50) / 100);
    serialOverride = true;  // disable CAN watchdog while using serial
    outputsInSafeState = false;
    lastCanRxMs = millis();
    applyOutputs(false);
    Serial.print("OK: DPO"); Serial.print(ch);
    Serial.print(" = "); Serial.print(pct); Serial.println("%");
    Serial.println("INFO: Serial override ON (CAN output control paused)");
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

// Initialize hardware PWM using GPT4, GPT5, GPT6 for 6 channels + GPIO for 2 channels
static void initHardwarePwm() {
  // Configure output pin modes
  configureGpioOutput(DIGITAL_OUT_PINS[6], false);  // DPO7 - GPIO only
  configureGpioOutput(DIGITAL_OUT_PINS[7], false);  // DPO8 - GPIO only

  // Use per-pair frequencies already loaded from config into outputFreq[].
  // Pairs: DPO1-2 → outputFreq[0] (GPT5), DPO3-4 → outputFreq[2] (GPT6),
  //        DPO5-6 → outputFreq[4] (GPT4).
  uint32_t freqGpt5 = outputFreq[0];
  uint32_t freqGpt6 = outputFreq[2];
  uint32_t freqGpt4 = outputFreq[4];
  
  // GPT5: Channels 0-1 (DPO1-2, P408-P409)
  configureGptPeripheral(DIGITAL_OUT_PINS[0]);  // P408 - DPO1
  configureGptPeripheral(DIGITAL_OUT_PINS[1]);  // P409 - DPO2
  FspTimer::force_use_of_pwm_reserved_timer();
  gpt5_out.begin(TIMER_MODE_PWM, GPT_TIMER, 5, freqGpt5, 50.0f);
  gpt5_out.add_pwm_extended_cfg();
  gpt5_out.enable_pwm_channel(CHANNEL_B);  // DPO1 on GTIOC5B
  gpt5_out.enable_pwm_channel(CHANNEL_A);  // DPO2 on GTIOC5A
  gpt5_out.open();
  gpt5_out.set_duty_cycle(0, CHANNEL_B);  // Initialize to 0% duty
  gpt5_out.set_duty_cycle(0, CHANNEL_A);
  gpt5_out.start();
  gpt5PeriodCounts = gpt5_out.get_period_raw();
  lastAppliedOutputFreqHz[0] = static_cast<uint16_t>(freqGpt5);

  // GPT6: Channels 2-3 (DPO3-4, P410-P411)
  configureGptPeripheral(DIGITAL_OUT_PINS[2]);  // P410 - DPO3
  configureGptPeripheral(DIGITAL_OUT_PINS[3]);  // P411 - DPO4
  FspTimer::force_use_of_pwm_reserved_timer();
  gpt6_out.begin(TIMER_MODE_PWM, GPT_TIMER, 6, freqGpt6, 50.0f);
  gpt6_out.add_pwm_extended_cfg();
  gpt6_out.enable_pwm_channel(CHANNEL_B);  // DPO3 on GTIOC6B
  gpt6_out.enable_pwm_channel(CHANNEL_A);  // DPO4 on GTIOC6A
  gpt6_out.open();
  gpt6_out.set_duty_cycle(0, CHANNEL_B);  // Initialize to 0% duty
  gpt6_out.set_duty_cycle(0, CHANNEL_A);
  gpt6_out.start();
  gpt6PeriodCounts = gpt6_out.get_period_raw();
  lastAppliedOutputFreqHz[1] = static_cast<uint16_t>(freqGpt6);

  // GPT4: Channels 4-5 (DPO5-6, P302-P301)
  configureGptPeripheral(DIGITAL_OUT_PINS[4]);  // P302 - DPO5
  configureGptPeripheral(DIGITAL_OUT_PINS[5]);  // P301 - DPO6
  FspTimer::force_use_of_pwm_reserved_timer();
  gpt4_out.begin(TIMER_MODE_PWM, GPT_TIMER, 4, freqGpt4, 50.0f);
  gpt4_out.add_pwm_extended_cfg();
  gpt4_out.enable_pwm_channel(CHANNEL_A);  // DPO5 on GTIOC4A
  gpt4_out.enable_pwm_channel(CHANNEL_B);  // DPO6 on GTIOC4B
  gpt4_out.open();
  gpt4_out.set_duty_cycle(0, CHANNEL_A);  // Initialize to 0% duty
  gpt4_out.set_duty_cycle(0, CHANNEL_B);
  gpt4_out.start();
  gpt4PeriodCounts = gpt4_out.get_period_raw();
  lastAppliedOutputFreqHz[2] = static_cast<uint16_t>(freqGpt4);

  Serial.print("[HW PWM] GPT5=");
  Serial.print(freqGpt5);
  Serial.print("Hz GPT6=");
  Serial.print(freqGpt6);
  Serial.print("Hz GPT4=");
  Serial.print(freqGpt4);
  Serial.println("Hz");
}

static void initCaptureInputs() {
  // Configure pins for GPT input capture and current pull-up mask
  applyInputPullups();

  initCaptureTimer(gpt0, gpt0Ctx, 0, 4, 3); // GPT0: A=DI5, B=DI4
  initCaptureTimer(gpt1, gpt1Ctx, 1, 2, 1); // GPT1: A=DI3, B=DI2
  initCaptureTimer(gpt2, gpt2Ctx, 2, 5, 0); // GPT2: A=DI6, B=DI1
  initCaptureTimer(gpt3, gpt3Ctx, 3, 7, 6); // GPT3: A=DI8, B=DI7

  // Force-enable NVIC for all capture IRQs and cycle-end IRQs.
  // IRQManager::addTimerCompareCaptureA/B allocates the IELSR slot and ISR
  // vector but does NOT call NVIC_EnableIRQ.  R_GPT_Open is supposed to
  // enable them via r_gpt_enable_irq, but GPT0 capture-A (the very first
  // slot) ends up with NVIC disabled in practice.  Explicitly enabling
  // all capture and cycle-end IRQs here is a safe no-op for already-enabled
  // ones and fixes the GPT0-A case.
  FspTimer *capTimers[] = {&gpt0, &gpt1, &gpt2, &gpt3};
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

  initAdc();

  for (int i = 0; i < NUM_DIGITAL_IN; i++) {
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
  for (int i = 0; i < NUM_DIGITAL_OUT; i++) {
    outputDuty[i] = 0;
    outputFreq[i] = DEFAULT_PWM_FREQ_HZ;
  }
  // Load per-pair frequencies from stored config
  for (int p = 0; p < 4; p++) {
    outputFreq[p * 2]     = config.outFreqHz[p];
    outputFreq[p * 2 + 1] = config.outFreqHz[p];
  }

  initHardwarePwm();

  // Initialize NeoPixel
  neopixel.begin();
  neopixel.setBrightness(BRIGHTNESS);
  neopixel.show();

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
    Serial.println();
    Serial.println("============================================");
    Serial.println("      PT-IO-Mini2  CAN I/O Expander");
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

  handleSerial();
  processCan();
  // Refresh now after processCan() so that time-delta checks below (safe-state
  // timeout, TX rate, LED, monitor) are never older than lastCanRxMs.  If the
  // stale pre-processCan() value were used and processCan() updated lastCanRxMs
  // to a newer millis(), unsigned subtraction (now - lastCanRxMs) would wrap to
  // ~4 billion and trigger the RX-timeout safe-state every single loop.
  now = millis();

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

  if (!outputsInSafeState && !serialOverride && (now - lastCanRxMs > config.rxTimeoutMs)) {
    applyOutputs(true);
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
