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
volatile uint32_t diCapSeq[NUM_DIGITAL_IN];  // incremented on each capture event

// Stale capture detection (main loop)
uint32_t diCapSeqLast[NUM_DIGITAL_IN];
uint32_t diCapTsLast[NUM_DIGITAL_IN];
const uint32_t DI_STALE_TIMEOUT_MS = 500;

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

// Timers - input capture
FspTimer gpt0; // DI4/DI5
FspTimer gpt1; // DI2/DI3
FspTimer gpt2; // DI1/DI6
FspTimer gpt3; // DI7/DI8

// Software PWM tick timer
// 10 kHz — keeps ISR CPU load under 20 % even at 24 MHz ICLK (UNO R4 Minima HOCO).
// At 300 Hz output freq → period = 33 ticks ≈ 3 % duty resolution.
#define SW_PWM_TICK_HZ 10000UL
static volatile int8_t swPwmIrqNum = -1;  // IELSR slot allocated for AGT1

struct SwPwmChannel {
  volatile uint16_t period;    // ticks per cycle  (SW_PWM_TICK_HZ / freq)
  volatile uint16_t highTicks; // ticks HIGH per cycle
};
SwPwmChannel swPwmCh[NUM_DIGITAL_OUT];
uint16_t swPwmCounter[NUM_DIGITAL_OUT]; // ISR-only counter (not volatile — reduces ISR overhead)
volatile uint32_t swPwmTickCount = 0;   // ISR tick counter for diagnostics

// Precomputed PORT register pointers for fast ISR pin toggling
// Uses POSR/PORR (atomic set/reset) — no PFS write-protection needed
struct SwPwmPin {
  volatile uint32_t *pcntr3; // PCNTR3 register (POSR=bits[15:0], PORR=bits[31:16])
  uint32_t setVal;           // value to write to PCNTR3 to set pin HIGH
  uint32_t clrVal;           // value to write to PCNTR3 to set pin LOW
};
SwPwmPin swPwmPins[NUM_DIGITAL_OUT];

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
  // Validate frequency values — corrupted EEPROM can yield garbage
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

  uint8_t idx = 0xFF;
  if (p_args->event == TIMER_EVENT_CAPTURE_A) {
    idx = ctx->idxA;
  } else if (p_args->event == TIMER_EVENT_CAPTURE_B) {
    idx = ctx->idxB;
  }
  if (idx >= NUM_DIGITAL_IN) return;

  bool level = readDigitalIn(idx);
  uint32_t captured = p_args->capture;

  diCapSeq[idx]++;

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
  // Pre-initialize context so callback has valid data as soon as timer starts
  ctx.idxA = idxA;
  ctx.idxB = idxB;
  ctx.maxCounts = 0xFFFFFFFF;
  ctx.timerFreqHz = 0;

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

// Software PWM tick ISR — bare-metal AGT1 underflow handler
// Clears ICU IR + AGT1 underflow flag, then updates all 8 outputs via batched PCNTR3 writes.
static void swPwmIsr() {
  // 1. Clear ICU Interrupt Request bit — without this the IRQ re-fires endlessly
  //    (same as R_BSP_IrqStatusClear in the FSP)
  IRQn_Type irq = R_FSP_CurrentIrqGet();
  R_ICU->IELSR_b[irq].IR = 0U;

  // 2. Clear AGT1 underflow flag (write 0 to TUNDF, preserve TSTART)
  R_AGT1->AGTCR = R_AGT1->AGTCR & ~(R_AGT0_AGTCR_TUNDF_Msk |
                                       R_AGT0_AGTCR_TCMAF_Msk |
                                       R_AGT0_AGTCR_TCMBF_Msk |
                                       R_AGT0_AGTCR_TEDGF_Msk);
  __DSB();  // Ensure writes complete before ISR returns

  swPwmTickCount++;

  // Accumulate POSR/PORR bits per port, then write once per port
  uint32_t p4 = 0, p3 = 0, p1 = 0;

  for (int i = 0; i < NUM_DIGITAL_OUT; i++) {
    uint16_t cnt = swPwmCounter[i] + 1;
    if (cnt >= swPwmCh[i].period) cnt = 0;
    swPwmCounter[i] = cnt;
    if (cnt < swPwmCh[i].highTicks) {
      if (i < 4) p4 |= swPwmPins[i].setVal;
      else if (i < 7) p3 |= swPwmPins[i].setVal;
      else p1 |= swPwmPins[i].setVal;
    } else {
      if (i < 4) p4 |= swPwmPins[i].clrVal;
      else if (i < 7) p3 |= swPwmPins[i].clrVal;
      else p1 |= swPwmPins[i].clrVal;
    }
  }

  *swPwmPins[0].pcntr3 = p4;  // PORT4
  *swPwmPins[4].pcntr3 = p3;  // PORT3
  *swPwmPins[7].pcntr3 = p1;  // PORT1
}

// Update software PWM parameters for all 8 output channels
static void applyOutputs(bool useSafeState) {
  for (int i = 0; i < NUM_DIGITAL_OUT; i++) {
    uint8_t duty = outputDuty[i];
    if (useSafeState) {
      bool safeOn = (config.safeMask >> i) & 0x01;
      duty = safeOn ? 255 : 0;
    }
    // Map 0-255 duty to a 0.0-1.0 fraction, apply polarity
    float dutyFrac = static_cast<float>(duty) / 255.0f;
    bool activeHigh = (config.activeMask >> i) & 0x01;
    if (!activeHigh) dutyFrac = 1.0f - dutyFrac;

    uint16_t freq = outputFreq[i];
    if (freq == 0) freq = DEFAULT_PWM_FREQ_HZ;
    uint16_t period = static_cast<uint16_t>(SW_PWM_TICK_HZ / freq);
    if (period < 2) period = 2;  // minimum 2 ticks for any toggling
    uint16_t high = static_cast<uint16_t>(period * dutyFrac + 0.5f);
    if (duty == 255 && activeHigh)  high = period; // guarantee 100 %
    if (duty == 0   && activeHigh)  high = 0;      // guarantee   0 %
    if (duty == 255 && !activeHigh) high = 0;
    if (duty == 0   && !activeHigh) high = period;

    noInterrupts();
    swPwmCh[i].period    = period;
    swPwmCh[i].highTicks = high;
    if (swPwmCounter[i] >= period) swPwmCounter[i] = 0;
    interrupts();
  }
}

static void handleCanRx(const CanMsg &msg) {
  bool changed = false;
  bool handled = canModeHandleRx(config.canMode,
                                 msg,
                                 config.rxBaseId,
                                 DEFAULT_PWM_FREQ_HZ,
                                 outputFreq,
                                 outputDuty,
                                 config.safeMask,
                                 config.activeMask,
                                 changed);

  if (!handled) {
    return;
  }

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
  // Zero-init the whole struct — CanMsg has fields beyond id/data_length/data
  // (e.g. rtr, extended flags) that the RA4M1 CAN controller reads.
  // Without this, those fields are garbage on calls 2-N and the controller
  // rejects or mangles the frame.
  CanMsg tx;
  memset(&tx, 0, sizeof(tx));
  tx.id = id;
  tx.data_length = len;
  memcpy(tx.data, data, len);

  // Retry for up to 10 ms to allow the TX mailbox to drain between back-to-back frames.
  const uint32_t txTimeoutUs = 10000UL;
  uint32_t startUs = micros();
  bool ok = false;
  do {
    ok = CAN.write(tx);
    if (!ok) {
      delayMicroseconds(100);
    }
  } while (!ok && (micros() - startUs < txTimeoutUs));

  if (ok) {
    if (canTxCount != 0xFFFF) {
      canTxCount++;
    }
    // Brief gap so the controller starts transmitting this frame before the
    // next CAN.write() call — otherwise back-to-back writes can clash on the
    // RA4M1 single TX mailbox even when write() returns true.
    delayMicroseconds(150);
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
    gpt_extended_cfg_t *ext = static_cast<gpt_extended_cfg_t *>(const_cast<void *>(t.timer->get_cfg()->p_extend));
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

  // SW PWM diagnostics — include AGT1 register dump
  Serial.print("--- SW PWM: tickCount=");
  Serial.print(swPwmTickCount);
  Serial.print("  IRQslot="); Serial.print(swPwmIrqNum);
  Serial.println(" ---");
  Serial.print("AGT1: AGT=0x"); Serial.print(R_AGT1->AGT, HEX);
  Serial.print(" AGTCR=0x"); Serial.print(R_AGT1->AGTCR, HEX);
  Serial.print(" AGTMR1=0x"); Serial.print(R_AGT1->AGTMR1, HEX);
  Serial.print(" AGTMR2=0x"); Serial.print(R_AGT1->AGTMR2, HEX);
  if (swPwmIrqNum >= 0) {
    Serial.print(" IELSR=0x"); Serial.print(R_ICU->IELSR[swPwmIrqNum], HEX);
    Serial.print(" NVIC="); Serial.print(NVIC_GetEnableIRQ((IRQn_Type)swPwmIrqNum) ? "EN" : "DIS");
  }
  Serial.println();
  for (int i = 0; i < NUM_DIGITAL_OUT; i++) {
    uint8_t port = DIGITAL_OUT_PINS[i] >> 8;
    uint8_t pin = DIGITAL_OUT_PINS[i] & 0xFF;
    uint32_t pfs = R_PFS->PORT[port].PIN[pin].PmnPFS;
    Serial.print("DPO"); Serial.print(i + 1);
    Serial.print(" ("); Serial.print(DIGITAL_OUT_PORT_NAMES[i]);
    Serial.print("): PFS=0x"); Serial.print(pfs, HEX);
    Serial.print(" PDR="); Serial.print((pfs >> 2) & 1);
    Serial.print(" PMR="); Serial.print((pfs >> 16) & 1);
    Serial.print(" period="); Serial.print(swPwmCh[i].period);
    Serial.print(" high="); Serial.print(swPwmCh[i].highTicks);
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
  Serial.println("  SAFE <ch> <0|1>         - Set safe state for output (1-8)");
  Serial.println("  ACTIVE <ch> <LOW|HIGH>  - Set active state for output (1-8)");
  Serial.println("  DEFAULTS                - Reset all config to factory defaults");
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
    Serial.print(" [cap=");
    Serial.print(diCapSeq[i]);
    Serial.print("]");
    Serial.println();
  }

  Serial.println();
  Serial.println("--- Digital Outputs ---");
  Serial.print("SafeState=");
  Serial.print(outputsInSafeState ? "YES" : "NO");
  Serial.print("  lastCanRxMs=");
  Serial.print(lastCanRxMs);
  Serial.print("  now=");
  Serial.print(millis());
  Serial.print("  timeout=");
  Serial.print(config.rxTimeoutMs);
  Serial.print("  swPwmTicks=");
  Serial.println(swPwmTickCount);
  for (int i = 0; i < NUM_DIGITAL_OUT; i++) {
    bool state = readDigitalOut(i);
    uint8_t port = DIGITAL_OUT_PINS[i] >> 8;
    uint8_t pin = DIGITAL_OUT_PINS[i] & 0xFF;
    uint32_t pfs = R_PFS->PORT[port].PIN[pin].PmnPFS;
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
    Serial.print(" Hz, PDR=");
    Serial.print((pfs >> 2) & 1);
    Serial.print(" hi=");
    Serial.print(swPwmCh[i].highTicks);
    Serial.print("/");
    Serial.print(swPwmCh[i].period);
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
    // Persist as pair frequency (pairs share EEPROM slot)
    uint8_t pair = (ch - 1) / 2;
    config.outFreqHz[pair] = hz;
    saveConfig();
    outputFreq[ch - 1] = hz;
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

static void initSwPwm() {
  // Configure ALL 8 output pins as plain GPIO outputs (LOW initially)
  // and precompute PORT register pointers for fast ISR toggling
  for (int i = 0; i < NUM_DIGITAL_OUT; i++) {
    configureGpioOutput(DIGITAL_OUT_PINS[i], false);
    swPwmCh[i].period    = static_cast<uint16_t>(SW_PWM_TICK_HZ / DEFAULT_PWM_FREQ_HZ);
    swPwmCh[i].highTicks = 0;
    swPwmCounter[i]      = 0;

    // PORT registers are spaced 0x20 bytes apart from R_PORT0
    uint8_t port = DIGITAL_OUT_PINS[i] >> 8;
    uint8_t pin  = DIGITAL_OUT_PINS[i] & 0xFF;
    R_PORT0_Type *portReg = (R_PORT0_Type *)((uintptr_t)R_PORT0 + port * 0x20u);
    swPwmPins[i].pcntr3 = &portReg->PCNTR3;
    uint32_t pinBit = (1u << pin);
    swPwmPins[i].setVal = pinBit;              // POSR = bits[15:0]
    swPwmPins[i].clrVal = pinBit << 16;        // PORR = bits[31:16]
  }

  // ---- Bare-metal AGT1 configuration (bypasses FspTimer entirely) ----
  // Step 1: Enable AGT1 module (clear MSTPCRD.MSTPD2)
  R_MSTP->MSTPCRD &= ~(1u << 2);

  // Step 2: Stop AGT1 if running
  R_AGT1->AGTCR = 0x04;  // TSTOP=1 → force stop, counter -> 0xFFFF
  while (R_AGT1->AGTCR_b.TCSTF) {}  // Wait for stop to take effect

  // Step 3: Configure count source and mode
  //   AGTMR1: TCK[6:4]=000 (PCLKB), TMOD[2:0]=000 (timer mode)
  R_AGT1->AGTMR1 = 0x00;
  //   AGTMR2: CKS[2:0]=000 (no sub-division), LPM=0
  R_AGT1->AGTMR2 = 0x00;

  // Step 4: Set reload period
  //   AGT is a 16-bit down-counter. It counts from AGT → 0, underflows, reloads.
  //   Period = PCLKB / SW_PWM_TICK_HZ
  const uint32_t pclkb = R_FSP_SystemClockHzGet(FSP_PRIV_CLOCK_PCLKB);
  uint16_t reload = static_cast<uint16_t>(pclkb / SW_PWM_TICK_HZ - 1);
  R_AGT1->AGT = reload;
  Serial.print("[SWPWM] PCLKB="); Serial.print(pclkb);
  Serial.print(" reload="); Serial.println(reload);

  // Step 5: Disable output pins / compare match (we only want underflow IRQ)
  R_AGT1->AGTIOC  = 0x00;
  R_AGT1->AGTCMSR = 0x00;
  R_AGT1->AGTCMA  = 0xFFFF;
  R_AGT1->AGTCMB  = 0xFFFF;

  // Step 6: Allocate IELSR slot for AGT1_INT and install our ISR
  //   Use the IRQManager’s last_interrupt_index to get a free slot.
  //   We can’t call IRQManager directly (private), but we CAN write IELSR
  //   and the vector table manually, same way IRQManager does.
  volatile uint32_t *irqVec = (volatile uint32_t *)SCB->VTOR;
  // Find a free IELSR slot (scan from 31 DOWN  IRQManager fills from 0 up)
  int slot = -1;
  for (int i = 31; i >= 0; i--) {
    if (R_ICU->IELSR[i] == 0) {
      slot = i;
      break;
    }
  }
  if (slot >= 0) {
    // Link AGT1_INT event (33 = 0x21) to this IELSR slot
    R_ICU->IELSR[slot] = 0x021;  // ELC_EVENT_AGT1_INT
    // Install our bare ISR into the vector table
    // VTOR points to the full vector table; programmable IRQs start at index 16 (after CM4 exceptions)
    irqVec[16 + slot] = (uint32_t)swPwmIsr;
    // Configure NVIC: set priority and enable
    NVIC_SetPriority((IRQn_Type)slot, 14);
    NVIC_ClearPendingIRQ((IRQn_Type)slot);
    NVIC_EnableIRQ((IRQn_Type)slot);
    swPwmIrqNum = (int8_t)slot;
    Serial.print("[SWPWM] IRQ slot="); Serial.print(slot);
    Serial.print(" IELSR=0x"); Serial.print(R_ICU->IELSR[slot], HEX);
    Serial.println(" NVIC=EN");
  } else {
    Serial.println("[SWPWM] ERR: no free IELSR slot!");
  }

  // Step 7: Clear all flags and start the counter
  R_AGT1->AGTCR = 0x01;  // TSTART=1, all flags cleared
  Serial.print("[SWPWM] AGTCR=0x"); Serial.print(R_AGT1->AGTCR, HEX);
  Serial.print(" TCSTF="); Serial.println(R_AGT1->AGTCR_b.TCSTF);
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

  // Force-enable NVIC for all capture IRQs.
  // IRQManager::addTimerCompareCaptureA/B allocates the IELSR slot and ISR
  // vector but does NOT call NVIC_EnableIRQ.  R_GPT_Open is supposed to
  // enable them via r_gpt_enable_irq, but GPT0 capture-A (the very first
  // slot) ends up with NVIC disabled in practice.  Explicitly enabling
  // all capture IRQs here is a safe no-op for already-enabled ones and
  // fixes the GPT0-A case.
  FspTimer *capTimers[] = {&gpt0, &gpt1, &gpt2, &gpt3};
  for (auto *t : capTimers) {
    auto *ext = static_cast<gpt_extended_cfg_t *>(const_cast<void *>(t->get_cfg()->p_extend));
    if (ext->capture_a_irq >= 0) NVIC_EnableIRQ((IRQn_Type)ext->capture_a_irq);
    if (ext->capture_b_irq >= 0) NVIC_EnableIRQ((IRQn_Type)ext->capture_b_irq);
  }
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
    diCapSeq[i] = 0;
    diCapSeqLast[i] = 0;
    diCapTsLast[i] = 0;
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

  initSwPwm();
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
      diHasPeriod[i] = false;
      diHasHigh[i] = false;
      diPeriodCounts[i] = 0;
      diHighCounts[i] = 0;
      diLastRise[i] = 0;
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
