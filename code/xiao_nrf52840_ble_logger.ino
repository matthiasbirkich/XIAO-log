#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <bluefruit.h>
#include <SPI.h>
#include <Adafruit_SPIFlash.h>
#include <SdFat.h>
#include <RTClib.h>
#include <LSM6DS3.h>
#include <nrf.h>
#include <nrf_gpio.h>
#include <nrf_sdm.h>

// =================================================================================
// Underwater Logger - XIAO nRF52840 Sense - v24 CLEAN battery + deployment metadata
//
// Board:
//   Seeed nRF52 Boards -> Seeed XIAO nRF52840 Sense
//   Non-mbed core, because Bluefruit is used.
//
// Hardware:
//   XIAO Logger HAT with PCF8563 RTC at I2C address 0x51
//   RTC INT -> D0
//   Reed switch -> D1 to GND
//   I2C SDA -> D4
//   I2C SCL -> D5
//   Sensor rail enable -> D10
//   Battery divider -> A3
//
// Design:
//   - No Wire.begin() is used.
//   - All RTC/sensor I2C access uses timeout-safe bit-banged I2C.
//   - Flash is accessed via SPI on the QSPI pins, as tested.
//   - RTC alarm wake and reed wake are configured directly with nRF52 GPIO SENSE.
//   - BLE service is text-based; CSV is sent on A002. No binary DUMP needed.
//
// BLE:
//   Name: UW52840-CLEAN
//   Service: A000
//   Command characteristic: A001
//   Status/CSV characteristic: A002
//
// Useful commands:
//   HELP
//   STATUS
//   RTC_CHECK
//   I2C_STATE
//   SET_TIME=YYYY-MM-DD HH:MM:SS
//   GET_TIME
//   SET_INTERVAL_MIN=N
//   GET_INTERVAL
//   MEASURE_NOW
//   START
//   STOP
//   SLEEP
//   COUNT
//   INFO
//   CSV
//   CLEAR
//
// Recommended test:
//   SET_TIME=2026-04-30 12:00:00
//   CLEAR
//   MEASURE_NOW
//   COUNT
//   CSV
//   SET_INTERVAL_MIN=1
//   START
//   Disconnect or send SLEEP
//   Wait 2-3 minutes
//   Wake with magnet
//   COUNT
// =================================================================================

// -----------------------------
// Pins and addresses
// -----------------------------
static constexpr uint8_t PIN_I2C_SDA   = D4;
static constexpr uint8_t PIN_I2C_SCL   = D5;
static constexpr uint8_t PIN_SENSOR_EN = D10;
static constexpr uint8_t PIN_RTC_INT   = D0;
static constexpr uint8_t PIN_REED      = D1;
static constexpr uint8_t PIN_BAT_ADC   = A3;

// Direct nRF52 GPIO numbers for XIAO nRF52840.
static constexpr uint32_t NRF_PIN_RTC_INT_D0 = 2;  // D0 = P0.02
static constexpr uint32_t NRF_PIN_REED_D1    = 3;  // D1 = P0.03

// XIAO onboard RGB LED pins, active-low.
static constexpr uint32_t PIN_RGB_RED   = 26; // P0.26
static constexpr uint32_t PIN_RGB_GREEN = 30; // P0.30
static constexpr uint32_t PIN_RGB_BLUE  = 6;  // P0.06

static constexpr uint8_t PCF8563_ADDR = 0x51;
static constexpr uint8_t SHT40_ADDR   = 0x44;
static constexpr uint8_t BH1750_ADDR  = 0x23;
static constexpr uint8_t IMU_ADDR     = 0x6A;

// -----------------------------
// Config and timing
// -----------------------------
static constexpr uint32_t DEFAULT_INTERVAL_MIN = 30;
static constexpr uint32_t SERVICE_TIMEOUT_MS   = 180000; // used only when disconnected
static constexpr uint32_t DISCONNECTED_SLEEP_DELAY_MS = 8000;

static constexpr float BATTERY_ADC_REF_V = 3.30f;
static constexpr float BATTERY_DIVIDER_RATIO = 2.0f;
static constexpr float BATTERY_LOW_VOLTAGE = 3.50f;

// -----------------------------
// BLE
// -----------------------------
BLEService        loggerService(0xA000);
BLECharacteristic cmdChar(0xA001);
BLECharacteristic statusChar(0xA002);

static constexpr char BLE_NAME[] = "UW52840-CLEAN24";

// Onboard XIAO nRF52840 Sense IMU.
// The Seeed_Arduino_LSM6DS3 library maps the onboard IMU to Wire1 internally
// for TARGET_SEEED_XIAO_NRF52840_SENSE, so it must not be read via the
// external D4/D5 bit-banged I2C bus.
LSM6DS3 g_imu(I2C_MODE, 0x6A);
bool g_imuReady = false;

static constexpr size_t STATUS_MAX_LEN = 200;
static constexpr size_t CMD_MAX_LEN    = 96;

bool g_bleStarted = false;
bool g_bleConnected = false;
uint32_t g_lastBleActivityMs = 0;

// -----------------------------
// Flash
// -----------------------------
static const SPIFlash_Device_t XIAO_P25Q16H_DEVICE = {
  .total_size = (1UL << 21),
  .start_up_time_us = 10000,
  .manufacturer_id = 0x85,
  .memory_type = 0x60,
  .capacity = 0x15,
  .max_clock_speed_mhz = 8,
  .quad_enable_bit_mask = 0x02,
  .has_sector_protection = 1,
  .supports_fast_read = 1,
  .supports_qspi = 1,
  .supports_qspi_writes = 1,
  .write_status_register_split = 1,
  .single_status_byte = 0,
  .is_fram = 0,
};

SPIClass flashSPI(NRF_SPIM2, PIN_QSPI_IO1, PIN_QSPI_SCK, PIN_QSPI_IO0); // MISO, SCK, MOSI
Adafruit_FlashTransport_SPI flashTransport(PIN_QSPI_CS, flashSPI);
Adafruit_SPIFlash flash(&flashTransport);
FatFileSystem fatfs;

bool g_flashReady = false;
bool g_fsReady = false;

static constexpr const char* LOG_FILE = "/log.bin";
static constexpr const char* CFG_FILE = "/config.bin";
static constexpr const char* META_FILE = "/meta.txt";

static constexpr uint32_t LOG_MAGIC   = 0x554C4734UL; // ULG4
static constexpr uint16_t LOG_VERSION = 3;
static constexpr uint32_t CFG_MAGIC   = 0x43464731UL; // CFG1
static constexpr uint16_t CFG_VERSION = 1;

// -----------------------------
// Data types
// -----------------------------
enum WakeCode : uint8_t {
  WAKE_UNKNOWN   = 0,
  WAKE_RTC_ALARM = 1,
  WAKE_REED      = 2,
  WAKE_POWERUP   = 3,
  WAKE_MANUAL    = 4
};

struct RuntimeConfig {
  uint32_t intervalMin = DEFAULT_INTERVAL_MIN;
  bool loggingEnabled = false;
  bool rtcValid = false;
};

RuntimeConfig g_cfg;

struct DeploymentMeta {
  String loggerId = "";
  bool hasSalinity = false;
  bool hasLat = false;
  bool hasLon = false;
  float salinityPsu = NAN;
  float latDeg = NAN;
  float lonDeg = NAN;
};

DeploymentMeta g_meta;

#pragma pack(push, 1)
struct ConfigRecord {
  uint32_t magic;
  uint16_t version;
  uint16_t intervalMin;
  uint8_t  loggingEnabled;
  uint8_t  rtcValid;
  uint8_t  reserved[6];
};

struct BinaryFileHeader {
  uint32_t magic;
  uint16_t version;
  uint16_t headerSize;
  uint16_t recordSize;
  uint16_t reserved;
};

struct LogRecord {
  uint32_t unixTime;
  int16_t  tempC_x100;
  uint8_t  bhMode;
  uint8_t  bhMtreg;
  uint16_t bhRaw;
  uint32_t bhLux_mlx;
  int16_t  ax_mg;
  int16_t  ay_mg;
  int16_t  az_mg;
  uint16_t vbat_mV;
  uint8_t  wakeCode;
  uint8_t  flags;

  // Relative humidity from SHT40 in %RH * 100.
  // INT16_MIN means invalid/not available.
  int16_t  rhPct_x100;
};
#pragma pack(pop)

static_assert(sizeof(ConfigRecord) == 16, "Unexpected ConfigRecord size");
static_assert(sizeof(BinaryFileHeader) == 12, "Unexpected BinaryFileHeader size");
static_assert(sizeof(LogRecord) == 26, "Unexpected LogRecord size");

struct Measurement {
  bool rtcOk = false;
  bool shtOk = false;
  bool bhOk = false;
  bool imuOk = false;

  uint32_t unixTime = 0;
  float tempC = NAN;
  float rhPct = NAN;
  float vbat = NAN;

  uint8_t bhMode = 255;
  uint8_t bhMtreg = 0;
  uint16_t bhRaw = 0;
  float bhLux = NAN;

  float ax_mg = NAN;
  float ay_mg = NAN;
  float az_mg = NAN;

  WakeCode wakeCode = WAKE_UNKNOWN;
};

// =================================================================================
// RGB LED
// =================================================================================
void rgbOff() {
  nrf_gpio_pin_set(PIN_RGB_RED);
  nrf_gpio_pin_set(PIN_RGB_GREEN);
  nrf_gpio_pin_set(PIN_RGB_BLUE);
}

void rgbRed() {
  nrf_gpio_pin_clear(PIN_RGB_RED);
  nrf_gpio_pin_set(PIN_RGB_GREEN);
  nrf_gpio_pin_set(PIN_RGB_BLUE);
}

void rgbGreen() {
  nrf_gpio_pin_set(PIN_RGB_RED);
  nrf_gpio_pin_clear(PIN_RGB_GREEN);
  nrf_gpio_pin_set(PIN_RGB_BLUE);
}

void rgbBlue() {
  nrf_gpio_pin_set(PIN_RGB_RED);
  nrf_gpio_pin_set(PIN_RGB_GREEN);
  nrf_gpio_pin_clear(PIN_RGB_BLUE);
}

void rgbOrange() {
  nrf_gpio_pin_clear(PIN_RGB_RED);
  nrf_gpio_pin_clear(PIN_RGB_GREEN);
  nrf_gpio_pin_set(PIN_RGB_BLUE);
}

void blinkColor(void (*color)(), uint8_t n, uint16_t onMs = 120, uint16_t offMs = 120) {
  for (uint8_t i = 0; i < n; i++) {
    color();
    delay(onMs);
    rgbOff();
    delay(offMs);
  }
}

// =================================================================================
// Utility
// =================================================================================
String trimCopy(String s) {
  s.trim();
  return s;
}

bool startsWithIgnoreCase(const String& s, const String& prefix) {
  if (s.length() < prefix.length()) return false;
  for (size_t i = 0; i < prefix.length(); i++) {
    if (tolower((unsigned char)s[i]) != tolower((unsigned char)prefix[i])) return false;
  }
  return true;
}

bool parseUInt32(const String& s, uint32_t& out) {
  if (s.length() == 0) return false;
  uint32_t v = 0;
  for (size_t i = 0; i < s.length(); i++) {
    if (!isdigit((unsigned char)s[i])) return false;
    uint32_t nv = v * 10UL + (uint32_t)(s[i] - '0');
    if (nv < v) return false;
    v = nv;
  }
  out = v;
  return true;
}

uint8_t toBCD(uint8_t v) {
  return (uint8_t)(((v / 10) << 4) | (v % 10));
}

uint8_t fromBCD(uint8_t v) {
  return (uint8_t)(((v >> 4) * 10) + (v & 0x0F));
}

bool parseDateTimeText(const String& input, DateTime& out) {
  String s = input;
  s.trim();

  // Robust input formats for nRF Connect / copy-paste:
  //   2026-04-30 12:34:56
  //   2026-04-30T12:34:56
  //   2026-04-30 12:34
  //   30.04.2026 12:34:56
  //   30.04.2026 12:34
  // Quotation marks are ignored.
  s.replace("T", " ");
  s.replace("t", " ");
  s.replace("\"", "");
  s.replace("'", "");
  s.trim();

  int Y = 0, M = 0, D = 0, h = 0, m = 0, sec = 0;

  if (sscanf(s.c_str(), "%d-%d-%d %d:%d:%d", &Y, &M, &D, &h, &m, &sec) == 6) {
    // ISO with seconds.
  }
  else if (sscanf(s.c_str(), "%d-%d-%d %d:%d", &Y, &M, &D, &h, &m) == 5) {
    sec = 0;
  }
  else if (sscanf(s.c_str(), "%d.%d.%d %d:%d:%d", &D, &M, &Y, &h, &m, &sec) == 6) {
    // German date with seconds.
  }
  else if (sscanf(s.c_str(), "%d.%d.%d %d:%d", &D, &M, &Y, &h, &m) == 5) {
    sec = 0;
  }
  else {
    return false;
  }

  if (Y < 2020 || Y > 2099 || M < 1 || M > 12 || D < 1 || D > 31 ||
      h < 0 || h > 23 || m < 0 || m > 59 || sec < 0 || sec > 59) {
    return false;
  }

  out = DateTime((uint16_t)Y, (uint8_t)M, (uint8_t)D, (uint8_t)h, (uint8_t)m, (uint8_t)sec);
  return true;
}

String wakeToString(WakeCode w) {
  switch (w) {
    case WAKE_RTC_ALARM: return "rtc";
    case WAKE_REED: return "reed";
    case WAKE_POWERUP: return "powerup";
    case WAKE_MANUAL: return "manual";
    default: return "unknown";
  }
}

float readBatteryVoltage() {
  // XIAO Logger HAT: D10 enables the voltage divider, and the halved
  // battery voltage is available on A3. Multiply by 2 to recover VBAT.
  bool railWasOn = (digitalRead(PIN_SENSOR_EN) == HIGH);

  if (!railWasOn) {
    digitalWrite(PIN_SENSOR_EN, HIGH);
    delay(80);
  }

  uint32_t sum = 0;
  const uint8_t n = 8;

  for (uint8_t i = 0; i < n; i++) {
    sum += analogRead(PIN_BAT_ADC);
    delay(2);
  }

  if (!railWasOn) {
    digitalWrite(PIN_SENSOR_EN, LOW);
  }

  float raw = (float)sum / (float)n;
  return (raw / 4095.0f) * BATTERY_ADC_REF_V * BATTERY_DIVIDER_RATIO;
}

void sensorRail(bool on) {
  digitalWrite(PIN_SENSOR_EN, on ? HIGH : LOW);
  if (on) delay(120);
}

// =================================================================================
// Timeout-safe bit-banged I2C
// =================================================================================
void i2cReleaseSDA() { pinMode(PIN_I2C_SDA, INPUT_PULLUP); }
void i2cReleaseSCL() { pinMode(PIN_I2C_SCL, INPUT_PULLUP); }
void i2cPullSDA() { pinMode(PIN_I2C_SDA, OUTPUT); digitalWrite(PIN_I2C_SDA, LOW); }
void i2cPullSCL() { pinMode(PIN_I2C_SCL, OUTPUT); digitalWrite(PIN_I2C_SCL, LOW); }

bool i2cWaitSCLHigh(uint16_t timeoutUs = 1000) {
  uint32_t start = micros();
  while (digitalRead(PIN_I2C_SCL) == LOW) {
    if ((uint32_t)(micros() - start) > timeoutUs) return false;
  }
  return true;
}

bool i2cClockHigh() {
  i2cReleaseSCL();
  if (!i2cWaitSCLHigh()) return false;
  delayMicroseconds(5);
  return true;
}

void i2cClockLow() {
  i2cPullSCL();
  delayMicroseconds(5);
}

void i2cBusInit() {
  sensorRail(true);

  i2cReleaseSDA();
  i2cReleaseSCL();
  delayMicroseconds(20);

  // Recover a stuck slave.
  for (uint8_t i = 0; i < 9; i++) {
    if (digitalRead(PIN_I2C_SDA) == HIGH) break;
    i2cClockLow();
    i2cClockHigh();
  }

  // STOP condition.
  i2cPullSDA();
  delayMicroseconds(5);
  i2cClockHigh();
  i2cReleaseSDA();
  delayMicroseconds(5);
}

const char* i2cStateText() {
  i2cBusInit();
  i2cReleaseSDA();
  i2cReleaseSCL();
  delayMicroseconds(20);

  bool sda = digitalRead(PIN_I2C_SDA);
  bool scl = digitalRead(PIN_I2C_SCL);

  if (sda && scl) return "OK BUS";
  if (!sda && !scl) return "ERR SDA SCL";
  if (!sda) return "ERR SDA";
  if (!scl) return "ERR SCL";
  return "ERR BUS";
}

bool i2cStart() {
  i2cReleaseSDA();
  if (!i2cClockHigh()) return false;
  if (digitalRead(PIN_I2C_SDA) == LOW) return false;
  i2cPullSDA();
  delayMicroseconds(5);
  i2cClockLow();
  return true;
}

bool i2cStop() {
  i2cPullSDA();
  delayMicroseconds(5);
  if (!i2cClockHigh()) return false;
  i2cReleaseSDA();
  delayMicroseconds(5);
  return true;
}

bool i2cWriteByte(uint8_t b) {
  for (uint8_t mask = 0x80; mask; mask >>= 1) {
    if (b & mask) i2cReleaseSDA();
    else i2cPullSDA();

    if (!i2cClockHigh()) return false;
    i2cClockLow();
  }

  i2cReleaseSDA();
  if (!i2cClockHigh()) return false;
  bool ack = (digitalRead(PIN_I2C_SDA) == LOW);
  i2cClockLow();
  return ack;
}

uint8_t i2cReadByte(bool ack) {
  uint8_t value = 0;
  i2cReleaseSDA();

  for (uint8_t i = 0; i < 8; i++) {
    value <<= 1;
    if (i2cClockHigh()) {
      if (digitalRead(PIN_I2C_SDA)) value |= 1;
    }
    i2cClockLow();
  }

  if (ack) i2cPullSDA();
  else i2cReleaseSDA();

  i2cClockHigh();
  i2cClockLow();
  i2cReleaseSDA();

  return value;
}

bool i2cAddressAck(uint8_t addr7) {
  i2cBusInit();
  if (!i2cStart()) return false;
  bool ack = i2cWriteByte((addr7 << 1) | 0);
  i2cStop();
  return ack;
}

bool i2cWriteReg(uint8_t addr7, uint8_t reg, uint8_t value) {
  i2cBusInit();

  if (!i2cStart()) return false;
  if (!i2cWriteByte((addr7 << 1) | 0)) { i2cStop(); return false; }
  if (!i2cWriteByte(reg)) { i2cStop(); return false; }
  if (!i2cWriteByte(value)) { i2cStop(); return false; }

  return i2cStop();
}

bool i2cReadReg(uint8_t addr7, uint8_t reg, uint8_t& value) {
  i2cBusInit();

  if (!i2cStart()) return false;
  if (!i2cWriteByte((addr7 << 1) | 0)) { i2cStop(); return false; }
  if (!i2cWriteByte(reg)) { i2cStop(); return false; }

  if (!i2cStart()) { i2cStop(); return false; }
  if (!i2cWriteByte((addr7 << 1) | 1)) { i2cStop(); return false; }

  value = i2cReadByte(false);
  return i2cStop();
}

bool i2cReadRegs(uint8_t addr7, uint8_t reg, uint8_t* buf, uint8_t len) {
  i2cBusInit();

  if (!i2cStart()) return false;
  if (!i2cWriteByte((addr7 << 1) | 0)) { i2cStop(); return false; }
  if (!i2cWriteByte(reg)) { i2cStop(); return false; }

  if (!i2cStart()) { i2cStop(); return false; }
  if (!i2cWriteByte((addr7 << 1) | 1)) { i2cStop(); return false; }

  for (uint8_t i = 0; i < len; i++) {
    buf[i] = i2cReadByte(i < (len - 1));
  }

  return i2cStop();
}

bool i2cWriteCommand(uint8_t addr7, uint8_t cmd) {
  i2cBusInit();

  if (!i2cStart()) return false;
  if (!i2cWriteByte((addr7 << 1) | 0)) { i2cStop(); return false; }
  if (!i2cWriteByte(cmd)) { i2cStop(); return false; }

  return i2cStop();
}

bool i2cReadBytes(uint8_t addr7, uint8_t* buf, uint8_t len) {
  i2cBusInit();

  if (!i2cStart()) return false;
  if (!i2cWriteByte((addr7 << 1) | 1)) { i2cStop(); return false; }

  for (uint8_t i = 0; i < len; i++) {
    buf[i] = i2cReadByte(i < (len - 1));
  }

  return i2cStop();
}

// =================================================================================
// PCF8563 RTC
// =================================================================================
bool rtcReadReg(uint8_t reg, uint8_t& value) {
  return i2cReadReg(PCF8563_ADDR, reg, value);
}

bool rtcWriteReg(uint8_t reg, uint8_t value) {
  return i2cWriteReg(PCF8563_ADDR, reg, value);
}

bool rtcBegin() {
  return i2cAddressAck(PCF8563_ADDR);
}

bool rtcNow(DateTime& out) {
  i2cBusInit();

  if (!i2cStart()) return false;
  if (!i2cWriteByte((PCF8563_ADDR << 1) | 0)) { i2cStop(); return false; }
  if (!i2cWriteByte(0x02)) { i2cStop(); return false; }

  if (!i2cStart()) { i2cStop(); return false; }
  if (!i2cWriteByte((PCF8563_ADDR << 1) | 1)) { i2cStop(); return false; }

  uint8_t secReg  = i2cReadByte(true);
  uint8_t minReg  = i2cReadByte(true);
  uint8_t hourReg = i2cReadByte(true);
  uint8_t dayReg  = i2cReadByte(true);
  (void)i2cReadByte(true); // weekday
  uint8_t monReg  = i2cReadByte(true);
  uint8_t yearReg = i2cReadByte(false);
  if (!i2cStop()) return false;

  if (secReg & 0x80) return false; // voltage-low / time invalid

  uint8_t sec  = fromBCD(secReg & 0x7F);
  uint8_t min  = fromBCD(minReg & 0x7F);
  uint8_t hour = fromBCD(hourReg & 0x3F);
  uint8_t day  = fromBCD(dayReg & 0x3F);
  uint8_t mon  = fromBCD(monReg & 0x1F);
  uint16_t year = 2000 + fromBCD(yearReg);

  if (mon < 1 || mon > 12 || day < 1 || day > 31 || hour > 23 || min > 59 || sec > 59) return false;

  out = DateTime(year, mon, day, hour, min, sec);
  return true;
}

bool rtcAdjust(const DateTime& dt) {
  bool ok = true;

  ok &= rtcWriteReg(0x00, 0x00);
  ok &= rtcWriteReg(0x01, 0x00);

  ok &= rtcWriteReg(0x02, toBCD((uint8_t)dt.second()) & 0x7F);
  ok &= rtcWriteReg(0x03, toBCD((uint8_t)dt.minute()) & 0x7F);
  ok &= rtcWriteReg(0x04, toBCD((uint8_t)dt.hour()) & 0x3F);
  ok &= rtcWriteReg(0x05, toBCD((uint8_t)dt.day()) & 0x3F);
  ok &= rtcWriteReg(0x06, toBCD((uint8_t)dt.dayOfTheWeek()) & 0x07);
  ok &= rtcWriteReg(0x07, toBCD((uint8_t)dt.month()) & 0x1F);
  ok &= rtcWriteReg(0x08, toBCD((uint8_t)(dt.year() % 100)));

  return ok;
}

bool rtcIntIsLow() {
  pinMode(PIN_RTC_INT, INPUT_PULLUP);
  delay(5);
  return digitalRead(PIN_RTC_INT) == LOW;
}

bool rtcClearAlarmFlag() {
  uint8_t ctrl2 = 0;
  if (!rtcReadReg(0x01, ctrl2)) return false;
  ctrl2 &= ~0x08; // clear AF
  return rtcWriteReg(0x01, ctrl2);
}

bool rtcDisableAlarm() {
  bool ok = true;

  uint8_t ctrl2 = 0;
  if (rtcReadReg(0x01, ctrl2)) {
    ctrl2 &= ~0x08; // clear AF
    ctrl2 &= ~0x02; // disable AIE
    ok &= rtcWriteReg(0x01, ctrl2);
  } else {
    ok = false;
  }

  ok &= rtcWriteReg(0x09, 0x80); // minute disabled
  ok &= rtcWriteReg(0x0A, 0x80); // hour disabled
  ok &= rtcWriteReg(0x0B, 0x80); // day disabled
  ok &= rtcWriteReg(0x0C, 0x80); // weekday disabled

  return ok;
}

bool rtcSetAlarmFor(DateTime dt) {
  bool ok = true;

  uint8_t ctrl2 = 0;
  ok &= rtcReadReg(0x01, ctrl2);
  ctrl2 &= ~0x08; // clear AF
  ctrl2 &= ~0x02; // disable AIE while configuring
  ok &= rtcWriteReg(0x01, ctrl2);

  // Match minute and hour. Day and weekday are disabled.
  ok &= rtcWriteReg(0x09, toBCD((uint8_t)dt.minute()) & 0x7F);
  ok &= rtcWriteReg(0x0A, toBCD((uint8_t)dt.hour()) & 0x3F);
  ok &= rtcWriteReg(0x0B, 0x80);
  ok &= rtcWriteReg(0x0C, 0x80);

  ok &= rtcReadReg(0x01, ctrl2);
  ctrl2 &= ~0x08; // clear AF
  ctrl2 |= 0x02;  // AIE
  ok &= rtcWriteReg(0x01, ctrl2);

  return ok;
}

bool setNextAlarmFromNow() {
  if (!g_cfg.loggingEnabled || !g_cfg.rtcValid) return rtcDisableAlarm();

  DateTime now;
  if (!rtcNow(now)) return false;

  uint32_t nextEpoch = now.unixtime() + (g_cfg.intervalMin * 60UL);
  uint32_t sec = nextEpoch % 60UL;
  if (sec != 0) nextEpoch += (60UL - sec);

  DateTime next(nextEpoch);
  return rtcSetAlarmFor(next);
}

String getRtcText() {
  DateTime now;
  if (!rtcNow(now)) return "RTC_ERR";

  char buf[32];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
           now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
  return String(buf);
}

// =================================================================================
// Flash and files
// =================================================================================
void prepareFlashPins() {
  pinMode(PIN_QSPI_IO2, OUTPUT);
  digitalWrite(PIN_QSPI_IO2, HIGH);

  pinMode(PIN_QSPI_IO3, OUTPUT);
  digitalWrite(PIN_QSPI_IO3, HIGH);

  pinMode(PIN_QSPI_CS, OUTPUT);
  digitalWrite(PIN_QSPI_CS, HIGH);

  delay(5);
}

bool beginFlashFS() {
  if (g_flashReady && g_fsReady) return true;

  prepareFlashPins();

  if (!g_flashReady) {
    if (!flash.begin(&XIAO_P25Q16H_DEVICE, 1)) return false;
    g_flashReady = true;
  }

  if (!g_fsReady) {
    if (!fatfs.begin(&flash)) return false;
    g_fsReady = true;
  }

  return true;
}

bool ensureLogFile() {
  if (!beginFlashFS()) return false;

  if (fatfs.exists(LOG_FILE)) {
    File32 f = fatfs.open(LOG_FILE, FILE_READ);
    if (!f) return false;

    BinaryFileHeader hdr;
    int n = f.read((uint8_t*)&hdr, sizeof(hdr));
    f.close();

    if (n == (int)sizeof(hdr) &&
        hdr.magic == LOG_MAGIC &&
        hdr.version == LOG_VERSION &&
        hdr.headerSize == sizeof(BinaryFileHeader) &&
        hdr.recordSize == sizeof(LogRecord)) {
      return true;
    }

    // Invalid old log. Rename it instead of deleting silently.
    fatfs.rename(LOG_FILE, "/log_bad.bin");
  }

  BinaryFileHeader hdr{};
  hdr.magic = LOG_MAGIC;
  hdr.version = LOG_VERSION;
  hdr.headerSize = sizeof(BinaryFileHeader);
  hdr.recordSize = sizeof(LogRecord);

  File32 f = fatfs.open(LOG_FILE, FILE_WRITE);
  if (!f) return false;

  size_t w = f.write((const uint8_t*)&hdr, sizeof(hdr));
  f.close();

  return w == sizeof(hdr);
}

uint32_t getLogFileSize() {
  if (!ensureLogFile()) return 0;
  File32 f = fatfs.open(LOG_FILE, FILE_READ);
  if (!f) return 0;
  uint32_t s = f.size();
  f.close();
  return s;
}

uint32_t getRecordCount() {
  uint32_t bytes = getLogFileSize();
  if (bytes < sizeof(BinaryFileHeader)) return 0;
  return (bytes - sizeof(BinaryFileHeader)) / sizeof(LogRecord);
}

bool appendRecord(const LogRecord& rec) {
  if (!ensureLogFile()) return false;

  File32 f = fatfs.open(LOG_FILE, O_RDWR);
  if (!f) return false;

  uint32_t size = f.size();
  if (!f.seek(size)) {
    f.close();
    return false;
  }

  size_t w = f.write((const uint8_t*)&rec, sizeof(rec));
  f.close();

  return w == sizeof(rec);
}

bool clearLogFile() {
  if (!beginFlashFS()) return false;
  if (fatfs.exists(LOG_FILE)) fatfs.remove(LOG_FILE);
  return ensureLogFile();
}

bool loadConfig() {
  g_cfg.intervalMin = DEFAULT_INTERVAL_MIN;
  g_cfg.loggingEnabled = false;
  g_cfg.rtcValid = false;

  if (!beginFlashFS()) return false;

  if (!fatfs.exists(CFG_FILE)) return false;

  File32 f = fatfs.open(CFG_FILE, FILE_READ);
  if (!f) return false;

  ConfigRecord cr;
  int n = f.read((uint8_t*)&cr, sizeof(cr));
  f.close();

  if (n != (int)sizeof(cr)) return false;
  if (cr.magic != CFG_MAGIC || cr.version != CFG_VERSION) return false;
  if (cr.intervalMin < 1 || cr.intervalMin > 1440) return false;

  g_cfg.intervalMin = cr.intervalMin;
  g_cfg.loggingEnabled = cr.loggingEnabled != 0;
  g_cfg.rtcValid = cr.rtcValid != 0;
  return true;
}

bool saveConfig() {
  if (!beginFlashFS()) return false;

  ConfigRecord cr{};
  cr.magic = CFG_MAGIC;
  cr.version = CFG_VERSION;
  cr.intervalMin = (uint16_t)g_cfg.intervalMin;
  cr.loggingEnabled = g_cfg.loggingEnabled ? 1 : 0;
  cr.rtcValid = g_cfg.rtcValid ? 1 : 0;

  if (fatfs.exists(CFG_FILE)) fatfs.remove(CFG_FILE);

  File32 f = fatfs.open(CFG_FILE, FILE_WRITE);
  if (!f) return false;

  size_t w = f.write((const uint8_t*)&cr, sizeof(cr));
  f.close();

  return w == sizeof(cr);
}


// =================================================================================
// Deployment metadata
// =================================================================================
String sanitizeMetaValue(String s) {
  s.trim();
  s.replace("\r", " ");
  s.replace("\n", " ");
  s.replace(",", ";");
  s.replace("=", "-");
  while (s.indexOf("  ") >= 0) s.replace("  ", " ");
  return s;
}

bool parseFloatValue(const String& s, float& out) {
  String t = s;
  t.trim();
  if (t.length() == 0) return false;

  char* endptr = nullptr;
  float v = strtof(t.c_str(), &endptr);
  if (endptr == t.c_str()) return false;

  while (*endptr) {
    if (!isspace((unsigned char)*endptr)) return false;
    endptr++;
  }

  out = v;
  return true;
}

bool saveMeta() {
  if (!beginFlashFS()) return false;

  if (fatfs.exists(META_FILE)) fatfs.remove(META_FILE);

  File32 f = fatfs.open(META_FILE, FILE_WRITE);
  if (!f) return false;

  String txt;
  txt.reserve(180);
  txt += "logger_id=" + sanitizeMetaValue(g_meta.loggerId) + "\n";
  txt += String("salinity_psu=") + (g_meta.hasSalinity ? String(g_meta.salinityPsu, 3) : "") + "\n";
  txt += String("lat_deg=") + (g_meta.hasLat ? String(g_meta.latDeg, 7) : "") + "\n";
  txt += String("lon_deg=") + (g_meta.hasLon ? String(g_meta.lonDeg, 7) : "") + "\n";

  size_t w = f.write((const uint8_t*)txt.c_str(), txt.length());
  f.close();

  return w == txt.length();
}

String metaLineValue(const String& content, const String& key) {
  int start = 0;
  while (start < (int)content.length()) {
    int end = content.indexOf('\n', start);
    if (end < 0) end = content.length();

    String line = content.substring(start, end);
    line.trim();

    if (startsWithIgnoreCase(line, key + "=")) {
      return line.substring(key.length() + 1);
    }

    start = end + 1;
  }

  return "";
}

bool loadMeta() {
  g_meta = DeploymentMeta();

  if (!beginFlashFS()) return false;
  if (!fatfs.exists(META_FILE)) return false;

  File32 f = fatfs.open(META_FILE, FILE_READ);
  if (!f) return false;

  String content;
  content.reserve(220);

  while (f.available()) {
    char c = (char)f.read();
    if (content.length() < 350) content += c;
  }
  f.close();

  g_meta.loggerId = metaLineValue(content, "logger_id");

  float v = NAN;
  String s = metaLineValue(content, "salinity_psu");
  if (parseFloatValue(s, v)) {
    g_meta.salinityPsu = v;
    g_meta.hasSalinity = true;
  }

  s = metaLineValue(content, "lat_deg");
  if (parseFloatValue(s, v)) {
    g_meta.latDeg = v;
    g_meta.hasLat = true;
  }

  s = metaLineValue(content, "lon_deg");
  if (parseFloatValue(s, v)) {
    g_meta.lonDeg = v;
    g_meta.hasLon = true;
  }

  return true;
}

String buildMetaString() {
  String s = "OK meta";
  s += "; id=";
  s += (g_meta.loggerId.length() ? g_meta.loggerId : "NA");
  s += "; salinity=";
  s += (g_meta.hasSalinity ? String(g_meta.salinityPsu, 3) : "NA");
  s += "; lat=";
  s += (g_meta.hasLat ? String(g_meta.latDeg, 7) : "NA");
  s += "; lon=";
  s += (g_meta.hasLon ? String(g_meta.lonDeg, 7) : "NA");
  return s;
}

// =================================================================================
// Sensors over bit-banged I2C
// =================================================================================
bool readSHT40TempHumidity(float& tempC, float& rhPct) {
  if (!i2cAddressAck(SHT40_ADDR)) return false;

  // SHT40 high precision measurement command.
  // Response: T_MSB, T_LSB, T_CRC, RH_MSB, RH_LSB, RH_CRC.
  // CRC is not checked here to keep the logger small and consistent with v22 behavior.
  if (!i2cWriteCommand(SHT40_ADDR, 0xFD)) return false;
  delay(12);

  uint8_t data[6] = {0};
  if (!i2cReadBytes(SHT40_ADDR, data, sizeof(data))) return false;

  uint16_t rawT  = ((uint16_t)data[0] << 8) | data[1];
  uint16_t rawRH = ((uint16_t)data[3] << 8) | data[4];

  tempC = -45.0f + 175.0f * ((float)rawT / 65535.0f);
  rhPct = -6.0f + 125.0f * ((float)rawRH / 65535.0f);

  if (rhPct < 0.0f) rhPct = 0.0f;
  if (rhPct > 100.0f) rhPct = 100.0f;

  return true;
}

bool readBH1750(uint8_t& mode, uint8_t& mtreg, uint16_t& raw, float& lux) {
  if (!i2cAddressAck(BH1750_ADDR)) return false;

  mtreg = 69;
  mode = 0; // one-time high-resolution mode 1

  if (!i2cWriteCommand(BH1750_ADDR, 0x01)) return false; // power on
  delay(10);
  if (!i2cWriteCommand(BH1750_ADDR, 0x20)) return false; // one-time H-resolution mode
  delay(180);

  uint8_t data[2] = {0};
  if (!i2cReadBytes(BH1750_ADDR, data, sizeof(data))) return false;

  raw = ((uint16_t)data[0] << 8) | data[1];
  lux = raw / 1.2f;
  return true;
}

bool beginOnboardIMU() {
  if (g_imuReady) return true;

  status_t rc = g_imu.begin();
  g_imuReady = (rc == IMU_SUCCESS);
  return g_imuReady;
}

bool readIMUAccel(float& ax, float& ay, float& az) {
  if (!beginOnboardIMU()) return false;

  // Seeed LSM6DS3 library returns acceleration in g.
  // The logger stores acceleration in mg, so multiply by 1000.
  ax = g_imu.readFloatAccelX() * 1000.0f;
  ay = g_imu.readFloatAccelY() * 1000.0f;
  az = g_imu.readFloatAccelZ() * 1000.0f;

  return true;
}

bool readIMUTemp(float& imuTempC) {
  if (!beginOnboardIMU()) return false;
  imuTempC = g_imu.readTempC();
  return true;
}

// =================================================================================
// Measurement
// =================================================================================
Measurement takeMeasurement(WakeCode wake) {
  Measurement m{};
  m.wakeCode = wake;

  DateTime now;
  m.rtcOk = rtcNow(now);
  if (m.rtcOk) m.unixTime = now.unixtime();

  sensorRail(true);

  m.vbat = readBatteryVoltage();
  m.shtOk = readSHT40TempHumidity(m.tempC, m.rhPct);
  m.bhOk = readBH1750(m.bhMode, m.bhMtreg, m.bhRaw, m.bhLux);
  m.imuOk = readIMUAccel(m.ax_mg, m.ay_mg, m.az_mg);

  sensorRail(false);
  return m;
}

LogRecord packRecord(const Measurement& m) {
  LogRecord rec{};
  rec.unixTime = m.unixTime;
  rec.tempC_x100 = isnan(m.tempC) ? INT16_MIN : (int16_t)lroundf(m.tempC * 100.0f);
  rec.rhPct_x100 = isnan(m.rhPct) ? INT16_MIN : (int16_t)lroundf(m.rhPct * 100.0f);
  rec.bhMode = m.bhMode;
  rec.bhMtreg = m.bhMtreg;
  rec.bhRaw = m.bhRaw;
  rec.bhLux_mlx = isnan(m.bhLux) ? 0UL : (uint32_t)lroundf(m.bhLux * 1000.0f);
  rec.ax_mg = isnan(m.ax_mg) ? INT16_MIN : (int16_t)lroundf(m.ax_mg);
  rec.ay_mg = isnan(m.ay_mg) ? INT16_MIN : (int16_t)lroundf(m.ay_mg);
  rec.az_mg = isnan(m.az_mg) ? INT16_MIN : (int16_t)lroundf(m.az_mg);
  rec.vbat_mV = isnan(m.vbat) ? 0U : (uint16_t)constrain((long)lroundf(m.vbat * 1000.0f), 0L, 65535L);
  rec.wakeCode = (uint8_t)m.wakeCode;

  rec.flags = 0;
  if (g_cfg.loggingEnabled) rec.flags |= 0x01;
  if (g_cfg.rtcValid) rec.flags |= 0x02;
  if (m.shtOk) rec.flags |= 0x04;
  if (m.bhOk) rec.flags |= 0x08;
  if (m.imuOk) rec.flags |= 0x10;

  return rec;
}

bool measureAndAppend(WakeCode wake) {
  Measurement m = takeMeasurement(wake);
  if (!m.rtcOk) {
    g_cfg.rtcValid = false;
    g_cfg.loggingEnabled = false;
    saveConfig();
    return false;
  }

  LogRecord rec = packRecord(m);
  return appendRecord(rec);
}

// =================================================================================
// Wake / sleep
// =================================================================================
bool reedActive() {
  return digitalRead(PIN_REED) == LOW;
}

WakeCode detectWakeSource() {
  if (reedActive()) return WAKE_REED;
  if (rtcIntIsLow()) return WAKE_RTC_ALARM;
  return WAKE_POWERUP;
}

void configureWakeSenseLow(uint32_t nrfPin) {
  nrf_gpio_cfg_sense_input(nrfPin, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
}

void disableSoftDeviceForSleep() {
  if (!g_bleStarted) return;

  Bluefruit.Advertising.stop();
  delay(50);

  uint8_t sdEnabled = 0;
  if (sd_softdevice_is_enabled(&sdEnabled) == NRF_SUCCESS && sdEnabled) {
    sd_softdevice_disable();
    delay(80);
  }

  g_bleStarted = false;
}

void enterSystemOff() {
  rgbOff();

  disableSoftDeviceForSleep();

  sensorRail(false);

  pinMode(PIN_REED, INPUT_PULLUP);
  pinMode(PIN_RTC_INT, INPUT_PULLUP);

  // Always allow magnet wake.
  configureWakeSenseLow(NRF_PIN_REED_D1);

  // Allow RTC wake only when routine logging is active.
  if (g_cfg.loggingEnabled && g_cfg.rtcValid) {
    configureWakeSenseLow(NRF_PIN_RTC_INT_D0);
  }

  delay(50);
  NRF_POWER->SYSTEMOFF = 1;
  __DSB();

  while (1) {
    delay(1000);
  }
}

bool prepareSleep() {
  if (g_cfg.loggingEnabled && g_cfg.rtcValid) {
    return setNextAlarmFromNow();
  } else {
    rtcDisableAlarm();
    return true;
  }
}

// =================================================================================
// BLE
// =================================================================================
bool isBleConnected() {
  return Bluefruit.connected();
}

void statusWrite(const String& s) {
  uint16_t n = (uint16_t)min((size_t)s.length(), STATUS_MAX_LEN);
  statusChar.write(s.c_str(), n);
  if (isBleConnected()) {
    statusChar.notify(s.c_str(), n);
    delay(10);
  }
}

String buildStatusString() {
  String s;
  s.reserve(190);

  s += "logging=";
  s += (g_cfg.loggingEnabled ? "1" : "0");
  s += "; rtcValid=";
  s += (g_cfg.rtcValid ? "1" : "0");
  s += "; intervalMin=";
  s += String(g_cfg.intervalMin);
  s += "; records=";
  s += String(getRecordCount());
  s += "; bytes=";
  s += String(getLogFileSize());
  s += "; vbat=";
  s += String(readBatteryVoltage(), 2);
  s += "; wake=";
  s += wakeToString(detectWakeSource());
  s += "; time=";
  s += getRtcText();

  return s;
}

void sendCsv() {
  if (!ensureLogFile()) {
    statusWrite("ERR CSV");
    return;
  }

  File32 f = fatfs.open(LOG_FILE, FILE_READ);
  if (!f) {
    statusWrite("ERR CSV OPEN");
    return;
  }

  BinaryFileHeader hdr;
  if (f.read((uint8_t*)&hdr, sizeof(hdr)) != (int)sizeof(hdr)) {
    f.close();
    statusWrite("ERR CSV HDR");
    return;
  }

  if (hdr.magic != LOG_MAGIC || hdr.recordSize != sizeof(LogRecord)) {
    f.close();
    statusWrite("ERR CSV MAGIC");
    return;
  }

  uint32_t idx = 0;
  statusWrite("CSV_BEGIN");
  statusWrite(String("META,logger_id,") + sanitizeMetaValue(g_meta.loggerId));
  statusWrite(String("META,salinity_psu,") + (g_meta.hasSalinity ? String(g_meta.salinityPsu, 3) : ""));
  statusWrite(String("META,lat_deg,") + (g_meta.hasLat ? String(g_meta.latDeg, 7) : ""));
  statusWrite(String("META,lon_deg,") + (g_meta.hasLon ? String(g_meta.lonDeg, 7) : ""));
  statusWrite("idx,unix,tempC,rhPct,vbat,bhMode,bhMtreg,bhRaw,lux,ax,ay,az,wake,flags");

  LogRecord r;
  while (f.available() >= (int)sizeof(LogRecord)) {
    if (f.read((uint8_t*)&r, sizeof(r)) != (int)sizeof(r)) break;

    float tempC = (r.tempC_x100 == INT16_MIN) ? NAN : ((float)r.tempC_x100 / 100.0f);
    float rhPct = (r.rhPct_x100 == INT16_MIN) ? NAN : ((float)r.rhPct_x100 / 100.0f);
    float lux = (float)r.bhLux_mlx / 1000.0f;

    char line[190];
    snprintf(line, sizeof(line),
             "%lu,%lu,%.2f,%.2f,%.3f,%u,%u,%u,%.3f,%d,%d,%d,%u,%u",
             (unsigned long)idx,
             (unsigned long)r.unixTime,
             isnan(tempC) ? -9999.99f : tempC,
             isnan(rhPct) ? -1.00f : rhPct,
             ((float)r.vbat_mV) / 1000.0f,
             (unsigned)r.bhMode,
             (unsigned)r.bhMtreg,
             (unsigned)r.bhRaw,
             lux,
             (int)r.ax_mg,
             (int)r.ay_mg,
             (int)r.az_mg,
             (unsigned)r.wakeCode,
             (unsigned)r.flags);

    statusWrite(String(line));
    idx++;
    delay(20);
  }

  f.close();
  statusWrite("CSV_END");
}

void processCommand(const String& raw) {
  String cmd = trimCopy(raw);
  g_lastBleActivityMs = millis();

  if (cmd.length() == 0) {
    statusWrite("ERR empty");
    return;
  }

  if (cmd.equalsIgnoreCase("HELP")) {
    statusWrite("OK HELP STATUS GET_META SET_ID SET_SALINITY SET_GPS RTC_CHECK I2C_STATE IMU_CHECK SET_TIME GET_TIME SET_INTERVAL_MIN START STOP SLEEP MEASURE_NOW COUNT INFO CSV CLEAR");
    return;
  }

  if (cmd.equalsIgnoreCase("STATUS")) {
    statusWrite(buildStatusString());
    return;
  }

  if (cmd.equalsIgnoreCase("GET_META")) {
    statusWrite(buildMetaString());
    return;
  }

  if (startsWithIgnoreCase(cmd, "SET_ID=") || startsWithIgnoreCase(cmd, "SET_LOGGER_ID=")) {
    int eq = cmd.indexOf('=');
    String id = sanitizeMetaValue(cmd.substring(eq + 1));

    if (id.length() > 24) id = id.substring(0, 24);
    if (id.length() == 0) {
      statusWrite("ERR id");
      return;
    }

    g_meta.loggerId = id;
    saveMeta();
    statusWrite(buildMetaString());
    return;
  }

  if (startsWithIgnoreCase(cmd, "SET_SALINITY=")) {
    float v = NAN;
    if (!parseFloatValue(cmd.substring(13), v) || v < 0.0f || v > 45.0f) {
      statusWrite("ERR salinity");
      return;
    }

    g_meta.salinityPsu = v;
    g_meta.hasSalinity = true;
    saveMeta();
    statusWrite(buildMetaString());
    return;
  }

  if (startsWithIgnoreCase(cmd, "SET_GPS=")) {
    String s = cmd.substring(8);
    s.trim();

    int comma = s.indexOf(',');
    if (comma < 0) comma = s.indexOf(';');

    if (comma < 0) {
      statusWrite("ERR gps fmt");
      return;
    }

    float lat = NAN, lon = NAN;
    if (!parseFloatValue(s.substring(0, comma), lat) ||
        !parseFloatValue(s.substring(comma + 1), lon) ||
        lat < -90.0f || lat > 90.0f || lon < -180.0f || lon > 180.0f) {
      statusWrite("ERR gps");
      return;
    }

    g_meta.latDeg = lat;
    g_meta.lonDeg = lon;
    g_meta.hasLat = true;
    g_meta.hasLon = true;
    saveMeta();
    statusWrite(buildMetaString());
    return;
  }

  if (cmd.equalsIgnoreCase("CLEAR_META")) {
    g_meta = DeploymentMeta();
    if (beginFlashFS() && fatfs.exists(META_FILE)) fatfs.remove(META_FILE);
    statusWrite(buildMetaString());
    return;
  }

  if (cmd.equalsIgnoreCase("I2C_STATE")) {
    statusWrite(i2cStateText());
    return;
  }

  if (cmd.equalsIgnoreCase("RTC_CHECK")) {
    statusWrite(rtcBegin() ? "OK T51" : "ERR RTC");
    return;
  }

  if (cmd.equalsIgnoreCase("IMU_CHECK")) {
    float ax = NAN, ay = NAN, az = NAN, t = NAN;
    if (!readIMUAccel(ax, ay, az)) {
      statusWrite("ERR imu");
      return;
    }

    String s = "OK imu ax=" + String(ax, 0) +
               "; ay=" + String(ay, 0) +
               "; az=" + String(az, 0);

    if (readIMUTemp(t)) {
      s += "; t=" + String(t, 2);
    }

    statusWrite(s);
    return;
  }

  if (cmd.equalsIgnoreCase("GET_TIME")) {
    statusWrite(String("OK time=") + getRtcText());
    return;
  }

  if (startsWithIgnoreCase(cmd, "SET_TIME=")) {
    DateTime dt;
    if (!parseDateTimeText(cmd.substring(9), dt)) {
      statusWrite("ERR time fmt: YYYY-MM-DD HH:MM[:SS]");
      return;
    }

    if (!rtcAdjust(dt)) {
      statusWrite("ERR rtc set");
      return;
    }

    g_cfg.rtcValid = true;
    saveConfig();
    statusWrite(String("OK time=") + getRtcText());
    return;
  }

  if (startsWithIgnoreCase(cmd, "SET_EPOCH=")) {
    uint32_t epoch = 0;
    if (!parseUInt32(cmd.substring(10), epoch)) {
      statusWrite("ERR epoch");
      return;
    }

    if (!rtcAdjust(DateTime(epoch))) {
      statusWrite("ERR rtc set");
      return;
    }

    g_cfg.rtcValid = true;
    saveConfig();
    statusWrite(String("OK time=") + getRtcText());
    return;
  }

  if (cmd.equalsIgnoreCase("GET_INTERVAL")) {
    statusWrite(String("OK intervalMin=") + String(g_cfg.intervalMin));
    return;
  }

  if (startsWithIgnoreCase(cmd, "SET_INTERVAL_MIN=")) {
    uint32_t v = 0;
    if (!parseUInt32(cmd.substring(17), v) || v < 1 || v > 1440) {
      statusWrite("ERR interval");
      return;
    }

    g_cfg.intervalMin = v;
    saveConfig();

    if (g_cfg.loggingEnabled && g_cfg.rtcValid) {
      if (!setNextAlarmFromNow()) statusWrite("ERR alarm");
      else statusWrite(String("OK intervalMin=") + String(g_cfg.intervalMin));
    } else {
      statusWrite(String("OK intervalMin=") + String(g_cfg.intervalMin));
    }
    return;
  }

  if (cmd.equalsIgnoreCase("START")) {
    if (!g_cfg.rtcValid) {
      statusWrite("ERR rtc not set");
      return;
    }

    g_cfg.loggingEnabled = true;

    if (!setNextAlarmFromNow()) {
      g_cfg.loggingEnabled = false;
      saveConfig();
      statusWrite("ERR alarm");
      return;
    }

    saveConfig();
    statusWrite("OK logging=1");
    return;
  }

  if (cmd.equalsIgnoreCase("STOP")) {
    g_cfg.loggingEnabled = false;
    saveConfig();
    rtcDisableAlarm();
    statusWrite("OK logging=0");
    return;
  }

  if (cmd.equalsIgnoreCase("SLEEP")) {
    statusWrite("OK sleep");
    delay(80);

    if (!prepareSleep()) {
      // Cannot report after this reliably, but avoid sleeping unarmed.
      setupBLE();
      statusWrite("ERR alarm");
      return;
    }

    enterSystemOff();
    return;
  }

  if (cmd.equalsIgnoreCase("MEASURE_NOW")) {
    bool ok = measureAndAppend(WAKE_MANUAL);
    statusWrite(ok ? String("OK measure rec=") + String(getRecordCount()) : "ERR measure");
    return;
  }

  if (cmd.equalsIgnoreCase("COUNT")) {
    statusWrite(String("OK records=") + String(getRecordCount()));
    return;
  }

  if (cmd.equalsIgnoreCase("INFO")) {
    statusWrite(String("OK rec=") + String(getRecordCount()) +
                "; bytes=" + String(getLogFileSize()) +
                "; recSize=" + String((uint32_t)sizeof(LogRecord)));
    return;
  }

  if (cmd.equalsIgnoreCase("CLEAR")) {
    if (clearLogFile()) statusWrite("OK cleared");
    else statusWrite("ERR clear");
    return;
  }

  if (cmd.equalsIgnoreCase("CSV")) {
    sendCsv();
    return;
  }

  statusWrite("ERR unknown");
}

void cmdWriteCallback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  (void)conn_hdl;
  (void)chr;

  char buf[CMD_MAX_LEN + 1];
  if (len > CMD_MAX_LEN) len = CMD_MAX_LEN;
  memcpy(buf, data, len);
  buf[len] = '\0';

  processCommand(String(buf));
}

void connectCallback(uint16_t conn_handle) {
  (void)conn_handle;
  g_bleConnected = true;
  g_lastBleActivityMs = millis();
  rgbGreen();
  delay(150);
  statusWrite("OK connected");
}

void disconnectCallback(uint16_t conn_handle, uint8_t reason) {
  (void)conn_handle;
  (void)reason;
  g_bleConnected = false;
  g_lastBleActivityMs = millis();
  rgbOff();

  if (g_bleStarted) {
    Bluefruit.Advertising.start(0);
  }
}

void setupBLE() {
  if (g_bleStarted) return;

  Bluefruit.begin();
  g_bleStarted = true;

  Bluefruit.autoConnLed(false);
  Bluefruit.Periph.setConnectCallback(connectCallback);
  Bluefruit.Periph.setDisconnectCallback(disconnectCallback);
  Bluefruit.setTxPower(4);
  Bluefruit.setName(BLE_NAME);

  loggerService.begin();

  cmdChar.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  cmdChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  cmdChar.setMaxLen(CMD_MAX_LEN);
  cmdChar.setWriteCallback(cmdWriteCallback);
  cmdChar.begin();

  statusChar.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  statusChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  statusChar.setMaxLen(STATUS_MAX_LEN);
  statusChar.begin();
  statusChar.write("BOOT", 4);

  Bluefruit.Advertising.stop();
  Bluefruit.Advertising.clearData();
  Bluefruit.ScanResponse.clearData();

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(loggerService);
  Bluefruit.ScanResponse.addName();

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);

  g_lastBleActivityMs = millis();
}

void runServiceMode() {
  setupBLE();
  statusWrite(buildStatusString());

  uint32_t startMs = millis();
  bool hadConnection = false;

  while (true) {
    if (isBleConnected()) {
      hadConnection = true;
      rgbGreen();
      delay(20);
      continue;
    }

    // Advertising indicator.
    rgbBlue();
    delay(250);
    rgbOff();
    delay(250);

    uint32_t nowMs = millis();

    // If nobody connects after power-up/reed wake, eventually sleep.
    if (!hadConnection && (nowMs - startMs > SERVICE_TIMEOUT_MS)) break;

    // If the user connected and then disconnected, sleep soon.
    if (hadConnection && (nowMs - g_lastBleActivityMs > DISCONNECTED_SLEEP_DELAY_MS)) break;
  }

  prepareSleep();
  enterSystemOff();
}

// =================================================================================
// Setup / main
// =================================================================================
void setup() {
  pinMode(PIN_SENSOR_EN, OUTPUT);
  sensorRail(false);

  pinMode(PIN_REED, INPUT_PULLUP);
  pinMode(PIN_RTC_INT, INPUT_PULLUP);

  i2cReleaseSDA();
  i2cReleaseSCL();

  nrf_gpio_cfg_output(PIN_RGB_RED);
  nrf_gpio_cfg_output(PIN_RGB_GREEN);
  nrf_gpio_cfg_output(PIN_RGB_BLUE);
  rgbOff();

  analogReadResolution(12);

  delay(150);

  beginFlashFS();
  loadConfig();
  loadMeta();
  ensureLogFile();

  WakeCode wake = detectWakeSource();

  bool validRoutineWake = (wake == WAKE_RTC_ALARM && g_cfg.loggingEnabled && g_cfg.rtcValid);

  if (validRoutineWake) {
    // A real scheduled wake: log one record and go back to sleep.
    blinkColor(rgbBlue, 1, 50, 20);

    rtcClearAlarmFlag();

    bool ok = measureAndAppend(WAKE_RTC_ALARM);
    if (!ok) {
      g_cfg.loggingEnabled = false;
      saveConfig();
    }

    if (g_cfg.loggingEnabled && g_cfg.rtcValid) {
      if (!setNextAlarmFromNow()) {
        g_cfg.loggingEnabled = false;
        saveConfig();
        rtcDisableAlarm();
      }
    } else {
      rtcDisableAlarm();
    }

    enterSystemOff();
  }

  // Everything else enters BLE service:
  // - power-up/reset
  // - reed wake
  // - stale RTC INT while logging is off
  // - RTC/config problem
  if (wake == WAKE_REED) blinkColor(rgbBlue, 3);
  else if (wake == WAKE_RTC_ALARM) blinkColor(rgbOrange, 3);
  else blinkColor(rgbRed, 3);

  runServiceMode();
}

void loop() {
  // Not used.
}
