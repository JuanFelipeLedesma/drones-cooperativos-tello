/*
 * MSP Rangefinder + Compass Bridge v4 — XIAO ESP32-S3 -> iNav (F405)
 * ------------------------------------------------------------------
 * Clean build, addresses CONFIRMED on this hardware (via external I2C scan):
 *   VL53L0X rangefinder = 0x29   ->  MSP2_SENSOR_RANGEFINDER (0x1F01)
 *   QMC5883L compass    = 0x0D   ->  MSP2_SENSOR_COMPASS    (0x1F04)
 *
 * No bus scanner. Mag reads use STOP (not repeated-start) + an I2C timeout, so a
 * disconnected mag returns cleanly instead of hanging the shared bus (the v3 bug).
 *
 * STILL TO VERIFY before flying poshold: AXIS ALIGNMENT (the 3 lines in loop()).
 *   Bench (serial): nose down -> magX up ; right side down -> magY up (N. hemisphere).
 *   Then confirm the heading tracks correctly in the iNav Setup tab.
 *
 * Wiring: both sensors SDA->D4(GPIO5) SCL->D5(GPIO6) VCC->3V3 GND->GND ;
 *   ESP TX D6(GPIO43)->FC UART3 RX ; shared GND ; M100 compass SDA/SCL on the XIAO.
 * iNav: Ports->MSP on UART3 @115200 ;
 *   CLI: set rangefinder_hardware=MSP / set mag_hardware=MSP / set align_mag=CW0 / save
 * Library: "VL53L0X" by Pololu.   Board: XIAO_ESP32S3.
 */

#include <Wire.h>
#include <VL53L0X.h>

// ---------------- Config ----------------
static const uint32_t FC_BAUD      = 115200;
static const int      PIN_SDA      = 5;      // D4
static const int      PIN_SCL      = 6;      // D5
static const int      PIN_FC_TX    = 43;     // D6
static const int      PIN_FC_RX    = 44;     // D7
static const uint16_t SEND_HZ      = 30;
static const int32_t  MAX_VALID_MM = 2000;
static const bool     DEBUG_USB    = true;
// ----------------------------------------

static const uint8_t QMC_ADDR = 0x0D;        // confirmed magnetometer

HardwareSerial FC(1);
VL53L0X sensor;
bool magPresent = false;

static const uint8_t  MSP_HDR_X               = 'X';
static const uint8_t  MSP_DIR_TO_FC           = '<';
static const uint16_t MSP2_SENSOR_RANGEFINDER = 0x1F01;
static const uint16_t MSP2_SENSOR_COMPASS     = 0x1F04;

// ----------------------- MSP v2 -----------------------
static uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a) {
  crc ^= a;
  for (uint8_t i = 0; i < 8; i++)
    crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
  return crc;
}
void mspSend(uint16_t function, const uint8_t *payload, uint16_t len) {
  uint8_t flag = 0, crc = 0;
  FC.write('$'); FC.write(MSP_HDR_X); FC.write(MSP_DIR_TO_FC);
  FC.write(flag);                      crc = crc8_dvb_s2(crc, flag);
  uint8_t fl = function & 0xFF;        FC.write(fl); crc = crc8_dvb_s2(crc, fl);
  uint8_t fh = (function >> 8) & 0xFF; FC.write(fh); crc = crc8_dvb_s2(crc, fh);
  uint8_t sl = len & 0xFF;             FC.write(sl); crc = crc8_dvb_s2(crc, sl);
  uint8_t sh = (len >> 8) & 0xFF;      FC.write(sh); crc = crc8_dvb_s2(crc, sh);
  for (uint16_t i = 0; i < len; i++) { FC.write(payload[i]); crc = crc8_dvb_s2(crc, payload[i]); }
  FC.write(crc);
}
// payload: quality(u8) + distanceMm(i32 LE). distance < 0 = out of range.
void sendRangefinder(uint8_t quality, int32_t mm) {
  uint8_t p[5] = { quality,
                   (uint8_t)(mm & 0xFF),         (uint8_t)((mm >> 8)  & 0xFF),
                   (uint8_t)((mm >> 16) & 0xFF), (uint8_t)((mm >> 24) & 0xFF) };
  mspSend(MSP2_SENSOR_RANGEFINDER, p, sizeof(p));
}
// payload: instance(u8) + timeMs(u32 LE) + magX,magY,magZ (i16 LE).
void sendCompass(int16_t bx, int16_t by, int16_t bz) {
  uint32_t t = millis();
  uint16_t ux = (uint16_t)bx, uy = (uint16_t)by, uz = (uint16_t)bz;
  uint8_t p[11] = {
    0,
    (uint8_t)(t & 0xFF),  (uint8_t)((t >> 8) & 0xFF),
    (uint8_t)((t >> 16) & 0xFF), (uint8_t)((t >> 24) & 0xFF),
    (uint8_t)(ux & 0xFF), (uint8_t)((ux >> 8) & 0xFF),
    (uint8_t)(uy & 0xFF), (uint8_t)((uy >> 8) & 0xFF),
    (uint8_t)(uz & 0xFF), (uint8_t)((uz >> 8) & 0xFF)
  };
  mspSend(MSP2_SENSOR_COMPASS, p, sizeof(p));
}

// ----------------------- QMC5883L -----------------------
void i2cWrite(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}
bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}
void magInit() {
  i2cWrite(QMC_ADDR, 0x0B, 0x01);   // set/reset period (recommended)
  i2cWrite(QMC_ADDR, 0x09, 0x1D);   // OSR=512, RNG=8G, ODR=200Hz, continuous
}
bool magReadRaw(int16_t &mx, int16_t &my, int16_t &mz) {
  uint8_t b[6];
  Wire.beginTransmission(QMC_ADDR); Wire.write(0x00); Wire.endTransmission(true);  // STOP
  if (Wire.requestFrom((int)QMC_ADDR, 6) != 6) return false;
  for (int i = 0; i < 6; i++) b[i] = Wire.read();
  mx = (int16_t)((b[1] << 8) | b[0]);   // little-endian, order X,Y,Z
  my = (int16_t)((b[3] << 8) | b[2]);
  mz = (int16_t)((b[5] << 8) | b[4]);
  return true;
}

// ----------------------- Rangefinder -----------------------
int32_t readDistanceMm() {
  uint16_t mm = sensor.readRangeContinuousMillimeters();
  if (sensor.timeoutOccurred())     return -1;
  if (mm == 0 || mm > MAX_VALID_MM) return -1;
  return (int32_t)mm;
}

void setup() {
  if (DEBUG_USB) Serial.begin(115200);
  FC.begin(FC_BAUD, SERIAL_8N1, PIN_FC_RX, PIN_FC_TX);
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  Wire.setTimeOut(25);              // ms: never block the bus forever on a dead device

  // Rangefinder (0x29)
  sensor.setTimeout(100);
  if (!sensor.init()) { if (DEBUG_USB) Serial.println("VL53L0X (0x29) not found"); }
  else {
    sensor.setSignalRateLimit(0.1);
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    sensor.setMeasurementTimingBudget(33000);
    sensor.startContinuous();
  }

  // Compass (0x0D)
  magPresent = i2cPresent(QMC_ADDR);
  if (magPresent) { magInit(); if (DEBUG_USB) Serial.println("QMC5883L (0x0D) OK"); }
  else if (DEBUG_USB)         Serial.println("QMC5883L (0x0D) not found - check wiring");
}

void loop() {
  static uint32_t last = 0;
  const uint32_t period = 1000 / SEND_HZ;
  if (millis() - last < period) return;
  last = millis();

  // Rangefinder -> iNav
  int32_t d = readDistanceMm();
  if (d < 0) sendRangefinder(0, -1);
  else       sendRangefinder(255, d);

  // Compass -> iNav
  int16_t mx, my, mz;
  if (magPresent && magReadRaw(mx, my, mz)) {
    // ===== AXIS ALIGNMENT — edit ONLY these 3 lines after the orientation check =====
    int16_t bx =  my;   // body X = forward
    int16_t by =  mx;   // body Y = right
    int16_t bz =  -mz;   // body Z = down
    // (example fix:  int16_t bx = -my;  int16_t by = mx;  int16_t bz = -mz;)
    // ===============================================================================
    sendCompass(bx, by, bz);
    if (DEBUG_USB) Serial.printf("dist=%ld mm | mag x=%d y=%d z=%d\n", d, bx, by, bz);
  } else if (DEBUG_USB) {
    Serial.printf("dist=%ld mm | mag: --\n", d);
  }
}
