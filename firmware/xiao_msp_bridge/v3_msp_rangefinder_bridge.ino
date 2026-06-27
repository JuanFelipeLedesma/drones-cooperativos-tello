/*
 * MSP Rangefinder + Compass Bridge v3 — XIAO ESP32-S3 -> iNav (F405)
 * ------------------------------------------------------------------
 * Streams to iNav over ONE MSP link (UART3 @115200):
 *   - VL53L0X rangefinder   -> MSP2_SENSOR_RANGEFINDER (0x1F01)   [proven in v1/v2]
 *   - HMC5883L/QMC5883L mag  -> MSP2_SENSOR_COMPASS    (0x1F04)
 *
 * Both I2C sensors share the XIAO Wire bus (different addresses, no conflict):
 *   VL53L0X = 0x29   HMC5883L = 0x1E   QMC5883L = 0x0D
 *
 * >>> TWO THINGS YOU MUST CONFIRM ON YOUR HARDWARE (see NOTES at the bottom) <<<
 *   1) WHICH MAG CHIP. Many "HMC5883" GPS modules are actually QMC5883L.
 *      The I2C scanner prints the address at boot:
 *        0x1E -> set MAG_TYPE = MAG_HMC5883L      0x0D -> set MAG_TYPE = MAG_QMC5883L
 *   2) AXIS ALIGNMENT. The mag must reach iNav in body frame (X fwd, Y right, Z down).
 *      Verify with the bench orientation test AND the iNav heading check before poshold.
 *
 * Wiring (XIAO ESP32-S3):
 *   Both sensors:  SDA -> D4 (GPIO5)   SCL -> D5 (GPIO6)   VCC -> 3V3   GND -> GND
 *   ESP TX D6 (GPIO43) -> FC UART3 RX        ESP GND <-> FC GND (shared ground required)
 *   M100 GPS: route its COMPASS SDA/SCL to the XIAO I2C; its GPS TX/RX stay on FC UART1.
 *   If I2C is flaky with both sensors on the bus, drop Wire.setClock to 100000.
 *
 * iNav:  Ports -> MSP on UART3 @115200
 *        CLI:  set rangefinder_hardware = MSP
 *              set mag_hardware = MSP
 *              set align_mag = CW0
 *              save
 *
 * Library: "VL53L0X" by Pololu.   Board: XIAO_ESP32S3.
 */

#include <Wire.h>
#include <VL53L0X.h>

// ---------------- Mag chip select ----------------
#define MAG_HMC5883L 0
#define MAG_QMC5883L 1
#define MAG_TYPE     MAG_HMC5883L     // set per the I2C scanner result at boot
// -------------------------------------------------

// ---------------- User config ----------------
static const uint32_t FC_BAUD       = 115200;
static const int      PIN_SDA       = 5;        // D4
static const int      PIN_SCL       = 6;        // D5
static const int      PIN_FC_TX     = 43;       // D6
static const int      PIN_FC_RX     = 44;       // D7
static const uint16_t SEND_HZ       = 30;
static const int32_t  MAX_VALID_MM  = 2000;
static const uint8_t  FILTER_WINDOW = 1;        // rangefinder MA: 1 = OFF (keep off for now)
static const bool     DEBUG_USB     = true;
// ---------------------------------------------

static const uint8_t HMC_ADDR = 0x1E;
static const uint8_t QMC_ADDR = 0x0D;

HardwareSerial FC(1);
VL53L0X sensor;

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
// MSP2_SENSOR_RANGEFINDER payload: quality(u8) + distanceMm(i32 LE). distance<0 = out of range.
void sendRangefinder(uint8_t quality, int32_t mm) {
  uint8_t p[5] = { quality,
                   (uint8_t)(mm & 0xFF),     (uint8_t)((mm >> 8)  & 0xFF),
                   (uint8_t)((mm >> 16) & 0xFF), (uint8_t)((mm >> 24) & 0xFF) };
  mspSend(MSP2_SENSOR_RANGEFINDER, p, sizeof(p));
}
// MSP2_SENSOR_COMPASS payload: instance(u8) + timeMs(u32 LE) + magX,magY,magZ (i16 LE).
void sendCompass(int16_t bx, int16_t by, int16_t bz) {
  uint32_t t = millis();
  uint16_t ux = (uint16_t)bx, uy = (uint16_t)by, uz = (uint16_t)bz;
  uint8_t p[11] = {
    0,                                                       // instance
    (uint8_t)(t & 0xFF),  (uint8_t)((t >> 8) & 0xFF),
    (uint8_t)((t >> 16) & 0xFF), (uint8_t)((t >> 24) & 0xFF), // timeMs (u32 LE)
    (uint8_t)(ux & 0xFF), (uint8_t)((ux >> 8) & 0xFF),        // magX (i16 LE)
    (uint8_t)(uy & 0xFF), (uint8_t)((uy >> 8) & 0xFF),        // magY
    (uint8_t)(uz & 0xFF), (uint8_t)((uz >> 8) & 0xFF)         // magZ
  };
  mspSend(MSP2_SENSOR_COMPASS, p, sizeof(p));
}

// ----------------------- I2C helpers -----------------------
void i2cWrite(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}
void i2cScan() {
  if (!DEBUG_USB) return;
  Serial.println("I2C scan:");
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print("  found 0x"); Serial.print(a, HEX);
      if (a == 0x29) Serial.print("  (VL53L0X)");
      if (a == 0x1E) Serial.print("  (HMC5883L -> MAG_TYPE = MAG_HMC5883L)");
      if (a == 0x0D) Serial.print("  (QMC5883L -> MAG_TYPE = MAG_QMC5883L)");
      Serial.println();
    }
  }
}

// ----------------------- Magnetometer -----------------------
void magInit() {
#if MAG_TYPE == MAG_HMC5883L
  i2cWrite(HMC_ADDR, 0x00, 0x70);   // 8 samples avg, 15 Hz, normal measurement
  i2cWrite(HMC_ADDR, 0x01, 0x20);   // gain +/-1.3 Ga
  i2cWrite(HMC_ADDR, 0x02, 0x00);   // continuous measurement
#else
  i2cWrite(QMC_ADDR, 0x0B, 0x01);   // set/reset period (recommended)
  i2cWrite(QMC_ADDR, 0x09, 0x1D);   // OSR=512, RNG=8G, ODR=200Hz, continuous
#endif
}
bool magReadRaw(int16_t &mx, int16_t &my, int16_t &mz) {
  uint8_t b[6];
#if MAG_TYPE == MAG_HMC5883L
  Wire.beginTransmission(HMC_ADDR); Wire.write(0x03); Wire.endTransmission(false);
  if (Wire.requestFrom((int)HMC_ADDR, 6) != 6) return false;
  for (int i = 0; i < 6; i++) b[i] = Wire.read();
  mx = (int16_t)((b[0] << 8) | b[1]);   // X  (big-endian; HMC order is X,Z,Y)
  mz = (int16_t)((b[2] << 8) | b[3]);   // Z
  my = (int16_t)((b[4] << 8) | b[5]);   // Y
#else
  Wire.beginTransmission(QMC_ADDR); Wire.write(0x00); Wire.endTransmission(false);
  if (Wire.requestFrom((int)QMC_ADDR, 6) != 6) return false;
  for (int i = 0; i < 6; i++) b[i] = Wire.read();
  mx = (int16_t)((b[1] << 8) | b[0]);   // X  (little-endian; QMC order is X,Y,Z)
  my = (int16_t)((b[3] << 8) | b[2]);   // Y
  mz = (int16_t)((b[5] << 8) | b[4]);   // Z
#endif
  return true;
}

// ----------------------- Rangefinder -----------------------
int32_t readRawMm() {
  uint16_t mm = sensor.readRangeContinuousMillimeters();
  if (sensor.timeoutOccurred())     return -1;
  if (mm == 0 || mm > MAX_VALID_MM) return -1;
  return (int32_t)mm;
}
int32_t movingAverage(int32_t s) {
  static int32_t buf[64]; static uint8_t idx = 0, count = 0;
  buf[idx] = s; idx = (idx + 1) % FILTER_WINDOW; if (count < FILTER_WINDOW) count++;
  int32_t sum = 0; for (uint8_t i = 0; i < count; i++) sum += buf[i];
  return sum / count;
}

void setup() {
  if (DEBUG_USB) Serial.begin(115200);
  FC.begin(FC_BAUD, SERIAL_8N1, PIN_FC_RX, PIN_FC_TX);
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  i2cScan();                 // tells you which mag chip is on the bus

  sensor.setTimeout(100);    // rangefinder
  if (!sensor.init()) { if (DEBUG_USB) Serial.println("VL53L0X not found"); }
  else {
    sensor.setSignalRateLimit(0.1);
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    sensor.setMeasurementTimingBudget(33000);
    sensor.startContinuous();
  }

  magInit();                 // magnetometer
}

void loop() {
  static uint32_t last = 0;
  const uint32_t period = 1000 / SEND_HZ;
  if (millis() - last < period) return;
  last = millis();

  // ---- Rangefinder ----
  int32_t raw = readRawMm(), out;
  if (raw < 0) { out = -1; sendRangefinder(0, -1); }
  else         { out = movingAverage(raw); sendRangefinder(255, out); }

  // ---- Magnetometer ----
  int16_t mx, my, mz;
  if (magReadRaw(mx, my, mz)) {
    // ===== AXIS ALIGNMENT — edit ONLY these 3 lines after the orientation check =====
    int16_t bx =  mx;   // body X = forward
    int16_t by =  my;   // body Y = right
    int16_t bz =  mz;   // body Z = down
    // (examples to fix it:  int16_t bx = -my;  int16_t by = mx;  int16_t bz = -mz;)
    // ===============================================================================
    sendCompass(bx, by, bz);
    if (DEBUG_USB) Serial.printf("dist=%ld mm | mag x=%d y=%d z=%d\n", out, bx, by, bz);
  } else if (DEBUG_USB) {
    Serial.printf("dist=%ld mm | mag READ FAIL\n", out);
  }
}
