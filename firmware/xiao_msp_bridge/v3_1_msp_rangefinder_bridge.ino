/*
 * MSP Rangefinder + Compass Bridge v3.1 — XIAO ESP32-S3 -> iNav (F405)
 * --------------------------------------------------------------------
 * v3.1 FIX: the magnetometer is now FAIL-SAFE. It is probed once at boot and only
 * read if it actually answers on the bus. Reads use a STOP (not repeated-start)
 * and the I2C bus has a timeout. A missing or misconfigured mag can no longer hang
 * the shared I2C bus or take down the rangefinder. (This was the v3 regression.)
 *
 * Streams to iNav over ONE MSP link (UART3 @115200):
 *   - VL53L0X rangefinder   -> MSP2_SENSOR_RANGEFINDER (0x1F01)
 *   - HMC5883L/QMC5883L mag  -> MSP2_SENSOR_COMPASS    (0x1F04)   [only if detected]
 *
 * Sensors share the XIAO Wire bus (different addresses):
 *   VL53L0X = 0x29   HMC5883L = 0x1E   QMC5883L = 0x0D
 *
 * Confirm on YOUR hardware (watch the boot I2C scan on the serial monitor):
 *   - mag address 0x1E -> MAG_TYPE = MAG_HMC5883L ;  0x0D -> MAG_TYPE = MAG_QMC5883L
 *   - axis alignment: verify with the bench orientation test AND the iNav heading check.
 *
 * Wiring: both sensors SDA->D4(GPIO5) SCL->D5(GPIO6) VCC->3V3 GND->GND ;
 *         ESP TX D6(GPIO43)->FC UART3 RX ; shared GND ; route M100 compass SDA/SCL to the XIAO.
 *
 * iNav: Ports->MSP on UART3 @115200 ; CLI: set rangefinder_hardware=MSP / set mag_hardware=MSP
 *       / set align_mag=CW0 / save
 *
 * Library: "VL53L0X" by Pololu.   Board: XIAO_ESP32S3.
 */

#include <Wire.h>
#include <VL53L0X.h>

// ---------------- Mag chip select ----------------
#define MAG_HMC5883L 0
#define MAG_QMC5883L 1
#define MAG_TYPE     MAG_QMC5883L     // set per the I2C scanner result at boot
// -------------------------------------------------

// ---------------- User config ----------------
static const uint32_t FC_BAUD       = 115200;
static const int      PIN_SDA       = 5;        // D4
static const int      PIN_SCL       = 6;        // D5
static const int      PIN_FC_TX     = 43;       // D6
static const int      PIN_FC_RX     = 44;       // D7
static const uint16_t SEND_HZ       = 30;
static const int32_t  MAX_VALID_MM  = 2000;
static const uint8_t  FILTER_WINDOW = 1;        // rangefinder MA: 1 = OFF
static const bool     DEBUG_USB     = true;
// ---------------------------------------------

static const uint8_t HMC_ADDR = 0x1E;
static const uint8_t QMC_ADDR = 0x0D;
static const uint8_t MAG_ADDR = (MAG_TYPE == MAG_HMC5883L) ? HMC_ADDR : QMC_ADDR;

HardwareSerial FC(1);
VL53L0X sensor;
bool magPresent = false;              // set in setup() after probing the bus

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
void sendRangefinder(uint8_t quality, int32_t mm) {
  uint8_t p[5] = { quality,
                   (uint8_t)(mm & 0xFF),     (uint8_t)((mm >> 8)  & 0xFF),
                   (uint8_t)((mm >> 16) & 0xFF), (uint8_t)((mm >> 24) & 0xFF) };
  mspSend(MSP2_SENSOR_RANGEFINDER, p, sizeof(p));
}
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

// ----------------------- I2C helpers -----------------------
void i2cWrite(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}
bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}
void i2cScan() {
  if (!DEBUG_USB) return;
  Serial.println("I2C scan:");
  bool any = false;
  for (uint8_t a = 1; a < 127; a++) {
    if (i2cPresent(a)) {
      any = true;
      Serial.print("  found 0x"); Serial.print(a, HEX);
      if (a == 0x29) Serial.print("  (VL53L0X)");
      if (a == 0x1E) Serial.print("  (HMC5883L -> MAG_TYPE = MAG_HMC5883L)");
      if (a == 0x0D) Serial.print("  (QMC5883L -> MAG_TYPE = MAG_QMC5883L)");
      Serial.println();
    }
  }
  if (!any) Serial.println("  (nothing found - check SDA/SCL wiring & power)");
}

// ----------------------- Magnetometer -----------------------
void magInit() {
#if MAG_TYPE == MAG_HMC5883L
  i2cWrite(HMC_ADDR, 0x00, 0x70);
  i2cWrite(HMC_ADDR, 0x01, 0x20);
  i2cWrite(HMC_ADDR, 0x02, 0x00);
#else
  i2cWrite(QMC_ADDR, 0x0B, 0x01);
  i2cWrite(QMC_ADDR, 0x09, 0x1D);
#endif
}
bool magReadRaw(int16_t &mx, int16_t &my, int16_t &mz) {
  uint8_t b[6];
#if MAG_TYPE == MAG_HMC5883L
  Wire.beginTransmission(HMC_ADDR); Wire.write(0x03); Wire.endTransmission(true);  // STOP (was repeated-start)
  if (Wire.requestFrom((int)HMC_ADDR, 6) != 6) return false;
  for (int i = 0; i < 6; i++) b[i] = Wire.read();
  mx = (int16_t)((b[0] << 8) | b[1]);   // X  (HMC order X,Z,Y; big-endian)
  mz = (int16_t)((b[2] << 8) | b[3]);   // Z
  my = (int16_t)((b[4] << 8) | b[5]);   // Y
#else
  Wire.beginTransmission(QMC_ADDR); Wire.write(0x00); Wire.endTransmission(true);  // STOP
  if (Wire.requestFrom((int)QMC_ADDR, 6) != 6) return false;
  for (int i = 0; i < 6; i++) b[i] = Wire.read();
  mx = (int16_t)((b[1] << 8) | b[0]);   // X  (QMC order X,Y,Z; little-endian)
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
  Wire.setTimeOut(25);          // ms: never block the bus forever on a dead device

  i2cScan();

  // Rangefinder
  sensor.setTimeout(100);
  if (!sensor.init()) { if (DEBUG_USB) Serial.println("VL53L0X not found"); }
  else {
    sensor.setSignalRateLimit(0.1);
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    sensor.setMeasurementTimingBudget(33000);
    sensor.startContinuous();
  }

  // Magnetometer — only initialize if it actually answers on the bus
  magPresent = i2cPresent(MAG_ADDR);
  if (magPresent) {
    magInit();
    if (DEBUG_USB) { Serial.print("MAG detected at 0x"); Serial.println(MAG_ADDR, HEX); }
  } else if (DEBUG_USB) {
    Serial.print("MAG NOT detected at 0x"); Serial.print(MAG_ADDR, HEX);
    Serial.println(" -> skipping mag (rangefinder still runs). Check wiring / MAG_TYPE.");
  }
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

  // ---- Magnetometer (only if present) ----
  if (magPresent) {
    int16_t mx, my, mz;
    if (magReadRaw(mx, my, mz)) {
      // ===== AXIS ALIGNMENT — edit ONLY these 3 lines after the orientation check =====
      int16_t bx =  mx;   // body X = forward
      int16_t by =  my;   // body Y = right
      int16_t bz =  mz;   // body Z = down
      // (example fix:  int16_t bx = -my;  int16_t by = mx;  int16_t bz = -mz;)
      // ===============================================================================
      sendCompass(bx, by, bz);
      if (DEBUG_USB) Serial.printf("dist=%ld mm | mag x=%d y=%d z=%d\n", out, bx, by, bz);
    }
  } else {
    if (DEBUG_USB) Serial.printf("dist=%ld mm | mag: none\n", out);
  }
}
