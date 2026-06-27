/*
 * MSP Rangefinder Bridge v2  —  XIAO ESP32-S3  ->  iNav (F405)
 * ----------------------------------------------------------------
 * v2 adds an optional MOVING-AVERAGE filter on the distance before sending it
 * to iNav.
 *
 * TRADE-OFF: a moving average reduces noise by ~1/sqrt(N) but ADDS LATENCY of
 * about (N-1)/2 samples. At 30 Hz, N=5 => ~67 ms of delay in the altitude loop.
 * Latency in the feedback path can WORSEN a control-loop oscillation, so keep
 * the window small and treat this as an experiment.
 *   - Set FILTER_WINDOW = 1 to disable the filter (raw passthrough) for A/B testing.
 *   - The serial monitor prints raw and filtered together so you can verify the
 *     smoothing on the bench (fixed distance) BEFORE flying.
 *
 * Sensor assumed: VL53L0X (~2 m). VL53L1X: swap class + use read(). TFmini: rewrite readRawMm().
 *
 * Wiring (XIAO ESP32-S3):
 *   Sensor SDA -> D4 (GPIO5)   Sensor SCL -> D5 (GPIO6)   Sensor VCC -> 3V3   GND -> GND
 *   ESP TX D6 (GPIO43) -> FC UART3 RX        ESP GND <-> FC GND (shared ground required)
 *   FC: Ports -> MSP on UART3 @115200 ;  CLI: set rangefinder_hardware = MSP / save
 *
 * Library: "VL53L0X" by Pololu.   Board: XIAO_ESP32S3.
 */

#include <Wire.h>
#include <VL53L0X.h>

// ---------------- User config ----------------
static const uint32_t FC_BAUD       = 115200;
static const int      PIN_SDA       = 5;        // D4
static const int      PIN_SCL       = 6;        // D5
static const int      PIN_FC_TX     = 43;       // D6
static const int      PIN_FC_RX     = 44;       // D7
static const uint16_t SEND_HZ       = 30;
static const int32_t  MAX_VALID_MM  = 2000;
static const uint8_t  FILTER_WINDOW = 10;        // 1 = filter OFF (raw). Try 3 / 5 / 8. Keep <= 15.
static const bool     DEBUG_USB     = true;
// ---------------------------------------------

HardwareSerial FC(1);
VL53L0X sensor;

static const uint8_t  MSP_HDR_X               = 'X';
static const uint8_t  MSP_DIR_TO_FC           = '<';
static const uint16_t MSP2_SENSOR_RANGEFINDER = 0x1F01;

static uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a) {
  crc ^= a;
  for (uint8_t i = 0; i < 8; i++) {
    if (crc & 0x80) crc = (uint8_t)((crc << 1) ^ 0xD5);
    else            crc = (uint8_t)(crc << 1);
  }
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

void sendRangefinder(uint8_t quality, int32_t distanceMm) {
  uint8_t p[5];
  p[0] = quality;
  p[1] = (uint8_t)( distanceMm        & 0xFF);
  p[2] = (uint8_t)((distanceMm >> 8 ) & 0xFF);
  p[3] = (uint8_t)((distanceMm >> 16) & 0xFF);
  p[4] = (uint8_t)((distanceMm >> 24) & 0xFF);
  mspSend(MSP2_SENSOR_RANGEFINDER, p, sizeof(p));
}

// Raw reading: mm, or -1 if invalid / out of range.
int32_t readRawMm() {
  uint16_t mm = sensor.readRangeContinuousMillimeters();   // VL53L1X: sensor.read();
  if (sensor.timeoutOccurred())     return -1;
  if (mm == 0 || mm > MAX_VALID_MM) return -1;
  return (int32_t)mm;
}

// Moving average over the last (up to FILTER_WINDOW) VALID samples.
// Only valid samples are ever pushed here, so a bad reading never pollutes the average.
int32_t movingAverage(int32_t sample) {
  static int32_t buf[64];                 // large enough for any sane window
  static uint8_t idx = 0, count = 0;
  buf[idx] = sample;
  idx = (idx + 1) % FILTER_WINDOW;
  if (count < FILTER_WINDOW) count++;
  int32_t sum = 0;
  for (uint8_t i = 0; i < count; i++) sum += buf[i];
  return sum / count;
}

void setup() {
  if (DEBUG_USB) Serial.begin(115200);
  FC.begin(FC_BAUD, SERIAL_8N1, PIN_FC_RX, PIN_FC_TX);
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  sensor.setTimeout(100);
  if (!sensor.init()) {
    if (DEBUG_USB) Serial.println("VL53L0X not found - check I2C wiring");
  } else {
    sensor.setSignalRateLimit(0.1);
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    sensor.setMeasurementTimingBudget(33000);   // ~30 Hz
    sensor.startContinuous();
  }
}

void loop() {
  static uint32_t last = 0;
  const uint32_t period = 1000 / SEND_HZ;
  if (millis() - last < period) return;
  last = millis();

  int32_t raw = readRawMm();
  int32_t out;
  if (raw < 0) {
    out = -1;                       // invalid: don't feed the filter; tell iNav out-of-range
    sendRangefinder(0, -1);
  } else {
    out = movingAverage(raw);
    sendRangefinder(255, out);
  }
  if (DEBUG_USB) Serial.printf("raw=%ld mm   filt=%ld mm\n", raw, out);
}
