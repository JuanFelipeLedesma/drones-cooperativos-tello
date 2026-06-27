/*
 * MSP Rangefinder Bridge  —  XIAO ESP32-S3  ->  iNav (F405)
 * ---------------------------------------------------------
 * Reads a VL53L0X laser ToF sensor over I2C and streams the distance to an
 * iNav flight controller using the MSP v2 message MSP2_SENSOR_RANGEFINDER (0x1F01).
 * The ESP is only a BRIDGE: the flight control (PID, alt-hold, pos-hold) stays in iNav.
 *
 * Sensor assumed: VL53L0X (~2 m range).
 *   - VL53L1X (~4 m):  include <VL53L1X.h>, swap the class, sensor.read() returns mm.
 *   - TFmini (serial): leave everything and just rewrite readDistanceMm() to read its UART.
 *
 * Wiring (XIAO ESP32-S3 silkscreen):
 *   Sensor SDA  -> D4  (GPIO5)
 *   Sensor SCL  -> D5  (GPIO6)
 *   Sensor VCC  -> 3V3
 *   Sensor GND  -> GND
 *   ESP TX  D6 (GPIO43) -> FC UART RX      (data flows ESP -> FC)
 *   ESP RX  D7 (GPIO44) <- FC UART TX      (optional; we only transmit)
 *   ESP GND            <-> FC GND          (MUST share ground)
 *   Power XIAO from the FC 5V BEC (5V pad) or its own source.
 *   UART pins are reassignable below if D6/D7 clash with anything on your build.
 *
 * iNav side:
 *   Ports tab:  enable MSP on the wired UART @ 115200.
 *   CLI:        set rangefinder_hardware = MSP   ->   save
 *   Verify in Configurator -> Sensors (sonar trace) before flying.
 *
 * Library: "VL53L0X" by Pololu (Arduino Library Manager).
 * Board:   XIAO_ESP32S3 (Seeed esp32 boards).
 */

#include <Wire.h>
#include <VL53L0X.h>

// ---------------- User config ----------------
static const uint32_t FC_BAUD      = 115200;   // must match iNav Ports tab
static const int      PIN_SDA      = 5;        // D4
static const int      PIN_SCL      = 6;        // D5
static const int      PIN_FC_TX    = 43;       // D6  (ESP -> FC RX)
static const int      PIN_FC_RX    = 44;       // D7  (FC -> ESP, unused for now)
static const uint16_t SEND_HZ      = 30;       // update rate to the FC
static const int32_t  MAX_VALID_MM = 2000;     // ignore readings beyond sensor range
static const bool     DEBUG_USB    = true;     // print distance on USB serial
// ---------------------------------------------

HardwareSerial FC(1);   // UART1
VL53L0X sensor;

// MSP v2
static const uint8_t  MSP_HDR_X               = 'X';
static const uint8_t  MSP_DIR_TO_FC           = '<';
static const uint16_t MSP2_SENSOR_RANGEFINDER = 0x1F01;

// CRC8 / DVB-S2 (MSP v2 checksum)
static uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a) {
  crc ^= a;
  for (uint8_t i = 0; i < 8; i++) {
    if (crc & 0x80) crc = (uint8_t)((crc << 1) ^ 0xD5);
    else            crc = (uint8_t)(crc << 1);
  }
  return crc;
}

// Send one MSP v2 frame: $ X < flag func_lo func_hi size_lo size_hi <payload> crc
// CRC covers flag .. last payload byte.
void mspSend(uint16_t function, const uint8_t *payload, uint16_t len) {
  uint8_t flag = 0;
  uint8_t crc  = 0;

  FC.write('$');
  FC.write(MSP_HDR_X);
  FC.write(MSP_DIR_TO_FC);

  FC.write(flag);                      crc = crc8_dvb_s2(crc, flag);
  uint8_t fl = function & 0xFF;        FC.write(fl); crc = crc8_dvb_s2(crc, fl);
  uint8_t fh = (function >> 8) & 0xFF; FC.write(fh); crc = crc8_dvb_s2(crc, fh);
  uint8_t sl = len & 0xFF;             FC.write(sl); crc = crc8_dvb_s2(crc, sl);
  uint8_t sh = (len >> 8) & 0xFF;      FC.write(sh); crc = crc8_dvb_s2(crc, sh);

  for (uint16_t i = 0; i < len; i++) {
    FC.write(payload[i]);
    crc = crc8_dvb_s2(crc, payload[i]);
  }
  FC.write(crc);
}

// Payload: uint8 quality, int32 distance_mm (little-endian). distance < 0 => out of range.
void sendRangefinder(uint8_t quality, int32_t distanceMm) {
  uint8_t p[5];
  p[0] = quality;
  p[1] = (uint8_t)( distanceMm        & 0xFF);
  p[2] = (uint8_t)((distanceMm >> 8 ) & 0xFF);
  p[3] = (uint8_t)((distanceMm >> 16) & 0xFF);
  p[4] = (uint8_t)((distanceMm >> 24) & 0xFF);
  mspSend(MSP2_SENSOR_RANGEFINDER, p, sizeof(p));
}

// Returns distance in mm, or -1 if invalid / out of range.
int32_t readDistanceMm() {
  uint16_t mm = sensor.readRangeContinuousMillimeters();   // VL53L1X: sensor.read();
  if (sensor.timeoutOccurred())     return -1;
  if (mm == 0 || mm > MAX_VALID_MM) return -1;             // ~8190 = no target
  return (int32_t)mm;
}

void setup() {
  if (DEBUG_USB) Serial.begin(115200);                     // USB CDC debug
  FC.begin(FC_BAUD, SERIAL_8N1, PIN_FC_RX, PIN_FC_TX);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  sensor.setTimeout(100);
  if (!sensor.init()) {
    if (DEBUG_USB) Serial.println("VL53L0X not found - check I2C wiring/address");
    // keep going so the FC link still initializes
  } else {
    // Long-range profile for better reach toward 2 m
    sensor.setSignalRateLimit(0.1);
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    sensor.setMeasurementTimingBudget(33000);              // ~33 ms -> ~30 Hz
    sensor.startContinuous();
  }
}

void loop() {
  static uint32_t last = 0;
  const uint32_t period = 1000 / SEND_HZ;
  uint32_t now = millis();
  if (now - last < period) return;
  last = now;

  int32_t d = readDistanceMm();
  if (d < 0) {
    sendRangefinder(0, -1);            // tell iNav: out of range -> ignore
  } else {
    sendRangefinder(255, d);           // good reading
    if (DEBUG_USB) Serial.printf("dist = %ld mm\n", d);
  }
}
