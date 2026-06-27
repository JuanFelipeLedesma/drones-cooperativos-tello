/*
 * Drone XIAO firmware v5 — Sensors (MSP) + Camera (WiFi), integrated
 * -----------------------------------------------------------------
 * Runs concurrently on the dual-core ESP32-S3:
 *   - Sensor bridge (dedicated FreeRTOS task, 30 Hz):
 *       VL53L0X (0x29) -> MSP2_SENSOR_RANGEFINDER (0x1F01)
 *       QMC5883L (0x0D) -> MSP2_SENSOR_COMPASS    (0x1F04)   to the FC on UART3 @115200
 *   - Camera: OV2640 MJPEG stream over WiFi (Access Point), viewable in a browser.
 *
 * Disjoint hardware -> no contention, no locks:
 *   Sensors: I2C0 (Wire) on D4/D5,  UART1 to FC on D6.
 *   Camera : own SCCB forced to I2C1 (sccb_i2c_port=1) + camera data pins + WiFi.
 *
 * Confirmed on this airframe (baked in):
 *   Mag alignment: bx=my, by=mx, bz=-mz   |   Camera 180-degree flip: vflip + hmirror.
 *
 * Arduino IDE: Board XIAO_ESP32S3 ; Tools -> PSRAM: "OPI PSRAM" (required) ;
 *   if it won't fit, Partition Scheme: "Huge APP (3MB No OTA)".
 * Camera: connect PC to WiFi "DRONE_CAM" (pass drone12345), open http://192.168.4.1/
 * Library: "VL53L0X" by Pololu.
 */

#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"
#include <Wire.h>
#include <VL53L0X.h>

// ---------------- Config ----------------
static const uint32_t FC_BAUD      = 115200;
static const int      PIN_SDA      = 5;      // D4 (sensors, I2C0)
static const int      PIN_SCL      = 6;      // D5
static const int      PIN_FC_TX    = 43;     // D6 -> FC UART3 RX
static const int      PIN_FC_RX    = 44;     // D7
static const uint16_t SEND_HZ      = 30;
static const int32_t  MAX_VALID_MM = 2000;
static const bool     DEBUG_USB    = true;

const char* AP_SSID = "DRONE_CAM";
const char* AP_PASS = "drone12345";          // >= 8 chars
// ----------------------------------------

static const uint8_t QMC_ADDR = 0x0D;

// ---- Camera pins: XIAO ESP32-S3 Sense ----
#define PWDN_GPIO_NUM   -1
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM   10
#define SIOD_GPIO_NUM   40
#define SIOC_GPIO_NUM   39
#define Y9_GPIO_NUM     48
#define Y8_GPIO_NUM     11
#define Y7_GPIO_NUM     12
#define Y6_GPIO_NUM     14
#define Y5_GPIO_NUM     16
#define Y4_GPIO_NUM     18
#define Y3_GPIO_NUM     17
#define Y2_GPIO_NUM     15
#define VSYNC_GPIO_NUM  38
#define HREF_GPIO_NUM   47
#define PCLK_GPIO_NUM   13

HardwareSerial FC(1);
VL53L0X sensor;
bool magPresent = false;
httpd_handle_t server = NULL;

static const uint8_t  MSP_HDR_X               = 'X';
static const uint8_t  MSP_DIR_TO_FC           = '<';
static const uint16_t MSP2_SENSOR_RANGEFINDER = 0x1F01;
static const uint16_t MSP2_SENSOR_COMPASS     = 0x1F04;

// ============================ MSP v2 ============================
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
                   (uint8_t)(mm & 0xFF),         (uint8_t)((mm >> 8)  & 0xFF),
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

// ===================== Sensors (I2C0 / Wire) =====================
void i2cWrite(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}
bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}
void magInit() {
  i2cWrite(QMC_ADDR, 0x0B, 0x01);
  i2cWrite(QMC_ADDR, 0x09, 0x1D);   // OSR=512, 8G, 200Hz, continuous
}
bool magReadRaw(int16_t &mx, int16_t &my, int16_t &mz) {
  uint8_t b[6];
  Wire.beginTransmission(QMC_ADDR); Wire.write(0x00); Wire.endTransmission(true);  // STOP
  if (Wire.requestFrom((int)QMC_ADDR, 6) != 6) return false;
  for (int i = 0; i < 6; i++) b[i] = Wire.read();
  mx = (int16_t)((b[1] << 8) | b[0]);
  my = (int16_t)((b[3] << 8) | b[2]);
  mz = (int16_t)((b[5] << 8) | b[4]);
  return true;
}
int32_t readDistanceMm() {
  uint16_t mm = sensor.readRangeContinuousMillimeters();
  if (sensor.timeoutOccurred())     return -1;
  if (mm == 0 || mm > MAX_VALID_MM) return -1;
  return (int32_t)mm;
}

// ========================= Camera + WiFi =========================
#define PART_BOUNDARY "frameboundary"
static const char* STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY     = "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART         = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static esp_err_t stream_handler(httpd_req_t *req) {
  esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  char part_buf[64];
  while (true) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) { res = ESP_FAIL; break; }
    res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    if (res == ESP_OK) {
      int hlen = snprintf(part_buf, sizeof(part_buf), STREAM_PART, fb->len);
      res = httpd_resp_send_chunk(req, part_buf, hlen);
    }
    if (res == ESP_OK)
      res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    if (res != ESP_OK) break;
  }
  return res;
}
static esp_err_t index_handler(httpd_req_t *req) {
  const char *html =
    "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>Drone Cam</title><style>body{margin:0;background:#111;text-align:center}"
    "img{width:100%;max-width:960px;height:auto}</style></head>"
    "<body><img src='/stream'></body></html>";
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, html, strlen(html));
}
void startServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.ctrl_port   = 32768;
  if (httpd_start(&server, &config) == ESP_OK) {
    httpd_uri_t index_uri  = { .uri = "/",       .method = HTTP_GET, .handler = index_handler,  .user_ctx = NULL };
    httpd_uri_t stream_uri = { .uri = "/stream", .method = HTTP_GET, .handler = stream_handler, .user_ctx = NULL };
    httpd_register_uri_handler(server, &index_uri);
    httpd_register_uri_handler(server, &stream_uri);
  }
}
bool startCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk  = XCLK_GPIO_NUM;   config.pin_pclk  = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;  config.pin_href  = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM; config.pin_sccb_scl = SIOC_GPIO_NUM;  // older cores: pin_sscb_*
  config.pin_pwdn  = PWDN_GPIO_NUM;   config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size   = FRAMESIZE_VGA;   // QVGA = smoother; SVGA/HD = more detail
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode    = CAMERA_GRAB_LATEST;
  config.fb_location  = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count     = 2;
  config.sccb_i2c_port = 1;              // camera SCCB on I2C1, separate from sensors' Wire (I2C0)

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) { Serial.printf("Camera init failed: 0x%x (PSRAM = OPI PSRAM?)\n", err); return false; }

  sensor_t *s = esp_camera_sensor_get();   // 180-degree flip (confirmed)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
  return true;
}

// ===================== Sensor task (FreeRTOS) =====================
void vTaskSensors(void *pv) {
  const TickType_t period = pdMS_TO_TICKS(1000 / SEND_HZ);
  TickType_t last = xTaskGetTickCount();
  uint16_t dbg = 0;
  for (;;) {
    int32_t d = readDistanceMm();
    if (d < 0) sendRangefinder(0, -1);
    else       sendRangefinder(255, d);

    int16_t mx, my, mz;
    bool magOk = (magPresent && magReadRaw(mx, my, mz));
    if (magOk) {
      // ===== Confirmed alignment for this airframe =====
      int16_t bx =  my;   // body X = forward
      int16_t by =  mx;   // body Y = right
      int16_t bz = -mz;   // body Z = down
      sendCompass(bx, by, bz);
    }

    if (DEBUG_USB && (++dbg >= 15)) {     // ~2 Hz status, doesn't flood serial
      dbg = 0;
      if (magOk) Serial.printf("dist=%ld mm | mag x=%d y=%d z=%d\n", d, mx, my, mz);
      else       Serial.printf("dist=%ld mm | mag: --\n", d);
    }
    vTaskDelayUntil(&last, period);
  }
}

void setup() {
  if (DEBUG_USB) Serial.begin(115200);

  // --- Sensors first (flight-critical) ---
  FC.begin(FC_BAUD, SERIAL_8N1, PIN_FC_RX, PIN_FC_TX);
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  Wire.setTimeOut(25);

  sensor.setTimeout(100);
  if (!sensor.init()) { if (DEBUG_USB) Serial.println("VL53L0X (0x29) not found"); }
  else {
    sensor.setSignalRateLimit(0.1);
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    sensor.setMeasurementTimingBudget(33000);
    sensor.startContinuous();
  }
  magPresent = i2cPresent(QMC_ADDR);
  if (magPresent) { magInit(); if (DEBUG_USB) Serial.println("QMC5883L (0x0D) OK"); }
  else if (DEBUG_USB) Serial.println("QMC5883L (0x0D) not found");

  // --- Camera + WiFi (best-effort; sensors run regardless) ---
  if (startCamera()) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    IPAddress ip = WiFi.softAPIP();
    if (DEBUG_USB)
      Serial.printf("Camera ready. WiFi: %s (pass %s) -> http://%s/\n",
                    AP_SSID, AP_PASS, ip.toString().c_str());
    startServer();
  } else if (DEBUG_USB) {
    Serial.println("Camera disabled - sensor bridge still running.");
  }

  // --- Launch the sensor bridge task (your convention: vTask* + xTaskCreate) ---
  xTaskCreate(vTaskSensors, "vTaskSensors", 8192, NULL, 6, NULL);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));   // nothing here; work is in the task + httpd
}
