/*
 * XIAO ESP32-S3 Sense — WiFi camera stream
 * ----------------------------------------
 * Streams the OV2640 camera as MJPEG over WiFi. View it in a browser on your PC.
 *
 * MODE: WiFi Access Point (the XIAO makes its own network — no router needed, good
 * for the field). Connect your PC to the network below, then open the IP printed on
 * the serial monitor (default http://192.168.4.1/).
 *
 * ** STANDALONE TEST SKETCH — it does NOT run the MSP sensor bridge. **
 * Flashing this replaces the bridge. Running camera + bridge together (dual-core)
 * is the next step, once the stream is confirmed working.
 *
 * Arduino IDE settings (IMPORTANT):
 *   Board:  XIAO_ESP32S3
 *   Tools -> PSRAM:  "OPI PSRAM"            <-- REQUIRED, camera init fails without it
 *   (if the sketch won't fit) Tools -> Partition Scheme: "Huge APP (3MB No OTA)"
 *
 * Range note: WiFi from the XIAO antenna reaches only tens of meters line-of-sight and
 * drops fast — fine for bench/close range and monitoring (~100-300 ms latency), not a
 * long-range or low-latency FPV link.
 */

#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"

// ---------------- WiFi AP config ----------------
const char* AP_SSID = "DRONE_CAM";
const char* AP_PASS = "drone12345";        // must be >= 8 chars; change it
// ------------------------------------------------

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

httpd_handle_t server = NULL;

#define PART_BOUNDARY "frameboundary"
static const char* STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY     = "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART         = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

// ---- MJPEG stream handler ----
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
    if (res != ESP_OK) break;       // client disconnected -> stop this stream
  }
  return res;
}

// ---- Landing page ----
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
    Serial.println("HTTP server started.");
  } else {
    Serial.println("HTTP server failed to start.");
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
  config.pin_sccb_sda = SIOD_GPIO_NUM; config.pin_sccb_scl = SIOC_GPIO_NUM;  // older cores: pin_sscb_sda/scl
  config.pin_pwdn  = PWDN_GPIO_NUM;   config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size   = FRAMESIZE_VGA;   // 640x480. QVGA = smoother/faster; SVGA/HD = more detail
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode    = CAMERA_GRAB_LATEST;
  config.fb_location  = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;               // lower = better quality but bigger frames
  config.fb_count     = 2;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x  (is PSRAM set to OPI PSRAM?)\n", err);
    return false;
  }
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(300);

  if (!startCamera()) return;

  sensor_t *s = esp_camera_sensor_get();
  s->set_vflip(s, 1);    // voltea vertical
  s->set_hmirror(s, 1);  // voltea horizontal

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  IPAddress ip = WiFi.softAPIP();

  Serial.println("\n--- Drone camera ready ---");
  Serial.printf("1) Connect your PC to WiFi:  %s   (password: %s)\n", AP_SSID, AP_PASS);
  Serial.printf("2) Open in your browser:     http://%s/\n", ip.toString().c_str());

  startServer();
}

void loop() {
  delay(1000);   // all work happens in the HTTP server task
}
