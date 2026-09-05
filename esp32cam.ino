/*
  ESP32-CAM + MLX90640 Combined Rescue Robot Imaging Hub

  Hardware:
    AI-Thinker ESP32-CAM + OV2640 camera
    MLX90640 thermal camera

  Wi-Fi:
    The ESP32-CAM creates its own Wi-Fi network.
    SSID: RESCUE-ROBOT
    Password: 12345678
    Open: http://192.168.4.1

  MLX90640 I2C on this AI-Thinker ESP32-CAM:
    SDA -> GPIO13
    SCL -> GPIO14

  IMPORTANT:
    GPIO13/14 are normally used by the microSD interface on AI-Thinker boards.
    Do not use the SD-card interface at the same time as the MLX90640.
*/

#include "esp_camera.h"
#include <WiFi.h>
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include "fb_gfx.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_http_server.h"
#include <Wire.h>
#include <HTTPClient.h>

#include <MLX90640_I2C_Driver.h>
#include <MLX90640_API.h>

#include "webpage.h"

// ---------------- Camera model ----------------
#define CAMERA_MODEL_AI_THINKER

#if defined(CAMERA_MODEL_AI_THINKER)
  #define PWDN_GPIO_NUM     32
  #define RESET_GPIO_NUM    -1
  #define XCLK_GPIO_NUM      0
  #define SIOD_GPIO_NUM     26
  #define SIOC_GPIO_NUM     27

  #define Y9_GPIO_NUM       35
  #define Y8_GPIO_NUM       34
  #define Y7_GPIO_NUM       39
  #define Y6_GPIO_NUM       36
  #define Y5_GPIO_NUM       21
  #define Y4_GPIO_NUM       19
  #define Y3_GPIO_NUM       18
  #define Y2_GPIO_NUM        5
  #define VSYNC_GPIO_NUM    25
  #define HREF_GPIO_NUM     23
  #define PCLK_GPIO_NUM     22
#else
  #error "Camera model not selected"
#endif

// ---------------- MLX90640 I2C pins ----------------
// GPIO13/14 are being repurposed because the camera already uses GPIO21/22.
#define I2C_SDA 13
#define I2C_SCL 14

// ---------------- MLX90640 settings ----------------
static const uint8_t MLX90640_ADDR = 0x33;
static const float EMISSIVITY = 0.95f;
static const float TA_SHIFT = 8.0f;
static const uint8_t MLX_REFRESH_RATE = 0x04; // 8 Hz
static const uint8_t MLX_RESOLUTION = 0x02;   // 18-bit
static const uint32_t I2C_CLOCK_HZ = 800000UL;
static const uint32_t FRAME_INTERVAL_MS = 110;

// ---------------- Robot Wi-Fi AP ----------------
const char* AP_SSID = "RESCUE-ROBOT";
const char* AP_PASS = "12345678";
// ESP32 DevKit joins this AP and uses 192.168.4.2 as its static IP.

// ---------------- Camera streaming ----------------
#define PART_BOUNDARY "123456789000000000000987654321"

static const char* STREAM_CONTENT_TYPE =
  "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;

static const char* STREAM_BOUNDARY =
  "\r\n--" PART_BOUNDARY "\r\n";

static const char* STREAM_PART =
  "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t web_httpd = NULL;

// ---------------- Thermal state ----------------
paramsMLX90640 mlxParams;
float thermalPixels[32 * 24];
uint16_t mlxFrame[834];

float minTemp = 0.0f;
float maxTemp = 0.0f;
float avgTemp = 0.0f;
float centerTemp = 0.0f;
float ambientTemp = 0.0f;

uint16_t minIndex = 0;
uint16_t maxIndex = 0;
uint8_t minX = 0;
uint8_t minY = 0;
uint8_t maxX = 0;
uint8_t maxY = 0;

bool sensorReady = false;
bool frameValid = false;
uint32_t lastFrameRequestMs = 0;
uint32_t lastFrameCompleteMs = 0;
uint32_t frameCounter = 0;
uint32_t lastReadDurationMs = 0;
float measuredFps = 0.0f;
String lastError = "Starting";

bool isMLXConnected()
{
  Wire.beginTransmission(MLX90640_ADDR);
  return Wire.endTransmission() == 0;
}

void calculateStats()
{
  minTemp = thermalPixels[0];
  maxTemp = thermalPixels[0];
  avgTemp = 0.0f;
  minIndex = 0;
  maxIndex = 0;

  for (uint16_t i = 0; i < 768; i++)
  {
    const float t = thermalPixels[i];
    avgTemp += t;

    if (t < minTemp)
    {
      minTemp = t;
      minIndex = i;
    }

    if (t > maxTemp)
    {
      maxTemp = t;
      maxIndex = i;
    }
  }

  avgTemp /= 768.0f;

  minX = minIndex % 32;
  minY = minIndex / 32;
  maxX = maxIndex % 32;
  maxY = maxIndex / 32;

  centerTemp = (thermalPixels[367] + thermalPixels[368] +
                thermalPixels[399] + thermalPixels[400]) / 4.0f;
}

bool readMLX90640Frame()
{
  if (!sensorReady)
  {
    lastError = "MLX90640 not initialized";
    return false;
  }

  const uint32_t startMs = millis();

  // MLX90640 uses two subpages. Keep the same read sequence as the supplied thermal sketch.
  for (uint8_t subpage = 0; subpage < 2; subpage++)
  {
    int status = MLX90640_GetFrameData(MLX90640_ADDR, mlxFrame);
    if (status < 0)
    {
      lastError = "MLX90640_GetFrameData error: " + String(status);
      frameValid = false;
      return false;
    }

    ambientTemp = MLX90640_GetTa(mlxFrame, &mlxParams);
    const float reflectedTemp = ambientTemp - TA_SHIFT;
    MLX90640_CalculateTo(mlxFrame, &mlxParams, EMISSIVITY, reflectedTemp, thermalPixels);
  }

  calculateStats();

  const uint32_t nowMs = millis();
  lastReadDurationMs = nowMs - startMs;

  if (lastFrameCompleteMs > 0)
  {
    const uint32_t dt = nowMs - lastFrameCompleteMs;
    if (dt > 0)
    {
      const float instantFps = 1000.0f / (float)dt;
      measuredFps = (measuredFps <= 0.01f)
        ? instantFps
        : (0.75f * measuredFps + 0.25f * instantFps);
    }
  }

  lastFrameCompleteMs = nowMs;
  frameCounter++;
  frameValid = true;
  lastError = "OK";
  return true;
}

String makeJsonData()
{
  if (!frameValid)
  {
    String err = "{\"ok\":false,\"error\":\"";
    err += lastError;
    err += "\"}";
    return err;
  }

  String json;
  json.reserve(10500);
  json += "{\"ok\":true,";
  json += "\"frame\":" + String(frameCounter) + ",";
  json += "\"min\":" + String(minTemp, 2) + ",";
  json += "\"max\":" + String(maxTemp, 2) + ",";
  json += "\"avg\":" + String(avgTemp, 2) + ",";
  json += "\"center\":" + String(centerTemp, 2) + ",";
  json += "\"ambient\":" + String(ambientTemp, 2) + ",";
  json += "\"fps\":" + String(measuredFps, 2) + ",";
  json += "\"readMs\":" + String(lastReadDurationMs) + ",";
  json += "\"minX\":" + String(minX) + ",";
  json += "\"minY\":" + String(minY) + ",";
  json += "\"maxX\":" + String(maxX) + ",";
  json += "\"maxY\":" + String(maxY) + ",";
  json += "\"pixels\":[";

  for (uint16_t i = 0; i < 768; i++)
  {
    if (i) json += ',';
    json += String(thermalPixels[i], 2);
  }

  json += "]}";
  return json;
}

static esp_err_t root_handler(httpd_req_t *req)
{
  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, WEBPAGE_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t data_handler(httpd_req_t *req)
{
  String json = makeJsonData();
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  httpd_resp_set_hdr(req, "Pragma", "no-cache");
  return httpd_resp_send(req, json.c_str(), json.length());
}

static String getQueryValue(httpd_req_t *req, const char *key)
{
  char query[128];
  size_t query_len = httpd_req_get_url_query_len(req);
  if (query_len == 0 || query_len >= sizeof(query)) return "";

  if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK)
    return "";

  char value[32];
  if (httpd_query_key_value(query, key, value, sizeof(value)) != ESP_OK)
    return "";

  return String(value);
}

static void sendMotorCommandToDevKit(const String &command)
{
  // The DevKit joins the ESP32-CAM access point using this fixed address.
  const String url = "http://192.168.4.2/cmd?move=" + command;

  HTTPClient http;
  http.setConnectTimeout(120);
  http.setTimeout(180);

  if (http.begin(url))
  {
    int code = http.GET();
    if (code <= 0)
      Serial.printf("DevKit command failed: %s\n", http.errorToString(code).c_str());
    http.end();
  }
  else
  {
    Serial.println("Could not create DevKit HTTP request");
  }
}

static esp_err_t move_handler(httpd_req_t *req)
{
  String command = getQueryValue(req, "cmd");
  command.toLowerCase();

  if (command != "forward" && command != "backward" &&
      command != "left" && command != "right" && command != "stop")
  {
    httpd_resp_set_status(req, "400 Bad Request");
    return httpd_resp_send(req, "Invalid command", HTTPD_RESP_USE_STRLEN);
  }

  sendMotorCommandToDevKit(command);

  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t health_handler(httpd_req_t *req)
{
  String health = "ESP32-CAM + MLX90640 Rescue Robot\n";
  health += "Sensor ready: " + String(sensorReady ? "yes" : "no") + "\n";
  health += "Frame valid: " + String(frameValid ? "yes" : "no") + "\n";
  health += "Frame count: " + String(frameCounter) + "\n";
  health += "Measured FPS: " + String(measuredFps, 2) + "\n";
  health += "Read duration ms: " + String(lastReadDurationMs) + "\n";
  health += "I2C SDA: GPIO" + String(I2C_SDA) + "\n";
  health += "I2C SCL: GPIO" + String(I2C_SCL) + "\n";
  health += "Last error: " + lastError + "\n";

  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_send(req, health.c_str(), health.length());
}

static esp_err_t stream_handler(httpd_req_t *req)
{
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  size_t jpg_buf_len = 0;
  uint8_t *jpg_buf = NULL;
  char part_buf[64];

  res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK)
    return res;

  while (true)
  {
    fb = esp_camera_fb_get();
    if (!fb)
    {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
    }
    else
    {
      if (fb->width > 400)
      {
        if (fb->format != PIXFORMAT_JPEG)
        {
          bool jpeg_converted = frame2jpg(fb, 80, &jpg_buf, &jpg_buf_len);
          esp_camera_fb_return(fb);
          fb = NULL;

          if (!jpeg_converted)
          {
            Serial.println("JPEG compression failed");
            res = ESP_FAIL;
          }
        }
        else
        {
          jpg_buf_len = fb->len;
          jpg_buf = fb->buf;
        }
      }
    }

    if (res == ESP_OK)
    {
      size_t hlen = snprintf(part_buf, sizeof(part_buf), STREAM_PART, jpg_buf_len);
      res = httpd_resp_send_chunk(req, part_buf, hlen);
    }

    if (res == ESP_OK)
      res = httpd_resp_send_chunk(req, (const char *)jpg_buf, jpg_buf_len);

    if (res == ESP_OK)
      res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));

    if (fb)
    {
      esp_camera_fb_return(fb);
      fb = NULL;
      jpg_buf = NULL;
    }
    else if (jpg_buf)
    {
      free(jpg_buf);
      jpg_buf = NULL;
    }

    if (res != ESP_OK)
      break;
  }

  return res;
}

void startWebServer()
{
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.max_uri_handlers = 8;

  if (httpd_start(&web_httpd, &config) != ESP_OK)
  {
    Serial.println("Web server start failed");
    return;
  }

  httpd_uri_t root_uri = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = root_handler,
    .user_ctx = NULL
  };

  httpd_uri_t stream_uri = {
    .uri = "/stream",
    .method = HTTP_GET,
    .handler = stream_handler,
    .user_ctx = NULL
  };

  httpd_uri_t data_uri = {
    .uri = "/data",
    .method = HTTP_GET,
    .handler = data_handler,
    .user_ctx = NULL
  };

  httpd_uri_t move_uri = {
    .uri = "/move",
    .method = HTTP_GET,
    .handler = move_handler,
    .user_ctx = NULL
  };

  httpd_uri_t health_uri = {
    .uri = "/health",
    .method = HTTP_GET,
    .handler = health_handler,
    .user_ctx = NULL
  };

  httpd_register_uri_handler(web_httpd, &root_uri);
  httpd_register_uri_handler(web_httpd, &move_uri);
  httpd_register_uri_handler(web_httpd, &stream_uri);
  httpd_register_uri_handler(web_httpd, &data_uri);
  httpd_register_uri_handler(web_httpd, &health_uri);

  Serial.println("Web server started");
  Serial.println("Movement commands: GET /move?cmd=forward|backward|left|right|stop");
}

bool initializeMLX90640()
{
  if (!isMLXConnected())
  {
    lastError = "MLX90640 not detected at I2C address 0x33. Check SDA/SCL and power.";
    return false;
  }

  uint16_t eeMLX90640[832];
  int status = MLX90640_DumpEE(MLX90640_ADDR, eeMLX90640);
  if (status != 0)
  {
    lastError = "MLX90640_DumpEE failed: " + String(status);
    return false;
  }

  status = MLX90640_ExtractParameters(eeMLX90640, &mlxParams);
  if (status != 0)
  {
    lastError = "MLX90640_ExtractParameters failed: " + String(status);
    return false;
  }

  status = MLX90640_SetRefreshRate(MLX90640_ADDR, MLX_REFRESH_RATE);
  if (status != 0)
  {
    lastError = "SetRefreshRate failed: " + String(status);
    return false;
  }

  status = MLX90640_SetResolution(MLX90640_ADDR, MLX_RESOLUTION);
  if (status != 0)
  {
    lastError = "SetResolution failed: " + String(status);
    return false;
  }

  status = MLX90640_SetChessMode(MLX90640_ADDR);
  if (status != 0)
  {
    lastError = "SetChessMode failed: " + String(status);
    return false;
  }

  sensorReady = true;
  lastError = "OK";
  return true;
}

void setupWiFi()
{
  WiFi.setSleep(false);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS, 1, false, 4);

  Serial.println();
  Serial.println("Wi-Fi AP started");
  Serial.print("SSID: ");
  Serial.println(AP_SSID);
  Serial.print("Password: ");
  Serial.println(AP_PASS);
  Serial.print("Open browser: http://");
  Serial.println(WiFi.softAPIP());
}

void setup()
{
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  setCpuFrequencyMhz(240);

  Serial.begin(115200);
  Serial.setDebugOutput(false);
  delay(800);

  Serial.println();
  Serial.println("ESP32-CAM + MLX90640 Rescue Robot");

  // -------- Camera --------
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound())
  {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  }
  else
  {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return;
  }

  // -------- MLX90640 --------
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_CLOCK_HZ);
  Wire.setTimeOut(1000);

  Serial.print("MLX90640 SDA: GPIO");
  Serial.println(I2C_SDA);
  Serial.print("MLX90640 SCL: GPIO");
  Serial.println(I2C_SCL);

  if (initializeMLX90640())
  {
    Serial.println("MLX90640 initialized successfully");
    readMLX90640Frame();
  }
  else
  {
    Serial.print("MLX90640 initialization failed: ");
    Serial.println(lastError);
  }

  // -------- Robot Wi-Fi + web server --------
  setupWiFi();
  startWebServer();
}

void loop()
{
  const uint32_t now = millis();

  if (now - lastFrameRequestMs >= FRAME_INTERVAL_MS)
  {
    lastFrameRequestMs = now;
    readMLX90640Frame();
  }

  delay(1);
}
