// esp32-cam high performance streaming server
// optimized for maximum fps and quality with psram

#include <WiFi.h>
#include <esp_camera.h>
#include <WebServer.h>
#include <esp_timer.h>
#include <esp_http_server.h>

// camera model - ov2640 camera pid
#define OV2640_PID 0x26

// ai thinker esp32-cam pins
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

// wifi configuration - access point mode for best performance
const char* AP_SSID = "ESP32-CAM-STREAM";
const char* AP_PASSWORD = "12345678";  // min 8 characters

// led flash
#define FLASH_GPIO_NUM 4
bool flashEnabled = false;

// flash control input pin (connect to esp32 master gpio 25)
#define FLASH_CONTROL_INPUT 13
bool lastFlashState = false;

// global variables
httpd_handle_t stream_httpd = NULL;

// function declarations
void setupCamera();
void setupWiFi();
void startStreamServer();

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(false);  // disable debug output for performance

  // set cpu frequency to maximum 240mhz for best performance
  setCpuFrequencyMhz(240);

  // initialize flash led with pwm for smoother control
  // using pwm prevents power surges that can interrupt camera streaming
  // esp32 arduino core 3.x uses new api
  ledcAttach(FLASH_GPIO_NUM, 5000, 8);  // pin, frequency, resolution
  ledcWrite(FLASH_GPIO_NUM, 0);  // start with flash off

  // setup flash control input pin
  pinMode(FLASH_CONTROL_INPUT, INPUT_PULLDOWN);
  Serial.println("Flash control GPIO input initialized on pin 13");

  Serial.println("\n===================================");
  Serial.println("ESP32-CAM High Performance Stream");
  Serial.println("===================================");
  Serial.printf("CPU Frequency: %d MHz\n", getCpuFrequencyMhz());
  Serial.printf("Free Heap: %d KB\n", ESP.getFreeHeap() / 1024);
  Serial.printf("Chip Model: %s\n", ESP.getChipModel());
  Serial.printf("Chip Revision: %d\n", ESP.getChipRevision());
  Serial.printf("Number of Cores: %d\n", ESP.getChipCores());

  // setup camera first
  setupCamera();

  // setup wifi
  setupWiFi();

  // start streaming server
  startStreamServer();

  Serial.println("\n=== READY TO STREAM ===");
  Serial.printf("Stream URL: http://%s/stream\n", WiFi.softAPIP().toString().c_str());
  Serial.println("========================\n");
}

void loop() {
  // check flash control gpio from esp32 master
  bool currentFlashState = digitalRead(FLASH_CONTROL_INPUT);

  if (currentFlashState != lastFlashState) {
    lastFlashState = currentFlashState;
    flashEnabled = currentFlashState;

    // control flash led
    if (flashEnabled) {
      ledcWrite(FLASH_GPIO_NUM, 230);  // 90% brightness
      Serial.println("Flash ON (via GPIO)");
    } else {
      ledcWrite(FLASH_GPIO_NUM, 0);
      Serial.println("Flash OFF (via GPIO)");
    }
  }

  delay(50);  // check every 50ms for responsive control
}

void setupCamera() {
  Serial.println("Initializing camera...");

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

  // optimal performance settings for 2mb psram module
  config.xclk_freq_hz = 24000000;  // 24mhz for ov2640 optimal performance
  config.pixel_format = PIXFORMAT_JPEG;  // jpeg for streaming
  config.grab_mode = CAMERA_GRAB_LATEST;  // always get latest frame

  // balanced configuration for performance and quality
  if(psramFound()){
    Serial.println("2MB PSRAM detected - balanced performance settings");
    // svga provides good balance - not too large, not too small
    config.frame_size = FRAMESIZE_SVGA;   // 800x600 - good balance
    config.jpeg_quality = 10;             // moderate quality for better fps
    config.fb_count = 2;                  // double buffer for smooth streaming
    config.fb_location = CAMERA_FB_IN_PSRAM;
  } else {
    Serial.println("No PSRAM - using reduced settings");
    config.frame_size = FRAMESIZE_VGA;    // 640x480 without psram
    config.jpeg_quality = 12;
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  // initialize camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    ESP.restart();
    return;
  }

  // optimize ov2640 sensor settings for maximum performance
  sensor_t * s = esp_camera_sensor_get();
  if (s != NULL) {
    // ov2640 specific optimizations
    if (s->id.PID == OV2640_PID) {
      Serial.println("OV2640 camera detected - applying optimizations");

      // image orientation
      s->set_vflip(s, 0);        // flip vertically if needed
      s->set_hmirror(s, 0);      // mirror horizontally if needed

      // optimal brightness/contrast for ov2640
      s->set_brightness(s, 0);   // -2 to 2 (0 is neutral)
      s->set_contrast(s, 0);     // -2 to 2 (0 is neutral)
      s->set_saturation(s, 0);   // -2 to 2 (0 is neutral)

      // ov2640 performance optimizations
      s->set_special_effect(s, 0);  // no special effect for maximum fps
      s->set_whitebal(s, 1);        // enable awb
      s->set_awb_gain(s, 1);        // enable awb gain
      s->set_wb_mode(s, 0);         // auto wb mode
      s->set_exposure_ctrl(s, 1);   // auto exposure on
      s->set_aec2(s, 1);            // aec dsp on for better exposure
      s->set_ae_level(s, 0);        // auto exposure level (0 = default)
      s->set_aec_value(s, 300);     // 0-1200 (300 = default)
      s->set_gain_ctrl(s, 1);       // auto gain on
      s->set_agc_gain(s, 0);        // 0-30 (0 = auto)
      s->set_gainceiling(s, (gainceiling_t)6);  // max 64x gain
      s->set_bpc(s, 1);             // bad pixel correction on
      s->set_wpc(s, 1);             // white pixel correction on
      s->set_raw_gma(s, 1);         // gamma on
      s->set_lenc(s, 1);            // lens correction on
      s->set_dcw(s, 1);             // dcw on for better performance
      s->set_colorbar(s, 0);        // disable test pattern

      // set balanced frame size for good performance
      if(psramFound()) {
        s->set_framesize(s, FRAMESIZE_SVGA);  // 800x600 balanced
        s->set_quality(s, 10);  // moderate quality
      } else {
        s->set_framesize(s, FRAMESIZE_VGA);  // 640x480 without psram
        s->set_quality(s, 12);
      }
    }
  }

  Serial.println("Camera initialized successfully!");
  Serial.printf("Resolution: %s\n", psramFound() ? "SVGA (800x600) - Balanced" : "VGA (640x480)");
  Serial.printf("JPEG Quality: %d\n", psramFound() ? 10 : 12);
  Serial.printf("PSRAM Size: %d MB\n", psramFound() ? (ESP.getPsramSize() / 1024 / 1024) : 0);
  Serial.printf("Free PSRAM: %d KB\n", psramFound() ? (ESP.getFreePsram() / 1024) : 0);
}

void setupWiFi() {
  Serial.println("Setting up WiFi Access Point...");

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);

  // configure ap settings for performance
  WiFi.setSleep(false);  // disable wifi sleep for consistent performance

  IPAddress IP = WiFi.softAPIP();
  Serial.print("Access Point Started - ");
  Serial.println(AP_SSID);
  Serial.print("Password: ");
  Serial.println(AP_PASSWORD);
  Serial.print("IP Address: ");
  Serial.println(IP);
}


esp_err_t stream_handler(httpd_req_t *req){
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t * _jpg_buf = NULL;
  char * part_buf[128];

  static int64_t last_frame = 0;
  static int64_t frame_count = 0;

  res = httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=frame");
  if(res != ESP_OK){
    return res;
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "X-Framerate", "60");

  while(true){
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
    } else {
      _jpg_buf_len = fb->len;
      _jpg_buf = fb->buf;
    }

    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, "--frame\r\n", 8);
    }
    if(res == ESP_OK){
      size_t hlen = snprintf((char *)part_buf, 128,
                            "Content-Type: image/jpeg\r\n"
                            "Content-Length: %u\r\n"
                            "X-Timestamp: %lld.%06ld\r\n"
                            "\r\n",
                            _jpg_buf_len,
                            esp_timer_get_time() / 1000000,
                            esp_timer_get_time() % 1000000);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, "\r\n", 2);
    }

    if(fb){
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if(_jpg_buf){
      free(_jpg_buf);
      _jpg_buf = NULL;
    }

    if(res != ESP_OK){
      Serial.println("Stream terminated");
      break;
    }

    // fps counter for debugging
    int64_t fr_end = esp_timer_get_time();
    int64_t frame_time = fr_end - last_frame;
    last_frame = fr_end;
    frame_count++;

    // print fps every 100 frames
    if (frame_count % 100 == 0) {
      Serial.printf("Stream: %lld frames, %.1f fps, %u KB\n",
                   frame_count,
                   1000000.0 / frame_time,
                   _jpg_buf_len / 1024);
    }

    // yield to prevent watchdog
    yield();
  }

  return res;
}

esp_err_t flash_handler(httpd_req_t *req) {
  char buf[64];
  size_t buf_len = httpd_req_get_url_query_len(req) + 1;

  if (buf_len > 1) {
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
      char param[32] = {0};
      if (httpd_query_key_value(buf, "state", param, sizeof(param)) == ESP_OK) {
        // use pwm for led control to prevent camera interference
        if (!strcmp(param, "on")) {
          flashEnabled = true;
          // use pwm at 90% duty cycle to reduce power draw
          ledcWrite(FLASH_GPIO_NUM, 230);  // write to pin directly in esp32 core 3.x
          Serial.println("Flash: ON (PWM)");
        } else if (!strcmp(param, "off")) {
          flashEnabled = false;
          ledcWrite(FLASH_GPIO_NUM, 0);  // turn off pwm
          Serial.println("Flash: OFF");
        } else if (!strcmp(param, "toggle")) {
          flashEnabled = !flashEnabled;
          ledcWrite(FLASH_GPIO_NUM, flashEnabled ? 230 : 0);
          Serial.printf("Flash: %s\n", flashEnabled ? "ON" : "OFF");
        }
      }
    }
  }

  // send response immediately without blocking
  httpd_resp_set_type(req, "text/plain");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
  char response[32];
  sprintf(response, "{\"flash\":%s}", flashEnabled ? "true" : "false");
  return httpd_resp_send(req, response, strlen(response));
}

void startStreamServer(){
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.ctrl_port = 32768;
  config.max_open_sockets = 10;
  config.max_uri_handlers = 12;
  config.max_resp_headers = 12;
  config.recv_wait_timeout = 10;
  config.send_wait_timeout = 10;
  config.stack_size = 8192;  // larger stack for stability with 2mb psram
  config.core_id = 1;        // pin to core 1 (core 0 handles wifi)

  httpd_uri_t stream_uri = {
    .uri       = "/stream",
    .method    = HTTP_GET,
    .handler   = stream_handler,
    .user_ctx  = NULL
  };

  httpd_uri_t flash_uri = {
    .uri       = "/flash",
    .method    = HTTP_GET,
    .handler   = flash_handler,
    .user_ctx  = NULL
  };

  // start streaming server
  Serial.printf("Starting stream server on port %d\n", config.server_port);
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
    httpd_register_uri_handler(stream_httpd, &flash_uri);
    Serial.println("Stream server started with flash control");
  } else {
    Serial.println("Failed to start stream server");
  }
}