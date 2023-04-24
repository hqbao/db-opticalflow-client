#include <WiFi.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "Coarse2FineFlowWrapper.h"

#define FIRMWARE_UPDATE_SIGNAL_PIN 16

// #define STREAMING
#define OPTICAL_FLOW

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

#ifdef OPTICAL_FLOW
#define IMAGE_WIDTH 320 // origin = 320
#define IMAGE_HEIGHT 240 // origin = 240
#define CROP_WIDTH 240
#define CROP_HEIGHT 240
#define PIXEL_SIZE 20
#define WIDTH ((int)(CROP_WIDTH/PIXEL_SIZE))
#define HEIGHT ((int)(CROP_HEIGHT/PIXEL_SIZE))
#define PIXEL_START_X (40 + (int)((IMAGE_WIDTH - CROP_WIDTH)/2))
#define PIXEL_START_Y (0 + (int)((IMAGE_HEIGHT - CROP_HEIGHT)/2))

#define SSID "Bao"
#define PASS "nopassword"

static double g_img0[HEIGHT*WIDTH];
static double g_img1[HEIGHT*WIDTH];
static double g_vy[HEIGHT*WIDTH];
static double g_vx[HEIGHT*WIDTH];
static double g_warpI2[HEIGHT*WIDTH];

void calc_otpflw(uint8_t *img_buf) {
  double *img = g_img1;

  // Scale
  int pixel_start_y = PIXEL_START_Y; // - anchor_y;
  int pixel_start_x = PIXEL_START_X; // + anchor_x;
  for (int i = 0; i < HEIGHT; i += 1) {
    for (int j = 0; j < WIDTH; j += 1) {
      double sum = 0;
      for (int k1 = 0; k1 < PIXEL_SIZE; k1 += 1)
        for (int k2 = 0; k2 < PIXEL_SIZE; k2 += 1)
          sum += img_buf[((pixel_start_y+PIXEL_SIZE*i)+k1)*IMAGE_WIDTH+(pixel_start_x+PIXEL_SIZE*j)+k2];
        
      uint8_t bw_pix = sum/(PIXEL_SIZE*PIXEL_SIZE);
      img[i*WIDTH+j] = bw_pix;
    }
  }

  float min = 255;
  float max = 0;
  float sum = 0;
  for (int i = 0; i < HEIGHT*WIDTH; i += 1) {
    if (min > img[i]) min = img[i];
    if (max < img[i]) max = img[i];
    sum += img[i];
  }
  float avg = sum/(HEIGHT*WIDTH);
  float lower = avg - min;
  float upper = max - avg;
  float clip = lower < upper ? lower : upper;
  float scale = 2*clip/253;
  float lim_lower = avg - clip;
  float lim_upper = avg + clip;
  for (int i = 0; i < HEIGHT*WIDTH; i += 1) {
    if (img[i] < lim_lower) img[i] = lim_lower;
    else if (img[i] > lim_upper) img[i] = lim_upper;
    img[i] = (img[i] - lim_lower) / scale + 1;
    if (img[i] > 254) img[i] = 254;
  }

  for (int i = 0; i < HEIGHT*WIDTH; i += 1) {
    img[i] /= 255;
  }

  int quality = lower < upper ? (int)(lower) : (int)(upper);
  if (quality == 0) quality = 1;

  Coarse2FineFlowWrapper(g_vx, g_vy, g_warpI2, g_img0, g_img1,
    0.00012, 0.9, 10, 1, 1, 1, 1, HEIGHT, WIDTH, 1);

  memcpy(g_img0, g_img1, HEIGHT*WIDTH*sizeof(double));

  double dx_raw = 0;
  double dy_raw = 0;
  
  for (int i = 0; i < HEIGHT; i += 1) {
    for (int j = 0; j < WIDTH; j += 1) {
      dx_raw -= g_vx[i*WIDTH + j];
      dy_raw += g_vy[i*WIDTH + j];
    }
  }

  Serial.printf("$%d,%d,%d,%d\n", 
    (int)(dx_raw*10), (int)(dy_raw*10), (int)(0), (int)(quality));
}
#endif

#ifdef STREAMING
#define MAX_URL_QUERY_SIZE 256
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t stream_httpd = NULL;
httpd_handle_t ctrl_httpd = NULL;

static esp_err_t stream_handler(httpd_req_t *req){
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
  char *part_buf[64];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
    } 
    else {
      _jpg_buf = fb->buf;
      _jpg_buf_len = fb->len;
    }

    if (res == ESP_OK)
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));

    if (res == ESP_OK) {
      size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }

    if (res == ESP_OK)
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    
    if (fb) {
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    }
    else if(_jpg_buf) {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }

    if (res != ESP_OK) break;
  }
  
  return res;
}

static esp_err_t cmd_handler(httpd_req_t *req) {
  char buf[MAX_URL_QUERY_SIZE] = {0,};  
  char dx_buf[8] = {0,};
  char dy_buf[8] = {0,};
  char dz_buf[8] = {0,};

  size_t buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len < 1 || buf_len >= MAX_URL_QUERY_SIZE)
    httpd_resp_send_404(req);

  if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
    httpd_query_key_value(buf, "dx", dx_buf, sizeof(dx_buf));
    httpd_query_key_value(buf, "dy", dy_buf, sizeof(dy_buf));
    httpd_query_key_value(buf, "dz", dz_buf, sizeof(dz_buf));
  }

  int dx = atoi(dx_buf);
  int dy = atoi(dy_buf);
  int dz = atoi(dz_buf);

  Serial.printf("$%d,%d,%d\n", dx, dy, dz);

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}

void start_server() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();

  httpd_uri_t cmd_uri = {
    .uri       = "/control",
    .method    = HTTP_GET,
    .handler   = cmd_handler,
    .user_ctx  = NULL
  };

  httpd_uri_t stream_uri = {
    .uri       = "/stream",
    .method    = HTTP_GET,
    .handler   = stream_handler,
    .user_ctx  = NULL
  };

  Serial.printf("Starting web server on port: '%d'\n", config.server_port);
  if (httpd_start(&ctrl_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(ctrl_httpd, &cmd_uri);
  }

  config.server_port += 1;
  config.ctrl_port += 1;
  Serial.printf("Starting stream server on port: '%d'\n", config.server_port);
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }
}

void connect_wifi() {
  int counter = 0;
  while (true) {
    Serial.printf("\nConnecting to %s\n", SSID);
    WiFi.begin(SSID, PASS);
    counter = 0;
    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      counter += 1;
      if (counter > 10) {
        Serial.println("Wifi connection time out");
        break;
      }
    }

    if (counter > 10) continue;
    else {
      Serial.printf("\nWiFi connected %s\n", WiFi.localIP().toString().c_str());
      break;
    }
  }
}
#endif

void init_cam() {
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
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
#ifdef STREAMING
  config.pixel_format = PIXFORMAT_JPEG;
#else
  config.pixel_format = PIXFORMAT_GRAYSCALE;
#endif
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 10;
  config.fb_count = 2;

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV2640_PID) {
    // s->set_gain_ctrl(s, 0); // auto gain off (1 or 0)
    // s->set_exposure_ctrl(s, 1); // auto exposure off (1 or 0)
    // s->set_agc_gain(s, 0); // set gain manually (0 - 30)
    // s->set_aec_value(s, 0); // set exposure manually (0-1200)
    // s->set_brightness(s, 2); // (-2 to 2) - set brightness
    // s->set_awb_gain(s, 0); // Auto White Balance?
    // s->set_lenc(s, 0); // lens correction? (1 or 0)
    // s->set_raw_gma(s, 1); // (1 or 0)?
    // s->set_quality(s, 1); // (0 - 63)
    // s->set_whitebal(s, 0); // white balance
    // s->set_wb_mode(s, 0); // white balance mode (0 to 4)
    // s->set_aec2(s, 1); // automatic exposure sensor? (0 or 1)
    // s->set_aec_value(s, 0); // automatic exposure correction? (0-1200)
    // s->set_saturation(s, -2); // (-2 to 2)
    // s->set_hmirror(s, 0); // (0 or 1) flip horizontally
    // s->set_gainceiling(s, GAINCEILING_2X); // Image gain (GAINCEILING_x2, x4, x8, x16, x32, x64 or x128)
    // s->set_contrast(s, 0); // (-2 to 2)
    // s->set_sharpness(s, 2); // (-2 to 2)
    // s->set_colorbar(s, 0); // (0 or 1) - testcard
    // s->set_special_effect(s, 0);
    // s->set_ae_level(s, 0); // auto exposure levels (-2 to 2)
    // s->set_bpc(s, 0); // black pixel correction
    // s->set_wpc(s, 0); // white pixel correction
    // s->set_dcw(s, 0); // downsize enable? (1 or 0)?
  }
  // drop down frame size for higher initial frame rate
  s->set_framesize(s, FRAMESIZE_QVGA);
}

void setup() {
  Serial.begin(9600);
  delay(500);

  // Initiaise camera
  init_cam();

#ifdef STREAMING
  connect_wifi();
  start_server();
#endif
}

long t = 0;
void loop() {
#ifdef OPTICAL_FLOW
  t = millis();
  
  static int failed_capture_counter = 0;
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    failed_capture_counter += 1;    
    return;
  }
  if (failed_capture_counter > 20) esp_restart();
  if (failed_capture_counter > 0) failed_capture_counter -= 1;

  // Serial.printf("%dx%d, %d, %d\n", fb->height, fb->width, fb->format, fb->len);
  esp_camera_fb_return(fb);

  calc_otpflw(fb->buf);
  // Serial.printf("FPS %d\n", (int)(1000)/(millis() - t));  
#endif
}
