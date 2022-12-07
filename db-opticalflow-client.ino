#include <WiFi.h>
#include <esp_pthread.h>
#include "esp_camera.h"
#include "camera_pins.h"
#include "Coarse2FineFlowWrapper.h"

#define LIMIT(number, min, max) (number > max ? max : (number < min ? min : number))

#define SSID "opticalflow_hub"
#define PASS 0
#define HOST "192.168.4.1"
#define PORT 80
#define ORIENTATION 'A' + (PORT - 80)

// #define OUTPUT_UART
// #define OUTPUT_PWM

#define LED_CH0 12
#define LED_CH1 14
#define FREQ 1000
#define RESOLUTION 8
#define LED_PIN_CH0 12
#define LED_PIN_CH1 14

#define HEIGHT 12
#define WIDTH 12
#define PIXEL_SIZE 20
#define PIXEL_START_Y ((240/PIXEL_SIZE - HEIGHT)/2)
#define PIXEL_START_X ((320/PIXEL_SIZE - WIDTH)/2)

hw_timer_t *timer1 = NULL;
hw_timer_t *timer2 = NULL;
portMUX_TYPE timer1Mux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timer2Mux = portMUX_INITIALIZER_UNLOCKED;
volatile int interrupt1Counter = 0;
volatile int interrupt2Counter = 0;

static double g_img0[HEIGHT*WIDTH];
static double g_img1[HEIGHT*WIDTH];
static double g_vy[HEIGHT*WIDTH];
static double g_vx[HEIGHT*WIDTH];
static double g_warpI2[HEIGHT*WIDTH];
volatile double g_dy = 0;
volatile double g_dx = 0;
volatile double g_sy = 0;
volatile double g_sx = 0;

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
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  
  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality 
  // for larger pre-allocated frame buffer.
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 10;
  config.fb_count = 2;

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_gain_ctrl(s, 0); // auto gain off (1 or 0)
    s->set_exposure_ctrl(s, 0); // auto exposure off (1 or 0)
    s->set_agc_gain(s, 0); // set gain manually (0 - 30)
    s->set_aec_value(s, 600); // set exposure manually (0-1200)
    s->set_brightness(s, 0); // (-2 to 2) - set brightness
    s->set_awb_gain(s, 0); // Auto White Balance?
    s->set_lenc(s, 0); // lens correction? (1 or 0)
    s->set_raw_gma(s, 1); // (1 or 0)?
    s->set_quality(s, 63); // (0 - 63)
    s->set_whitebal(s, 1); // white balance
    s->set_wb_mode(s, 1); // white balance mode (0 to 4)
    s->set_aec2(s, 0); // automatic exposure sensor? (0 or 1)
    s->set_aec_value(s, 0); // automatic exposure correction? (0-1200)
    s->set_saturation(s, 0); // (-2 to 2)
    s->set_hmirror(s, 0); // (0 or 1) flip horizontally
    s->set_gainceiling(s, GAINCEILING_32X); // Image gain (GAINCEILING_x2, x4, x8, x16, x32, x64 or x128)
    s->set_contrast(s, 0); // (-2 to 2)
    s->set_sharpness(s, 0); // (-2 to 2)
    s->set_colorbar(s, 0); // (0 or 1) - testcard
    s->set_special_effect(s, 0);
    s->set_ae_level(s, 0); // auto exposure levels (-2 to 2)
    s->set_bpc(s, 0); // black pixel correction
    s->set_wpc(s, 0); // white pixel correction
    s->set_dcw(s, 0); // downsize enable? (1 or 0)?
  }
  // drop down frame size for higher initial frame rate
  s->set_framesize(s, FRAMESIZE_QVGA);

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif
}

void calc_otpflw() {
  unsigned long t = millis();
#ifdef OUTPUT_UART
  Serial.printf("$%d,%d\n", 
    LIMIT((int)(g_dy), -128, 127),
    LIMIT((int)(g_dx), -128, 127));
#endif

#ifdef OUTPUT_PWM
  ledcWrite(LED_CH0, LIMIT(g_dy+128, 0, 255));
  ledcWrite(LED_CH1, LIMIT(g_dx+128, 0, 255));
#endif

  static int failed_capture_counter = 0;
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.printf("Camera capture failed\n");
    failed_capture_counter += 1;    
    return;
  }
  if (failed_capture_counter > 20) esp_restart();
  if (failed_capture_counter > 0) failed_capture_counter -= 1;

  // Serial.printf("%dx%d, %d, %d\n", fb->height, fb->width, fb->format, fb->len);
  esp_camera_fb_return(fb);

  // Scale
  for (int i = PIXEL_START_Y; i < PIXEL_START_Y+HEIGHT; i += 1) {
    for (int j = PIXEL_START_X; j < PIXEL_START_X+WIDTH; j += 1) {
      double sum = 0;
      for (int k1 = 0; k1 < PIXEL_SIZE; k1 += 1)
        for (int k2 = 0; k2 < PIXEL_SIZE; k2 += 1)
          sum += fb->buf[(PIXEL_SIZE*i+k1)*fb->width+PIXEL_SIZE*j+k2];

      uint8_t bw_pix = sum/(PIXEL_SIZE*PIXEL_SIZE);
      g_img1[(i-PIXEL_START_Y)*WIDTH+(j-PIXEL_START_X)] = (double)bw_pix/255;
    }
  }

  Coarse2FineFlowWrapper(g_vx, g_vy, g_warpI2, g_img0, g_img1,
    0.00012, 0.75, 9, 1, 1, 1, 1, HEIGHT, WIDTH, 1);

  memcpy(g_img0, g_img1, HEIGHT*WIDTH*sizeof(double));

  double dy = 0;
  double dx = 0;
  for (int i = 0; i < HEIGHT*WIDTH; i += 1) {
    dy += g_vy[i];
    dx -= g_vx[i];
  }

  dy = dy / (HEIGHT*WIDTH);
  dx = dx / (HEIGHT*WIDTH);
  g_dy = dy*100;
  g_dx = dx*100;
  g_sy += dy;
  g_sx += dx;

  Serial.printf("$%d,%d,%d\n", (int)g_dy, (int)g_dx, (int)(1000/(millis() - t)));
  // Serial.print("y: ");
  // for (int i = 0; i < abs(g_dy); i += 1) {
  //   if (g_dy >= 0) Serial.print("+");
  //   else Serial.print("-");
  // }
  // Serial.println();
  // Serial.print("x: ");
  // for (int i = 0; i < abs(g_dx); i += 1) {
  //   if (g_dx >= 0) Serial.print("+");
  //   else Serial.print("-");
  // }
  // Serial.println();
}

void IRAM_ATTR onTimer1() {
  portENTER_CRITICAL_ISR(&timer1Mux);
  interrupt1Counter++;
  portEXIT_CRITICAL_ISR(&timer1Mux);
}

void IRAM_ATTR onTimer2() {}

// timer 8mhz. 80,000,000/prescaler(8000)/counter(1000) = 10hz
void start_timer1() {  
  timer1 = timerBegin(0, 8000, true);
  timerAttachInterrupt(timer1, &onTimer1, true);
  timerAlarmWrite(timer1, 800, true);
  timerAlarmEnable(timer1);
}

// 8hz
void start_timer2() {
  timer2 = timerBegin(1, 1000, true);
  timerAttachInterrupt(timer2, &onTimer2, true);
  timerAlarmWrite(timer2, 10000, true);
  timerAlarmEnable(timer2);
}

void *run_tcp_client(void *pointer) {
  WiFiClient client;
  static char msg[10];
  static int counter = 0;
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

    Serial.printf("\nWiFi connected %s\n", WiFi.localIP().toString());

    while (true) {
      Serial.println("Opening a connection...");
      counter = 0;  
      while (!client.connect(HOST, PORT)) {
        delay(100);
        counter += 1;
        if (counter > 10) {
          Serial.println("Connection time out");
          break;
        }
      }
      
      if (counter > 10) break;

      Serial.println("Done");
      Serial.println("Start streaming");
      
      while (true) {
        sprintf(msg, "%d,%d%c", (int)(g_sy*10), (int)(g_sx*10), ORIENTATION);
        client.print(msg);

        counter = 0;
        bool connection_lost = false;
        while (!client.available() || client.read() != '$') {
          if (counter > 500) {
            connection_lost = true;
            break;
          }

          delay(10);
          counter += 1;
        }

        if (connection_lost) break;
        delay(10);
      }

      client.stop();
      Serial.println("Stop streaming");
      delay(100);
    }
  }
}

void setup() {
  Serial.begin(9600);
  Serial.printf("Start program\n");

#ifdef OUTPUT_PWM
  ledcSetup(LED_CH0, FREQ, RESOLUTION);
  ledcSetup(LED_CH1, FREQ, RESOLUTION);
  ledcAttachPin(LED_PIN_CH0, LED_CH0);
  ledcAttachPin(LED_PIN_CH1, LED_CH1);
#endif
  
  init_cam();
  start_timer1();
  // start_timer2();

  // https://docs.espressif.com/projects/esp-idf/en/v4.3-beta2/esp32/api-reference/system/esp_pthread.html
  // esp_pthread_cfg_t cfg = esp_pthread_get_default_config();
  // cfg.stack_size = (3*1024);
  // esp_pthread_set_cfg(&cfg);
    
  // pthread_t thread;
  // if (pthread_create(&thread, nullptr, run_tcp_client, (void *)0)) {
  //   Serial.printf("Can't create thread run_tcp_client");
  //   esp_restart();
  // }
}

void loop() {
  if (interrupt1Counter > 0) {
    portENTER_CRITICAL(&timer1Mux);
    interrupt1Counter--;
    portEXIT_CRITICAL(&timer1Mux);

    calc_otpflw();
  }
}
