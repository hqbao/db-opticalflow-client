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
#define OUTPUT_PWM

#define LED_CH0 12
#define LED_CH1 14
#define FREQ 5000
#define RESOLUTION 8
#define LED_PIN_CH0 12
#define LED_PIN_CH1 14

#define HEIGHT 12
#define WIDTH 16

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
  if (psramFound()) {
    Serial.printf("PSRAM found\n");
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

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
    s->set_vflip(s, 1); // flip it back
    s->set_brightness(s, 1); // up the brightness just a bit
    s->set_saturation(s, -2); // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  s->set_framesize(s, FRAMESIZE_QVGA);

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif
}

void calc_otpflw() {
  #ifdef OUTPUT_UART
  Serial.printf("$%d,%d\n", 
    LIMIT((int)(g_dy*500), -128, 127),
    LIMIT((int)(g_dx*500), -128, 127));
#endif

#ifdef OUTPUT_PWM
  ledcWrite(LED_CH0, LIMIT(g_dy*500+128, 0, 255));
  ledcWrite(LED_CH1, LIMIT(g_dx*500+128, 0, 255));
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
  for (int i = 0; i < HEIGHT; i += 1) {
    for (int j = 0; j < WIDTH; j += 1) {
      double sum = 0;
      for (int k1 = 0; k1 < 20; k1 += 1)
        for (int k2 = 0; k2 < 20; k2 += 1)
          sum += fb->buf[(20*i+k1)*fb->width+20*j+k2];

      uint8_t bw_pix = sum/(20*20);
      g_img1[i*WIDTH+j] = (double)bw_pix/255;
    }
  }

  Coarse2FineFlowWrapper(g_vx, g_vy, g_warpI2, g_img0, g_img1,
    0.0012, 0.75, 12, 1, 1, 1, 1, HEIGHT, WIDTH, 1);

  memcpy(g_img0, g_img1, HEIGHT*WIDTH*sizeof(double));

  double dy = 0;
  double dx = 0;
  for (int i = 0; i < HEIGHT*WIDTH; i += 1) {
    dy += g_vy[i];
    dx += g_vx[i];
  }

  dy = dy / (HEIGHT*WIDTH);
  dx = dx / (HEIGHT*WIDTH);
  g_dy = dy;
  g_dx = dx;
  g_sy += dy * 0.125; // v*t = s
  g_sx += dx * 0.125;

  // Serial.printf("y = %f, x = %f, fps: %d\n", sy, sx, (int)(1000/(millis() - t)));
  // Serial.print("y: ");
  // for (int i = 0; i < abs(g_sy); i += 1) {
  //   if (g_sy >= 0) Serial.print("+");
  //   else Serial.print("-");
  // }
  // Serial.println();
  // Serial.print("x: ");
  // for (int i = 0; i < abs(g_sx); i += 1) {
  //   if (g_sx >= 0) Serial.print("+");
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

// timer 8mhz. 80,000,000/prescaler(1000)/counter(10000) = 8hz
void start_timer1() {  
  timer1 = timerBegin(0, 1000, true);
  timerAttachInterrupt(timer1, &onTimer1, true);
  timerAlarmWrite(timer1, 10000, true);
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

  ledcSetup(LED_CH0, FREQ, RESOLUTION);
  ledcSetup(LED_CH1, FREQ, RESOLUTION);
  ledcAttachPin(LED_PIN_CH0, LED_CH0);
  ledcAttachPin(LED_PIN_CH1, LED_CH1);
  
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
