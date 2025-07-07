#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include "secrets.h"
#include "esp_camera.h"
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"


const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;
const char* websocket_server_host = WS_SERVER_HOST;
const uint16_t websocket_server_port = WS_SERVER_PORT;
const char* websocket_server_path = WS_SERVER_PATH;

WebSocketsClient webSocket;

bool streaming = false;  // start 명령 받으면 true

void cameraInit(void);
void sendVideoFrame();
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length);

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  cameraInit();

  WiFi.begin(ssid, password);
  Serial.print("Connecting Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected Wi-Fi!");

  webSocket.begin(websocket_server_host, websocket_server_port, websocket_server_path);
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);
}

void loop() {
  webSocket.loop();
  if (streaming) {
    sendVideoFrame();
    delay(30); // 30ms 간격으로 전송 (33fps)
  }
}

void cameraInit(void) {
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
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  if (psramFound()) {
    config.jpeg_quality = 10;
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  s->set_vflip(s, 1);
  s->set_brightness(s, 1);
  s->set_saturation(s, 0);
}

void sendVideoFrame() {
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }
  webSocket.sendBIN(fb->buf, fb->len);
  esp_camera_fb_return(fb);
} 

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("[WSc] Disconnected!\n");
      streaming = false;  // 연결이 끊기면 스트리밍 중지
      break;

    case WStype_CONNECTED:
      Serial.printf("[WSc] Connected to url: %s\n", payload);
      webSocket.sendTXT("{\"status\": \"ready\"}");
      break;

    case WStype_TEXT:
      Serial.printf("[WSc] get text: %s\n", payload);
      if(strcmp((char*)payload, "{\"command\":\"start\"}") == 0) {
        streaming = true;  // start 명령 수신 시 스트리밍 시작
        webSocket.sendTXT("{\"status\": \"started\"}");
        Serial.println("[WSc] sent: started");
      }
      else if (strcmp((char*)payload, "{\"command\":\"stop\"}") == 0) {
        streaming = false;  // stop 명령 수신 시 스트리밍 중지
        webSocket.sendTXT("{\"status\": \"stopped\"}");
        Serial.println("[WSc] sent: stopped");
      }
      break;

  }
}
