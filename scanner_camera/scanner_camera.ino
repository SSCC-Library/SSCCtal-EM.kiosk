// ESP32-S3-WROOM-1 + OV2640을 사용하여 카메라 영상 스트리밍을 WebSocket을 통해 전송하는 코드
#include "esp_camera.h"   // ESP32 카메라 라이브러리 포함 (카메라 제어를 위해 필요)
#include <WiFi.h>  // Wi-Fi 연결을 위한 라이브러리 포함 (인터넷 연결 기능 제공)
#include <WebSocketsClient.h>  // WebSocket 클라이언트 라이브러리 포함 (서버와 실시간 통신을 위해 필요)
//#include "ipconfig.h"  // Wi-Fi 및 WebSocket 서버 정보가 저장된 설정 파일 포함 (보안 데이터 보호)

// 카메라 모델 선택 (ESP32-S3-WROOM-1 + OV2640 사용)
#define CAMERA_MODEL_ESP32S3_EYE // PSRAM이 포함된 ESP32-S3-EYE 보드 사용
#include "camera_pins.h"  // ESP32와 OV2640 카메라 모듈의 핀 매핑 정보 포함

// WebSocket 클라이언트 객체 생성 (서버와 실시간 통신을 수행할 객체)
WebSocketsClient webSocket;

// 함수 선언 (각 기능을 구현하는 함수 정의)
void cameraInit(void);  // 카메라 초기화 함수 (카메라 설정 및 활성화)
void sendVideoFrame();  // 비디오 프레임을 WebSocket으로 전송하는 함수
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length);  // WebSocket 이벤트 핸들러 함수 (연결, 데이터 수신 처리)

void setup() {
  Serial.begin(115200);  // 시리얼 통신 시작 (속도: 115200bps, 디버깅 메시지 출력용)
  Serial.setDebugOutput(true);  // 디버그 출력 활성화 (오류 메시지 출력 가능)
  Serial.println();

  // 카메라 초기화 (OV2640 카메라 모듈 설정 및 활성화)
  cameraInit();
  
  // Wi-Fi 연결 (저장된 "SSCC" 및 비밀번호로 네트워크 접속 시도)
  WiFi.begin("SSCC", "2025sscc");
  while (WiFi.status() != WL_CONNECTED) { // 연결될 때까지 대기
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");  // Wi-Fi 연결 성공 메시지 출력

  // WebSocket 서버 연결 (서버 주소, 포트 및 경로 설정)
  webSocket.begin("192.168.0.209", 8000, "/ws/video_feed");
  webSocket.onEvent(webSocketEvent);  // WebSocket 이벤트 핸들러 설정 (연결 및 데이터 수신 처리)
}

void loop() {
  webSocket.loop();  // WebSocket 이벤트 처리 (서버와의 연결 유지 및 데이터 수신 대기)

  sendVideoFrame();  // 카메라 프레임 캡처 및 WebSocket을 통한 전송
  delay(30);  // 프레임 전송 속도 조절 (약 33fps, 지연을 추가하여 안정적인 송출 유지)
}

// 카메라 초기화 함수 (카메라 설정을 적용하고 활성화)
void cameraInit(void) {
  camera_config_t config;  // 카메라 설정 구조체 선언
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;  // 데이터 라인 설정
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;  // 카메라 클럭 핀 설정
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;  // 수직 동기화 핀 설정
  config.pin_href = HREF_GPIO_NUM;  // 수평 동기화 핀 설정
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;  // XCLK 주파수 설정 (20MHz)
  config.frame_size = FRAMESIZE_UXGA;  // 기본 해상도를 UXGA(1600x1200)로 설정
  config.pixel_format = PIXFORMAT_JPEG;  // 영상 데이터를 JPEG 포맷으로 설정
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;  // 기본 JPEG 품질 설정 (값이 낮을수록 화질 증가, 용량 증가)
  config.fb_count = 1;  // 기본 프레임 버퍼 개수 설정

  if (psramFound()) {
    config.jpeg_quality = 10;  // PSRAM이 존재하면 JPEG 품질 향상 (화질 개선)
    config.fb_count = 2;  // 추가 프레임 버퍼 사용 (성능 향상)
    config.grab_mode = CAMERA_GRAB_LATEST;
  } else {
    config.frame_size = FRAMESIZE_SVGA;  // PSRAM이 없을 경우 해상도 낮춤 (SVGA)
    config.fb_location = CAMERA_FB_IN_DRAM;  // DRAM을 사용하여 프레임 버퍼 저장
  }

  esp_err_t err = esp_camera_init(&config);  // 카메라 초기화 실행
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);  // 초기화 실패 시 오류 메시지 출력
    return;
  }

  sensor_t * s = esp_camera_sensor_get();  // 카메라 센서 객체 가져오기
  s->set_vflip(s, 1);  // 이미지 상하 반전
  s->set_brightness(s, 1);  // 밝기 증가
  s->set_saturation(s, 0);  // 채도 기본값 유지
}

// WebSocket을 통해 비디오 프레임을 전송하는 함수
void sendVideoFrame() {
  camera_fb_t * fb = esp_camera_fb_get();  // 카메라에서 프레임 캡처 (JPEG 형식)
  if (!fb) {
    Serial.println("Camera capture failed");  // 캡처 실패 시 오류 메시지 출력
    return;
  }

  webSocket.sendBIN(fb->buf, fb->len);  // WebSocket을 통해 바이너리 데이터(영상 프레임) 전송

  esp_camera_fb_return(fb);  // 사용한 프레임 버퍼 반환 (메모리 누수 방지)
}

// WebSocket 이벤트 핸들러 (연결 상태 및 수신 데이터 처리)
void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.println("WebSocket Disconnected");  // WebSocket 연결 해제 시 출력
      break;
    case WStype_CONNECTED:
      Serial.println("WebSocket Connected");  // WebSocket 연결 성공 시 출력
      break;
    case WStype_TEXT:
      Serial.printf("WebSocket Text Message: %s\n", payload);  // 텍스트 메시지 수신 시 출력
      break;
    case WStype_BIN:
      Serial.println("WebSocket Binary Message Received");  // 바이너리 메시지 수신 시 출력
      break;
    default:
      break;  
  }
}
