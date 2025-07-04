#include <Arduino.h>
#include <WiFi.h>
#include <WebSocketsClient.h>
#include "secrets.h"

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;
const char* websocket_server_host = WS_SERVER_HOST;
const uint16_t websocket_server_port = WS_SERVER_PORT;
const char* websocket_path = WS_SERVER_PATH;

WebSocketsClient webSocket;

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_TEXT:
      Serial.printf("[WSc] get text: %s\n", payload);
      delay(100);
      if(strcmp((char*)payload, "{\"command\":\"start\"}") == 0) {
        webSocket.sendTXT("{\"status\": \"started\"}");
        Serial.println("[WSc] sent: started");
      }
      else if (strcmp((char*)payload, "{\"command\":\"stop\"}") == 0) {
        webSocket.sendTXT("{\"status\": \"stopped\"}");
        Serial.println("[WSc] sent: stopped");
      }
      break;

    case WStype_DISCONNECTED:
      Serial.printf("[WSc] Disconnected!\n");
      break;
    case WStype_CONNECTED:
      Serial.printf("[WSc] Connected to url: %s\n", payload);
      webSocket.sendTXT("{\"status\": \"ready\"}");
      break;
  }
}

void setup() {
  Serial.begin(115200);

  WiFi.begin(ssid, password);
  Serial.print("Connecting Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected Wi-Fi!");

  webSocket.begin(websocket_server_host, websocket_server_port, websocket_path);
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);
}

void loop() {
  webSocket.loop();
}
