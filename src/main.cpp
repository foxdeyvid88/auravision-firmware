#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <SPI.h>
#include <SD.h>
#include "Audio.h"
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsClient.h>
#include "esp_camera.h"

// ── Nombre BLE ───────────────────────────────────────────────
#define DEVICE_NAME "ESP32-AuraVisión"

// ── UUIDs estándar UART over BLE (Nordic UART Service) ───────────────────────
#define SERVICE_UUID   "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHAR_UUID_RX   "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  // Teléfono → ESP32
#define CHAR_UUID_TX   "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // ESP32 → Teléfono

// ==== WiFi ====
const char* WIFI_SSID = "";
const char* WIFI_PASS = "";

// ==== Backend ====
const char* WS_HOST        = "191.237.250.3";
const uint16_t WS_PORT     = 8000;
const char* DISPOSITIVO_ID = "";
const char* ACCESS_TOKEN   = "";

// ── Pines I2S ────────────────────────────────────────────────────────────────
#define I2S_BCLK  2
#define I2S_LRC   3
#define I2S_DOUT  1

// ── Pines SD ─────────────────────────────────────────────────────────────────
#define SD_SCK   7
#define SD_MISO  8
#define SD_MOSI  9
#define SD_CS    21

// ==== Pines camara XIAO ESP32S3 Sense (OV2640) ====
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     10
#define SIOD_GPIO_NUM     40
#define SIOC_GPIO_NUM     39
#define Y9_GPIO_NUM       48
#define Y8_GPIO_NUM       11
#define Y7_GPIO_NUM       12
#define Y6_GPIO_NUM       14
#define Y5_GPIO_NUM       16
#define Y4_GPIO_NUM       18
#define Y3_GPIO_NUM       17
#define Y2_GPIO_NUM       15
#define VSYNC_GPIO_NUM    38
#define HREF_GPIO_NUM     47
#define PCLK_GPIO_NUM     13

BLEServer*          pServer  = nullptr;
BLECharacteristic*  pTxChar  = nullptr;
WebSocketsClient webSocket;
Audio audio;


// Volumen 0 - 21
  audio.setVolume(16);

  // Ecualización
  audio.setTone(-8, 0, -4);