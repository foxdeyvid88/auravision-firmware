/**
 * AuraVision Firmware - ESP32 XIAO SENSE (OV2640)
 *
 * Flujo:
 *  1. Iniciar BLE (Nordic UART Service) → recibir WiFi/Backend config
 *  2. Guardar parámetros en NVS (Preferences)
 *  3. Conectar a WiFi
 *  4. Registrar dispositivo en backend (POST /api/v1/dispositivos)
 *  5. Guardar id_dispositivo en NVS
 *  6. Crear sesión  (POST /api/v1/sesiones_dispositivos)
 *  7. Abrir WebSocket /api/v1/ws/dispositivos/<id>/vision?token=<token>
 *  8. Loop: capturar JPEG 640×480 cada 500 ms → enviar por WS
 *  9. Recibir JSON de detecciones → priorizar → reproducir audio desde SD
 */
#include <Arduino.h>
#include <Preferences.h>
// ── BLE ──────────────────────────────────────────────────────────────────────
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
// ── Red ──────────────────────────────────────────────────────────────────────
#include <WiFi.h>
#include <HTTPClient.h>
#include <WebSocketsClient.h>
// ── Cámara ───────────────────────────────────────────────────────────────────
#include "esp_camera.h"
// ── Audio / SD ───────────────────────────────────────────────────────────────
#include <SPI.h>
#include <SD.h>
#include "Audio.h"
// ── JSON ──────────────────────────────────────────────────────────────────────
#include <ArduinoJson.h>
// =============================================================================
//  Constantes del dispositivo
// =============================================================================
#define FIRMWARE_VERSION  "1.0.0"
#define DEVICE_NAME       "AuraVision"
// ── UUIDs Nordic UART Service ─────────────────────────────────────────────────
#define BLE_SERVICE_UUID  "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define BLE_CHAR_RX_UUID  "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  // Teléfono → ESP32
#define BLE_CHAR_TX_UUID  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // ESP32 → Teléfono
// ── Pines I2S ─────────────────────────────────────────────────────────────────
#define I2S_BCLK  2
#define I2S_LRC   3
#define I2S_DOUT  1
// ── Pines SD (SPI secundario) ─────────────────────────────────────────────────
#define SD_SCK   7
#define SD_MISO  8
#define SD_MOSI  9
#define SD_CS    21
// ── Pines cámara OV2640 (XIAO ESP32S3 Sense) ─────────────────────────────────
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    10
#define SIOD_GPIO_NUM    40
#define SIOC_GPIO_NUM    39
#define Y9_GPIO_NUM      48
#define Y8_GPIO_NUM      11
#define Y7_GPIO_NUM      12
#define Y6_GPIO_NUM      14
#define Y5_GPIO_NUM      16
#define Y4_GPIO_NUM      18
#define Y3_GPIO_NUM      17
#define Y2_GPIO_NUM      15
#define VSYNC_GPIO_NUM   38
#define HREF_GPIO_NUM    47
#define PCLK_GPIO_NUM    13
// ── Timing ───────────────────────────────────────────────────────────────────
#define CAPTURE_INTERVAL_MS  500
// =============================================================================
//  Matriz de priorización de obstáculos (Nivel riesgo × Posición)
//  Posición: frente_izquierda=1, frente_derecha=1, frente=2
//  Nivel riesgo obstáculo: Bajo=1, Medio=2, Alto=3
//  Puntaje = NivelRiesgo × PesoPostion  (de la hoja "Matriz de Riesgo")
// =============================================================================
struct ObstacleRisk {
    const char* label;
    int         riskLevel;   // 1=Bajo, 2=Medio, 3=Alto
};
static const ObstacleRisk RISK_TABLE[] = {
    { "Chair",           1 },
    { "Table",           1 },
    { "Door",            1 },
    { "Traffic_Light",   1 },
    { "Cross_Walk",      1 },
    { "Trash_Can",       1 },
    { "Bicycle",         2 },
    { "Pole",            2 },
    { "Person",          2 },
    { "Car",             3 },
    { "Bus",             3 },
    { "Truck",           3 },
    { "Motorcycle",      3 },
    { "Electric_Scooter",3 },
};
/** Devuelve el nivel de riesgo (1-3) de un obstáculo dado su label. */
static int getRiskLevel(const String& label) {
    for (auto& r : RISK_TABLE) {
        if (label.equalsIgnoreCase(r.label)) return r.riskLevel;
    }
    return 1; // desconocido → riesgo bajo
}
/** Peso de posición según la matriz (Frente=2, Laterales=1). */
static int getPositionWeight(const String& pos) {
    if (pos.equalsIgnoreCase("frente")) return 2;
    return 1; // frente_izquierda, frente_derecha
}
/** Puntuación final: riskLevel × positionWeight. */
static int getPriority(const String& label, const String& pos) {
    return getRiskLevel(label) * getPositionWeight(pos);
}
// =============================================================================
//  Variables globales de configuración (persistidas en NVS)
// =============================================================================
Preferences prefs;
String g_wifiSSID;
String g_wifiPass;
String g_wsHost;
uint16_t g_wsPort      = 8000;
String g_accessToken;
String g_urlBackend;
String g_dispositivoId;
// =============================================================================
//  Estado de la máquina
// =============================================================================
enum class State {
    BLE_WAITING,       // Esperando config por BLE
    WIFI_CONNECTING,   // Conectando a WiFi
    REGISTERING,       // POST /api/v1/dispositivos
    CREATING_SESSION,  // POST /api/v1/sesiones_dispositivos
    WS_CONNECTING,     // Conectando WebSocket
    STREAMING,         // Capturando y enviando frames
};
State g_state = State::BLE_WAITING;
// =============================================================================
//  BLE
// =============================================================================
BLEServer*         g_bleServer   = nullptr;
BLECharacteristic* g_bleTxChar   = nullptr;
bool               g_bleConnected = false;
String             g_bleBuffer;   // buffer para mensajes largos fragmentados
/** Envía una cadena al teléfono vía BLE (TX characteristic). */
void bleSend(const String& msg) {
    if (g_bleTxChar && g_bleConnected) {
        g_bleTxChar->setValue(msg.c_str());
        g_bleTxChar->notify();
        Serial.printf("[BLE TX] → %s\n", msg.c_str());
    } else if (!g_bleConnected) {
        Serial.println("[BLE TX] Sin cliente conectado, no se envió respuesta");
    }
}
class BleServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) override {
        g_bleConnected = true;
        Serial.println("[BLE] ══════════════════════════════════════");
        Serial.println("[BLE] ✔ Cliente CONECTADO");
        Serial.println("[BLE] ══════════════════════════════════════");
        Serial.println("[BLE] Esperando JSON de configuración por RX...");
    }
    void onDisconnect(BLEServer*) override {
        g_bleConnected = false;
        Serial.println("[BLE] ──────────────────────────────────────");
        Serial.println("[BLE] ✘ Cliente DESCONECTADO");
        Serial.println("[BLE] Reanudando advertising...");
        Serial.println("[BLE] ──────────────────────────────────────");
        BLEDevice::startAdvertising();
    }
};
/**
 * Interpreta el JSON recibido por BLE con los parámetros de configuración.
 * Formato esperado:
 *   {"Wifi_SSID":"...","Wifi_PASS":"...","URL_BACKEND":"http://host:port","ACCESS_TOKEN":"..."}
 */
void parseAndSaveConfig(const String& json) {
    Serial.println("[BLE] Procesando JSON recibido...");
    Serial.printf ("[BLE] Contenido: %s\n", json.c_str());
    StaticJsonDocument<512> doc;
    DeserializationError err = deserializeJson(doc, json);
    if (err) {
        Serial.printf("[BLE] ✘ JSON inválido: %s\n", err.c_str());
        bleSend("{\"status\":\"error\",\"msg\":\"JSON invalido\"}");
        return;
    }
    bool changed = false;
    // Soportar variantes comunes por si la app las envía con mayúsculas/minúsculas diferentes
    if (doc.containsKey("Wifi_SSID"))      { g_wifiSSID = doc["Wifi_SSID"].as<String>(); changed = true; }
    else if (doc.containsKey("wifi_ssid")) { g_wifiSSID = doc["wifi_ssid"].as<String>(); changed = true; }
    else if (doc.containsKey("ssid"))      { g_wifiSSID = doc["ssid"].as<String>();      changed = true; }
    if (doc.containsKey("Wifi_PASS"))      { g_wifiPass = doc["Wifi_PASS"].as<String>(); changed = true; }
    else if (doc.containsKey("wifi_pass")) { g_wifiPass = doc["wifi_pass"].as<String>(); changed = true; }
    else if (doc.containsKey("password"))  { g_wifiPass = doc["password"].as<String>();  changed = true; }
    if (doc.containsKey("URL_BACKEND"))      { g_urlBackend = doc["URL_BACKEND"].as<String>(); changed = true; }
    else if (doc.containsKey("url_backend")) { g_urlBackend = doc["url_backend"].as<String>(); changed = true; }
    if (doc.containsKey("ACCESS_TOKEN"))      { g_accessToken = doc["ACCESS_TOKEN"].as<String>(); changed = true; }
    else if (doc.containsKey("access_token")) { g_accessToken = doc["access_token"].as<String>(); changed = true; }
    else if (doc.containsKey("token"))        { g_accessToken = doc["token"].as<String>();        changed = true; }
    Serial.printf("[BLE] ✔ SSID     = \"%s\"\n", g_wifiSSID.c_str());
    Serial.printf("[BLE] ✔ PASS     = \"%s\"\n", g_wifiPass.c_str());
    Serial.printf("[BLE] ✔ BACKEND  = \"%s\"\n", g_urlBackend.c_str());
    Serial.printf("[BLE] ✔ TOKEN    = \"%s\"\n", g_accessToken.c_str());
    if (!changed) {
        Serial.println("[BLE] ⚠ JSON recibido sin campos reconocidos (sin cambios)");
        return;
    }
    // Extraer host y puerto de URL_BACKEND (ej. http://1.2.3.4:8000 o http://host:8000)
    String url = g_urlBackend;
    if (url.startsWith("http://"))  url = url.substring(7);
    if (url.startsWith("https://")) url = url.substring(8);
    int colonIdx = url.lastIndexOf(':');
    if (colonIdx > 0) {
        g_wsHost = url.substring(0, colonIdx);
        int slashIdx = url.indexOf('/', colonIdx);
        String portStr = (slashIdx > 0) ? url.substring(colonIdx + 1, slashIdx)
                                        : url.substring(colonIdx + 1);
        g_wsPort = (uint16_t)portStr.toInt();
        if (g_wsPort == 0) g_wsPort = 8000;
    } else {
        g_wsHost = url;
        g_wsPort = 8000;
    }
    Serial.printf("[BLE] WS_HOST extraído  = \"%s\"\n", g_wsHost.c_str());
    Serial.printf("[BLE] WS_PORT extraído  = %u\n",    g_wsPort);
    // Guardar en NVS
    Serial.println("[BLE] Guardando en NVS...");
    prefs.begin("auravision", false);
    prefs.putString("wifi_ssid",    g_wifiSSID);
    prefs.putString("wifi_pass",    g_wifiPass);
    prefs.putString("url_backend",  g_urlBackend);
    prefs.putString("ws_host",      g_wsHost);
    prefs.putUInt  ("ws_port",      g_wsPort);
    prefs.putString("access_token", g_accessToken);
    prefs.end();
    Serial.println("[BLE] ✔ Config guardada en NVS correctamente");
    bleSend("{\"status\":\"ok\",\"msg\":\"Configuracion guardada\"}");
    // Transicionar a conexión WiFi
    if (g_wifiSSID.length() > 0) {
        Serial.println("[BLE] ✔ Transicionando a WIFI_CONNECTING...");
        g_state = State::WIFI_CONNECTING;
    } else {
        Serial.println("[BLE] ⚠ No se inició la conexión WiFi porque el SSID recibido está vacío.");
        Serial.println("[BLE] ⚠ Revisa el JSON que está enviando la app. Asegúrate de incluir el campo SSID con datos.");
    }
}
class BleRxCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pChar) override {
        String val = pChar->getValue().c_str();
        Serial.printf("[BLE RX] Fragmento recibido (%u bytes): %s\n",
                      (unsigned)val.length(), val.c_str());
        g_bleBuffer += val;
        // Intentar parsear cuando llegue un JSON completo (termina en '}')
        int endIdx = g_bleBuffer.lastIndexOf('}');
        if (endIdx >= 0) {
            Serial.printf("[BLE RX] JSON completo detectado (%d bytes en buffer)\n", endIdx + 1);
            String jsonStr = g_bleBuffer.substring(0, endIdx + 1);
            g_bleBuffer = g_bleBuffer.substring(endIdx + 1);
            parseAndSaveConfig(jsonStr);
        } else {
            Serial.printf("[BLE RX] Esperando más fragmentos... (buffer actual: %u bytes)\n",
                          (unsigned)g_bleBuffer.length());
        }
    }
};
void initBLE() {
    Serial.println("[BLE] Inicializando BLE...");
    Serial.printf ("[BLE] Nombre del dispositivo : \"%s\"\n", DEVICE_NAME);
    Serial.printf ("[BLE] Service UUID  : %s\n", BLE_SERVICE_UUID);
    Serial.printf ("[BLE] RX Char UUID  : %s  (WRITE)\n",  BLE_CHAR_RX_UUID);
    Serial.printf ("[BLE] TX Char UUID  : %s  (NOTIFY)\n", BLE_CHAR_TX_UUID);
    BLEDevice::init(DEVICE_NAME);
    g_bleServer = BLEDevice::createServer();
    g_bleServer->setCallbacks(new BleServerCallbacks());
    BLEService* pService = g_bleServer->createService(BLE_SERVICE_UUID);
    // Característica TX (ESP32 → Teléfono) con Notify
    g_bleTxChar = pService->createCharacteristic(BLE_CHAR_TX_UUID,
                                                   BLECharacteristic::PROPERTY_NOTIFY);
    g_bleTxChar->addDescriptor(new BLE2902());
    // Característica RX (Teléfono → ESP32) con Write
    BLECharacteristic* pRxChar = pService->createCharacteristic(BLE_CHAR_RX_UUID,
                                                                  BLECharacteristic::PROPERTY_WRITE |
                                                                  BLECharacteristic::PROPERTY_WRITE_NR);
    pRxChar->setCallbacks(new BleRxCallbacks());
    pService->start();
    BLEAdvertising* pAdv = BLEDevice::getAdvertising();
    pAdv->addServiceUUID(BLE_SERVICE_UUID);
    pAdv->setScanResponse(true);
    BLEDevice::startAdvertising();
    Serial.println("[BLE] ✔ Advertising activo — esperando conexión de la app...");
}
// =============================================================================
//  NVS – Cargar parámetros guardados
// =============================================================================
void loadConfig() {
    prefs.begin("auravision", true);
    g_wifiSSID     = prefs.getString("wifi_ssid",    "");
    g_wifiPass     = prefs.getString("wifi_pass",    "");
    g_urlBackend   = prefs.getString("url_backend",  "");
    g_wsHost       = prefs.getString("ws_host",      "");
    g_wsPort       = prefs.getUInt  ("ws_port",      8000);
    g_accessToken  = prefs.getString("access_token", "");
    g_dispositivoId= prefs.getString("dispositivo_id","");
    prefs.end();
    Serial.printf("[NVS] SSID=%s Host=%s Port=%u DevID=%s\n",
                  g_wifiSSID.c_str(), g_wsHost.c_str(), g_wsPort, g_dispositivoId.c_str());
}
void saveDispositivoId(const String& id) {
    g_dispositivoId = id;
    prefs.begin("auravision", false);
    prefs.putString("dispositivo_id", id);
    prefs.end();
    Serial.printf("[NVS] id_dispositivo guardado: %s\n", id.c_str());
}
// =============================================================================
//  WiFi
// =============================================================================
String getWifiStatusString(wl_status_t status) {
    switch(status) {
        case WL_IDLE_STATUS:      return "IDLE_STATUS (0)";
        case WL_NO_SSID_AVAIL:    return "NO_SSID_AVAIL (1) - Red no encontrada";
        case WL_SCAN_COMPLETED:   return "SCAN_COMPLETED (2)";
        case WL_CONNECTED:        return "CONNECTED (3)";
        case WL_CONNECT_FAILED:   return "CONNECT_FAILED (4) - Clave incorrecta / Error de auth";
        case WL_CONNECTION_LOST:  return "CONNECTION_LOST (5)";
        case WL_DISCONNECTED:     return "DISCONNECTED (6)";
        default:                  return String("UNKNOWN (") + String(status) + ")";
    }
}
bool connectWiFi() {
    Serial.println("\n[WiFi] ══════════════════════════════════════");
    Serial.printf ("[WiFi] Intentando conectar a la red...\n");
    Serial.printf ("[WiFi] SSID: '%s'\n", g_wifiSSID.c_str());
    Serial.printf ("[WiFi] PASS: '%s' (Longitud: %u caracteres)\n", 
                   g_wifiPass.c_str(), (unsigned)g_wifiPass.length());
    
    // Limpieza inicial por si el módem quedó colgado
    WiFi.disconnect(true, true);
    delay(500);
    WiFi.mode(WIFI_STA);
    Serial.printf ("[WiFi] MAC del ESP32: %s\n", WiFi.macAddress().c_str());
    WiFi.begin(g_wifiSSID.c_str(), g_wifiPass.c_str());
    
    uint32_t t0 = millis();
    wl_status_t lastStatus = WiFi.status();
    
    Serial.print("[WiFi] Conectando ");
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
        delay(500);
        Serial.print(".");
        
        wl_status_t currentStatus = WiFi.status();
        if (currentStatus != lastStatus) {
            Serial.printf("\n[WiFi] Estado cambió a: %s\n[WiFi] Conectando ", getWifiStatusString(currentStatus).c_str());
            lastStatus = currentStatus;
        }
    }
    
    Serial.println(); // Salto de línea después de los puntos
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("[WiFi] ✔ Conexión exitosa!");
        Serial.printf ("[WiFi] IP Asignada: %s\n", WiFi.localIP().toString().c_str());
        Serial.printf ("[WiFi] RSSI (Fuerza de señal): %d dBm\n", WiFi.RSSI());
        Serial.println("[WiFi] ══════════════════════════════════════");
        return true;
    }
    
    Serial.println("[WiFi] ✘ Falló la conexión");
    Serial.printf ("[WiFi] Razón final (Estado): %s\n", getWifiStatusString(WiFi.status()).c_str());
    Serial.println("[WiFi] ══════════════════════════════════════");
    return false;
}
// =============================================================================
//  HTTP helpers
// =============================================================================
/**
 * POST JSON a una ruta relativa del backend.
 * Devuelve el código HTTP; rellena 'responseBody' con el cuerpo.
 */
int httpPost(const String& path, const String& jsonBody, String& responseBody) {
    String url = g_urlBackend;
    // Quitar trailing slash de url y leading slash de path para evitar doble //
    if (url.endsWith("/"))   url.remove(url.length() - 1);
    if (!path.startsWith("/")) url += "/";
    url += path;
    Serial.println("[HTTP] --------------------------------------------------");
    Serial.printf ("[HTTP] POST a: %s\n", url.c_str());
    Serial.printf ("[HTTP] Payload: %s\n", jsonBody.c_str());
    HTTPClient http;
    http.begin(url);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("Authorization", String("Bearer ") + g_accessToken);
    int code = http.POST(jsonBody);
    responseBody = (code > 0) ? http.getString() : "";
    
    Serial.printf ("[HTTP] Status Code: %d\n", code);
    if (responseBody.length() > 0) {
        Serial.printf ("[HTTP] Respuesta: %s\n", responseBody.c_str());
    } else {
        Serial.println("[HTTP] Respuesta: (Vacía)");
    }
    Serial.println("[HTTP] --------------------------------------------------");
    
    http.end();
    return code;
}
// =============================================================================
//  Registro del dispositivo
// =============================================================================
bool registerDevice() {
    String mac = WiFi.macAddress();
    Serial.printf("[REG] Iniciando registro de dispositivo (MAC: %s)...\n", mac.c_str());
    StaticJsonDocument<256> doc;
    doc["nombre"]           = DEVICE_NAME;
    doc["numero_serie"]  = mac;
    doc["version_firmware"] = FIRMWARE_VERSION;
    doc["estado_conexion"]  = "desconectado";
    String body;
    serializeJson(doc, body);
    String resp;
    int code = httpPost("/api/v1/dispositivos", body, resp);
    if (code == 201) {
        Serial.println("[REG] ✔ Dispositivo creado exitosamente (201 Created)");
        StaticJsonDocument<256> respDoc;
        if (deserializeJson(respDoc, resp) == DeserializationError::Ok) {
            String id = respDoc["id_dispositivo"].as<String>();
            if (id.length() > 0) {
                Serial.printf("[REG] ID asignado por el servidor: %s\n", id.c_str());
                saveDispositivoId(id);
                return true;
            }
        }
        Serial.println("[REG] ⚠ Advertencia: El servidor devolvió 201 pero no se encontró 'id_dispositivo' en la respuesta.");
    } else if (code == 200 || code == 409) {
        Serial.println("[REG] ℹ El dispositivo ya estaba registrado.");
        StaticJsonDocument<256> respDoc;
        if (deserializeJson(respDoc, resp) == DeserializationError::Ok &&
            respDoc.containsKey("id_dispositivo")) {
            String id = respDoc["id_dispositivo"].as<String>();
            Serial.printf("[REG] ID recuperado del servidor: %s\n", id.c_str());
            saveDispositivoId(id);
            return true;
        } else {
            Serial.println("[REG] ✘ Error: No se pudo extraer 'id_dispositivo' del dispositivo existente.");
        }
    } else {
        Serial.printf("[REG] ✘ Falló el registro. Código HTTP inesperado: %d\n", code);
    }
    return false;
}
// =============================================================================
//  Creación de sesión
// =============================================================================
bool createSession() {
    Serial.printf("[SES] Solicitando creación de sesión para el dispositivo ID: %s...\n", g_dispositivoId.c_str());
    
    StaticJsonDocument<128> doc;
    doc["id_dispositivo_fk"] = g_dispositivoId;
    String body;
    serializeJson(doc, body);
    String resp;
    int code = httpPost("/api/v1/sesiones-dispositivo", body, resp);
    
    if (code == 201 || code == 200) {
        Serial.println("[SES] ✔ Sesión creada con éxito en el backend.");
        return true;
    } else {
        Serial.printf("[SES] ✘ Falló la creación de sesión (HTTP %d).\n", code);
        return false;
    }
}
// =============================================================================
//  Cámara
// =============================================================================
bool initCamera() {
    camera_config_t cfg = {};
    cfg.ledc_channel = LEDC_CHANNEL_0;
    cfg.ledc_timer   = LEDC_TIMER_0;
    cfg.pin_d0       = Y2_GPIO_NUM;
    cfg.pin_d1       = Y3_GPIO_NUM;
    cfg.pin_d2       = Y4_GPIO_NUM;
    cfg.pin_d3       = Y5_GPIO_NUM;
    cfg.pin_d4       = Y6_GPIO_NUM;
    cfg.pin_d5       = Y7_GPIO_NUM;
    cfg.pin_d6       = Y8_GPIO_NUM;
    cfg.pin_d7       = Y9_GPIO_NUM;
    cfg.pin_xclk     = XCLK_GPIO_NUM;
    cfg.pin_pclk     = PCLK_GPIO_NUM;
    cfg.pin_vsync    = VSYNC_GPIO_NUM;
    cfg.pin_href     = HREF_GPIO_NUM;
    cfg.pin_sscb_sda = SIOD_GPIO_NUM;
    cfg.pin_sscb_scl = SIOC_GPIO_NUM;
    cfg.pin_pwdn     = PWDN_GPIO_NUM;
    cfg.pin_reset    = RESET_GPIO_NUM;
    cfg.xclk_freq_hz = 20000000;
    cfg.pixel_format = PIXFORMAT_JPEG;
    cfg.frame_size   = FRAMESIZE_VGA;   // 640×480
    cfg.jpeg_quality = 12;
    cfg.fb_count     = 2;
    cfg.grab_mode    = CAMERA_GRAB_LATEST;
    esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK) {
        Serial.printf("[CAM] Error init: 0x%x\n", err);
        return false;
    }
    Serial.println("[CAM] Inicializada (640×480 JPEG)");
    return true;
}
// =============================================================================
//  Audio / SD
// =============================================================================
SPIClass sdSPI(HSPI);
Audio    audio;
bool     g_audioReady = false;
bool     g_audioPlaying = false;
/** Cola de archivos pendientes (hasta 2: obstáculo + posición). */
String g_audioQueue[2];
int    g_audioQueueSize = 0;
void audioFinished() {
    g_audioPlaying = false;
}
bool initSD() {
    sdSPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    if (!SD.begin(SD_CS, sdSPI)) {
        Serial.println("[SD] Error al montar");
        return false;
    }
    Serial.println("[SD] Montada correctamente");
    return true;
}
bool initAudio() {
    audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
    audio.setVolume(16);
    audio.setTone(-8, 0, -4);
    return true;
}
/** Reproduce el siguiente audio de la cola si no hay uno en curso. */
void processAudioQueue() {
    if (g_audioPlaying) {
        audio.loop();
        return;
    }
    if (g_audioQueueSize > 0) {
        String file = g_audioQueue[0];
        // Desplazar cola
        g_audioQueue[0] = g_audioQueue[1];
        g_audioQueueSize--;
        if (SD.exists(file.c_str())) {
            Serial.printf("[AUDIO] Reproduciendo: %s\n", file.c_str());
            g_audioPlaying = audio.connecttoFS(SD, file.c_str());
        } else {
            Serial.printf("[AUDIO] Archivo no encontrado: %s\n", file.c_str());
        }
    }
}
// Callback de la librería Audio para detectar fin de reproducción
void audio_eof_mp3(const char* info) {
    Serial.printf("[AUDIO] Fin: %s\n", info);
    g_audioPlaying = false;
}
/** Encola los audios para el obstáculo más riesgoso. */
void enqueueAlerts(const String& label, const String& relPos) {
    g_audioQueueSize = 0;
    g_audioQueue[g_audioQueueSize++] = String("/obstaculos/") + label + ".mp3";
    if (!relPos.equalsIgnoreCase("None") && relPos.length() > 0) {
        g_audioQueue[g_audioQueueSize++] = String("/direcciones/") + relPos + ".mp3";
    }
}
// =============================================================================
//  WebSocket
// =============================================================================
WebSocketsClient g_ws;
bool             g_wsConnected = false;
uint32_t         g_lastCapture = 0;
struct Detection {
    String label;
    float  confidence;
    String relativePosition;
};
/** Procesa el JSON de detecciones, prioriza y dispara audio. */
void handleDetections(const String& json) {
    StaticJsonDocument<2048> doc;
    DeserializationError err = deserializeJson(doc, json);
    if (err) {
        Serial.printf("[WS] ✘ JSON inválido: %s\n", err.c_str());
        return;
    }
    // Si el backend notificó un error procesando la imagen
    if (doc.containsKey("ok") && doc["ok"] == false) {
        String errorMsg = doc["error"] | "Error desconocido del backend";
        Serial.printf("[WS] ⚠ Error reportado por el backend: %s\n", errorMsg.c_str());
        return;
    }
    JsonArray arr = doc.as<JsonArray>();
    if (!arr) {
        // El backend de AuraVision envía: {"ok": true, "resultado": {"detections": [...]}}
        if (doc.containsKey("resultado") && doc["resultado"].containsKey("detections")) {
            arr = doc["resultado"]["detections"].as<JsonArray>();
        } else {
            arr = doc["detections"].as<JsonArray>();
        }
    }
    
    if (!arr || arr.size() == 0) {
        // No hay detecciones o el array está vacío
        return; 
    }
    int    bestPriority = -1;
    String bestLabel;
    String bestPos;
    for (JsonObject det : arr) {
        String label = det["label"]             | "";
        String pos   = det["relative_position"] | "None";
        if (pos.equalsIgnoreCase("None")) continue; // sin posición → ignorar
        int prio = getPriority(label, pos);
        if (prio > bestPriority) {
            bestPriority = prio;
            bestLabel    = label;
            bestPos      = pos;
        }
    }
    if (bestLabel.length() > 0) {
        Serial.printf("[DETECT] Mejor: label=%s pos=%s prio=%d\n",
                      bestLabel.c_str(), bestPos.c_str(), bestPriority);
        enqueueAlerts(bestLabel, bestPos);
    }
}
void webSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
    switch (type) {
        case WStype_CONNECTED:
            g_wsConnected = true;
            Serial.println("[WS] ✔ Conectado al servidor WebSocket");
            g_state = State::STREAMING;
            break;
        case WStype_DISCONNECTED:
            g_wsConnected = false;
            Serial.println("[WS] ✘ Desconectado – reconectando...");
            if (g_state == State::STREAMING) g_state = State::WS_CONNECTING;
            break;
        case WStype_TEXT: {
            String jsonContent = String((char*)payload);
            Serial.println("[WS] --------------------------------------------------");
            Serial.printf ("[WS] Datos recibidos (%u bytes):\n", (unsigned)length);
            Serial.println(jsonContent);
            Serial.println("[WS] --------------------------------------------------");
            handleDetections(jsonContent);
            break;
        }
        case WStype_BIN:
            // No esperado desde el servidor
            break;
        case WStype_ERROR:
            Serial.printf("[WS] Error\n");
            break;
        default:
            break;
    }
}
void initWebSocket() {
    String wsPath = String("/api/v1/ws/dispositivos/") + g_dispositivoId
                    + "/vision?token=" + g_accessToken;
    Serial.printf("[WS] Conectando a ws://%s:%u%s\n",
                  g_wsHost.c_str(), g_wsPort, wsPath.c_str());
    g_ws.begin(g_wsHost.c_str(), g_wsPort, wsPath.c_str());
    g_ws.onEvent(webSocketEvent);
    g_ws.setReconnectInterval(5000);
}
// =============================================================================
//  setup
// =============================================================================
void setup() {
    Serial.begin(115200);
    
    // Esperar hasta 3 segundos a que el puerto Serial nativo se inicialice
    uint32_t t = millis();
    while (!Serial && millis() - t < 3000) {
        delay(10);
    }
    
    Serial.println("\n\n========================================");
    Serial.println("=== AuraVision Firmware " FIRMWARE_VERSION " ===");
    Serial.println("========================================\n");
    // 1. Cargar configuración persistida
    loadConfig();
    // 2. Iniciar BLE siempre (permite actualizar config en cualquier momento)
    initBLE();
    // 3. Iniciar SD y Audio
    if (initSD()) {
        g_audioReady = true;
        initAudio();
    }
    // 4. Si ya tenemos SSID guardado, pasar directo a WiFi
    if (g_wifiSSID.length() > 0) {
        Serial.println("[BOOT] Credenciales encontradas → intentando WiFi");
        g_state = State::WIFI_CONNECTING;
    } else {
        Serial.println("[BOOT] Sin credenciales → esperando config BLE");
        g_state = State::BLE_WAITING;
    }
}
// =============================================================================
//  loop
// =============================================================================
void loop() {
    // Audio siempre activo
    if (g_audioReady) processAudioQueue();
    switch (g_state) {
        // ──────────────────────────────────────────────────────────────────
        case State::BLE_WAITING:
            // Nada especial: los callbacks BLE manejan la transición
            delay(100);
            break;
        // ──────────────────────────────────────────────────────────────────
        case State::WIFI_CONNECTING: {
            if (connectWiFi()) {
                // Inicializar cámara una sola vez
                static bool camInit = false;
                if (!camInit) camInit = initCamera();
                g_state = (g_dispositivoId.length() > 0)
                          ? State::CREATING_SESSION
                          : State::REGISTERING;
            } else {
                Serial.println("[BOOT] Reintentando WiFi en 10 s...");
                delay(10000);
            }
            break;
        }
        // ──────────────────────────────────────────────────────────────────
        case State::REGISTERING:
            Serial.println("[BOOT] Registrando dispositivo...");
            if (registerDevice()) {
                g_state = State::CREATING_SESSION;
            } else {
                Serial.println("[REG] Fallo – reintentando en 10 s");
                delay(10000);
            }
            break;
        // ──────────────────────────────────────────────────────────────────
        case State::CREATING_SESSION:
            Serial.println("[BOOT] Creando sesión...");
            if (createSession()) {
                g_state = State::WS_CONNECTING;
                initWebSocket();
            } else {
                Serial.println("[SES] Fallo – reintentando en 10 s");
                delay(10000);
            }
            break;
        // ──────────────────────────────────────────────────────────────────
        case State::WS_CONNECTING:
            g_ws.loop();  // La reconexión la maneja la librería
            break;
        // ──────────────────────────────────────────────────────────────────
        case State::STREAMING: {
            g_ws.loop();
            uint32_t now = millis();
            if (now - g_lastCapture >= CAPTURE_INTERVAL_MS) {
                g_lastCapture = now;
                camera_fb_t* fb = esp_camera_fb_get();
                if (!fb) {
                    Serial.println("[CAM] Error al capturar frame");
                    break;
                }
                if (g_wsConnected) {
                    bool sent = g_ws.sendBIN(fb->buf, fb->len);
                    if (!sent) {
                        Serial.println("[WS] Error enviando frame");
                    }
                }
                esp_camera_fb_return(fb);
            }
            break;
        }
    }
}