/**
 * AuraVision Firmware - ESP32 XIAO S3 Sense (OV2640)
 *
 * FLUJO:
 *
 *  1. Encender ESP32
 *  2. Cargar configuración NO sensible desde NVS
 *  3. Iniciar BLE
 *  4. Esperar conexión del teléfono
 *  5. Recibir por BLE:
 *
 *       Wifi_SSID
 *       Wifi_PASS
 *       URL_BACKEND
 *       ACCESS_TOKEN
 *
 *  6. Conectar WiFi SOLO si BLE sigue conectado
 *  7. Registrar dispositivo si aún no existe
 *  8. Crear sesión:
 *
 *       POST /api/v1/sesiones-dispositivo
 *
 *  9. Abrir WebSocket
 * 10. Capturar imágenes y enviarlas
 *
 *
 * AL DESCONECTARSE BLUETOOTH:
 *
 *  1. DELETE /api/v1/sesiones-dispositivo/actual
 *  2. Cerrar WebSocket
 *  3. Desconectar WiFi
 *  4. Apagar WiFi
 *  5. Borrar ACCESS_TOKEN de RAM
 *  6. Limpiar buffer BLE
 *  7. Volver a BLE_WAITING
 *  8. Reanudar advertising
 *
 *
 * SEGURIDAD:
 *
 * - ACCESS_TOKEN NO se guarda en NVS.
 * - ACCESS_TOKEN existe solamente en RAM.
 * - Al reiniciar el ESP32, ACCESS_TOKEN está vacío.
 * - Sin BLE conectado no se permite:
 *
 *      WiFi
 *      HTTP
 *      WebSocket
 *      Streaming
 */

#include <Arduino.h>
#include <Preferences.h>

// =============================================================================
// BLE
// =============================================================================

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// =============================================================================
// Red
// =============================================================================

#include <WiFi.h>
#include <HTTPClient.h>
#include <WebSocketsClient.h>

// =============================================================================
// Cámara
// =============================================================================

#include "esp_camera.h"

// =============================================================================
// Audio / SD
// =============================================================================

#include <SPI.h>
#include <SD.h>
#include "Audio.h"

// =============================================================================
// JSON
// =============================================================================

#include <ArduinoJson.h>


// =============================================================================
// Constantes
// =============================================================================

#define FIRMWARE_VERSION "1.0.0"
#define DEVICE_NAME      "AuraVision"


// =============================================================================
// BLE Nordic UART Service
// =============================================================================

#define BLE_SERVICE_UUID \
    "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"

#define BLE_CHAR_RX_UUID \
    "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

#define BLE_CHAR_TX_UUID \
    "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"


// =============================================================================
// Pines I2S
// =============================================================================

#define I2S_BCLK  2
#define I2S_LRC   3 
#define I2S_DOUT  1 


// =============================================================================
// Pines SD
// =============================================================================

#define SD_SCK    7
#define SD_MISO   8
#define SD_MOSI   9
#define SD_CS     21


// =============================================================================
// Pines cámara OV2640 - XIAO ESP32S3 Sense
// =============================================================================

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


// =============================================================================
// Timing
// =============================================================================

#define CAPTURE_INTERVAL_MS 500


// =============================================================================
// Matriz de priorización
// =============================================================================

struct ObstacleRisk {

    const char* label;

    int riskLevel;
};


static const ObstacleRisk RISK_TABLE[] = {

    { "Chair",             1 },
    { "Table",             1 },
    { "Door",              1 },
    { "Traffic_Light",     1 },
    { "Cross_Walk",        1 },
    { "Trash_Can",         1 },

    { "Bicycle",           2 },
    { "Pole",              2 },
    { "Person",            2 },

    { "Car",               3 },
    { "Bus",               3 },
    { "Truck",             3 },
    { "Motorcycle",        3 },
    { "Electric_Scooter",  3 }
};


// =============================================================================
// Riesgo
// =============================================================================

static int getRiskLevel(
    const String& label
) {

    for (auto& r : RISK_TABLE) {

        if (
            label.equalsIgnoreCase(
                r.label
            )
        ) {

            return r.riskLevel;
        }
    }


    return 1;
}


// =============================================================================
// Peso posición
// =============================================================================

static int getPositionWeight(
    const String& pos
) {

    if (
        pos.equalsIgnoreCase(
            "frente"
        )
    ) {

        return 2;
    }


    return 1;
}


// =============================================================================
// Prioridad
// =============================================================================

static int getPriority(
    const String& label,
    const String& pos
) {

    return
        getRiskLevel(label)
        *
        getPositionWeight(pos);
}


// =============================================================================
// Preferences
// =============================================================================

Preferences prefs;


// =============================================================================
// Configuración persistente
// =============================================================================

String g_wifiSSID;

String g_wifiPass;

String g_urlBackend;

String g_wsHost;

uint16_t g_wsPort = 8000;

String g_dispositivoId;


// =============================================================================
// ACCESS TOKEN
//
// IMPORTANTE:
//
// Este valor NO se guarda en NVS.
// Existe solamente en RAM.
// =============================================================================

String g_accessToken;


// =============================================================================
// Máquina de estados
// =============================================================================

enum class State {

    BLE_WAITING,

    WIFI_CONNECTING,

    REGISTERING,

    CREATING_SESSION,

    WS_CONNECTING,

    STREAMING
};


State g_state =
    State::BLE_WAITING;


// =============================================================================
// BLE
// =============================================================================

BLEServer* g_bleServer =
    nullptr;


BLECharacteristic* g_bleTxChar =
    nullptr;


bool g_bleConnected =
    false;


String g_bleBuffer;


// =============================================================================
// WebSocket
// =============================================================================

WebSocketsClient g_ws;


bool g_wsConnected =
    false;


uint32_t g_lastCapture =
    0;


// =============================================================================
// Cámara
// =============================================================================

bool g_cameraInitialized =
    false;


// =============================================================================
// Audio
// =============================================================================

SPIClass sdSPI(
    HSPI
);


Audio audio;


bool g_audioReady =
    false;


bool g_audioPlaying =
    false;


String g_audioQueue[2];


int g_audioQueueSize =
    0;


// =============================================================================
// Control de secuencia de alertas de audio
//
// Mientras se reproduce:
//     obstaculo -> direccion
//
// no se aceptan nuevas alertas.
// Al terminar la secuencia completa se aplica un cooldown corto para evitar
// repetir inmediatamente el mismo objeto en cada frame.
// =============================================================================

#define AUDIO_ALERT_COOLDOWN_MS 2000

bool g_audioSequenceActive =
    false;

uint32_t g_lastAudioAlertFinished =
    0;


// =============================================================================
// Prototipos
// =============================================================================

void stopActiveSession();

bool finishCurrentSession();

void initWebSocket();


// =============================================================================
// BLE SEND
// =============================================================================

void bleSend(
    const String& msg
) {

    if (
        g_bleTxChar != nullptr &&
        g_bleConnected
    ) {

        g_bleTxChar->setValue(
            msg.c_str()
        );


        g_bleTxChar->notify();


        Serial.printf(
            "[BLE TX] %s\n",
            msg.c_str()
        );
    }
}


// =============================================================================
// Finalizar sesión actual en Backend
//
// DELETE /api/v1/sesiones-dispositivo/actual
//
// IMPORTANTE:
//
// Esta función puede ejecutarse cuando:
//     g_bleConnected == false
//
// porque justamente se utiliza cuando Bluetooth acaba de desconectarse.
//
// NO debemos comprobar BLE aquí.
//
// Necesitamos todavía:
//
//     WiFi
//     ACCESS_TOKEN
// =============================================================================

bool finishCurrentSession() {

    Serial.println();

    Serial.println(
        "[SES-END] ========================================"
    );

    Serial.println(
        "[SES-END] Finalizando sesión actual..."
    );

    Serial.println(
        "[SES-END] ========================================"
    );


    // =========================================================================
    // WiFi obligatorio
    // =========================================================================

    if (
        WiFi.status() !=
        WL_CONNECTED
    ) {

        Serial.println(
            "[SES-END] No se puede finalizar: WiFi desconectado"
        );


        return false;
    }


    // =========================================================================
    // Token obligatorio
    // =========================================================================

    if (
        g_accessToken.length() == 0
    ) {

        Serial.println(
            "[SES-END] No se puede finalizar: ACCESS_TOKEN vacío"
        );


        return false;
    }


    // =========================================================================
    // Backend obligatorio
    // =========================================================================

    if (
        g_urlBackend.length() == 0
    ) {

        Serial.println(
            "[SES-END] No se puede finalizar: URL_BACKEND vacío"
        );


        return false;
    }


    // =========================================================================
    // Crear URL
    // =========================================================================

    String url =
        g_urlBackend;


    if (
        url.endsWith("/")
    ) {

        url.remove(
            url.length() - 1
        );
    }


    url +=
        "/api/v1/sesiones-dispositivo/actual";


    Serial.printf(
        "[SES-END] DELETE %s\n",
        url.c_str()
    );


    // =========================================================================
    // HTTP
    // =========================================================================

    HTTPClient http;


    bool started =
        http.begin(
            url
        );


    if (!started) {

        Serial.println(
            "[SES-END] Error iniciando HTTPClient"
        );


        return false;
    }


    // =========================================================================
    // Authorization
    // =========================================================================

    http.addHeader(
        "Authorization",
        String("Bearer ") +
        g_accessToken
    );


    http.addHeader(
        "Content-Type",
        "application/json"
    );


    // =========================================================================
    // DELETE
    // =========================================================================

    int code =
        http.sendRequest(
            "DELETE"
        );


    String responseBody;


    if (
        code > 0
    ) {

        responseBody =
            http.getString();
    }


    Serial.printf(
        "[SES-END] HTTP Status: %d\n",
        code
    );


    if (
        responseBody.length() > 0
    ) {

        Serial.printf(
            "[SES-END] Respuesta: %s\n",
            responseBody.c_str()
        );
    }


    http.end();


    // =========================================================================
    // Cualquier 2xx es éxito
    // =========================================================================

    if (
        code >= 200 &&
        code < 300
    ) {

        Serial.println(
            "[SES-END] Sesión finalizada correctamente"
        );


        return true;
    }


    Serial.printf(
        "[SES-END] Error cerrando sesión. HTTP %d\n",
        code
    );


    return false;
}


// =============================================================================
// Cerrar sesión completa del dispositivo
// =============================================================================

void stopActiveSession() {

    Serial.println();

    Serial.println(
        "[SESSION] ========================================"
    );

    Serial.println(
        "[SESSION] Cerrando sesión AuraVision..."
    );

    Serial.println(
        "[SESSION] ========================================"
    );


    // =========================================================================
    // 1. FINALIZAR SESIÓN EN BACKEND
    //
    // IMPORTANTE:
    //
    // Debe ocurrir ANTES de:
    //
    //      cerrar WiFi
    //      borrar ACCESS_TOKEN
    //
    // =========================================================================

    if (
        WiFi.status() ==
            WL_CONNECTED
        &&
        g_accessToken.length() > 0
    ) {

        Serial.println(
            "[SESSION] Notificando al backend..."
        );


        bool sessionClosed =
            finishCurrentSession();


        if (
            sessionClosed
        ) {

            Serial.println(
                "[SESSION] Backend confirmó cierre"
            );

        } else {

            Serial.println(
                "[SESSION] No fue posible confirmar cierre en backend"
            );
        }

    } else {

        Serial.println(
            "[SESSION] No es posible notificar al backend"
        );


        if (
            WiFi.status() !=
            WL_CONNECTED
        ) {

            Serial.println(
                "[SESSION] - WiFi no conectado"
            );
        }


        if (
            g_accessToken.length() == 0
        ) {

            Serial.println(
                "[SESSION] - ACCESS_TOKEN vacío"
            );
        }
    }


    // =========================================================================
    // 2. Cerrar WebSocket
    // =========================================================================

    Serial.println(
        "[SESSION] Cerrando WebSocket..."
    );


    g_ws.disconnect();


    g_wsConnected =
        false;


    Serial.println(
        "[SESSION] WebSocket cerrado"
    );


    // =========================================================================
    // 3. Desconectar WiFi
    // =========================================================================

    if (
        WiFi.status() ==
        WL_CONNECTED
    ) {

        Serial.println(
            "[SESSION] Desconectando WiFi..."
        );


        WiFi.disconnect(
            true,
            false
        );


        delay(100);
    }


    // =========================================================================
    // 4. Apagar WiFi
    // =========================================================================

    WiFi.mode(
        WIFI_OFF
    );


    Serial.println(
        "[SESSION] WiFi apagado"
    );


    // =========================================================================
    // 5. Borrar TOKEN
    // =========================================================================

    if (
        g_accessToken.length() > 0
    ) {

        Serial.println(
            "[SECURITY] Eliminando ACCESS_TOKEN de RAM..."
        );


        g_accessToken = "";
    }


    Serial.println(
        "[SECURITY] ACCESS_TOKEN eliminado"
    );


    // =========================================================================
    // 6. Limpiar buffer BLE
    // =========================================================================

    g_bleBuffer = "";


    // =========================================================================
    // 7. Limpiar captura
    // =========================================================================

    g_lastCapture = 0;


    // =========================================================================
    // 8. Volver a espera BLE
    // =========================================================================

    g_state =
        State::BLE_WAITING;


    Serial.println(
        "[SESSION] Estado = BLE_WAITING"
    );


    Serial.println(
        "[SESSION] ========================================"
    );
}


// =============================================================================
// BLE SERVER CALLBACKS
// =============================================================================

class BleServerCallbacks :
    public BLEServerCallbacks {

    void onConnect(
        BLEServer* pServer
    ) override {

        g_bleConnected =
            true;


        // =====================================================================
        // Toda nueva conexión BLE comienza sin token.
        //
        // La app debe volver a enviarlo.
        // =====================================================================

        g_accessToken = "";


        // =====================================================================
        // Limpiar datos fragmentados anteriores
        // =====================================================================

        g_bleBuffer = "";


        Serial.println();

        Serial.println(
            "[BLE] ========================================"
        );

        Serial.println(
            "[BLE] TELÉFONO CONECTADO"
        );

        Serial.println(
            "[BLE] ========================================"
        );


        Serial.println(
            "[BLE] ACCESS_TOKEN vacío"
        );


        Serial.println(
            "[BLE] Esperando configuración desde la app..."
        );
    }


    void onDisconnect(
        BLEServer* pServer
    ) override {

        // =====================================================================
        // Bluetooth ya está desconectado.
        // =====================================================================

        g_bleConnected =
            false;


        Serial.println();

        Serial.println(
            "[BLE] ========================================"
        );

        Serial.println(
            "[BLE] TELÉFONO DESCONECTADO"
        );

        Serial.println(
            "[BLE] ========================================"
        );


        // =====================================================================
        // MUY IMPORTANTE:
        //
        // Todavía conservamos:
        //
        //      WiFi
        //      ACCESS_TOKEN
        //
        // stopActiveSession():
        //
        // 1. DELETE sesión
        // 2. WebSocket OFF
        // 3. WiFi OFF
        // 4. borrar TOKEN
        //
        // =====================================================================

        stopActiveSession();


        // =====================================================================
        // Volver a advertising
        // =====================================================================

        delay(300);


        BLEDevice::startAdvertising();


        Serial.println(
            "[BLE] Advertising reanudado"
        );


        Serial.println(
            "[BLE] Esperando nuevamente al teléfono..."
        );
    }
};


// =============================================================================
// Procesar configuración BLE
// =============================================================================

void parseAndSaveConfig(
    const String& json
) {

    // =========================================================================
    // BLE obligatorio
    // =========================================================================

    if (
        !g_bleConnected
    ) {

        Serial.println(
            "[BLE] Config ignorada: teléfono desconectado"
        );


        return;
    }


    Serial.println();

    Serial.println(
        "[BLE] Procesando configuración..."
    );


    // =========================================================================
    // JSON
    // =========================================================================

    StaticJsonDocument<2048> doc;


    DeserializationError err =
        deserializeJson(
            doc,
            json
        );


    if (err) {

        Serial.printf(
            "[BLE] JSON inválido: %s\n",
            err.c_str()
        );


        bleSend(
            "{\"status\":\"error\",\"msg\":\"JSON invalido\"}"
        );


        return;
    }


    bool changed =
        false;


    // =========================================================================
    // SSID
    // =========================================================================

    if (
        doc.containsKey(
            "Wifi_SSID"
        )
    ) {

        g_wifiSSID =
            doc["Wifi_SSID"]
            .as<String>();


        changed = true;

    } else if (
        doc.containsKey(
            "wifi_ssid"
        )
    ) {

        g_wifiSSID =
            doc["wifi_ssid"]
            .as<String>();


        changed = true;

    } else if (
        doc.containsKey(
            "ssid"
        )
    ) {

        g_wifiSSID =
            doc["ssid"]
            .as<String>();


        changed = true;
    }


    // =========================================================================
    // WIFI PASSWORD
    // =========================================================================

    if (
        doc.containsKey(
            "Wifi_PASS"
        )
    ) {

        g_wifiPass =
            doc["Wifi_PASS"]
            .as<String>();


        changed = true;

    } else if (
        doc.containsKey(
            "wifi_pass"
        )
    ) {

        g_wifiPass =
            doc["wifi_pass"]
            .as<String>();


        changed = true;

    } else if (
        doc.containsKey(
            "password"
        )
    ) {

        g_wifiPass =
            doc["password"]
            .as<String>();


        changed = true;
    }


    // =========================================================================
    // BACKEND
    // =========================================================================

    if (
        doc.containsKey(
            "URL_BACKEND"
        )
    ) {

        g_urlBackend =
            doc["URL_BACKEND"]
            .as<String>();


        changed = true;

    } else if (
        doc.containsKey(
            "url_backend"
        )
    ) {

        g_urlBackend =
            doc["url_backend"]
            .as<String>();


        changed = true;
    }


    // =========================================================================
    // ACCESS TOKEN
    //
    // SOLAMENTE RAM
    // =========================================================================

    if (
        doc.containsKey(
            "ACCESS_TOKEN"
        )
    ) {

        g_accessToken =
            doc["ACCESS_TOKEN"]
            .as<String>();


        changed = true;

    } else if (
        doc.containsKey(
            "access_token"
        )
    ) {

        g_accessToken =
            doc["access_token"]
            .as<String>();


        changed = true;

    } else if (
        doc.containsKey(
            "token"
        )
    ) {

        g_accessToken =
            doc["token"]
            .as<String>();


        changed = true;
    }


    // =========================================================================
    // Ningún campo reconocido
    // =========================================================================

    if (
        !changed
    ) {

        Serial.println(
            "[BLE] JSON sin campos reconocidos"
        );


        return;
    }


    // =========================================================================
    // Mostrar datos NO sensibles
    // =========================================================================

    Serial.printf(
        "[BLE] SSID = \"%s\"\n",
        g_wifiSSID.c_str()
    );


    Serial.printf(
        "[BLE] BACKEND = \"%s\"\n",
        g_urlBackend.c_str()
    );


    if (
        g_accessToken.length() > 0
    ) {

        Serial.printf(
            "[BLE] ACCESS_TOKEN recibido (%u caracteres)\n",
            (unsigned)
            g_accessToken.length()
        );

    } else {

        Serial.println(
            "[BLE] ACCESS_TOKEN vacío"
        );
    }


    // =========================================================================
    // Extraer HOST y PORT
    // =========================================================================

    String url =
        g_urlBackend;


    if (
        url.startsWith(
            "http://"
        )
    ) {

        url =
            url.substring(7);
    }


    if (
        url.startsWith(
            "https://"
        )
    ) {

        url =
            url.substring(8);
    }


    // =========================================================================
    // Quitar path
    // =========================================================================

    int slashIdx =
        url.indexOf('/');


    if (
        slashIdx >= 0
    ) {

        url =
            url.substring(
                0,
                slashIdx
            );
    }


    // =========================================================================
    // Puerto
    // =========================================================================

    int colonIdx =
        url.lastIndexOf(':');


    if (
        colonIdx > 0
    ) {

        g_wsHost =
            url.substring(
                0,
                colonIdx
            );


        String portStr =
            url.substring(
                colonIdx + 1
            );


        g_wsPort =
            (uint16_t)
            portStr.toInt();


        if (
            g_wsPort == 0
        ) {

            g_wsPort =
                8000;
        }

    } else {

        g_wsHost =
            url;


        g_wsPort =
            8000;
    }


    Serial.printf(
        "[BLE] WS_HOST = \"%s\"\n",
        g_wsHost.c_str()
    );


    Serial.printf(
        "[BLE] WS_PORT = %u\n",
        g_wsPort
    );


    // =========================================================================
    // Guardar SOLO configuración persistente
    //
    // NO ACCESS_TOKEN
    // =========================================================================

    prefs.begin(
        "auravision",
        false
    );


    prefs.putString(
        "wifi_ssid",
        g_wifiSSID
    );


    prefs.putString(
        "wifi_pass",
        g_wifiPass
    );


    prefs.putString(
        "url_backend",
        g_urlBackend
    );


    prefs.putString(
        "ws_host",
        g_wsHost
    );


    prefs.putUInt(
        "ws_port",
        g_wsPort
    );


    prefs.end();


    Serial.println(
        "[BLE] Configuración persistente guardada"
    );


    // =========================================================================
    // Validar configuración completa
    // =========================================================================

    bool configCompleta =

        g_bleConnected

        &&

        g_wifiSSID.length() > 0

        &&

        g_urlBackend.length() > 0

        &&

        g_wsHost.length() > 0

        &&

        g_accessToken.length() > 0;


    if (
        !configCompleta
    ) {

        Serial.println();

        Serial.println(
            "[BLE] CONFIGURACIÓN INCOMPLETA"
        );


        if (
            !g_bleConnected
        ) {

            Serial.println(
                "[BLE] - BLE desconectado"
            );
        }


        if (
            g_wifiSSID.length() == 0
        ) {

            Serial.println(
                "[BLE] - Falta Wifi_SSID"
            );
        }


        if (
            g_urlBackend.length() == 0
        ) {

            Serial.println(
                "[BLE] - Falta URL_BACKEND"
            );
        }


        if (
            g_accessToken.length() == 0
        ) {

            Serial.println(
                "[BLE] - Falta ACCESS_TOKEN"
            );
        }


        bleSend(
            "{\"status\":\"error\",\"msg\":\"Configuracion incompleta\"}"
        );


        g_state =
            State::BLE_WAITING;


        return;
    }


    // =========================================================================
    // Todo correcto
    // =========================================================================

    bleSend(
        "{\"status\":\"ok\",\"msg\":\"Configuracion recibida\"}"
    );


    Serial.println();

    Serial.println(
        "[BLE] CONFIGURACIÓN COMPLETA"
    );


    Serial.println(
        "[BLE] BLE conectado"
    );


    Serial.println(
        "[BLE] Iniciando conexión WiFi..."
    );


    g_state =
        State::WIFI_CONNECTING;
}


// =============================================================================
// BLE RX CALLBACK
// =============================================================================

class BleRxCallbacks :
    public BLECharacteristicCallbacks {

    void onWrite(
        BLECharacteristic* pChar
    ) override {

        // =====================================================================
        // Sin conexión BLE no procesar
        // =====================================================================

        if (
            !g_bleConnected
        ) {

            return;
        }


        // =====================================================================
        // Fragmento
        // =====================================================================

        String val =
            pChar
            ->getValue()
            .c_str();


        Serial.printf(
            "[BLE RX] Fragmento: %u bytes\n",
            (unsigned)
            val.length()
        );


        // =====================================================================
        // Acumular
        // =====================================================================

        g_bleBuffer +=
            val;


        // =====================================================================
        // Protección buffer
        // =========================================================================

        if (
            g_bleBuffer.length() >
            8192
        ) {

            Serial.println(
                "[BLE RX] Buffer demasiado grande"
            );


            g_bleBuffer = "";


            return;
        }


        // =====================================================================
        // Detectar final JSON
        // =====================================================================

        int endIdx =
            g_bleBuffer
            .lastIndexOf('}');


        if (
            endIdx >= 0
        ) {

            String jsonStr =
                g_bleBuffer.substring(
                    0,
                    endIdx + 1
                );


            g_bleBuffer =
                g_bleBuffer.substring(
                    endIdx + 1
                );


            Serial.printf(
                "[BLE RX] JSON completo: %u bytes\n",
                (unsigned)
                jsonStr.length()
            );


            parseAndSaveConfig(
                jsonStr
            );

        } else {

            Serial.printf(
                "[BLE RX] Esperando más fragmentos... (%u bytes)\n",
                (unsigned)
                g_bleBuffer.length()
            );
        }
    }
};


// =============================================================================
// Inicializar BLE
// =============================================================================

void initBLE() {

    Serial.println();

    Serial.println(
        "[BLE] Inicializando..."
    );


    BLEDevice::init(
        DEVICE_NAME
    );


    // =========================================================================
    // Server
    // =========================================================================

    g_bleServer =
        BLEDevice::createServer();


    g_bleServer->setCallbacks(
        new BleServerCallbacks()
    );


    // =========================================================================
    // Service
    // =========================================================================

    BLEService* pService =
        g_bleServer
        ->createService(
            BLE_SERVICE_UUID
        );


    // =========================================================================
    // TX
    // =========================================================================

    g_bleTxChar =
        pService
        ->createCharacteristic(

            BLE_CHAR_TX_UUID,

            BLECharacteristic::
                PROPERTY_NOTIFY
        );


    g_bleTxChar
        ->addDescriptor(
            new BLE2902()
        );


    // =========================================================================
    // RX
    // =========================================================================

    BLECharacteristic* pRxChar =
        pService
        ->createCharacteristic(

            BLE_CHAR_RX_UUID,

            BLECharacteristic::
                PROPERTY_WRITE

            |

            BLECharacteristic::
                PROPERTY_WRITE_NR
        );


    pRxChar
        ->setCallbacks(
            new BleRxCallbacks()
        );


    // =========================================================================
    // Start service
    // =========================================================================

    pService->start();


    // =========================================================================
    // Advertising
    // =========================================================================

    BLEAdvertising* pAdv =
        BLEDevice
        ::getAdvertising();


    pAdv->addServiceUUID(
        BLE_SERVICE_UUID
    );


    pAdv->setScanResponse(
        true
    );


    BLEDevice
        ::startAdvertising();


    Serial.println(
        "[BLE] Advertising activo"
    );


    Serial.println(
        "[BLE] Esperando teléfono..."
    );
}


// =============================================================================
// Cargar configuración NVS
// =============================================================================

void loadConfig() {

    prefs.begin(
        "auravision",
        true
    );


    g_wifiSSID =
        prefs.getString(
            "wifi_ssid",
            ""
        );


    g_wifiPass =
        prefs.getString(
            "wifi_pass",
            ""
        );


    g_urlBackend =
        prefs.getString(
            "url_backend",
            ""
        );


    g_wsHost =
        prefs.getString(
            "ws_host",
            ""
        );


    g_wsPort =
        prefs.getUInt(
            "ws_port",
            8000
        );


    g_dispositivoId =
        prefs.getString(
            "dispositivo_id",
            ""
        );


    prefs.end();


    // =========================================================================
    // ACCESS TOKEN JAMÁS se carga desde NVS
    // =========================================================================

    g_accessToken = "";


    Serial.println();

    Serial.println(
        "[NVS] Configuración cargada"
    );


    Serial.printf(
        "[NVS] SSID: %s\n",
        g_wifiSSID.c_str()
    );


    Serial.printf(
        "[NVS] HOST: %s\n",
        g_wsHost.c_str()
    );


    Serial.printf(
        "[NVS] PORT: %u\n",
        g_wsPort
    );


    Serial.printf(
        "[NVS] DEVICE ID: %s\n",
        g_dispositivoId.c_str()
    );


    Serial.println(
        "[NVS] ACCESS_TOKEN: NO PERSISTENTE"
    );
}


// =============================================================================
// Eliminar token antiguo de NVS
//
// Esto limpia tokens guardados por versiones anteriores.
// =============================================================================

void removeLegacyToken() {

    prefs.begin(
        "auravision",
        false
    );


    if (
        prefs.isKey(
            "access_token"
        )
    ) {

        prefs.remove(
            "access_token"
        );


        Serial.println(
            "[SECURITY] Token antiguo eliminado de NVS"
        );
    }


    prefs.end();


    g_accessToken =
        "";
}


// =============================================================================
// Guardar ID dispositivo
// =============================================================================

void saveDispositivoId(
    const String& id
) {

    g_dispositivoId =
        id;


    prefs.begin(
        "auravision",
        false
    );


    prefs.putString(
        "dispositivo_id",
        id
    );


    prefs.end();


    Serial.printf(
        "[NVS] id_dispositivo guardado: %s\n",
        id.c_str()
    );
}


// =============================================================================
// WiFi status string
// =============================================================================

String getWifiStatusString(
    wl_status_t status
) {

    switch (
        status
    ) {

        case WL_IDLE_STATUS:

            return
                "IDLE_STATUS";


        case WL_NO_SSID_AVAIL:

            return
                "NO_SSID_AVAIL";


        case WL_SCAN_COMPLETED:

            return
                "SCAN_COMPLETED";


        case WL_CONNECTED:

            return
                "CONNECTED";


        case WL_CONNECT_FAILED:

            return
                "CONNECT_FAILED";


        case WL_CONNECTION_LOST:

            return
                "CONNECTION_LOST";


        case WL_DISCONNECTED:

            return
                "DISCONNECTED";


        default:

            return
                String("UNKNOWN(")
                +
                String(status)
                +
                ")";
    }
}


// =============================================================================
// Conectar WiFi
// =============================================================================

bool connectWiFi() {

    // =========================================================================
    // BLE obligatorio
    // =========================================================================

    if (
        !g_bleConnected
    ) {

        Serial.println(
            "[WiFi] CANCELADO: BLE desconectado"
        );


        return false;
    }


    // =========================================================================
    // Token obligatorio
    // =========================================================================

    if (
        g_accessToken.length() == 0
    ) {

        Serial.println(
            "[WiFi] CANCELADO: ACCESS_TOKEN vacío"
        );


        return false;
    }


    // =========================================================================
    // SSID obligatorio
    // =========================================================================

    if (
        g_wifiSSID.length() == 0
    ) {

        Serial.println(
            "[WiFi] CANCELADO: SSID vacío"
        );


        return false;
    }


    Serial.println();

    Serial.println(
        "[WiFi] ========================================"
    );


    Serial.printf(
        "[WiFi] SSID: \"%s\"\n",
        g_wifiSSID.c_str()
    );


    // =========================================================================
    // Limpiar estado anterior
    // =========================================================================

    WiFi.disconnect(
        true,
        true
    );


    delay(300);


    // =========================================================================
    // BLE podría haberse desconectado
    // =========================================================================

    if (
        !g_bleConnected
    ) {

        Serial.println(
            "[WiFi] BLE perdido antes de conectar"
        );


        return false;
    }


    // =========================================================================
    // STA
    // =========================================================================

    WiFi.mode(
        WIFI_STA
    );


    WiFi.begin(
        g_wifiSSID.c_str(),
        g_wifiPass.c_str()
    );


    uint32_t t0 =
        millis();


    wl_status_t lastStatus =
        WiFi.status();


    Serial.print(
        "[WiFi] Conectando "
    );


    // =========================================================================
    // Espera hasta 15 segundos
    // =========================================================================

    while (

        WiFi.status() !=
            WL_CONNECTED

        &&

        millis() - t0 <
            15000

    ) {

        // =====================================================================
        // Si BLE desaparece, cancelar inmediatamente
        // =====================================================================

        if (
            !g_bleConnected
        ) {

            Serial.println();

            Serial.println(
                "[WiFi] BLE perdido durante conexión"
            );


            WiFi.disconnect(
                true,
                false
            );


            WiFi.mode(
                WIFI_OFF
            );


            return false;
        }


        delay(
            250
        );


        Serial.print(".");


        wl_status_t currentStatus =
            WiFi.status();


        if (
            currentStatus !=
            lastStatus
        ) {

            Serial.printf(
                "\n[WiFi] Estado: %s\n",
                getWifiStatusString(
                    currentStatus
                ).c_str()
            );


            lastStatus =
                currentStatus;
        }
    }


    Serial.println();


    // =========================================================================
    // BLE nuevamente
    // =========================================================================

    if (
        !g_bleConnected
    ) {

        WiFi.disconnect(
            true,
            false
        );


        WiFi.mode(
            WIFI_OFF
        );


        return false;
    }


    // =========================================================================
    // Conectado
    // =========================================================================

    if (
        WiFi.status() ==
        WL_CONNECTED
    ) {

        Serial.println(
            "[WiFi] CONEXIÓN EXITOSA"
        );


        Serial.printf(
            "[WiFi] IP: %s\n",
            WiFi.localIP()
                .toString()
                .c_str()
        );


        Serial.printf(
            "[WiFi] RSSI: %d dBm\n",
            WiFi.RSSI()
        );


        Serial.println(
            "[WiFi] ========================================"
        );


        return true;
    }


    Serial.println(
        "[WiFi] No fue posible conectar"
    );


    return false;
}


// =============================================================================
// HTTP POST
//
// Utilizado para operaciones normales.
//
// Aquí SÍ exigimos BLE porque estas operaciones solamente deben ocurrir
// mientras el teléfono está conectado.
// =============================================================================

int httpPost(
    const String& path,
    const String& jsonBody,
    String& responseBody
) {

    // =========================================================================
    // BLE
    // =========================================================================

    if (
        !g_bleConnected
    ) {

        Serial.println(
            "[HTTP] CANCELADO: BLE desconectado"
        );


        return -1;
    }


    // =========================================================================
    // Token
    // =========================================================================

    if (
        g_accessToken.length() == 0
    ) {

        Serial.println(
            "[HTTP] CANCELADO: ACCESS_TOKEN vacío"
        );


        return -1;
    }


    // =========================================================================
    // WiFi
    // =========================================================================

    if (
        WiFi.status() !=
        WL_CONNECTED
    ) {

        Serial.println(
            "[HTTP] CANCELADO: WiFi desconectado"
        );


        return -1;
    }


    // =========================================================================
    // Crear URL
    // =========================================================================

    String url =
        g_urlBackend;


    if (
        url.endsWith("/")
    ) {

        url.remove(
            url.length() - 1
        );
    }


    if (
        !path.startsWith("/")
    ) {

        url += "/";
    }


    url +=
        path;


    Serial.println(
        "[HTTP] ----------------------------------------"
    );


    Serial.printf(
        "[HTTP] POST %s\n",
        url.c_str()
    );


    // =========================================================================
    // HTTP
    // =========================================================================

    HTTPClient http;


    if (
        !http.begin(
            url
        )
    ) {

        Serial.println(
            "[HTTP] Error iniciando conexión"
        );


        return -1;
    }


    http.addHeader(
        "Content-Type",
        "application/json"
    );


    http.addHeader(
        "Authorization",
        String("Bearer ")
        +
        g_accessToken
    );


    // =========================================================================
    // POST
    // =========================================================================

    int code =
        http.POST(
            jsonBody
        );


    if (
        code > 0
    ) {

        responseBody =
            http.getString();

    } else {

        responseBody =
            "";
    }


    Serial.printf(
        "[HTTP] Status: %d\n",
        code
    );


    if (
        responseBody.length() > 0
    ) {

        Serial.printf(
            "[HTTP] Respuesta: %s\n",
            responseBody.c_str()
        );
    }


    http.end();


    Serial.println(
        "[HTTP] ----------------------------------------"
    );


    return code;
}


// =============================================================================
// Registrar dispositivo
// =============================================================================

bool registerDevice() {

    if (
        !g_bleConnected
    ) {

        return false;
    }


    String mac =
        WiFi.macAddress();


    Serial.printf(
        "[REG] Registrando dispositivo. MAC=%s\n",
        mac.c_str()
    );


    // =========================================================================
    // JSON
    // =========================================================================

    StaticJsonDocument<256> doc;


    doc["nombre"] =
        DEVICE_NAME;


    doc["numero_serie"] =
        mac;


    doc["version_firmware"] =
        FIRMWARE_VERSION;


    doc["estado_conexion"] =
        "desconectado";


    String body;


    serializeJson(
        doc,
        body
    );


    String resp;


    // =========================================================================
    // POST
    // =========================================================================

    int code =
        httpPost(

            "/api/v1/dispositivos",

            body,

            resp
        );


    // =========================================================================
    // BLE pudo caer durante HTTP
    // =========================================================================

    if (
        !g_bleConnected
    ) {

        return false;
    }


    // =========================================================================
    // Creado
    // =========================================================================

    if (
        code == 201
    ) {

        Serial.println(
            "[REG] Dispositivo creado"
        );


        StaticJsonDocument<256>
            respDoc;


        if (

            deserializeJson(
                respDoc,
                resp
            )

            ==

            DeserializationError::Ok

        ) {

            String id =
                respDoc[
                    "id_dispositivo"
                ]
                .as<String>();


            if (
                id.length() > 0
            ) {

                saveDispositivoId(
                    id
                );


                return true;
            }
        }
    }


    // =========================================================================
    // Ya existe
    // =========================================================================

    if (
        code == 200 ||
        code == 409
    ) {

        Serial.println(
            "[REG] Dispositivo ya registrado"
        );


        StaticJsonDocument<256>
            respDoc;


        if (

            deserializeJson(
                respDoc,
                resp
            )

            ==

            DeserializationError::Ok

            &&

            respDoc.containsKey(
                "id_dispositivo"
            )

        ) {

            String id =
                respDoc[
                    "id_dispositivo"
                ]
                .as<String>();


            if (
                id.length() > 0
            ) {

                saveDispositivoId(
                    id
                );


                return true;
            }
        }
    }


    Serial.printf(
        "[REG] Error HTTP %d\n",
        code
    );


    return false;
}


// =============================================================================
// Crear sesión
// =============================================================================

bool createSession() {

    // =========================================================================
    // BLE
    // =========================================================================

    if (
        !g_bleConnected
    ) {

        return false;
    }


    // =========================================================================
    // Device ID
    // =========================================================================

    if (
        g_dispositivoId.length() == 0
    ) {

        Serial.println(
            "[SES] id_dispositivo vacío"
        );


        return false;
    }


    Serial.printf(
        "[SES] Creando sesión para dispositivo %s\n",
        g_dispositivoId.c_str()
    );


    // =========================================================================
    // Body
    // =========================================================================

    StaticJsonDocument<128> doc;


    doc["id_dispositivo_fk"] =
        g_dispositivoId;


    String body;


    serializeJson(
        doc,
        body
    );


    String resp;


    // =========================================================================
    // POST
    // =========================================================================

    int code =
        httpPost(

            "/api/v1/sesiones-dispositivo",

            body,

            resp
        );


    // =========================================================================
    // BLE pudo caer
    // =========================================================================

    if (
        !g_bleConnected
    ) {

        return false;
    }


    // =========================================================================
    // OK
    // =========================================================================

    if (
        code == 201 ||
        code == 200
    ) {

        Serial.println(
            "[SES] Sesión creada correctamente"
        );


        return true;
    }


    Serial.printf(
        "[SES] Error creando sesión. HTTP %d\n",
        code
    );


    return false;
}


// =============================================================================
// Cámara
// =============================================================================

bool initCamera() {

    // =========================================================================
    // Evitar inicialización duplicada
    // =========================================================================

    if (
        g_cameraInitialized
    ) {

        return true;
    }


    camera_config_t cfg =
        {};


    cfg.ledc_channel =
        LEDC_CHANNEL_0;


    cfg.ledc_timer =
        LEDC_TIMER_0;


    cfg.pin_d0 =
        Y2_GPIO_NUM;


    cfg.pin_d1 =
        Y3_GPIO_NUM;


    cfg.pin_d2 =
        Y4_GPIO_NUM;


    cfg.pin_d3 =
        Y5_GPIO_NUM;


    cfg.pin_d4 =
        Y6_GPIO_NUM;


    cfg.pin_d5 =
        Y7_GPIO_NUM;


    cfg.pin_d6 =
        Y8_GPIO_NUM;


    cfg.pin_d7 =
        Y9_GPIO_NUM;


    cfg.pin_xclk =
        XCLK_GPIO_NUM;


    cfg.pin_pclk =
        PCLK_GPIO_NUM;


    cfg.pin_vsync =
        VSYNC_GPIO_NUM;


    cfg.pin_href =
        HREF_GPIO_NUM;


    cfg.pin_sscb_sda =
        SIOD_GPIO_NUM;


    cfg.pin_sscb_scl =
        SIOC_GPIO_NUM;


    cfg.pin_pwdn =
        PWDN_GPIO_NUM;


    cfg.pin_reset =
        RESET_GPIO_NUM;


    cfg.xclk_freq_hz =
        20000000;


    cfg.pixel_format =
        PIXFORMAT_JPEG;


    cfg.frame_size =
        FRAMESIZE_VGA;


    cfg.jpeg_quality =
        12;


    cfg.fb_count =
        2;


    cfg.grab_mode =
        CAMERA_GRAB_LATEST;


    // =========================================================================
    // Inicializar
    // =========================================================================

    esp_err_t err =
        esp_camera_init(
            &cfg
        );


    if (
        err != ESP_OK
    ) {

        Serial.printf(
            "[CAM] Error init: 0x%x\n",
            err
        );


        return false;
    }


    g_cameraInitialized =
        true;


    Serial.println(
        "[CAM] Inicializada 640x480 JPEG"
    );


    return true;
}


// =============================================================================
// SD
// =============================================================================

bool initSD() {

    sdSPI.begin(

        SD_SCK,

        SD_MISO,

        SD_MOSI,

        SD_CS
    );


    if (

        !SD.begin(
            SD_CS,
            sdSPI
        )

    ) {

        Serial.println(
            "[SD] Error al montar"
        );


        return false;
    }


    Serial.println(
        "[SD] Montada correctamente"
    );


    return true;
}


// =============================================================================
// Audio
// =============================================================================

bool initAudio() {

    audio.setPinout(

        I2S_BCLK,

        I2S_LRC,

        I2S_DOUT
    );


    audio.setVolume(
        21
    );


    audio.setTone(
        -8,
        0,
        -4
    );


    return true;
}


// =============================================================================
// Audio EOF
// =============================================================================

void audio_eof_mp3(
    const char* info
) {

    Serial.printf(
        "[AUDIO] Fin: %s\n",
        info
    );


    g_audioPlaying =
        false;


    // =========================================================================
    // La secuencia termina solamente cuando ya no quedan audios pendientes.
    //
    // Ejemplo:
    //
    //     Person.mp3
    //     frente.mp3
    //
    // Al terminar Person.mp3 todavía queda frente.mp3 en la cola, por lo que
    // NO liberamos la secuencia todavía.
    // =========================================================================

    if (
        g_audioQueueSize <= 0
    ) {

        g_audioSequenceActive =
            false;


        g_lastAudioAlertFinished =
            millis();


        Serial.println(
            "[AUDIO] Secuencia completa terminada"
        );
    }
}


// =============================================================================
// Procesar cola audio
// =============================================================================

void processAudioQueue() {

    if (
        g_audioPlaying
    ) {

        audio.loop();


        return;
    }


    if (
        g_audioQueueSize <= 0
    ) {

        return;
    }


    // =========================================================================
    // Primer archivo
    // =========================================================================

    String file =
        g_audioQueue[0];


    // =========================================================================
    // Desplazar cola
    // =========================================================================

    g_audioQueue[0] =
        g_audioQueue[1];


    g_audioQueueSize--;


    // =========================================================================
    // Reproducir
    // =========================================================================

    if (
        SD.exists(
            file.c_str()
        )
    ) {

        Serial.printf(
            "[AUDIO] Reproduciendo: %s\n",
            file.c_str()
        );


        g_audioPlaying =
            audio.connecttoFS(

                SD,

                file.c_str()
            );


        // =====================================================================
        // Si la librería no pudo iniciar el archivo, no dejamos la secuencia
        // bloqueada para siempre.
        // =====================================================================

        if (
            !g_audioPlaying
        ) {

            Serial.printf(
                "[AUDIO] No fue posible iniciar: %s\n",
                file.c_str()
            );


            if (
                g_audioQueueSize <= 0
            ) {

                g_audioSequenceActive =
                    false;


                g_lastAudioAlertFinished =
                    millis();
            }
        }

    } else {

        Serial.printf(
            "[AUDIO] Archivo no encontrado: %s\n",
            file.c_str()
        );


        // =====================================================================
        // Si tampoco quedan más archivos, terminar la secuencia.
        // =====================================================================

        if (
            g_audioQueueSize <= 0
        ) {

            g_audioSequenceActive =
                false;


            g_lastAudioAlertFinished =
                millis();
        }
    }
}


// =============================================================================
// Encolar alertas
// =============================================================================

void enqueueAlerts(
    const String& label,
    const String& relPos
) {

    // =========================================================================
    // No interrumpir una secuencia que ya se está reproduciendo.
    //
    // Esto evita que una nueva detección recibida 500 ms después reemplace
    // la dirección pendiente de la alerta anterior.
    // =========================================================================

    if (
        g_audioSequenceActive
        ||
        g_audioPlaying
        ||
        g_audioQueueSize > 0
    ) {

        Serial.println(
            "[AUDIO] Nueva deteccion ignorada: alerta en reproduccion"
        );


        return;
    }


    // =========================================================================
    // Cooldown después de terminar obstáculo + dirección.
    // =========================================================================

    if (
        g_lastAudioAlertFinished > 0
        &&
        millis() - g_lastAudioAlertFinished
            < AUDIO_ALERT_COOLDOWN_MS
    ) {

        Serial.println(
            "[AUDIO] Nueva deteccion ignorada: cooldown"
        );


        return;
    }


    // =========================================================================
    // Comenzar nueva secuencia
    // =========================================================================

    g_audioSequenceActive =
        true;


    // =========================================================================
    // Reset cola
    // =========================================================================

    g_audioQueueSize =
        0;


    // =========================================================================
    // Obstáculo
    // =========================================================================

    g_audioQueue[
        g_audioQueueSize++
    ] =

        String(
            "/obstaculos/"
        )

        +

        label

        +

        ".mp3";


    // =========================================================================
    // Dirección
    // =========================================================================

    if (

        !relPos.equalsIgnoreCase(
            "None"
        )

        &&

        relPos.length() > 0

    ) {

        g_audioQueue[
            g_audioQueueSize++
        ] =

            String(
                "/direcciones/"
            )

            +

            relPos

            +

            ".mp3";
    }


    Serial.printf(
        "[AUDIO] Nueva alerta: %s | %s\n",
        label.c_str(),
        relPos.c_str()
    );
}


// =============================================================================
// Procesar detecciones
// =============================================================================

void handleDetections(
    const String& json
) {

    // =========================================================================
    // Si BLE desapareció no procesamos nada
    // =========================================================================

    if (
        !g_bleConnected
    ) {

        return;
    }


    StaticJsonDocument<2048> doc;


    DeserializationError err =
        deserializeJson(
            doc,
            json
        );


    if (
        err
    ) {

        Serial.printf(
            "[WS] JSON inválido: %s\n",
            err.c_str()
        );


        return;
    }


    // =========================================================================
    // Error backend
    // =========================================================================

    if (

        doc.containsKey(
            "ok"
        )

        &&

        doc["ok"] ==
            false

    ) {

        String errorMsg =
            doc["error"]
            |
            "Error desconocido";


        Serial.printf(
            "[WS] Backend: %s\n",
            errorMsg.c_str()
        );


        return;
    }


    // =========================================================================
    // Detecciones
    // =========================================================================

    JsonArray arr =
        doc.as<JsonArray>();


    if (
        !arr
    ) {

        // =====================================================================
        // Formato:
        //
        // {
        //   "ok": true,
        //   "resultado": {
        //      "detections": [...]
        //   }
        // }
        // =====================================================================

        if (

            doc.containsKey(
                "resultado"
            )

            &&

            doc[
                "resultado"
            ]
            .containsKey(
                "detections"
            )

        ) {

            arr =
                doc[
                    "resultado"
                ]
                [
                    "detections"
                ]
                .as<JsonArray>();

        } else {

            arr =
                doc[
                    "detections"
                ]
                .as<JsonArray>();
        }
    }


    // =========================================================================
    // Sin detecciones
    // =========================================================================

    if (

        !arr

        ||

        arr.size() == 0

    ) {

        return;
    }


    // =========================================================================
    // Priorizar
    // =========================================================================

    int bestPriority =
        -1;


    String bestLabel;

    String bestPos;


    for (
        JsonObject det :
        arr
    ) {

        String label =
            det["label"]
            |
            "";


        String pos =
            det["relative_position"]
            |
            "None";


        // =====================================================================
        // Sin posición
        // =====================================================================

        if (
            pos.equalsIgnoreCase(
                "None"
            )
        ) {

            continue;
        }


        // =====================================================================
        // Prioridad
        // =====================================================================

        int prio =
            getPriority(
                label,
                pos
            );


        if (
            prio >
            bestPriority
        ) {

            bestPriority =
                prio;


            bestLabel =
                label;


            bestPos =
                pos;
        }
    }


    // =========================================================================
    // Audio
    // =========================================================================

    if (
        bestLabel.length() > 0
    ) {

        Serial.printf(
            "[DETECT] %s | %s | prioridad=%d\n",

            bestLabel.c_str(),

            bestPos.c_str(),

            bestPriority
        );


        enqueueAlerts(

            bestLabel,

            bestPos
        );
    }
}


// =============================================================================
// WebSocket Event
// =============================================================================

void webSocketEvent(

    WStype_t type,

    uint8_t* payload,

    size_t length

) {

    switch (
        type
    ) {


        // =====================================================================
        // CONNECTED
        // =====================================================================

        case WStype_CONNECTED: {

            // =================================================================
            // Nunca aceptar WebSocket si ya no existe BLE/token
            // =================================================================

            if (

                !g_bleConnected

                ||

                g_accessToken.length() == 0

            ) {

                Serial.println(
                    "[WS] Conexión rechazada: BLE/token inexistente"
                );


                g_ws.disconnect();


                g_wsConnected =
                    false;


                g_state =
                    State::BLE_WAITING;


                break;
            }


            g_wsConnected =
                true;


            Serial.println(
                "[WS] CONECTADO AL SERVIDOR"
            );


            g_state =
                State::STREAMING;


            break;
        }


        // =====================================================================
        // DISCONNECTED
        // =====================================================================

        case WStype_DISCONNECTED: {

            g_wsConnected =
                false;


            Serial.println(
                "[WS] Desconectado"
            );


            // =================================================================
            // Reconectar SOLO si BLE sigue activo
            // =================================================================

            if (

                g_bleConnected

                &&

                g_accessToken.length() > 0

            ) {

                if (
                    g_state ==
                    State::STREAMING
                ) {

                    Serial.println(
                        "[WS] Intentando reconectar..."
                    );


                    g_state =
                        State::WS_CONNECTING;
                }

            } else {

                Serial.println(
                    "[WS] No se reconectará porque BLE no está activo"
                );


                g_state =
                    State::BLE_WAITING;
            }


            break;
        }


        // =====================================================================
        // TEXT
        // =====================================================================

        case WStype_TEXT: {

            if (
                !g_bleConnected
            ) {

                break;
            }


            String jsonContent =
                String(
                    (char*)payload
                );


            Serial.printf(
                "[WS] Datos recibidos: %u bytes\n",
                (unsigned)
                length
            );


            handleDetections(
                jsonContent
            );


            break;
        }


        // =====================================================================
        // BIN
        // =====================================================================

        case WStype_BIN: {

            break;
        }


        // =====================================================================
        // ERROR
        // =====================================================================

        case WStype_ERROR: {

            Serial.println(
                "[WS] Error"
            );


            break;
        }


        default: {

            break;
        }
    }
}


// =============================================================================
// Inicializar WebSocket
// =============================================================================

void initWebSocket() {

    // =========================================================================
    // BLE
    // =========================================================================

    if (
        !g_bleConnected
    ) {

        Serial.println(
            "[WS] CANCELADO: BLE desconectado"
        );


        g_state =
            State::BLE_WAITING;


        return;
    }


    // =========================================================================
    // Token
    // =========================================================================

    if (
        g_accessToken.length() == 0
    ) {

        Serial.println(
            "[WS] CANCELADO: ACCESS_TOKEN vacío"
        );


        g_state =
            State::BLE_WAITING;


        return;
    }


    // =========================================================================
    // WiFi
    // =========================================================================

    if (
        WiFi.status() !=
        WL_CONNECTED
    ) {

        Serial.println(
            "[WS] CANCELADO: WiFi desconectado"
        );


        return;
    }


    // =========================================================================
    // Device ID
    // =========================================================================

    if (
        g_dispositivoId.length() == 0
    ) {

        Serial.println(
            "[WS] CANCELADO: dispositivo ID vacío"
        );


        return;
    }


    // =========================================================================
    // Path
    // =========================================================================

    String wsPath =

        String(
            "/api/v1/ws/dispositivos/"
        )

        +

        g_dispositivoId

        +

        "/vision?token="

        +

        g_accessToken;


    // =========================================================================
    // NO imprimir token
    // =========================================================================

    Serial.printf(
        "[WS] Conectando a ws://%s:%u/api/v1/ws/dispositivos/%s/vision\n",

        g_wsHost.c_str(),

        g_wsPort,

        g_dispositivoId.c_str()
    );


    // =========================================================================
    // Begin
    // =========================================================================

    g_ws.begin(

        g_wsHost.c_str(),

        g_wsPort,

        wsPath.c_str()
    );


    g_ws.onEvent(
        webSocketEvent
    );


    g_ws.setReconnectInterval(
        5000
    );
}


// =============================================================================
// SETUP
// =============================================================================

void setup() {

    Serial.begin(
        115200
    );


    // =========================================================================
    // Esperar Serial máximo 3 segundos
    // =========================================================================

    uint32_t t =
        millis();


    while (

        !Serial

        &&

        millis() - t <
        3000

    ) {

        delay(
            10
        );
    }


    Serial.println();

    Serial.println();

    Serial.println(
        "========================================"
    );

    Serial.println(
        "=== AuraVision Firmware 1.0.0 ==="
    );

    Serial.println(
        "========================================"
    );


    // =========================================================================
    // 1. Cargar configuración persistente
    // =========================================================================

    loadConfig();


    // =========================================================================
    // 2. Eliminar token de firmware antiguo
    // =========================================================================

    removeLegacyToken();


    // =========================================================================
    // 3. FORZAR WiFi apagado al iniciar
    //
    // Aunque haya SSID guardado NO conectamos.
    // =========================================================================

    WiFi.disconnect(
        true,
        false
    );


    WiFi.mode(
        WIFI_OFF
    );


    // =========================================================================
    // 4. BLE
    // =========================================================================

    initBLE();


    // =========================================================================
    // 5. SD + Audio
    // =========================================================================

    if (
        initSD()
    ) {

        g_audioReady =
            true;


        initAudio();
    }


    // =========================================================================
    // 6. SIEMPRE esperar Bluetooth
    // =========================================================================

    g_state =
        State::BLE_WAITING;


    g_accessToken =
        "";


    Serial.println();

    Serial.println(
        "[BOOT] ========================================"
    );


    Serial.println(
        "[BOOT] Sistema iniciado"
    );


    Serial.println(
        "[BOOT] BLE: esperando teléfono"
    );


    Serial.println(
        "[BOOT] WiFi: OFF"
    );


    Serial.println(
        "[BOOT] Backend: OFF"
    );


    Serial.println(
        "[BOOT] WebSocket: OFF"
    );


    Serial.println(
        "[BOOT] ACCESS_TOKEN: VACÍO"
    );


    Serial.println(
        "[BOOT] ========================================"
    );
}


// =============================================================================
// LOOP
// =============================================================================

void loop() {

    // =========================================================================
    // Audio
    // =========================================================================

    if (
        g_audioReady
    ) {

        processAudioQueue();
    }


    // =========================================================================
    // PROTECCIÓN GLOBAL
    //
    // Si BLE desapareció durante cualquier operación:
    //
    //      DELETE sesión
    //      WS OFF
    //      WiFi OFF
    //      TOKEN eliminado
    //
    // =========================================================================

    if (

        !g_bleConnected

        &&

        g_state !=
            State::BLE_WAITING

    ) {

        Serial.println(
            "[SECURITY] BLE perdido -> cerrando sesión"
        );


        stopActiveSession();


        delay(
            100
        );


        return;
    }


    // =========================================================================
    // Estados
    // =========================================================================

    switch (
        g_state
    ) {


        // =====================================================================
        // BLE WAITING
        // =====================================================================

        case State::BLE_WAITING: {

            // =================================================================
            // Aquí NO hacemos:
            //
            // WiFi.begin()
            // HTTP
            // WebSocket
            // Cámara streaming
            //
            // Solo esperamos callbacks BLE.
            // =================================================================

            delay(
                50
            );


            break;
        }


        // =====================================================================
        // WIFI CONNECTING
        // =====================================================================

        case State::WIFI_CONNECTING: {

            // =================================================================
            // BLE obligatorio
            // =================================================================

            if (
                !g_bleConnected
            ) {

                stopActiveSession();


                break;
            }


            // =================================================================
            // Conectar
            // =================================================================

            if (
                connectWiFi()
            ) {

                // =============================================================
                // Verificar BLE otra vez
                // =============================================================

                if (
                    !g_bleConnected
                ) {

                    stopActiveSession();


                    break;
                }


                // =============================================================
                // Informar conexión WiFi exitosa a la aplicación
                // =============================================================

                {
                    StaticJsonDocument<256> doc;


                    doc["type"] =
                        "wifi_status";


                    doc["status"] =
                        "wifi_connected";


                    doc["connected"] =
                        true;


                    doc["msg"] =
                        "Conexion WiFi exitosa";


                    doc["ip"] =
                        WiFi.localIP().toString();


                    String response;


                    serializeJson(
                        doc,
                        response
                    );


                    bleSend(
                        response
                    );
                }

                delay(1000);

                // =============================================================
                // Inicializar cámara
                // =============================================================

                if (
                    !g_cameraInitialized
                ) {

                    bool cameraOK =
                        initCamera();


                    if (
                        !cameraOK
                    ) {

                        Serial.println(
                            "[CAM] No se pudo inicializar cámara"
                        );
                    }
                }


                // =============================================================
                // Si ya tenemos ID no registramos de nuevo
                // =============================================================

                if (
                    g_dispositivoId.length() > 0
                ) {

                    g_state =
                        State::CREATING_SESSION;

                } else {

                    g_state =
                        State::REGISTERING;
                }

            } else {

                // =============================================================
                // BLE desapareció durante el intento de conexión
                // =============================================================

                if (
                    !g_bleConnected
                ) {

                    stopActiveSession();


                    break;
                }


                // =============================================================
                // Informar fallo WiFi a la aplicación
                // =============================================================

                {
                    StaticJsonDocument<256> doc;


                    doc["type"] =
                        "wifi_status";


                    doc["status"] =
                        "wifi_disconnected";


                    doc["connected"] =
                        false;


                    doc["msg"] =
                        "No fue posible conectar al WiFi";


                    doc["wifi_status"] =
                        getWifiStatusString(
                            WiFi.status()
                        );


                    String response;


                    serializeJson(
                        doc,
                        response
                    );


                    bleSend(
                        response
                    );
                }


                Serial.println(
                    "[WiFi] Estado de error enviado a la aplicación"
                );


                // =============================================================
                // Dar tiempo a que la notificación BLE llegue al teléfono
                // =============================================================

                delay(1000);


                // =============================================================
                // Apagar WiFi antes de cortar Bluetooth
                // =============================================================

                WiFi.disconnect(
                    true,
                    false
                );


                WiFi.mode(
                    WIFI_OFF
                );


                // =============================================================
                // Desconectar automáticamente el teléfono por BLE
                //
                // onDisconnect() se encargará de:
                //
                //  - limpiar la sesión
                //  - volver a BLE_WAITING
                //  - reactivar el advertising
                // =============================================================

                if (
                    g_bleServer != nullptr
                    &&
                    g_bleConnected
                ) {

                    Serial.println(
                        "[BLE] Desconectando teléfono por fallo WiFi..."
                    );


                    g_bleServer->disconnect(
                        g_bleServer->getConnId()
                    );
                }
            }


            break;
        }


        // =====================================================================
        // REGISTERING
        // =====================================================================

        case State::REGISTERING: {

            if (
                !g_bleConnected
            ) {

                stopActiveSession();


                break;
            }


            Serial.println(
                "[BOOT] Registrando dispositivo..."
            );


            if (
                registerDevice()
            ) {

                if (
                    g_bleConnected
                ) {

                    g_state =
                        State::CREATING_SESSION;
                }

            } else {

                // =============================================================
                // BLE perdido
                // =============================================================

                if (
                    !g_bleConnected
                ) {

                    stopActiveSession();


                    break;
                }


                Serial.println(
                    "[REG] Reintentando en 5 segundos..."
                );


                for (
                    int i = 0;
                    i < 50;
                    i++
                ) {

                    if (
                        !g_bleConnected
                    ) {

                        break;
                    }


                    delay(
                        100
                    );
                }
            }


            break;
        }


        // =====================================================================
        // CREATING SESSION
        // =====================================================================

        case State::CREATING_SESSION: {

            if (
                !g_bleConnected
            ) {

                stopActiveSession();


                break;
            }


            Serial.println(
                "[BOOT] Creando sesión..."
            );


            if (
                createSession()
            ) {

                // =============================================================
                // BLE sigue conectado
                // =============================================================

                if (
                    g_bleConnected
                ) {

                    g_state =
                        State::WS_CONNECTING;


                    initWebSocket();
                }

            } else {

                if (
                    !g_bleConnected
                ) {

                    stopActiveSession();


                    break;
                }


                Serial.println(
                    "[SES] Reintentando en 5 segundos..."
                );


                for (
                    int i = 0;
                    i < 50;
                    i++
                ) {

                    if (
                        !g_bleConnected
                    ) {

                        break;
                    }


                    delay(
                        100
                    );
                }
            }


            break;
        }


        // =====================================================================
        // WEBSOCKET CONNECTING
        // =====================================================================

        case State::WS_CONNECTING: {

            // =================================================================
            // BLE
            // =================================================================

            if (
                !g_bleConnected
            ) {

                stopActiveSession();


                break;
            }


            // =================================================================
            // Token
            // =================================================================

            if (
                g_accessToken.length() == 0
            ) {

                Serial.println(
                    "[SECURITY] ACCESS_TOKEN inexistente"
                );


                stopActiveSession();


                break;
            }


            // =================================================================
            // WiFi
            // =================================================================

            if (
                WiFi.status() !=
                WL_CONNECTED
            ) {

                Serial.println(
                    "[WiFi] WiFi perdido antes de conectar WebSocket"
                );


                g_state =
                    State::WIFI_CONNECTING;


                break;
            }


            // =================================================================
            // Procesar WebSocket
            // =================================================================

            g_ws.loop();


            break;
        }


        // =====================================================================
        // STREAMING
        // =====================================================================

        case State::STREAMING: {

            // =================================================================
            // BLE obligatorio
            // =================================================================

            if (
                !g_bleConnected
            ) {

                stopActiveSession();


                break;
            }


            // =================================================================
            // Token obligatorio
            // =================================================================

            if (
                g_accessToken.length() == 0
            ) {

                Serial.println(
                    "[SECURITY] ACCESS_TOKEN perdido"
                );


                stopActiveSession();


                break;
            }


            // =================================================================
            // WiFi obligatorio
            // =================================================================

            if (
                WiFi.status() !=
                WL_CONNECTED
            ) {

                Serial.println(
                    "[WiFi] Conexión perdida"
                );


                g_ws.disconnect();


                g_wsConnected =
                    false;


                // =============================================================
                // BLE sigue conectado:
                // intentar recuperar WiFi
                // =============================================================

                if (
                    g_bleConnected
                ) {

                    g_state =
                        State::WIFI_CONNECTING;

                } else {

                    stopActiveSession();
                }


                break;
            }


            // =================================================================
            // WebSocket
            // =================================================================

            g_ws.loop();


            // =================================================================
            // PRIORIDAD ABSOLUTA AL AUDIO
            //
            // Mientras se reproduce una alerta completa (obstáculo + dirección),
            // NO capturamos ni enviamos nuevas imágenes. Esto permite que
            // audio.loop() se ejecute con mucha frecuencia y evita cortes.
            // =================================================================

            if (

                g_audioPlaying

                ||

                g_audioSequenceActive

                ||

                g_audioQueueSize > 0

            ) {

                processAudioQueue();

                // Reiniciar el temporizador de captura para no disparar una
                // captura inmediatamente al terminar el audio.
                g_lastCapture = millis();

                return;
            }


            // =================================================================
            // Captura
            // =================================================================

            uint32_t now =
                millis();


            if (

                now -
                g_lastCapture

                >=

                CAPTURE_INTERVAL_MS

            ) {

                g_lastCapture =
                    now;


                // =============================================================
                // BLE + WebSocket obligatorios
                // =============================================================

                if (

                    !g_bleConnected

                    ||

                    !g_wsConnected

                ) {

                    break;
                }


                // =============================================================
                // Cámara
                // =============================================================

                camera_fb_t* fb =
                    esp_camera_fb_get();


                if (
                    !fb
                ) {

                    Serial.println(
                        "[CAM] Error capturando frame"
                    );


                    break;
                }


                // =============================================================
                // Verificar nuevamente antes de enviar
                // =============================================================

                if (

                    g_bleConnected

                    &&

                    g_wsConnected

                    &&

                    g_accessToken.length() > 0

                ) {

                    bool sent =
                        g_ws.sendBIN(

                            fb->buf,

                            fb->len
                        );


                    if (
                        !sent
                    ) {

                        Serial.println(
                            "[WS] Error enviando frame"
                        );
                    }
                }


                // =============================================================
                // Liberar frame
                // =============================================================

                esp_camera_fb_return(
                    fb
                );
            }


            break;
        }
    }
}