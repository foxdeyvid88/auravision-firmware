# Requerimientos Funcionales del Proyecto

Este documento describe las funcionalidades requeridas para el dispositivo (ESP32 XIAO SENSE) en el proyecto AuraVision.

## 1. Conexión BLE y Configuración Inicial
Al encender el dispositivo (ESP32 XIAO SENSE), este debe activar su interfaz Bluetooth Low Energy (BLE) y hacerse visible para los dispositivos cercanos.

*   **Espera de conexión:** El dispositivo debe esperar a que la aplicación móvil se conecte vía BLE. **Importante:** No se debe intentar ninguna conexión a internet (WiFi) si el dispositivo no ha establecido conexión BLE previamente.
*   **Recepción de parámetros:** Una vez establecida la conexión con el teléfono, el ESP32 debe recibir los siguientes parámetros de configuración:
    *   `Wifi_SSID`
    *   `Wifi_PASS`
    *   `URL_BACKEND` (incluyendo/reemplazando `WS_HOST` y `WS_PORT`)
    *   `ACCESS_TOKEN`
*   **Almacenamiento persistente:** El ESP32 debe guardar estas variables de manera permanente en su memoria no volátil (NVS, EEPROM, etc.) para que persistan incluso si el dispositivo se apaga o reinicia.
*   **Actualización de datos:** Si la aplicación móvil envía datos nuevos en cualquier momento vía BLE, el ESP32 debe actualizar y sobreescribir las variables ya guardadas.

## 2. Conexión a Internet
Con las credenciales recibidas y almacenadas vía BLE, el dispositivo debe intentar conectarse a la red WiFi utilizando:
- `Wifi_SSID`
- `Wifi_PASS`

## 3. Registro del Dispositivo
Una vez establecida la conexión a internet, el dispositivo debe registrarse en el servidor utilizando los parámetros guardados de URL y el `ACCESS_TOKEN`.

*   **Ruta:** `/api/v1/dispositivos`
*   **Método:** `POST`
*   **Cabeceras:** Debe incluir el token de acceso (`ACCESS_TOKEN`).
*   **Cuerpo (JSON):**
    ```json
    {
      "nombre": "<nombre_dispositivo>",
      "numero de serie": "<MAC_DEL_ESP32>",
      "version_firmware": "<version>",
      "estado_conexion": "<estado>"
    }
    ```
    *Nota: El "numero de serie" se obtiene de la dirección MAC del ESP32.*

## 4. Almacenamiento del ID de Dispositivo
Si la respuesta del servidor al registro es un código `201 Created`, el servidor devolverá una respuesta similar a esta:
```json
{
  "nombre": "...",
  "numero de serie": "...",
  "version_firmware": "...",
  "estado_conexion": "...",
  "id_dispositivo": "<ID_ASIGNADO>"
}
```
El dispositivo debe extraer el `id_dispositivo` y guardarlo de manera permanente en su memoria no volátil.

## 5. Creación de Sesión
Teniendo el `id_dispositivo` (ya sea recién obtenido o leído de la memoria permanente), el dispositivo debe crear una sesión en el servidor.

*   **Ruta:** `/api/v1/sesiones_dispositivos`
*   **Método:** `POST`
*   **Cabeceras:** Debe incluir el token de acceso (`ACCESS_TOKEN`).
*   **Cuerpo (JSON):**
    ```json
    {
      "id_dispositivo": "<ID_GUARDADO>"
    }
    ```

## 6. Conexión por WebSocket
Si la respuesta a la creación de sesión es `201 Created`, el dispositivo debe establecer una conexión WebSocket con el backend para la transmisión de datos.

**Ejemplo de implementación (C++):**
```cpp
  wsPath = String("/api/v1/ws/dispositivos/") + DISPOSITIVO_ID + "/vision?token=" + ACCESS_TOKEN;
  Serial.printf("[WS] Conectando a ws://%s:%u%s\n", WS_HOST, WS_PORT, wsPath.c_str());
  webSocket.begin(WS_HOST, WS_PORT, wsPath);
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);
```

## 7. Transmisión de Imágenes
Con el dispositivo conectado exitosamente vía WebSocket, se debe iniciar la captura y envío de imágenes:
*   **Resolución:** 640x480 píxeles.
*   **Frecuencia:** Una imagen cada 0.5 segundos.

## 8. Recepción de Detecciones
El backend procesará las imágenes enviadas y responderá a través del WebSocket enviando un JSON que contiene un arreglo de detecciones de obstáculos. Cada detección tendrá la siguiente estructura:
```json
{
  "label": "<nombre_obstaculo>",
  "confidence": <nivel_confianza>,
  "relative_position": "<frente | frente_izquierda | frente_derecha | None>"
}
```

## 9. Priorización de Riesgos
Una vez recibidas las detecciones, el dispositivo debe procesarlas y priorizarlas basándose en:
1.  **Matriz de riesgo** (definida en un archivo Excel "Art. Matriz de Priorización de Obstaculos.xlsx").
2.  **Posición relativa** (`frente`, `frente_izquierda`, `frente_derecha`).

El objetivo de esta etapa es identificar cuál es el obstáculo más riesgoso o urgente a comunicar al usuario.

## 10. Reproducción de Audio (Alerta de Obstáculos)
Después de priorizar el obstáculo más riesgoso, el ESP32 XIAO SENSE debe reproducir audios desde su memoria micro SD para alertar al usuario. Se reproducen dos audios en secuencia:

1.  **Audio del obstáculo:** Se reproduce el archivo de audio correspondiente al `label` del obstáculo (el archivo de audio debe llamarse igual que el valor de *label*).
2.  **Audio de la posición relativa:** Se reproduce el archivo de audio correspondiente a la `relative_position` (el archivo de audio debe llamarse igual que la *relative_position*). 
    *   *Nota: Si la posición relativa es `None`, este segundo audio no se debe reproducir.*
