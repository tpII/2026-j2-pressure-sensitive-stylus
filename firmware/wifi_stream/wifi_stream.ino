// Paso 1 de la transmision por WiFi.
//
// El ESP32 levanta su propia red (SoftAP), de modo que no depende de ninguna
// infraestructura: la PC se une a esa red y el navegador abre el WebSocket
// contra 192.168.4.1. No se lee ningun sensor; se emite un seno sintetico a
// 50 Hz para validar el camino de red completo antes de sumar el filtro.
//
// El campo seq permite detectar paquetes perdidos y el campo t (micros() en el
// instante del envio) permite medir el jitter de llegada. Ambos se conservan en
// la version final, porque son los numeros del analisis de limites.

#include <WiFi.h>
#include <WebSocketsServer.h>

static const char   *AP_SSID        = "J2-Stylus";
static const char   *AP_PASS        = "stylus2026";   // minimo 8 caracteres
static const uint8_t AP_CHANNEL     = 6;              // fijo, para elegir canal libre en el aula
static const uint16_t WS_PORT       = 81;
static const uint32_t SEND_PERIOD_MS = 20;            // 50 Hz

WebSocketsServer ws(WS_PORT);

static uint32_t seq      = 0;
static uint32_t lastSend = 0;

void onWsEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t len) {
    switch (type) {
        case WStype_CONNECTED:
            Serial.printf("[ws] cliente %u conectado desde %s\n",
                          num, ws.remoteIP(num).toString().c_str());
            break;
        case WStype_DISCONNECTED:
            Serial.printf("[ws] cliente %u desconectado\n", num);
            break;
        default:
            break;
    }
}

void setup() {
    Serial.begin(115200);
    delay(200);

    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS, AP_CHANNEL);

    Serial.println();
    Serial.printf("SSID: %s\n", AP_SSID);
    Serial.printf("clave: %s\n", AP_PASS);
    Serial.printf("WebSocket: ws://%s:%u/\n",
                  WiFi.softAPIP().toString().c_str(), WS_PORT);

    ws.begin();
    ws.onEvent(onWsEvent);

    lastSend = millis();
}

void loop() {
    ws.loop();

    // mismo patron de temporizacion que madgwick_filter: el instante objetivo
    // avanza un periodo fijo, de modo que el atraso de una iteracion no se acumula
    uint32_t now = millis();
    if ((int32_t)(now - lastSend) < (int32_t)SEND_PERIOD_MS) return;
    lastSend += SEND_PERIOD_MS;
    if ((int32_t)(now - lastSend) > (int32_t)(5 * SEND_PERIOD_MS)) lastSend = now;

    float value = sinf(now / 500.0f);   // seno de ~0,32 Hz, visible a simple vista

    char msg[96];
    int n = snprintf(msg, sizeof(msg),
                     "{\"seq\":%lu,\"t\":%lu,\"value\":%.4f}",
                     (unsigned long)seq++, (unsigned long)micros(), value);
    ws.broadcastTXT(msg, n);
}
