// ENSAYO DE LATENCIA WI-FI - J2 Pressure Sensitive Stylus | Taller de Proyecto II - UNLP
//
// Es firmware/wifi_stream (paso 2, sin cambios en el sensado ni en el envio de
// tramas) con UNA sola cosa agregada: un eco para medir la latencia de ida y
// vuelta (RTT) desde el navegador, con tools/medir_latencia_wifi/index.html.
//
// La correccion del docente al plan dice que la latencia inalambrica no es un
// requisito de tiempo real (el Wi-Fi no es deterministico) sino algo que se mide
// y se informa como limitacion. Este ensayo la mide con las tramas de 50 Hz
// circulando, o sea con la misma carga que la version final.
//
//   navegador -> ESP32   "p,<id>,<t_envio_ms>"
//   ESP32 -> navegador   {"pong":"<id>,<t_envio_ms>","t_esp":<micros>}
// La respuesta se arma dentro de ws.loop() (tarea de red, core 0), en el mismo
// lugar donde se arman las tramas: el RTT incluye la espera en esa tarea.
// La latencia de un solo sentido se estima como RTT / 2 (los relojes del ESP32
// y de la PC no estan sincronizados).
//
// Lo demas, igual que wifi_stream:
//
// El requerimiento de tiempo real esta acotado al muestreo dentro del ESP32
// (informe de avance, seccion 1.4.b), asi que la red no puede frenar el sensado.
// Para eso:
//
//   core 1  tarea de sensado, prioridad alta, disparada por un timer de
//           hardware cada 10 ms. Empuja cada muestra a una cola.
//   core 0  tarea de red. Saca muestras de la cola, arma lotes y los envia.
//
// La cola se escribe con timeout 0: si esta llena se descarta la muestra mas
// vieja y se incrementa un contador. De ese modo un bloqueo de la red nunca
// retrasa el muestreo; se pierden datos, pero la cadencia se mantiene.
//
// Todavia no se leen sensores: la tarea de sensado calcula el mismo seno
// sintetico del paso 1. Lo que se valida aca es que la cadencia de 100 Hz se
// sostenga con el WiFi transmitiendo.

#include <WiFi.h>
#include <WebSocketsServer.h>

#define USAR_WIFI 1          // 0 = sin red, solo estadisticas por serie (medicion de referencia)
#define FALLA_RED  0         // 1 = prueba 3: frena la tarea de red 500 ms por vuelta (falla inyectada)

static const char    *AP_SSID        = "J2-Stylus";
static const char    *AP_PASS        = "stylus2026";
static const uint8_t  AP_CHANNEL     = 6;
static const uint16_t WS_PORT        = 81;

static const uint32_t PERIODO_US     = 10000;   // 100 Hz
static const uint32_t TARDE_US       = 11000;   // una muestra por encima de esto cuenta como atrasada
static const uint8_t  LOTE           = 2;       // muestras por trama -> 50 tramas/s
static const uint8_t  COLA_LARGO     = 32;      // ~320 ms de amortiguacion

struct Muestra {
    uint32_t seq;
    uint32_t t_us;
    float    value;
};

WebSocketsServer ws(WS_PORT);
QueueHandle_t    cola;
TaskHandle_t     hSensado = NULL;

// Estadisticas publicadas por la tarea de sensado y leidas desde el otro nucleo.
// Son enteros de 32 bits alineados, cuya escritura es atomica en esta
// arquitectura; en el peor caso se lee una mezcla de dos segundos distintos, lo
// que es aceptable para un diagnostico.
static volatile uint32_t stHz     = 0;     // muestras tomadas en el ultimo segundo
static volatile uint32_t stDtMin  = 0;     // periodo mas corto del ultimo segundo [us]
static volatile uint32_t stDtMax  = 0;     // periodo mas largo del ultimo segundo [us]
static volatile uint32_t stTarde  = 0;     // acumulado de muestras por encima de TARDE_US
static volatile uint32_t stDrop   = 0;     // acumulado de muestras descartadas por cola llena
static volatile uint32_t stNetMax = 0;     // vuelta mas larga de la tarea de red [us]
static volatile uint32_t stPings  = 0;     // pings respondidos (ensayo de latencia)

// ---------------------------------------------------------------- timer ----

void IRAM_ATTR onTimer() {
    BaseType_t debeCeder = pdFALSE;
    vTaskNotifyGiveFromISR(hSensado, &debeCeder);
    portYIELD_FROM_ISR(debeCeder);
}

// --------------------------------------------------------------- sensado ---

void tareaSensado(void *arg) {
    uint32_t seq = 0;
    uint32_t tPrev = 0, tVentana = 0;
    uint32_t dtMin = UINT32_MAX, dtMax = 0, cuenta = 0;

    Serial.printf("[sensado] corriendo en el core %d\n", xPortGetCoreID());

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        uint32_t ahora = micros();

        // periodo real entre muestras consecutivas
        if (tPrev) {
            uint32_t dt = ahora - tPrev;
            if (dt < dtMin) dtMin = dt;
            if (dt > dtMax) dtMax = dt;
            if (dt > TARDE_US) stTarde++;
        }
        tPrev = ahora;
        cuenta++;

        // en el paso 3 esto se reemplaza por la lectura del MPU6050 y el filtro
        Muestra m = { seq++, ahora, sinf(ahora / 500000.0f) };

        if (xQueueSend(cola, &m, 0) != pdTRUE) {
            Muestra descartada;
            xQueueReceive(cola, &descartada, 0);   // se tira la mas vieja
            xQueueSend(cola, &m, 0);
            stDrop++;
        }

        // resumen una vez por segundo
        if (ahora - tVentana >= 1000000) {
            tVentana = ahora;
            stHz = cuenta; stDtMin = dtMin; stDtMax = dtMax;
            cuenta = 0; dtMin = UINT32_MAX; dtMax = 0;
        }
    }
}

// ------------------------------------------------------------------ eco ---

// Responde cada ping del navegador. Corre dentro de ws.loop(), en la tarea de red.
void onWsEvento(uint8_t num, WStype_t tipo, uint8_t *payload, size_t largo) {
    if (tipo != WStype_TEXT || largo < 3 || payload[0] != 'p' || payload[1] != ',') return;
    size_t n = largo - 2;
    if (n > 60) n = 60;
    char resp[112];
    int p = snprintf(resp, sizeof(resp), "{\"pong\":\"%.*s\",\"t_esp\":%lu}",
                     (int)n, (const char *)payload + 2, (unsigned long)micros());
    ws.sendTXT(num, resp, p);
    stPings++;
}

// ------------------------------------------------------------------- red ---

void tareaRed(void *arg) {
    Muestra lote[LOTE];
    uint8_t n = 0;
#if USAR_WIFI
    char    msg[256];
#endif

    Serial.printf("[red] corriendo en el core %d\n", xPortGetCoreID());

    for (;;) {
        uint32_t t0 = micros();

#if FALLA_RED
        delay(500);   // prueba 3: freno deliberado de la tarea de red
#endif

#if USAR_WIFI
        ws.loop();
#endif

        // el timeout corto garantiza que ws.loop() se atienda aunque no lleguen muestras
        if (xQueueReceive(cola, &lote[n], pdMS_TO_TICKS(5)) == pdTRUE && ++n >= LOTE) {
#if USAR_WIFI
            int p = snprintf(msg, sizeof(msg),
                             "{\"hz\":%lu,\"dtmin\":%lu,\"dtmax\":%lu,"
                             "\"tarde\":%lu,\"drop\":%lu,\"s\":[",
                             (unsigned long)stHz, (unsigned long)stDtMin,
                             (unsigned long)stDtMax, (unsigned long)stTarde,
                             (unsigned long)stDrop);
            for (uint8_t i = 0; i < n; i++) {
                p += snprintf(msg + p, sizeof(msg) - p, "%s[%lu,%lu,%.4f]",
                              i ? "," : "", (unsigned long)lote[i].seq,
                              (unsigned long)lote[i].t_us, lote[i].value);
            }
            p += snprintf(msg + p, sizeof(msg) - p, "]}");
            ws.broadcastTXT(msg, p);
#endif
            n = 0;   // con USAR_WIFI 0 la cola igual se vacia: solo no se transmite
        }

        uint32_t vuelta = micros() - t0;
        if (vuelta > stNetMax) stNetMax = vuelta;
    }
}

// ----------------------------------------------------------------- setup ---

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.printf("\n[setup] corriendo en el core %d\n", xPortGetCoreID());

#if USAR_WIFI
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS, AP_CHANNEL);
    Serial.printf("SSID: %s  clave: %s\n", AP_SSID, AP_PASS);
    Serial.printf("WebSocket: ws://%s:%u/\n",
                  WiFi.softAPIP().toString().c_str(), WS_PORT);
    ws.begin();
    ws.onEvent(onWsEvento);
#else
    WiFi.mode(WIFI_OFF);
    Serial.println("WiFi deshabilitado (USAR_WIFI 0): medicion de referencia");
#endif

    cola = xQueueCreate(COLA_LARGO, sizeof(Muestra));

    // el sensado va en el core 1 con prioridad alta; la red en el core 0
    xTaskCreatePinnedToCore(tareaSensado, "sensado", 4096, NULL, 10, &hSensado, 1);
    xTaskCreatePinnedToCore(tareaRed,     "red",     8192, NULL,  5, NULL,      0);

    // timer de hardware a 1 MHz: 1 tick = 1 us. No se usa vTaskDelayUntil porque
    // el tick de FreeRTOS es de 1 ms y arruinaria el jitter del muestreo.
    hw_timer_t *timer = timerBegin(1000000);
    timerAttachInterrupt(timer, &onTimer);
    timerAlarm(timer, PERIODO_US, true, 0);
}

// loop() queda en el core 1 con prioridad baja: solo informa por serie una vez
// por segundo. Si se viera starveado, eso mismo seria un dato.
void loop() {
    Serial.printf("hz=%lu  dt=[%lu..%lu]us  tarde=%lu  drop=%lu  red_max=%luus  pings=%lu\n",
                  (unsigned long)stHz, (unsigned long)stDtMin, (unsigned long)stDtMax,
                  (unsigned long)stTarde, (unsigned long)stDrop, (unsigned long)stNetMax,
                  (unsigned long)stPings);
    delay(1000);
}
