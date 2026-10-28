# wifi_stream — transmision de datos por WiFi

Paso 1: el ESP32 levanta su propia red WiFi (SoftAP), abre un servidor
WebSocket y emite un seno sintetico a 50 Hz. **No se leen sensores.**

El objetivo es validar el camino de red completo (AP, WebSocket, llegada de
tramas, reconexion) por separado del firmware de sensado, para que cuando algo
falle se sepa de que lado esta el problema.

## Por que una red propia

El dia de la entrega no se depende del WiFi de la facultad: la notebook se une
a la red `J2-Stylus` que crea la placa. Sin router, sin internet, sin credenciales.

Contrapartida: mientras la notebook esta en esa red, no tiene internet.

## Preparar el IDE de Arduino

1. **Placa:** en *Gestor de tarjetas* instalar `esp32` de Espressif Systems.
   Despues elegir **Heltec WiFi LoRa 32(V3)** en Herramientas > Placa.
   Para la placa de la catedra, **ESP32 Dev Module**.
2. **Libreria:** en *Gestor de librerias* instalar **WebSockets** de
   Markus Sattler.
3. Abrir `wifi_stream.ino` y cargar con el boton de subida.

El monitor serie va a 115200 baudios. Tiene que aparecer el SSID y la URL del
WebSocket.

Si el monitor no muestra nada, revisar que *USB CDC On Boot* este en `Disabled`:
la Heltec V3 usa un CP2102, no el USB nativo del S3.

## Levantar la interfaz

La SPA esta en `webapp/` en la raiz del repositorio. No abrirla con `file://`:
el origen es `null` y el WebSocket se comporta distinto segun el navegador.

    cd webapp
    python -m http.server 5173

Despues, con la notebook conectada a la red `J2-Stylus` con la contraseña 'stylus2026', abrir
<http://localhost:5173>.

Si la placa esta en modo estacion en vez de AP, se le puede pasar otra
direccion: `http://localhost:5173/?host=192.168.1.50`

## Que hay que verificar

1. La red `J2-Stylus` aparece en la lista de redes de la notebook.
2. La pagina pasa a «conectado» sola.
3. **Tramas/s** se queda cerca de 50 y estable.
4. **Perdidas** se mantiene en 0 (se calcula con los huecos del campo `seq`).
5. La curva del seno se ve lisa, sin tirones ni escalones.
6. Al apagar y prender la placa, la pagina se reconecta sola.

El jitter pico a pico es la diferencia entre el intervalo de llegada mas largo
y el mas corto del ultimo segundo. Unos pocos milisegundos es normal; decenas
indica congestion del canal.

## Formato del mensaje

    {"seq":1432,"t":14320000,"value":0.7071}

- `seq`: numero de secuencia, detecta paquetes perdidos.
- `t`: `micros()` del ESP32 en el instante del envio, para medir jitter.
- `value`: el seno sintetico; en el paso 3 lo reemplazan `q[]` y `p[]`.

`seq` y `t` se conservan en la version final: son los datos del analisis de
limites del proyecto.

## Notas sobre la placa Heltec V3 (para los pasos siguientes)

La Heltec WiFi LoRa 32 V3 monta un **ESP32-S3**, no el ESP-WROOM-32 del
informe de avance. Sigue siendo de doble nucleo, asi que el reparto de tareas
entre cores del paso 2 no cambia. Lo que si cambia son los pines:

| Que | Informe (WROOM-32) | En la Heltec V3 |
|---|---|---|
| I2C del MPU6050 | GPIO21 / GPIO22 | **GPIO22 no existe en el S3.** Usar el bus del OLED (SDA 17, SCL 18) o un bus propio en GPIO41/42 |
| ADC del KY-024 | GPIO34 | **GPIO34 no es pin de ADC en el S3.** Usar ADC1: GPIO2 a GPIO7 |

Pines que **no** hay que tocar en esta placa:

- GPIO8 a GPIO14: radio LoRa (SX1262)
- GPIO17 / GPIO18: I2C del OLED
- GPIO19 / GPIO20: USB nativo
- GPIO21: reset del OLED
- GPIO43 / GPIO44: UART del CP2102 (el monitor serie)
- GPIO0: pulsador de usuario
- GPIO36: control de Vext
- GPIO45 / GPIO46: pines de arranque (strapping)

Dos detalles mas:

- **ADC2 no se puede usar con WiFi activo.** El KY-024 tiene que ir si o si en
  un canal de ADC1 (GPIO2 a GPIO7 en esta placa).
- **Alimentar el MPU6050 desde 3V3, no desde Ve.** Los pines `Ve` dependen del
  control de Vext (GPIO36) y quedan sin tension hasta habilitarlo.

Antes de cablear el MPU6050 conviene correr `firmware/i2c_scan` con los pines
nuevos para confirmar que responde en 0x68.
