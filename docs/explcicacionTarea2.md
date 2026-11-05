# Transmisión por Wi-Fi y primera versión de la SPA

Explicación del código escrito para los dos últimos ítems del cronograma:
*Transmisión de datos por Wi-Fi* y *Primera versión de la SPA*.

Archivos involucrados:

| Archivo | Rol |
|---|---|
| `firmware/wifi_stream/wifi_stream.ino` | Firmware del ESP32: red, muestreo y transmisión |
| `firmware/wifi_stream/README.md` | Cómo cargarlo y el procedimiento de verificación |
| `webapp/index.html` | SPA: cliente WebSocket, gráfico y telemetría |

## 1. Alcance

El objetivo del paso es **validar el camino de red de forma independiente del
sensado**. Por eso el firmware todavía no lee el MPU6050: la tarea de muestreo
calcula un seno sintético en lugar de leer el sensor.

La decisión es deliberada. Si se implementara todo junto y la cadencia de 100 Hz
se degradara, no habría forma de saber si la culpa es del Wi-Fi, del I2C o del
filtro. Midiendo primero con un dato sintético, cualquier degradación posterior
es atribuible a lo que se agregó después.

Lo que sí está implementado y medido:

* El ESP32 levanta su propia red Wi-Fi.
* Sirve un servidor WebSocket al que el navegador se conecta directamente.
* El muestreo corre a 100 Hz en un núcleo y la transmisión en el otro, con una
  cola intermedia.
* El firmware publica estadísticas propias que permiten demostrar que la red no
  interrumpe el muestreo.

## 2. La red

El ESP32 no se une a una red existente: **crea la suya** en modo punto de acceso
(SoftAP). La computadora se une a esa red como a cualquier otra.

| Parámetro | Valor |
|---|---|
| SSID | `J2-Stylus` |
| Clave | `stylus2026` |
| Canal | 6 |
| IP del ESP32 | `192.168.4.1` (la que asigna el SoftAP por defecto) |
| WebSocket | `ws://192.168.4.1:81/` |

El motivo es una indicación docente: el sistema no puede depender de la red de la
facultad el día de la entrega. Así no hace falta router, ni credenciales, ni
conexión a internet. El enlace es punto a punto y dedicado, lo que además
simplifica el resto del diseño (ver sección 9).

La SPA **no se aloja en la placa**. Se sirve desde la computadora con un servidor
local y se abre en el navegador:

    cd webapp
    python -m http.server 5173

y después `http://localhost:5173`. El navegador carga la página desde
`localhost` y los datos desde el ESP32; son dos orígenes distintos y eso está
bien, porque WebSocket no aplica la política de mismo origen.

## 3. Arquitectura del firmware

El ESP32 es de doble núcleo. El reparto es el siguiente:

```
            timer de hardware (1 MHz, alarma cada 10000 us)
                             |
                             v
                        onTimer()   <-- ISR
                             |  notificación
                             v
    +----------------------+                      +----------------------+
    |  núcleo 1            |     cola de 32       |  núcleo 0            |
    |  tareaSensado        | -------------------> |  tareaRed            |
    |  prioridad 10        |   struct Muestra     |  prioridad 5         |
    +----------------------+                      +----------------------+
                                                            |  WebSocket
                                                            v
                                                    navegador (notebook)
```

La afirmación que sostiene el diseño es concreta: **la red no puede frenar el
muestreo**. El requerimiento de tiempo real está acotado al muestreo dentro del
ESP32 (informe de avance, sección 1.4.b), mientras que la transmisión y el
dibujado no tienen plazo estricto.

### El recorrido de una muestra

1. El timer de hardware vence cada 10000 µs y dispara `onTimer()`.
2. La ISR notifica a `tareaSensado` y cede el procesador inmediatamente.
3. `tareaSensado` se despierta, toma la marca de tiempo, mide el período real
   contra la muestra anterior, arma la muestra y la empuja a la cola. Si la cola
   está llena **no espera**: descarta la más vieja, mete la nueva y suma un
   contador.
4. `tareaRed`, en el otro núcleo, saca muestras de la cola. Cuando juntó dos,
   arma una trama JSON y la emite por WebSocket a todos los clientes conectados.
5. El navegador la recibe, la grafica y actualiza el panel de métricas.

El punto clave es el paso 3. La tarea de sensado nunca se bloquea esperando a la
red: ante una red trabada se degrada el dato transmitido, nunca la cadencia de
muestreo.

## 4. Las funciones del firmware

### `onTimer()`

Rutina de atención de la interrupción del timer. Está marcada `IRAM_ATTR` para
que resida en RAM interna y no dependa de la caché de la flash.

Hace una sola cosa: `vTaskNotifyGiveFromISR()` sobre la tarea de sensado. No
imprime, no calcula en punto flotante y no toma semáforos, porque todo eso está
prohibido o es peligroso dentro de una ISR. El `portYIELD_FROM_ISR()` final hace
que al salir de la interrupción se ejecute la tarea notificada de inmediato, en
lugar de esperar al próximo tick del planificador.

### `tareaSensado(void *arg)`

Núcleo 1, prioridad 10, 4096 bytes de stack. Es la tarea crítica.

* Se bloquea en `ulTaskNotifyTake(pdTRUE, portMAX_DELAY)` hasta que la ISR la
  despierta. Bloqueada no consume procesador.
* Mide `dt`, la diferencia de `micros()` contra la muestra anterior, y mantiene
  el mínimo y el máximo de la ventana de un segundo.
* Si `dt` supera `TARDE_US`, incrementa el contador de muestras atrasadas.
* Arma la muestra. **Acá va el reemplazo del paso 3**: hoy es
  `sinf(ahora / 500000.0f)`, mañana la lectura del MPU6050 y el filtro Madgwick.
* La empuja con `xQueueSend(..., 0)`. El timeout en cero es lo que garantiza que
  no se bloquee.
* Una vez por segundo publica el resumen en las variables de estadística y
  reinicia los acumuladores de la ventana.

### `tareaRed(void *arg)`

Núcleo 0, prioridad 5, 8192 bytes de stack (más grande porque la pila de red
consume más).

* Llama a `ws.loop()` en cada vuelta, que es lo que atiende los handshakes, los
  pings y las desconexiones de la librería de WebSocket.
* Saca una muestra con `xQueueReceive(..., pdMS_TO_TICKS(5))`. El timeout de 5 ms
  es necesario: si esperara indefinidamente, `ws.loop()` no se atendería durante
  los períodos sin muestras y la conexión se caería.
* Acumula en `lote[]` hasta juntar `LOTE` muestras, serializa el JSON con
  `snprintf` y lo emite con `ws.broadcastTXT()`.
* Mide cuánto tardó su propia vuelta y mantiene el máximo histórico.

### `setup()`

Corre en el núcleo 1. Inicializa el puerto serie, levanta el punto de acceso,
arranca el servidor WebSocket, crea la cola, crea las dos tareas con
`xTaskCreatePinnedToCore()` y por último configura el timer:

```cpp
hw_timer_t *timer = timerBegin(1000000);   // 1 MHz: 1 tick = 1 us
timerAttachInterrupt(timer, &onTimer);
timerAlarm(timer, PERIODO_US, true, 0);    // true = recarga automática
```

Esta es la API del core ESP32 Arduino **3.x**. La mayoría de los ejemplos que
circulan usan la de 2.x (`timerAlarmWrite` / `timerAlarmEnable`), que no compila
con el core actual.

El timer se configura **después** de crear las tareas, porque `onTimer()`
notifica a `hSensado` y ese handle tiene que existir antes de la primera
interrupción.

### `loop()`

Queda en el núcleo 1 con prioridad baja. Solo imprime una línea de diagnóstico
por segundo. Que esté en la prioridad más baja es intencional: si se viera
inanido, eso mismo sería un dato sobre la carga del sistema.

## 5. Parámetros configurables

| Constante | Valor | Qué significa |
|---|---|---|
| `PERIODO_US` | 10000 | Período de muestreo en µs. 100 Hz |
| `TARDE_US` | 11000 | Umbral para contar una muestra como atrasada (10 % sobre el nominal) |
| `LOTE` | 2 | Muestras por trama. Con 100 Hz da 50 tramas/s |
| `COLA_LARGO` | 32 | Lugares de la cola. A 100 Hz son 320 ms de amortiguación |
| `WS_PORT` | 81 | Puerto del servidor WebSocket |

Y dos banderas de compilación, ambas para las pruebas:

| Bandera | Efecto |
|---|---|
| `USAR_WIFI 0` | Apaga la radio. Las dos tareas y la cola siguen funcionando igual; lo único que no pasa es que se transmita. Sirve como medición de referencia |
| `FALLA_RED 1` | Mete un `delay(500)` dentro del lazo de la tarea de red. Falla inyectada para la prueba del desacople |

Para operación normal quedan en `USAR_WIFI 1` y `FALLA_RED 0`.

## 6. La trama que se envía

Es un objeto JSON en un solo mensaje de texto WebSocket. Ejemplo real:

```json
{"hz":100,"dtmin":9994,"dtmax":10008,"tarde":0,"drop":0,"s":[[12345,98765432,0.7071],[12346,98775432,0.7075]]}
```

Campos:

| Campo | Tipo | Contenido |
|---|---|---|
| `hz` | entero | Muestras tomadas en el último segundo |
| `dtmin` | entero | Período más corto del último segundo, en µs |
| `dtmax` | entero | Período más largo del último segundo, en µs |
| `tarde` | entero | Acumulado de muestras atrasadas desde el arranque |
| `drop` | entero | Acumulado de muestras descartadas por cola llena |
| `s` | arreglo | Las muestras del lote |

Cada elemento de `s` es una terna `[seq, t_us, value]`:

* `seq` — número de secuencia, incremental, uno por muestra. Es lo que permite
  detectar pérdidas en el receptor: si entre dos muestras consecutivas el número
  salta, lo que falta se perdió.
* `t_us` — marca de tiempo en microsegundos del ESP32, tomada en el instante del
  muestreo, no en el del envío.
* `value` — el dato. Hoy el seno sintético, con cuatro decimales.

### Cada cuánto

| Evento | Período |
|---|---|
| Muestra tomada | 10 ms (100 Hz) |
| Trama emitida | 20 ms (50 tramas/s), porque `LOTE = 2` |
| Estadísticas recalculadas en el ESP32 | 1 s |
| Línea de diagnóstico por serie | 1 s |
| Refresco del panel de la SPA | 500 ms |
| Redibujado del gráfico | por `requestAnimationFrame`, ~60 veces/s |
| Reintento de reconexión | 1 s |

Los campos de estadística se recalculan una vez por segundo pero viajan en
**todas** las tramas, o sea unas 50 veces repetidos. Es redundante, pero son unos
40 bytes sobre una trama de 110 en un enlace dedicado: cuesta menos que agregar
un segundo tipo de mensaje y manejarlo aparte.

### Tamaño y ancho de banda

La trama del ejemplo son 110 bytes. En el peor caso, con los contadores y la
marca de tiempo en sus valores más largos, llega a unos 145. A 50 tramas por
segundo eso da entre **5,5 y 7 kB/s**, contra los varios Mb/s que da el enlace.

El buffer de armado es de 256 bytes, cerca del doble del peor caso. El `snprintf`
acumula el retorno en `p` y limita cada escritura a `sizeof(msg) - p`, así que un
desborde truncaría la trama en lugar de corromper memoria.

### Por qué JSON y no binario

Una estructura binaria empacada ocuparía unos 12 bytes por muestra contra los ~50
del JSON. Sobre este enlace la diferencia es irrelevante, y el JSON se lee
directamente en las herramientas de desarrollo del navegador, lo que durante el
desarrollo vale más que el ancho de banda ahorrado. Si en el paso 3 el volumen
creciera mucho, el cambio queda acotado a `tareaRed` y a `onmessage`.

## 7. Las variables de diagnóstico

Son seis variables `volatile uint32_t` que escribe el sensado y lee el otro
núcleo. Se imprimen por serie así:

```
hz=100  dt=[9994..10008]us  tarde=0  drop=0  red_max=812us
```

| | Qué mide | Sano | Se reinicia |
|---|---|---|---|
| `hz` | Muestras realmente tomadas en el último segundo | 100 | cada 1 s |
| `dt` | Período **real** entre muestras, `[mínimo..máximo]` del último segundo | ~[9990..10010] | cada 1 s |
| `tarde` | Muestras con `dt` mayor a `TARDE_US` | 0 | nunca |
| `drop` | Muestras descartadas por cola llena | 0 | nunca |
| `red_max` | Vuelta más larga del lazo de la tarea de red, en µs | < 10000 | nunca |

Tres aclaraciones que no son obvias:

**`dt` son los extremos, no el promedio.** La distancia entre el mínimo y el
máximo es el jitter. El promedio escondería el problema: si 99 muestras salen
perfectas y una se atrasa 3 ms, el promedio casi no se mueve y el máximo sí.

**`tarde` y `dt` parecen redundantes pero no lo son.** `dt` se reinicia cada
segundo, así que un evento aislado desaparece de la pantalla y no se ve nunca
más. `tarde` acumula y sobrevive. `dt` dice qué tan mal está ahora, `tarde` dice
cuántas veces pasó en total.

**`red_max` y `drop` son causa y consecuencia.** `red_max` es cuánto tarda la
red; `drop` es el daño que esa demora causó. Entre los dos está el colchón de la
cola, 320 ms. Un `red_max` de 5300 µs es el 1,6 % del colchón, y ese es el margen
real disponible.

Un detalle sobre `red_max` en reposo: da del orden de 5000 µs y no de 800 porque
cuando no hay muestras esperando, la vuelta se consume entera en el timeout de
5 ms de la cola. No es un problema, es el lazo esperando trabajo.

Las estadísticas se escriben sin mutex. Son enteros de 32 bits alineados, cuya
escritura es atómica en esta arquitectura, así que en el peor caso se lee una
mezcla de dos segundos distintos. Es aceptable para un diagnóstico, y la
alternativa sería agregar un bloqueo en la ruta del sensado, que es exactamente
lo que el diseño busca proteger.

## 8. La SPA

Un único `index.html` sin dependencias ni compilación: HTML, CSS y JavaScript en
el mismo archivo. El gráfico es Canvas 2D a mano. TypeScript y Vite se
incorporarán en el paso 4, cuando aparezca el lienzo de dibujo y el código
empiece a justificar una estructura.

El destino del WebSocket es `ws://192.168.4.1:81/`, y se puede cambiar con un
parámetro en la URL: `?host=otra.ip`.

### Las funciones

**`conectar()`** — abre el WebSocket e instala los cuatro manejadores.
`onmessage` es el que hace el trabajo: parsea el JSON dentro de un `try` (un
mensaje corrupto se ignora en lugar de romper la página), guarda las
estadísticas, registra la marca de llegada y recorre las muestras del lote. Por
cada una compara el `seq` con el anterior para acumular pérdidas y agrega el
valor a la serie, que se mantiene acotada a 600 puntos descartando por el frente.

`onclose` reprograma `conectar()` al segundo, y `onerror` cierra para que el
cierre dispare ese mismo camino. Así la reconexión es automática y hay un solo
lugar donde se decide reintentar: se puede apagar y prender la placa con la
página abierta y vuelve sola.

**`setEstado(ok, txt)`** y **`poner(id, texto, alerta)`** — utilidades de
presentación. `poner` además enciende el color de alerta cuando el valor sale del
rango esperado, que es lo que hace que un problema se vea sin leer números.

**El `setInterval` de 500 ms** — recalcula las métricas. Las tramas por segundo
salen de filtrar el arreglo de marcas de llegada a la última ventana de un
segundo; el jitter de llegada es el pico a pico de las diferencias entre marcas
consecutivas. Los umbrales de alerta son: `hz` fuera de 100 ± 1, `dtmax` por
encima de 11000 µs, y cualquier valor mayor a cero en atrasadas, descartes o
tramas perdidas.

**`dibujar()`** — se reprograma con `requestAnimationFrame`. Ajusta el tamaño del
canvas por `devicePixelRatio` para que la curva no se vea borrosa en pantallas de
alta densidad, dibuja el eje y después la serie. Los colores se leen de las
variables CSS, así que el gráfico sigue el tema claro u oscuro del sistema.

El panel está dividido en dos grupos a propósito. **Red** (tramas/s, jitter de
llegada, tramas perdidas, recibidas) son métricas medidas en la computadora e
incluyen el aire. **Muestreo en el ESP32** (muestras/s, período, atrasadas,
descartes de cola) son las que reporta el firmware. La separación importa: si se
degrada el primer grupo y el segundo se mantiene, el problema está en la red y el
muestreo está intacto, que es justamente lo que el diseño promete.

## 9. Decisiones de diseño

**Punto de acceso propio en lugar de unirse a una red existente.** Indicación
docente: no depender de la red de la facultad el día de la entrega. Como efecto
secundario, el enlace queda dedicado, lo que vuelve despreciable el costo del
JSON y del envío redundante de estadísticas.

**Timer de hardware con notificación a una tarea, en lugar de
`vTaskDelayUntil()`.** El tick de FreeRTOS es de 1 ms, así que una espera basada
en el planificador cuantizaría el muestreo a múltiplos de 1 ms y arruinaría el
jitter medido en el Ensayo 2 del informe. El timer de hardware trabaja con
resolución de 1 µs.

**Cola con timeout cero, descartando la muestra más vieja.** Tres propiedades: el
sensado nunca se bloquea; la degradación es de dato y no de cadencia; y lo que se
conserva es lo más reciente, que para un lápiz es lo útil, porque una posición de
hace 300 ms no sirve para nada.

**`LOTE = 2`.** Agrupar dos muestras por trama baja de 100 a 50 tramas por
segundo y reparte el encabezado entre dos muestras en lugar de una. El costo es
latencia: la primera muestra del par espera hasta 10 ms a su compañera. Para el
dibujado es imperceptible y no hay plazo estricto en ese lado.

**Prioridad 10 para el sensado contra 5 para la red.** Estando en núcleos
distintos casi no interviene, pero fija el criterio para cualquier cosa que
llegue a serializarse entre ambas.

**La SPA se sirve desde la computadora y no desde la flash del ESP32.** Mantiene
el esquema de la Figura 1 del informe sin cambios que justificar, permite iterar
la interfaz sin recargar el firmware, y evita LittleFS y la compresión de los
archivos estáticos.

## 10. Verificación

El procedimiento completo está en `firmware/wifi_stream/README.md`, en cinco
pruebas. Resumidas:

| Prueba | Qué comprueba |
|---|---|
| 0 | Las tareas quedaron en los núcleos correctos |
| 1 | Medición de referencia con `USAR_WIFI 0` |
| 2 | La misma medición con la radio transmitiendo |
| 3 | Con `FALLA_RED 1`, el muestreo sobrevive a una red trabada |
| 4 | Operación normal sostenida durante diez minutos |

Lo que demuestra la afirmación del paso son la 2 y la 3. La 2 compara **sin red
contra con red**: si `hz` y el rango de `dt` no se degradan, el muestreo no se
enteró de que hay Wi-Fi. La 3 es el contraejemplo y es el dato más fuerte, porque
muestra el mecanismo actuando bajo falla y no solo funcionando en condiciones
buenas: con la tarea de red frenada a propósito, `drop` y `red_max` se van al
techo mientras `hz` se queda en 100 y `tarde` en 0.

Resultados medidos sobre la placa, a completar:

| | Prueba 1 (sin red) | Prueba 2 (con red) |
|---|---|---|
| `hz` | | |
| `dt` | | |
| `red_max` | | |

## 11. Qué falta

* **Paso 3** — reemplazar el seno sintético por la lectura del MPU6050, el
  cuaternión del filtro Madgwick, la posición estimada y el estado de contacto.
* **Paso 4** — lienzo de dibujo y modo de trazo por inclinación en la SPA,
  portando la lógica de `tools/visor_orientacion_dibujo`. Acá entran TypeScript y Vite.
* **Paso 5** — modo de desplazamiento y conexión del sensor Hall KY-024.

Queda además una decisión abierta sobre el hardware. La placa usada en estas
pruebas es una Heltec Wi-Fi LoRa 32 V3, que monta un **ESP32-S3** y no el
ESP-WROOM-32 documentado en la sección 3.2 del informe. Sigue siendo de doble
núcleo, así que el reparto de tareas no cambia, pero los pines no son
compatibles: GPIO22 no existe en el S3 y GPIO34 no es canal de ADC. Además el
ADC2 queda inutilizable con el Wi-Fi activo, por lo que el sensor Hall tiene que
ir a un canal del ADC1 (GPIO2 a GPIO7). Si el grupo trabaja con las dos placas,
los pines deberían quedar en un `config.h` por placa; si se adopta solo la
Heltec, hay que corregir la sección 3.2 y la Figura 2 del informe.
