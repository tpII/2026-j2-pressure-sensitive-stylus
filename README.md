# 2026-j2-pressure-sensitive-stylus
Taller de Proyecto II 2026 — Grupo J2 — Pressure Sensitive Stylus

## Versión final (entrega)

| Carpeta | Contenido |
|---|---|
| `firmware/lapiz_orientacion` | Firmware final: Madgwick embebido en el ESP32 (β adaptativo 0,033 / 0,1), sin desplazamiento. Manda la orientación, el acelerómetro, el giróscopo y el contacto del Hall. |
| `tools/visor_orientacion_vectores` | Visor de la demostración final (Processing): modelo 3D con la orientación, el vector gravedad (acelerómetro) y el vector giro (giróscopo, con el sentido de giro); dibujo por inclinación con el lápiz bajado por el Hall. |

Uso: cargar `firmware/lapiz_orientacion` en el ESP32 (placa ESP32 Dev Module, MPU6050 en SDA 21 / SCL 22, Hall SS49E en GPIO 4), dejar el lápiz quieto mientras calibra y abrir `tools/visor_orientacion_vectores` (puerto en la línea `PUERTO`).

## Desarrollo y ensayos

### Base de la orientación

| Carpeta | Contenido |
|---|---|
| `firmware/madgwick_filter` | Firmware de los informes de avance: orientación con Madgwick (β adaptativo 0,033 / 0,1) y desplazamiento con ZUPT. Base de `lapiz_orientacion`. |
| `firmware/i2c_scan` | Escáner I2C para verificar el cableado del MPU6050. |
| `tools/visor_orientacion_dibujo` | Visor (Processing): modelo 3D del sensor y dibujo por inclinación. |
| `tools/visor_trayectoria_3d` | Visor (Processing): orientación y trayectoria de desplazamiento de `madgwick_filter`. |

### Transmisión Wi-Fi

| Carpeta | Contenido |
|---|---|
| `firmware/wifi_stream` | Transmisión Wi-Fi: SoftAP + WebSocket hacia la SPA. |
| `webapp` | SPA de telemetría. |
| `firmware/ensayo_latencia_wifi` | Ensayo: `wifi_stream` con un eco para medir la latencia de ida y vuelta (RTT) desde el navegador. |
| `tools/medir_latencia_wifi` | Pagina web que mide la latencia del Wi-Fi (RTT, perdidas, separacion de tramas) con `ensayo_latencia_wifi`. |

### Sensor Hall

| Carpeta | Contenido |
|---|---|
| `firmware/ensayo_hall` | Ensayo (PlatformIO, Arduino UNO): lectura del sensor Hall SS49E con filtro EMA y descarte de picos. |
| `tools/visor_hall` | Visor web (Web Serial) de la señal del sensor Hall. |
| `tools/calibrar_presion_hall.py` | Curva de presion del Hall con una balanza: fuerza minima de contacto, resolucion e histeresis; genera `tools/calibracion_presion_hall.json`. |

### Desplazamiento (límite de la investigación)

| Carpeta | Contenido |
|---|---|
| `firmware/ensayo_beta_adaptativo_vs_fijo` | Ensayo: β adaptativo contra β fijo 0,1, dos filtros en paralelo sobre las mismas muestras. |
| `firmware/ensayo_desplazamiento_integracion_pura` | Ensayo: doble integración sin correcciones, con y sin Madgwick. |
| `firmware/ensayo_desplazamiento_hall` | El mismo ensayo con el Hall SS49E leído en el ESP32 (GPIO 4): agrega el contacto a cada muestra. |
| `tools/visor_desplazamiento_integracion_pura` | Visor (Processing) del ensayo de desplazamiento, con o sin Hall; graba las corridas en `corridas/`. |
| `tools/visor_dibujo_hall` | Visor (Processing) de dibujo con IMU + Hall, muestra de la versión final: trazos por contacto, grosor por presión, corrección al frenar. Con `ensayo_desplazamiento_hall`. |
| `tools/visor_lapiz_hall` | Visor (Processing) del lapiz armado: trazos cortos con IMU + Hall, bias medido con el lapiz acostado, pausas en la mano, posicion de la punta (IMU a 12,5 cm, eje Y hacia arriba). Con `ensayo_desplazamiento_hall`. |
| `tools/visor_joystick_hall` | Visor (Processing) de la alternativa joystick: el cursor sigue la inclinacion (palanca x giro), con embrague por el Hall, grosor por presion y estadisticas de cada trazo. Con `ensayo_desplazamiento_hall`. En prueba, no es la version final. |
| `tools/analizar_desplazamiento.py` | Análisis de las corridas del ensayo de desplazamiento. |
| `tools/calibrar_imu.py` | Calibración del acelerómetro (6 caras) y de la escala del giroscopo (giros de 360°) desde corridas del visor; genera `tools/calibracion_imu.json`. |
| `tools/reprocesar_desplazamiento.py` | Reprocesa la lectura cruda de las corridas y compara variantes del cálculo del desplazamiento (β = 0 en el trazo, corrección de velocidad, posición de la punta). |

### DMP contra Madgwick

| Carpeta | Contenido |
|---|---|
| `firmware/ensayo_dmp_vs_madgwick` | Ensayo: cuaternion del DMP del MPU6050 y lectura cruda en la misma muestra (libreria MPU6050 de Electronic Cats). |
| `tools/comparar_dmp_madgwick.py` | Graba corridas de `ensayo_dmp_vs_madgwick` y compara el DMP con las variantes del Madgwick (reposo, caras, deslizar, giros). |
| `tools/corridas_dmp`, `tools/resultados_dmp` | Corridas del ensayo DMP contra Madgwick (11/11) y su analisis (`resumen_dmp_vs_madgwick.md`, `revision_resultados.md`). |

### Otros

| Carpeta | Contenido |
|---|---|
| `docs/resultados/ensayos_iniciales_t1-t4` | Resultados de los ensayos iniciales (I2C, lectura cruda, calibración, Madgwick con β fijo y validación de ángulos). |
| `BITACORA.md` | Bitácora del proyecto. |
