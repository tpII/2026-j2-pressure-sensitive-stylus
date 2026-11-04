# SS49E · Visualizador Hall RAW

Streaming crudo (sin filtros) de un SS49E en A0 de un Arduino Uno → dashboard web con Web Serial API.

```
firmware/ensayo_hall/platformio.ini   Proyecto PlatformIO (env uno, COM5, monitor 115200)
firmware/ensayo_hall/src/main.cpp     Firmware (analogRead directo, 50 Hz con millis(), JSON @115200)
tools/visor_hall/web/index.html       Dashboard (HTML + Canvas, 0 dependencias)
tools/visor_hall/serve.js             Servidor estático Node, 0 dependencias
```

## 1. Firmware (PlatformIO)
**VS Code:** abrir la carpeta `firmware/ensayo_hall/` como proyecto PlatformIO → *Upload* (→) en la barra inferior.

**CLI:**
```powershell
pio run -d firmware/ensayo_hall -t upload      # compila y flashea en COM5
pio device monitor -d firmware/ensayo_hall     # monitor serie a 115200
```
Salida: `{"raw":342,"voltage":1.672}` × 50 por segundo.

## 2. Dashboard
```powershell
node tools/visor_hall/serve.js
```
Abrir **http://localhost:8080** en **Chrome o Edge** → *Conectar (COM5)* → elegir COM5 en el selector.

> Cerrá el Monitor Serie del Arduino IDE antes de conectar: solo un programa puede tener el COM5 abierto.

## Notas
- Ningún filtrado en firmware ni en el dashboard: la curva es una polilínea recta entre muestras crudas.
- El eje temporal se reconstruye a 20 ms/muestra (el USB entrega en ráfagas); los valores no se modifican.
- Escala Y: Manual (por defecto 1.00–3.00 V) o Auto (min/max de la ventana). Unidad V o cuentas ADC.
- Con VCC del sensor a 3.3 V y ADC referenciado a 5 V (AVcc), la resolución es ~4.9 mV/cuenta y el valor
  depende de la tensión real del USB (≈4.8–5.1 V). Es normal ver ±1–3 cuentas de ruido.

