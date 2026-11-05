# 2026-j2-pressure-sensitive-stylus
Taller de Proyecto II 2026 — Grupo J2 — Pressure Sensitive Stylus

## Estructura

| Carpeta | Contenido |
|---|---|
| `firmware/madgwick_filter` | Firmware principal: orientación con Madgwick (β adaptativo 0,033 / 0,1) y desplazamiento con ZUPT. |
| `firmware/wifi_stream` | Transmisión Wi-Fi: SoftAP + WebSocket hacia la SPA. |
| `firmware/i2c_scan` | Escáner I2C para verificar el cableado del MPU6050. |
| `firmware/ensayo_beta_adaptativo_vs_fijo` | Ensayo: β adaptativo contra β fijo 0,1, dos filtros en paralelo sobre las mismas muestras. |
| `firmware/ensayo_desplazamiento_integracion_pura` | Ensayo: doble integración sin correcciones, con y sin Madgwick. |
| `tools/visor_orientacion_dibujo` | Visor (Processing): modelo 3D del sensor y dibujo por inclinación. |
| `tools/visor_trayectoria_3d` | Visor (Processing): orientación y trayectoria de desplazamiento de `madgwick_filter`. |
| `tools/visor_desplazamiento_integracion_pura` | Visor (Processing) del ensayo de desplazamiento; graba las corridas en `corridas/`. |
| `tools/analizar_desplazamiento.py` | Análisis de las corridas del ensayo de desplazamiento. |
| `webapp` | SPA de telemetría. |
| `docs/resultados/ensayos_iniciales_t1-t4` | Resultados de los ensayos iniciales (I2C, lectura cruda, calibración, Madgwick con β fijo y validación de ángulos). |
| `BITACORA.md` | Bitácora del proyecto. |
