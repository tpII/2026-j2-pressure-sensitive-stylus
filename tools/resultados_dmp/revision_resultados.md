# Revisión de las corridas DMP vs Madgwick (11/11)

Corridas que NO valen:
- 20261111_104403_reposo: |a| = 2 g en reposo (escala mal leída en la primera versión del firmware), todas las muestras saturadas.
- 20261111_111302_reposo: |a| = 0,95 g (misma causa).
- 20261111_113448_deslizar_10cm: vacía.
Válidas: 111527_reposo, 112542_caras (112254 es corta, 2 poses), 113713_deslizar_10cm, 114101_giros.

Cuidados al leerlas:
- Al abrir el puerto el ESP32 se reinicia, así que reposo, deslizar y giros incluyen los 20 s de arranque del DMP. En reposo se recalculó desde t = 25 s: DMP ruido 0,005°, deriva de yaw 0,25 °/min.
- Caras: el error de ~1,2° del DMP aparece porque la "verdad" es el acelerómetro corregido con calibracion_acc_dmp.json, que sale de esa misma corrida, y al DMP no se le puede aplicar. Sin esa corrección el DMP da 0,02°. En estático, los dos convergen a su propio acelerómetro: la exactitud la pone la calibración, no el filtro.
- Deslizar sin calibración: DMP 32 %, M giro 12 %, M β fijo 45–70 %. Con calibración: DMP 27 %, M giro 4,9 %.
