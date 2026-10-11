## 30-09-2026

**Actividades realizadas**

* Recepción de los materiales provistos por la cátedra para el desarrollo del proyecto:
  * Placa de desarrollo ESP32
  * Módulo sensor MPU6050
  * Protoboard
  * Cable USB
* Verificación física de los componentes recibidos.

<img src="./docs/img/materiales_recibidos.jpeg" width="200">

## 01-10-2026

**Actividades realizadas**

* Montaje del circuito en protoboard conectando el ESP32 con el MPU6050 vía I2C.
* Configuración del entorno de desarrollo en PlatformIO.
* Carga del firmware en el ESP32 y verificación del entorno.
* Desarrollo y ejecución de 4 tests unitarios/funcionales:
  * **Test I2C:** Verificación de comunicación exitosa y detección de ID correspondiente a MPU6050 original (dirección `0x68`).
  * **Test de lectura cruda:** Adquisición continua a 100 Hz estables con un jitter de ±1 µs.
  * **Test de calibración:** Medición de la deriva del giróscopo, registrando 30°/min sin corregir y reduciéndose a menos de 1°/min tras aplicar compensación de bias.
  * **Test de Madgwick:** Validación del filtro con un tiempo de cómputo de 25 µs por muestra, adoptando un factor β = 0.1.

## 03-10-2026

**Actividades realizadas**

* Se realizaron pruebas del firmware en un segundo ESP32 disponible, ya que el hardware principal no estaba accesible en ese momento.
* Fue necesario ajustar los pines de comunicación I2C ya que la asignación de pines difería entre las placas.
* Se detectó que el bus I2C fallaba de forma intermitente en la protoboard a alta velocidad. Se redujo la frecuencia de comunicación y la conexión se estabilizó.
* Se verificó mediante un programa de escaneo I2C que el MPU6050 es detectado correctamente en la dirección esperada.


## 05-10-2026

**Actividades realizadas**

* Se detectó un problema recurrente: luego de algunos movimientos bruscos, el firmware dejaba de responder y enviaba siempre los mismos valores por serial.
* Se identificó que la librería de comunicación I2C quedaba bloqueada ante errores sin ser detectados. Se agregó verificación de errores en cada lectura y una rutina de recuperación automática que reinicia el bus tras varios fallos consecutivos.
* Se agregó una espera al inicio para asegurarse de que el MPU6050 esté listo antes de continuar con la inicialización.

## 06-10-2026

**Actividades realizadas**

* Se desarrolló un visualizador 3D en Processing 4 para validar el movimiento del lápiz en pantalla contra el movimiento físico del MPU6050.
* El visualizador recibe la orientación por puerto serie y representa el lápiz como un objeto 3D rotando en tiempo real.
* Se implementó la funcionalidad de resetear la orientación de referencia en cualquier momento, útil para compensar la posición inicial del sensor.
* Al validar el movimiento con el visualizador se observó que el lápiz rotaba solo incluso cuando el sensor estaba quieto, lo que indicó un problema en la implementación del filtro.

## 08-10-2026

**Actividades realizadas**

* En reunión con el docente se identificó el error en la implementación del filtro de Madgwick: se usaba un intervalo de tiempo variable medido en cada iteración, cuando el filtro requiere un intervalo fijo definido por la frecuencia de muestreo. Usar un intervalo variable altera la ganancia del filtro y genera deriva artificial en los ángulos.
* Se releyó el paper de Madgwick, en particular el Apéndice A dedicado a la implementación optimizada en C, y se reescribió la función del filtro siguiendo esa referencia.
* Se ajustó el valor de β al recomendado en el paper para uso sin magnetómetro.

## 09-10-2026

**Actividades realizadas**
* Se agregó calibración de bias del giróscopo al inicio: se toman varias muestras con el sensor en reposo, se promedian y se restan en cada lectura antes de alimentar el filtro. Esto reduce la deriva significativamente.
* Con estos cambios el lápiz dejó de rotar solo durante los primeros minutos de uso. La deriva en yaw persiste a largo plazo, lo cual es esperable al no contar con magnetómetro.
* Se desarrolló un script de validación de ángulos con una secuencia guiada de posiciones conocidas (0°, ±45°, ±90°) en cada eje.
* Primera validación: los ejes estaban bien asignados, pero el filtro sobreestimaba entre 3° y 6°. Se identificó como causa un offset en el acelerómetro.
* Se incorporó la corrección de offset del acelerómetro en el firmware. Segunda validación: el error bajó a menos de 2° en el rango ±90°. El pitch en ±90° presenta singularidad (gimbal lock), lo cual es inherente a la representación de Euler.
* Se subió al repositorio el firmware y el visualizador.

**Pendientes**

* Iniciar el desarrollo de la lógica para la estimación del desplazamiento lineal.
* Evaluar la adopción de β = 0,1 (mejor resultado del ensayo de calibración del 01-10) frente al β sugerido en el paper de Madgwick, que es el que usa el firmware actual.

## 10-10-2026

**Actividades realizadas**

* Primer prototipo de estimación de desplazamiento: la aceleración se rota al marco del mundo con el cuaternión de Madgwick, se resta la gravedad (1 g en Z) y se integra dos veces (aceleración → velocidad → posición) a 100 Hz.
* Se implementó ZUPT (actualización de velocidad cero): cuando el sensor está en reposo, la velocidad se pone en cero para frenar la deriva de la integración.
* Configuración del MPU6050: filtro pasabajos digital (DLPF) en 44 Hz, giróscopo ±500 °/s y acelerómetro ±4 g, para reducir ruido y evitar saturación en movimientos rápidos.
* Temporización con período fijo de 10 ms sin acumulación de atraso, y puerto serie a 921600 baudios.
* La orientación inicial se calcula a partir del acelerómetro y se usa un β adaptativo: 0,033 en movimiento (valor sugerido por Madgwick) y 0,5 en reposo para corregir rápido la inclinación.
* El reposo se detecta con el módulo del acelerómetro (|a| ≈ 1 g) y el giro, promediados en ~0,1 s, de forma independiente de la orientación estimada. Antes, un error de orientación impedía detectar el reposo y la posición se desplazaba en línea recta.
* Mitigación de la deriva por doble integración: zona muerta de 0,02 g en la aceleración lineal y atenuación de la velocidad (factor 0,98 por muestra).
* Visualizador en Processing: se corrigió un cuelgue por acceso concurrente a la trayectoria desde el hilo serie, se agregó escala ajustable (+/-) y reinicio de orientación (tecla Z).
* Resultado: el lápiz digital sigue trazos cortos en el plano XY y se mantiene quieto en reposo. El eje Z presenta mayor deriva, al concentrar el residuo de la gravedad.

**Pendientes**

* Cuantificar el error de desplazamiento con trazos de longitud conocida.
* Evaluar la adopción de β = 0,1 (ensayo del 01-10) frente al β adaptativo actual.
* Medir el offset del eje X del acelerómetro o realizar una calibración en seis posiciones.
* Definir la orientación del sensor dentro del cuerpo del lápiz.

## 11-10-2026

**Actividades realizadas**

* Se desarrolló un visor de orientación en Processing 4 (`visor_lapiz`) que funciona con el firmware de ensayo `t4_madgwick` sin modificarlo: al conectarse espera el fin de la calibración y envía el comando `l` para recibir el cuaternión en formato CSV. Si solo llega la salida para Teleplot, reconstruye el cuaternión desde roll, pitch y yaw.
* El visor tiene dos paneles: un modelo 3D del módulo GY-521 con sus ejes, rotado según la orientación relativa a una pose de referencia (tecla R), y un lienzo donde se dibuja inclinando el sensor.
* En la primera versión el dibujo no respondía de forma intuitiva: el cursor seguía el ángulo de giro alrededor de cada eje, pero girar alrededor de un eje inclina el sensor en la dirección perpendicular, y el giro sobre el propio eje vertical también movía el cursor. Se reescribió como joystick: al tomar la referencia se detecta qué eje del sensor queda vertical y el cursor sigue la inclinación de ese eje. Así el giro sobre la vertical (y la deriva del yaw) no afecta el trazo.
* Se grabó el video demostrativo con el visor: respuesta del modelo 3D a rotaciones en cada eje, estabilidad al soltar el sensor y dejarlo en reposo, y un trazo de prueba por inclinación. El desplazamiento no se incluyó por no estar validado.


**Pendientes**

* Cuantificar el error de desplazamiento con trazos de longitud conocida.
* Evaluar la adopción de β = 0,1 (ensayo del 01-10) frente al β adaptativo actual.
* Ensamblaje mecánico del lápiz, pendiente de la entrega del sensor KY-024.