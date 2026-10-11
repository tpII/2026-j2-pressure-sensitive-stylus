// =============================================================================
// J2 - Pressure Sensitive Stylus  |  Visor de orientacion + dibujo por inclinacion
// Taller de Proyecto II - UNLP
//
// Lee por serie el firmware t4_madgwick (sin modificarlo):
//   - al conectar, espera la calibracion y manda "l" para pasar a modo CSV
//     (t_us,ax,ay,az,gx,gy,gz,q0,q1,q2,q3,roll,pitch,yaw,beta)
//   - si solo llegan lineas Teleplot (>roll/>pitch/>yaw) tambien funciona,
//     armando el cuaternion desde los angulos de Euler.
//
// IMPORTANTE: el trazo se controla con la INCLINACION (orientacion relativa),
// no con el desplazamiento real del lapiz ni con el sensor de contacto, que
// todavia no estan integrados. Por eso el lapiz se baja con la barra espaciadora.
//
// Teclas:
//   ESPACIO  bajar / levantar el lapiz          R  tomar referencia (pose "cero")
//   C        borrar el lienzo                   S  guardar captura PNG
//   M        cambiar que ejes mueven X / Y      X / Y  invertir eje horizontal / vertical
//   + / -    sensibilidad (px por grado)        G  recalibrar giroscopo (dejar quieto)
//   1..4     color del trazo                    [ / ]  grosor del trazo
//
// Cerrar el monitor serie de PlatformIO antes de abrir este sketch.
// =============================================================================

import processing.serial.*;

// ---------------- configuracion ----------------
final String PUERTO = "";       // "" = ultimo puerto de la lista; o por ejemplo "COM5"
final int    BAUD   = 921600;   // igual que monitor_speed en platformio.ini

// ---------------- estado de la orientacion ----------------
Serial esp;
volatile float[] q = {1, 0, 0, 0};   // cuaternion actual (w, x, y, z)
float[] qRef = {1, 0, 0, 0};          // pose de referencia
volatile float roll, pitch, yaw, betaFw = -1;
float trRoll, trPitch, trYaw;         // acumuladores para modo Teleplot
boolean simulacion = false;
boolean refTomada = false;
String modo = "conectando...";
String ultimoMsg = "";
int tConexion;
boolean pedidoCSV = false;
volatile boolean calibracionOK = false;

// tasa de muestras medida en el PC
volatile int muestras = 0;
int tVentana = 0;
float hz = 0;

// ---------------- dibujo ----------------
PGraphics vista3d, lienzo;
final int MARGEN = 20, ARRIBA = 78, ANCHO3D = 500;
int lienzoX, lienzoY, lienzoW, lienzoH;
boolean lapizAbajo = false;
float ganancia = 12;                  // px por grado de inclinacion
int mapeo = 0;
final int[][] PARES = {{0, 1}, {1, 0}, {0, 2}, {2, 0}, {1, 2}, {2, 1}};
final String[] NOMBRE_EJE = {"X", "Y", "Z"};
boolean invX = false, invY = true;
float cx, cy, px, py;
boolean primerPunto = true;
final float SUAVIZADO = 0.35;
final color[] COLORES = {color(25, 45, 110), color(190, 40, 40), color(30, 120, 60), color(20, 20, 20)};
int colorIdx = 0;
float grosor = 4;
float[] rvActual = {0, 0, 0};

PFont fuente, fuenteChica, fuenteTitulo;

void settings() {
  size(1280, 720, P3D);
  smooth(8);
}

void setup() {
  surface.setTitle("J2 - Pressure Sensitive Stylus | Visor");
  fuente = createFont("SansSerif", 14);
  fuenteChica = createFont("SansSerif", 12);
  fuenteTitulo = createFont("SansSerif.bold", 18);

  lienzoX = MARGEN + ANCHO3D + MARGEN;
  lienzoY = ARRIBA;
  lienzoW = width - lienzoX - MARGEN;
  lienzoH = height - ARRIBA - 60;

  vista3d = createGraphics(ANCHO3D, lienzoH, P3D);
  lienzo = createGraphics(lienzoW, lienzoH);
  borrarLienzo();
  cx = px = lienzoW / 2.0;
  cy = py = lienzoH / 2.0;

  conectar();
  tVentana = millis();
}

void conectar() {
  String[] lista = Serial.list();
  println("Puertos disponibles:");
  printArray(lista);
  String nombre = PUERTO;
  if (nombre.equals("") && lista.length > 0) nombre = lista[lista.length - 1];
  if (nombre.equals("")) {
    simulacion = true;
    modo = "SIMULACION (sin ESP32): mover el mouse";
    return;
  }
  try {
    esp = new Serial(this, nombre, BAUD);
    esp.bufferUntil('\n');
    modo = "conectado a " + nombre + ", esperando calibracion...";
    tConexion = millis();
  }
  catch (Exception e) {
    println("No se pudo abrir " + nombre + ": " + e.getMessage());
    simulacion = true;
    modo = "SIMULACION (no se pudo abrir " + nombre + ")";
  }
}

// ---------------- recepcion serie ----------------
void serialEvent(Serial p) {
  String s = p.readStringUntil('\n');
  if (s == null) return;
  s = trim(s);
  if (s.length() == 0) return;

  if (s.startsWith("#")) {                       // mensajes del firmware
    ultimoMsg = s.substring(1).trim();
    if (s.indexOf("Calibracion OK") >= 0) calibracionOK = true;
    return;
  }
  if (s.startsWith("t_us")) {                    // cabecera del modo CSV
    modo = "CSV (cuaternion del Madgwick)";
    return;
  }
  if (s.startsWith(">")) {                       // modo Teleplot
    int dp = s.indexOf(':');
    if (dp < 0) return;
    String k = s.substring(1, dp);
    float v = float(s.substring(dp + 1));
    if (Float.isNaN(v)) return;
    if (k.equals("roll")) trRoll = v;
    else if (k.equals("pitch")) trPitch = v;
    else if (k.equals("beta")) betaFw = v;
    else if (k.equals("yaw")) {
      trYaw = v;
      roll = trRoll; pitch = trPitch; yaw = trYaw;
      q = eulerACuat(roll, pitch, yaw);
      muestras++;
      if (!modo.startsWith("CSV")) modo = "Teleplot (Euler -> cuaternion)";
    }
    return;
  }
  String[] f = split(s, ',');                    // fila CSV
  if (f.length == 15) {
    float w = float(f[7]), x = float(f[8]), y = float(f[9]), z = float(f[10]);
    if (Float.isNaN(w) || Float.isNaN(z)) return;
    q = new float[] {w, x, y, z};
    roll = float(f[11]); pitch = float(f[12]); yaw = float(f[13]);
    betaFw = float(f[14]);
    muestras++;
  }
}

// ---------------- bucle principal ----------------
void draw() {
  background(24, 26, 30);

  // pasar el firmware a modo CSV una vez calibrado (o si ya venia corriendo)
  if (esp != null && !pedidoCSV && !modo.startsWith("CSV")) {
    if (calibracionOK || (muestras > 50 && millis() - tConexion > 3000)) {
      esp.write("l\n");
      pedidoCSV = true;
    }
  }

  if (simulacion) simularConMouse();

  // tasa medida
  if (millis() - tVentana >= 1000) {
    hz = muestras * 1000.0 / (millis() - tVentana);
    muestras = 0;
    tVentana = millis();
  }

  // primera referencia automatica cuando llegan datos
  if (!refTomada && (hz > 5 || simulacion)) tomarReferencia();

  float[] qRel = cuatMult(cuatConj(qRef), q);
  rvActual = vectorRotacionGrados(qRel);

  actualizarCursor();
  dibujarVista3D(qRel);

  image(vista3d, MARGEN, ARRIBA);
  image(lienzo, lienzoX, lienzoY);
  dibujarCursor();
  dibujarHUD();
}

// ---------------- dibujo por inclinacion ----------------
void actualizarCursor() {
  int a = PARES[mapeo][0], b = PARES[mapeo][1];
  float dx = rvActual[a] * ganancia * (invX ? -1 : 1);
  float dy = rvActual[b] * ganancia * (invY ? -1 : 1);
  float tx = constrain(lienzoW / 2.0 + dx, 0, lienzoW);
  float ty = constrain(lienzoH / 2.0 + dy, 0, lienzoH);
  cx = lerp(cx, tx, SUAVIZADO);
  cy = lerp(cy, ty, SUAVIZADO);

  if (lapizAbajo) {
    if (primerPunto) { px = cx; py = cy; primerPunto = false; }
    lienzo.beginDraw();
    lienzo.stroke(COLORES[colorIdx]);
    lienzo.strokeWeight(grosor);
    lienzo.strokeCap(ROUND);
    lienzo.line(px, py, cx, cy);
    lienzo.endDraw();
  }
  px = cx;
  py = cy;
}

void borrarLienzo() {
  lienzo.beginDraw();
  lienzo.background(250, 248, 242);
  lienzo.stroke(225, 222, 212);
  lienzo.strokeWeight(1);
  for (int x = 0; x < lienzoW; x += 40) lienzo.line(x, 0, x, lienzoH);
  for (int y = 0; y < lienzoH; y += 40) lienzo.line(0, y, lienzoW, y);
  lienzo.endDraw();
}

void dibujarCursor() {
  pushStyle();
  float x = lienzoX + cx, y = lienzoY + cy;
  noFill();
  strokeWeight(2);
  if (lapizAbajo) {
    stroke(COLORES[colorIdx]);
    fill(COLORES[colorIdx]);
    ellipse(x, y, grosor + 6, grosor + 6);
  } else {
    stroke(120);
    ellipse(x, y, 16, 16);
    line(x - 12, y, x - 5, y); line(x + 5, y, x + 12, y);
    line(x, y - 12, x, y - 5); line(x, y + 5, x, y + 12);
  }
  popStyle();
}

// ---------------- vista 3D del modulo ----------------
// Ejes del sensor -> ejes de Processing: X -> X, Y -> -Z, Z -> -Y
// (Processing tiene Y hacia abajo). Ese cambio de base es una reflexion,
// por eso el angulo cambia de signo.
void dibujarVista3D(float[] qr) {
  PGraphics g = vista3d;
  g.beginDraw();
  g.background(34, 37, 43);
  g.lights();
  g.translate(g.width / 2.0, g.height / 2.0 + 20, 0);
  g.rotateX(-0.45);
  g.rotateY(0.55);

  // grilla de referencia (plano de la mesa)
  g.stroke(70, 75, 85);
  g.strokeWeight(1);
  for (int i = -200; i <= 200; i += 40) {
    g.line(i, 90, -200, i, 90, 200);
    g.line(-200, 90, i, 200, 90, i);
  }

  float w = constrain(qr[0], -1, 1);
  float ang = 2 * acos(w);
  float s = sqrt(max(0, 1 - w * w));
  g.pushMatrix();
  if (s > 1e-5) {
    g.rotate(-ang, qr[1] / s, -qr[3] / s, -qr[2] / s);
  }
  // placa GY-521
  g.noStroke();
  g.fill(20, 90, 160);
  g.box(170, 8, 110);
  // chip MPU6050 (cara +Z del sensor)
  g.pushMatrix();
  g.translate(0, -8, 0);
  g.fill(25);
  g.box(40, 8, 40);
  g.popMatrix();
  // pines
  g.fill(210, 180, 60);
  for (int i = 0; i < 8; i++) {
    g.pushMatrix();
    g.translate(-70 + i * 20, -6, 48);
    g.box(6, 6, 6);
    g.popMatrix();
  }
  // ejes del sensor
  eje(g, 130, 0, 0, color(230, 70, 70), "X");
  eje(g, 0, 0, -130, color(80, 200, 100), "Y");
  eje(g, 0, -130, 0, color(80, 150, 255), "Z");
  g.popMatrix();
  g.endDraw();
}

void eje(PGraphics g, float x, float y, float z, color c, String nombre) {
  g.stroke(c);
  g.strokeWeight(4);
  g.line(0, 0, 0, x, y, z);
  g.pushMatrix();
  g.translate(x, y, z);
  g.noStroke();
  g.fill(c);
  g.sphere(6);
  g.fill(255);
  g.textFont(fuente);
  g.textSize(16);
  g.text(nombre, 8, -8);
  g.popMatrix();
}

// ---------------- textos ----------------
void dibujarHUD() {
  hint(DISABLE_DEPTH_TEST);
  pushStyle();
  fill(240);
  textFont(fuenteTitulo);
  textAlign(LEFT, BASELINE);
  text("J2 - Pressure Sensitive Stylus  |  Orientacion en vivo (Madgwick embebido en ESP32)", MARGEN, 30);

  textFont(fuente);
  fill(185);
  String datos = String.format("roll %7.1f   pitch %7.1f   yaw %7.1f  [grados]      tasa %5.1f Hz", roll, pitch, yaw, hz);
  if (betaFw > 0) datos += String.format("      beta %.3f", betaFw);
  text(datos, MARGEN, 54);
  textFont(fuenteChica);
  fill(130);
  text("Fuente: " + modo + (ultimoMsg.length() > 0 ? "   |   firmware: " + ultimoMsg : ""), MARGEN, 70);

  // titulos de los paneles
  fill(240);
  textFont(fuente);
  text("Modulo MPU6050 (relativo a la referencia)", MARGEN + 10, ARRIBA + 22);
  fill(200);
  textFont(fuenteChica);
  text(String.format("inclinacion relativa  X %6.1f   Y %6.1f   Z %6.1f", rvActual[0], rvActual[1], rvActual[2]),
    MARGEN + 10, ARRIBA + lienzoH - 12);

  // aviso sobre el lienzo
  fill(60);
  textFont(fuente);
  text("Trazo controlado por INCLINACION (" + NOMBRE_EJE[PARES[mapeo][0]] + " -> horizontal, "
    + NOMBRE_EJE[PARES[mapeo][1]] + " -> vertical)", lienzoX + 10, lienzoY + 22);
  fill(110);
  textFont(fuenteChica);
  text("Sin estimacion de desplazamiento ni sensor de contacto en esta etapa: el lapiz se baja con ESPACIO.",
    lienzoX + 10, lienzoY + 40);
  fill(lapizAbajo ? color(190, 40, 40) : color(110));
  textFont(fuente);
  textAlign(RIGHT, BASELINE);
  text(lapizAbajo ? "LAPIZ ABAJO" : "lapiz arriba", lienzoX + lienzoW - 12, lienzoY + 22);

  // ayuda
  textAlign(LEFT, BASELINE);
  textFont(fuenteChica);
  fill(150);
  text("ESPACIO lapiz   R referencia   C borrar   S captura   M ejes   X/Y invertir   +/- sensibilidad ("
    + nf(ganancia, 0, 0) + " px/grado)   G recalibrar   1-4 color   [ ] grosor",
    MARGEN, height - 22);

  if (simulacion) {
    fill(255, 190, 60);
    textAlign(RIGHT, BASELINE);
    text("MODO SIMULACION", width - MARGEN, 30);
  }
  popStyle();
  hint(ENABLE_DEPTH_TEST);
}

// ---------------- teclado ----------------
void keyPressed() {
  if (key == ' ') {
    lapizAbajo = !lapizAbajo;
    primerPunto = true;
  } else if (key == 'r' || key == 'R') {
    tomarReferencia();
  } else if (key == 'c' || key == 'C') {
    borrarLienzo();
  } else if (key == 's' || key == 'S') {
    String nombre = "captura_" + nf(year(), 4) + nf(month(), 2) + nf(day(), 2) + "_"
      + nf(hour(), 2) + nf(minute(), 2) + nf(second(), 2) + ".png";
    saveFrame(nombre);
    println("Guardado " + nombre);
  } else if (key == 'm' || key == 'M') {
    mapeo = (mapeo + 1) % PARES.length;
  } else if (key == 'x' || key == 'X') {
    invX = !invX;
  } else if (key == 'y' || key == 'Y') {
    invY = !invY;
  } else if (key == '+' || key == '=') {
    ganancia = min(ganancia + 2, 60);
  } else if (key == '-' || key == '_') {
    ganancia = max(ganancia - 2, 2);
  } else if (key == 'g' || key == 'G') {
    if (esp != null) esp.write("c\n");
    refTomada = false;
  } else if (key >= '1' && key <= '4') {
    colorIdx = key - '1';
  } else if (key == '[') {
    grosor = max(1, grosor - 1);
  } else if (key == ']') {
    grosor = min(20, grosor + 1);
  }
}

void tomarReferencia() {
  qRef = q.clone();
  refTomada = true;
  cx = px = lienzoW / 2.0;
  cy = py = lienzoH / 2.0;
  primerPunto = true;
}

// ---------------- simulacion sin hardware ----------------
void simularConMouse() {
  float r = map(mouseY, 0, height, -45, 45);
  float p = map(mouseX, 0, width, -45, 45);
  roll = r; pitch = p; yaw = 0;
  q = eulerACuat(r, p, 0);
  muestras++;
}

// ---------------- matematica de cuaterniones ----------------
float[] cuatMult(float[] a, float[] b) {
  return new float[] {
    a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
    a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
    a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
    a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0]
  };
}

float[] cuatConj(float[] a) {
  return new float[] {a[0], -a[1], -a[2], -a[3]};
}

// Vector de rotacion (eje * angulo) en grados. Para inclinaciones chicas cada
// componente es el giro alrededor de ese eje del sensor, sin singularidades.
float[] vectorRotacionGrados(float[] qr) {
  float w = qr[0], x = qr[1], y = qr[2], z = qr[3];
  if (w < 0) { w = -w; x = -x; y = -y; z = -z; }   // camino corto
  float s = sqrt(x*x + y*y + z*z);
  if (s < 1e-6) return new float[] {2 * degrees(x), 2 * degrees(y), 2 * degrees(z)};
  float ang = 2 * atan2(s, w);
  float k = degrees(ang) / s;
  return new float[] {x * k, y * k, z * k};
}

// Euler ZYX (grados) -> cuaternion, misma convencion que roll/pitch/yaw del firmware
float[] eulerACuat(float rDeg, float pDeg, float yDeg) {
  float r = radians(rDeg) / 2, p = radians(pDeg) / 2, y = radians(yDeg) / 2;
  float cr = cos(r), sr = sin(r), cp = cos(p), sp = sin(p), cyw = cos(y), syw = sin(y);
  return new float[] {
    cr*cp*cyw + sr*sp*syw,
    sr*cp*cyw - cr*sp*syw,
    cr*sp*cyw + sr*cp*syw,
    cr*cp*syw - sr*sp*cyw
  };
}
