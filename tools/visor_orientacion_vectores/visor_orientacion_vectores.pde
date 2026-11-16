// =============================================================================
// J2 - Pressure Sensitive Stylus  |  Visor de orientacion con vectores (version final)
// Taller de Proyecto II - UNLP
//
// Demostracion de la version final del lapiz: es el visor de orientacion del primer
// informe de avance (visor_orientacion_dibujo) con dos vectores agregados sobre el
// modelo 3D:
//   - GRAVEDAD (amarillo): lo que mide el acelerometro, dado vuelta. Con el lapiz
//     quieto apunta siempre hacia abajo: de ahi sale la inclinacion. Al sacudirlo
//     se aparta, porque el acelerometro mide tambien la aceleracion del movimiento.
//   - GIRO (magenta): velocidad de giro que mide el giroscopo. La flecha es el eje
//     de giro (regla de la mano derecha) y su largo, la velocidad; el arco alrededor
//     muestra el sentido en que esta girando el lapiz.
// El panel derecho dibuja por inclinacion (como en el primer informe) y baja el
// lapiz con el contacto del Hall; la barra espaciadora tambien lo baja.
//
// Firmware: firmware/lapiz_orientacion (Madgwick embebido en el ESP32, beta
// adaptativo 0,033 / 0,1). Lineas: O,q0,q1,q2,q3,ax,ay,az,gx,gy,gz,quieto,hall_mV,contacto
// Sin ESP32 conectado arranca en modo simulacion (mover el mouse).
//
// Teclas:
//   R  referencia: el modelo mira de frente y el cursor vuelve al centro
//   G  reiniciar la orientacion en el ESP32 (lapiz quieto)
//   C  borrar el lienzo        ESPACIO  bajar / levantar el lapiz a mano
//   H  usar / ignorar el Hall  M  cambiar que ejes mueven X / Y   X / Y  invertir
//   + / -  sensibilidad del dibujo      S  captura PNG
//
// Cerrar el monitor serie del Arduino IDE antes de abrir este sketch.
// =============================================================================

import processing.serial.*;

final String PUERTO = "COM3";   // "" = ultimo puerto de la lista
final int    BAUD   = 921600;
final float  ESC_G    = 110;    // px por g (vector gravedad)
final float  ESC_GIRO = 0.9;    // px por grado/s (vector giro)
final float  GIRO_MIN = 8;      // grados/s: por debajo no se dibuja el giro (ruido / pulso)

// ---------------- estado ----------------
Serial esp;
final Object lock = new Object();
float[] q = {1, 0, 0, 0};          // cuaternion actual (sensor -> mundo)
float[] qRef = {1, 0, 0, 0};       // referencia para el dibujo por inclinacion
float[] qRumbo = {1, 0, 0, 0};     // quita el rumbo (yaw) para que el modelo mire de frente
float[] acc = {0, 0, 1};           // acelerometro [g]
float[] giro = {0, 0, 0};          // giroscopo filtrado [grados/s]
boolean quieto = false, contactoHall = false, hayHall = false, usarHall = true;
float hallMV = 0, hallBase = 0;
boolean simulacion = false, refTomada = false;
String modo = "conectando...", ultimoMsg = "";
volatile int muestras = 0;
int tVentana = 0;
float hz = 0;

// ---------------- dibujo por inclinacion ----------------
PGraphics vista3d, lienzo;
final int MARGEN = 20, ARRIBA = 78, ANCHO3D = 640;
int lienzoX, lienzoY, lienzoW, lienzoH;
boolean lapizManual = false;
float ganancia = 12;                 // px por grado
int mapeo = 0;
final int[][] PARES = {{0, 1}, {1, 0}, {0, 2}, {2, 0}, {1, 2}, {2, 1}};
final String[] NOMBRE_EJE = {"X", "Y", "Z"};
boolean invX = false, invY = true;
float cx, cy, px, py;
boolean primerPunto = true;
float[] rvActual = {0, 0, 0};

PFont fuente, fuenteChica, fuenteTitulo;

void settings() {
  size(1300, 760, P3D);
  smooth(8);
}

void setup() {
  surface.setTitle("J2 - Pressure Sensitive Stylus | Orientacion con vectores");
  fuente = createFont("SansSerif", 14);
  fuenteChica = createFont("SansSerif", 12);
  fuenteTitulo = createFont("SansSerif.bold", 18);
  lienzoX = MARGEN + ANCHO3D + MARGEN;
  lienzoY = ARRIBA;
  lienzoW = width - lienzoX - MARGEN;
  lienzoH = height - ARRIBA - 50;
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
  printArray(lista);
  String nombre = PUERTO;
  if (nombre.equals("") && lista.length > 0) nombre = lista[lista.length - 1];
  if (nombre.equals("")) { simulacion = true; modo = "SIMULACION (sin ESP32): mover el mouse"; return; }
  try {
    esp = new Serial(this, nombre, BAUD);
    esp.bufferUntil('\n');
    modo = "conectado a " + nombre + ", esperando al firmware (lapiz QUIETO)...";
  }
  catch (Exception e) {
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
  synchronized (lock) {
    if (s.startsWith("#")) {
      int ib = s.indexOf("hall_base_mV=");
      if (ib >= 0) {
        String[] pz = splitTokens(s.substring(ib + 13), " ");
        if (pz.length > 0 && !Float.isNaN(float(pz[0]))) hallBase = float(pz[0]);
      }
      if (s.indexOf("Calibracion OK") >= 0) modo = "recibiendo datos";
      if (!s.startsWith("#cols") && !s.startsWith("#cfg")) ultimoMsg = s.substring(1).trim();
      return;
    }
    if (!s.startsWith("O,")) return;
    String[] f = split(s, ',');
    if (f.length != 15) return;
    float[] v = new float[14];
    for (int i = 0; i < 14; i++) {
      v[i] = float(f[i + 1]);
      if (Float.isNaN(v[i])) return;
    }
    q = new float[] {v[0], v[1], v[2], v[3]};
    acc = new float[] {v[4], v[5], v[6]};
    for (int i = 0; i < 3; i++) giro[i] += 0.3 * (v[7 + i] - giro[i]);   // suavizado para la vista
    quieto = v[10] > 0.5;
    hallMV = v[11];
    contactoHall = v[12] > 0.5;
    hayHall = hallBase > 300 && hallBase < 3000;
    muestras++;
  }
}

boolean lapizAbajo() {
  return lapizManual || (usarHall && hayHall && contactoHall);
}

// ---------------- bucle principal ----------------
void draw() {
  background(24, 26, 30);
  if (simulacion) simularConMouse();
  if (millis() - tVentana >= 1000) {
    hz = muestras * 1000.0 / (millis() - tVentana);
    muestras = 0;
    tVentana = millis();
  }
  float[] qAct, a, w;
  synchronized (lock) {
    if (!refTomada && (hz > 5 || simulacion)) tomarReferencia();
    qAct = q.clone(); a = acc.clone(); w = giro.clone();
  }
  rvActual = vectorRotacionGrados(cuatMult(cuatConj(qRef), qAct));
  actualizarCursor();
  dibujarVista3D(cuatMult(qRumbo, qAct), a, w);
  image(vista3d, MARGEN, ARRIBA);
  image(lienzo, lienzoX, lienzoY);
  dibujarCursor();
  dibujarHUD(qAct, a, w);
}

// ---------------- vista 3D ----------------
// Ejes del sensor (y del mundo) -> ejes de Processing: X -> X, Y -> -Z, Z -> -Y
// (Processing tiene Y hacia abajo). Es una reflexion, por eso el angulo cambia de signo.
float[] aP(float[] v, float esc) {
  return new float[] {v[0] * esc, -v[2] * esc, -v[1] * esc};
}

void dibujarVista3D(float[] qd, float[] a, float[] w) {
  PGraphics g = vista3d;
  g.beginDraw();
  g.background(34, 37, 43);
  g.lights();
  g.translate(g.width / 2.0, g.height / 2.0 + 10, 0);
  g.rotateX(-0.42);
  g.rotateY(0.6);

  // plano de la mesa (horizontal del mundo)
  g.stroke(70, 75, 85);
  g.strokeWeight(1);
  for (int i = -220; i <= 220; i += 40) {
    g.line(i, 150, -220, i, 150, 220);
    g.line(-220, 150, i, 220, 150, i);
  }

  float ww = constrain(qd[0], -1, 1);
  float ang = 2 * acos(ww);
  float s = sqrt(max(0, 1 - ww * ww));
  g.pushMatrix();
  if (s > 1e-5) g.rotate(-ang, qd[1] / s, -qd[3] / s, -qd[2] / s);

  // placa GY-521
  g.noStroke();
  g.fill(20, 90, 160);
  g.box(170, 8, 110);
  g.pushMatrix();
  g.translate(0, -8, 0);
  g.fill(25);
  g.box(40, 8, 40);
  g.popMatrix();
  g.fill(210, 180, 60);
  for (int i = 0; i < 8; i++) {
    g.pushMatrix();
    g.translate(-70 + i * 20, -6, 48);
    g.box(6, 6, 6);
    g.popMatrix();
  }
  // ejes del sensor (finos, para no tapar los vectores)
  eje(g, new float[] {110, 0, 0}, color(230, 70, 70), "X", 2);
  eje(g, new float[] {0, 0, -110}, color(80, 200, 100), "Y", 2);
  eje(g, new float[] {0, -110, 0}, color(80, 150, 255), "Z", 2);

  // GRAVEDAD: el acelerometro mide +1 g hacia arriba en reposo; la gravedad es -a
  float[] gv = aP(new float[] {-a[0], -a[1], -a[2]}, ESC_G);
  flecha(g, gv, color(250, 205, 60), "gravedad", 5);

  // GIRO: eje (mano derecha) y largo = velocidad; arco = sentido de giro
  float wn = sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
  if (wn > GIRO_MIN) {
    float largo = min(wn * ESC_GIRO, 230);
    float[] eje = {w[0] / wn, w[1] / wn, w[2] / wn};
    float[] wv = aP(eje, largo);
    flecha(g, wv, color(235, 80, 220), "giro", 5);
    arcoGiro(g, eje, largo, color(235, 80, 220));
  }
  g.popMatrix();
  g.endDraw();
}

void eje(PGraphics g, float[] p, color c, String nombre, float grosor) {
  g.stroke(c);
  g.strokeWeight(grosor);
  g.line(0, 0, 0, p[0], p[1], p[2]);
  g.pushMatrix();
  g.translate(p[0], p[1], p[2]);
  g.fill(c);
  g.textFont(fuenteChica);
  g.text(nombre, 6, -6);
  g.popMatrix();
}

void flecha(PGraphics g, float[] p, color c, String nombre, float grosor) {
  g.stroke(c);
  g.strokeWeight(grosor);
  g.line(0, 0, 0, p[0], p[1], p[2]);
  g.pushMatrix();
  g.translate(p[0], p[1], p[2]);
  g.noStroke();
  g.fill(c);
  g.sphere(7);
  g.textFont(fuente);
  g.text(nombre, 10, -10);
  g.popMatrix();
}

// Arco de 270 grados alrededor del eje de giro, cerca de la punta del vector, con
// una punta de flecha: muestra hacia donde gira (se arma en ejes del sensor y se
// pasa a Processing con la misma transformacion que el resto).
void arcoGiro(PGraphics g, float[] e, float largo, color c) {
  float[] ref = abs(e[0]) < 0.9 ? new float[] {1, 0, 0} : new float[] {0, 1, 0};
  float[] u = normal(cruz(e, ref));
  float[] v = cruz(e, u);                       // (u, v, e) de mano derecha
  float r = 28, d = largo * 0.7;
  g.noFill();
  g.stroke(c);
  g.strokeWeight(3);
  g.beginShape();
  float[] ult = null, prev = null;
  for (int k = 0; k <= 30; k++) {
    float t = k / 30.0 * 1.5 * PI;
    float[] pt = {e[0] * d + r * (cos(t) * u[0] + sin(t) * v[0]),
                  e[1] * d + r * (cos(t) * u[1] + sin(t) * v[1]),
                  e[2] * d + r * (cos(t) * u[2] + sin(t) * v[2])};
    float[] pp = aP(pt, 1);
    g.vertex(pp[0], pp[1], pp[2]);
    prev = ult;
    ult = pp;
  }
  g.endShape();
  if (prev != null) {                           // punta de flecha al final del arco
    g.pushMatrix();
    g.translate(ult[0], ult[1], ult[2]);
    g.noStroke();
    g.fill(c);
    g.sphere(5);
    g.popMatrix();
  }
}

float[] cruz(float[] a, float[] b) {
  return new float[] {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

float[] normal(float[] a) {
  float n = sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
  return new float[] {a[0] / n, a[1] / n, a[2] / n};
}

// ---------------- dibujo por inclinacion ----------------
void actualizarCursor() {
  int ia = PARES[mapeo][0], ib = PARES[mapeo][1];
  float dx = rvActual[ia] * ganancia * (invX ? -1 : 1);
  float dy = rvActual[ib] * ganancia * (invY ? -1 : 1);
  float tx = constrain(lienzoW / 2.0 + dx, 0, lienzoW);
  float ty = constrain(lienzoH / 2.0 + dy, 0, lienzoH);
  cx = lerp(cx, tx, 0.35);
  cy = lerp(cy, ty, 0.35);
  if (lapizAbajo()) {
    if (primerPunto) { px = cx; py = cy; primerPunto = false; }
    lienzo.beginDraw();
    lienzo.stroke(25, 45, 110);
    lienzo.strokeWeight(4);
    lienzo.strokeCap(ROUND);
    lienzo.line(px, py, cx, cy);
    lienzo.endDraw();
  } else primerPunto = true;
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
  hint(DISABLE_DEPTH_TEST);
  pushStyle();
  float x = lienzoX + cx, y = lienzoY + cy;
  strokeWeight(2);
  if (lapizAbajo()) { stroke(25, 45, 110); fill(25, 45, 110); ellipse(x, y, 10, 10); }
  else {
    noFill(); stroke(120);
    ellipse(x, y, 16, 16);
    line(x - 12, y, x - 5, y); line(x + 5, y, x + 12, y);
    line(x, y - 12, x, y - 5); line(x, y + 5, x, y + 12);
  }
  popStyle();
  hint(ENABLE_DEPTH_TEST);
}

// ---------------- textos ----------------
void dibujarHUD(float[] qa, float[] a, float[] w) {
  hint(DISABLE_DEPTH_TEST);
  pushStyle();
  textAlign(LEFT, BASELINE);
  fill(240);
  textFont(fuenteTitulo);
  text("J2 - Pressure Sensitive Stylus  |  Orientacion (Madgwick embebido en el ESP32)", MARGEN, 30);
  textFont(fuente);
  fill(185);
  float an = sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
  float wn = sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
  // inclinacion del lapiz: angulo entre su eje (Y del sensor, hacia arriba) y la vertical
  float incl = degrees(acos(constrain(2 * (qa[2] * qa[3] + qa[0] * qa[1]), -1, 1)));
  text(String.format("|a| %.2f g    giro %5.0f grados/s    inclinacion del lapiz %5.1f grados    %s    %.0f Hz",
    an, wn, incl, quieto ? "QUIETO (beta 0,1)" : "en movimiento (beta 0,033)", hz), MARGEN, 52);
  textFont(fuenteChica);
  fill(130);
  text("Fuente: " + modo + (ultimoMsg.length() > 0 ? "   |   firmware: " + ultimoMsg : ""), MARGEN, 68);

  // leyenda de la vista 3D
  int y0 = ARRIBA + 22;
  fill(240); textFont(fuente);
  text("Lapiz (MPU6050) - orientacion absoluta, rumbo fijado con R", MARGEN + 10, y0);
  textFont(fuenteChica);
  fill(250, 205, 60);
  text("amarillo: gravedad segun el acelerometro (quieto = vertical hacia abajo)", MARGEN + 10, y0 + 18);
  fill(235, 80, 220);
  text("magenta: giro segun el giroscopo (eje y velocidad; el arco marca el sentido)", MARGEN + 10, y0 + 34);
  fill(200);
  text(String.format("acelerometro [g]  %6.2f %6.2f %6.2f      giro [grados/s]  %6.0f %6.0f %6.0f",
    a[0], a[1], a[2], w[0], w[1], w[2]), MARGEN + 10, ARRIBA + lienzoH - 12);

  // lienzo
  fill(60); textFont(fuente);
  text("Dibujo por INCLINACION (" + NOMBRE_EJE[PARES[mapeo][0]] + " -> horizontal, "
    + NOMBRE_EJE[PARES[mapeo][1]] + " -> vertical)", lienzoX + 10, lienzoY + 22);
  textFont(fuenteChica);
  fill(110);
  String hall = !hayHall ? "Hall no detectado: el lapiz se baja con ESPACIO"
    : (!usarHall ? String.format("Hall ignorado (%.0f mV): el lapiz se baja con ESPACIO", hallMV)
    : String.format("Hall %.0f mV (base %.0f): %s", hallMV, hallBase, contactoHall ? "CONTACTO" : "sin contacto"));
  text(hall, lienzoX + 10, lienzoY + 40);
  textAlign(RIGHT, BASELINE);
  textFont(fuente);
  fill(lapizAbajo() ? color(190, 40, 40) : color(110));
  text(lapizAbajo() ? "LAPIZ ABAJO" : "lapiz arriba", lienzoX + lienzoW - 12, lienzoY + 22);

  textAlign(LEFT, BASELINE);
  textFont(fuenteChica);
  fill(150);
  text("R referencia   G reiniciar orientacion   C borrar   ESPACIO lapiz   H usar/ignorar Hall   M ejes   X/Y invertir   +/- sensibilidad ("
    + nf(ganancia, 0, 0) + " px/grado)   S captura", MARGEN, height - 18);
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
  char k = Character.toLowerCase(key);
  if (key == ' ') { lapizManual = !lapizManual; primerPunto = true; }
  else if (k == 'r') { synchronized (lock) { tomarReferencia(); } }
  else if (k == 'g') { if (esp != null) esp.write("z"); refTomada = false; }
  else if (k == 'c') borrarLienzo();
  else if (k == 'h') usarHall = !usarHall;
  else if (k == 'm') mapeo = (mapeo + 1) % PARES.length;
  else if (k == 'x') invX = !invX;
  else if (k == 'y') invY = !invY;
  else if (key == '+' || key == '=') ganancia = min(ganancia + 2, 60);
  else if (key == '-' || key == '_') ganancia = max(ganancia - 2, 2);
  else if (k == 's') {
    String n = "captura_" + nf(year(), 4) + nf(month(), 2) + nf(day(), 2) + "_" + nf(hour(), 2) + nf(minute(), 2) + nf(second(), 2) + ".png";
    saveFrame(n);
    println("Guardado " + n);
  }
}

// Referencia: el cursor vuelve al centro y se quita el rumbo actual (yaw), asi el
// modelo 3D queda de frente sin perder la inclinacion real.
void tomarReferencia() {
  qRef = q.clone();
  float yaw = atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3]));
  qRumbo = new float[] {cos(-yaw / 2), 0, 0, sin(-yaw / 2)};
  refTomada = true;
  cx = px = lienzoW / 2.0;
  cy = py = lienzoH / 2.0;
  primerPunto = true;
}

// ---------------- simulacion sin hardware ----------------
float[] qSimAnt = {1, 0, 0, 0};
void simularConMouse() {
  float r = radians(map(mouseY, 0, height, -60, 60)), p = radians(map(mouseX, 0, width, -60, 60));
  float[] qn = cuatMult(new float[] {cos(r / 2), sin(r / 2), 0, 0}, new float[] {cos(p / 2), 0, sin(p / 2), 0});
  synchronized (lock) {
    // giro: diferencia entre muestras, en ejes del sensor (dq = q_ant* x q_nuevo)
    float[] dq = cuatMult(cuatConj(qSimAnt), qn);
    float fps = max(frameRate, 1);
    for (int i = 0; i < 3; i++) giro[i] += 0.3 * (degrees(2 * dq[i + 1]) * fps - giro[i]);
    q = qn;
    // acelerometro quieto: la vertical del mundo vista en ejes del sensor
    acc = new float[] {2 * (qn[1] * qn[3] - qn[0] * qn[2]), 2 * (qn[0] * qn[1] + qn[2] * qn[3]),
                       qn[0] * qn[0] - qn[1] * qn[1] - qn[2] * qn[2] + qn[3] * qn[3]};
    qSimAnt = qn;
    muestras++;
  }
}

// ---------------- cuaterniones ----------------
float[] cuatMult(float[] a, float[] b) {
  return new float[] {
    a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
    a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
    a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
    a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0]
  };
}

float[] cuatConj(float[] a) { return new float[] {a[0], -a[1], -a[2], -a[3]}; }

float[] vectorRotacionGrados(float[] qr) {
  float w = qr[0], x = qr[1], y = qr[2], z = qr[3];
  if (w < 0) { w = -w; x = -x; y = -y; z = -z; }
  float s = sqrt(x*x + y*y + z*z);
  if (s < 1e-6) return new float[] {2 * degrees(x), 2 * degrees(y), 2 * degrees(z)};
  float k = degrees(2 * atan2(s, w)) / s;
  return new float[] {x * k, y * k, z * k};
}
