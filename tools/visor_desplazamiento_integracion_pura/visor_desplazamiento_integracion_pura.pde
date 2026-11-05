// =============================================================================
// J2 - Pressure Sensitive Stylus  |  Visor de DESPLAZAMIENTO (integracion pura)
// Taller de Proyecto II - UNLP
//
// Se usa con el firmware firmware/ensayo_desplazamiento_integracion_pura. Muestra en tiempo real,
// vista desde arriba (plano de la mesa), las dos posiciones que calcula el
// firmware con las mismas muestras y SIN correcciones (la integracion no se
// frena nunca, solo se reinicia con C):
//   A (rojo)  CON MADGWICK: aceleracion rotada al mundo menos la gravedad.
//   B (azul)  SIN MADGWICK: aceleracion menos la gravedad fija de la referencia
//             (solo valido si el sensor no se inclina).
// Queda el historial completo de ambas desde la ultima referencia, y un grafico
// de la distancia al origen en el tiempo, con curvas de referencia de lo que
// produciria un error constante de aceleracion de 1 y 10 mg.
//
// Cada referencia (tecla C) abre una CORRIDA nueva que se graba completa en
// corridas/AAAAMMDD_HHMMSS_<etiqueta>.csv (carpeta del sketch), con todas las
// muestras, incluida la lectura cruda del sensor. Esos archivos se analizan con
// tools/analizar_desplazamiento.py.
//
// Teclas:
//   E        etiqueta de la PROXIMA corrida (reposo / trazo_5cm / trazo_10cm / trazo_20cm / libre)
//   C        nueva referencia (sensor QUIETO 1 s) y nueva corrida
//   M        marca en la corrida (por ejemplo, inicio / fin de un movimiento)
//   1 2      mostrar / ocultar A, B
//   F        encuadrar las trayectorias      + / -  zoom      S  captura PNG
//
// Cerrar el monitor serie del Arduino IDE antes de abrir este sketch.
// =============================================================================

import processing.serial.*;

// ---------------- configuracion ----------------
final String PUERTO = "";       // "" = ultimo puerto de la lista; o por ejemplo "COM5"
final int    BAUD   = 921600;
final float  INCL_MAX = 3.0;    // [grados] por encima B deja de ser valido
final String[] ETIQUETAS = {"reposo", "trazo_5cm", "trazo_10cm", "trazo_20cm", "libre"};
final int    MAX_MUESTRAS = 180000;   // 30 min a 100 Hz

// ---------------- datos (los escribe el hilo serie) ----------------
Serial esp;
final Object lock = new Object();
ArrayList<float[]> hist = new ArrayList<float[]>();   // {t_s, Au, Av, Bu, Bv}
float Au, Av, An, Bu, Bv, Bn, incl, tSeg;
float inclMax = 0;
boolean verA = true, verB = true;

// corrida (archivo)
PrintWriter corrida = null;
String nombreCorrida = "";
int etiquetaIdx = 0;
int marcas = 0;

// estadisticas del firmware
long muestras = 0, satTotal = 0, perdidas = 0;
long ultimoT = -1;
float cicloMax = 0, cicloSuma = 0;
int muestrasVentana = 0, tVentana = 0;
float hz = 0;

String modo = "conectando...";
String ultimoMsg = "";

// ---------------- vista ----------------
float escala = 12;              // px por cm
float centroU = 0, centroV = 0; // cm en el centro del lienzo
PFont fuente, fuenteChica, fuenteTitulo, fuenteMono;
final int PANEL = 440;

void settings() {
  size(1300, 780);
  smooth(4);
}

void setup() {
  surface.setTitle("J2 - Pressure Sensitive Stylus | Desplazamiento");
  fuente = createFont("SansSerif", 14);
  fuenteChica = createFont("SansSerif", 12);
  fuenteTitulo = createFont("SansSerif.bold", 18);
  fuenteMono = createFont("Monospaced", 13);
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
    modo = "SIN ESP32: no hay puertos serie";
    return;
  }
  try {
    esp = new Serial(this, nombre, BAUD);
    esp.bufferUntil('\n');
    modo = "conectado a " + nombre + ", esperando calibracion (sensor QUIETO)...";
  }
  catch (Exception e) {
    modo = "no se pudo abrir " + nombre + ": " + e.getMessage();
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
      if (s.indexOf("Referencia tomada") >= 0) nuevaCorrida();
      if (corrida != null) corrida.println(s);
      if (!s.startsWith("#cols") && !s.startsWith("#cfg")) ultimoMsg = s.substring(1).trim();
      if (s.indexOf("Calibracion OK") >= 0) modo = "recibiendo datos";
      return;
    }
    if (!s.startsWith("D,")) return;
    String[] f = split(s, ',');
    if (f.length != 17) return;
    float[] v = new float[16];
    for (int i = 0; i < 16; i++) {
      v[i] = float(f[i + 1]);
      if (Float.isNaN(v[i])) return;
    }
    if (corrida != null) corrida.println(s);

    long t = (long) v[0];
    tSeg = t / 1000.0;
    Au = v[1]; Av = v[2]; An = v[3];
    Bu = v[4]; Bv = v[5]; Bn = v[6];
    incl = v[7];
    inclMax = max(inclMax, incl);
    if (v[8] > 0.5) satTotal++;
    cicloMax = max(cicloMax, v[9]);
    cicloSuma += v[9];
    muestras++; muestrasVentana++;
    if (ultimoT >= 0 && t - ultimoT > 15) perdidas += round((t - ultimoT) / 10.0) - 1;
    ultimoT = t;

    hist.add(new float[] {tSeg, Au, Av, Bu, Bv});
    if (hist.size() > MAX_MUESTRAS) hist.remove(0);
  }
}

// Se llama con el lock tomado
void nuevaCorrida() {
  cerrarCorrida();
  hist.clear();
  muestras = 0; satTotal = 0; perdidas = 0; ultimoT = -1;
  cicloMax = 0; cicloSuma = 0; inclMax = 0; marcas = 0;
  Au = Av = An = Bu = Bv = Bn = 0; tSeg = 0;
  centroU = 0; centroV = 0;
  String sello = nf(year(), 4) + nf(month(), 2) + nf(day(), 2) + "_" + nf(hour(), 2) + nf(minute(), 2) + nf(second(), 2);
  nombreCorrida = "corridas/" + sello + "_" + ETIQUETAS[etiquetaIdx] + ".csv";
  corrida = createWriter(nombreCorrida);
  corrida.println("#etiqueta=" + ETIQUETAS[etiquetaIdx]);
  println("Corrida nueva: " + nombreCorrida);
}

void cerrarCorrida() {
  if (corrida == null) return;
  corrida.flush();
  corrida.close();
  corrida = null;
  println("Corrida guardada: " + nombreCorrida + " (" + muestras + " muestras)");
}

void exit() {
  synchronized (lock) {
    cerrarCorrida();
  }
  super.exit();
}

// ---------------- dibujo ----------------
void draw() {
  background(24, 26, 30);
  if (millis() - tVentana >= 1000) {
    hz = muestrasVentana * 1000.0 / (millis() - tVentana);
    muestrasVentana = 0;
    tVentana = millis();
  }
  synchronized (lock) {
    dibujarLienzo();
    dibujarPanel();
    dibujarGrafico();
  }
  dibujarEncabezado();
}

float pantallaX(float u, float cx) { return cx + (u - centroU) * escala; }
float pantallaY(float v, float cy) { return cy - (v - centroV) * escala; }

void dibujarLienzo() {
  int x0 = 20, y0 = 80, w = width - PANEL - 40, h = height - y0 - 50;
  pushStyle();
  fill(250, 248, 242);
  noStroke();
  rect(x0, y0, w, h);

  float cx = x0 + w / 2.0, cy = y0 + h / 2.0;
  // grilla: paso de 1, 5, 10, 50... cm segun el zoom (lineas separadas >= 8 px)
  float paso = 1;
  while (paso * escala < 8) paso *= (str(paso).startsWith("1") ? 5 : 2);
  float uMin = centroU - (w / 2.0) / escala, uMax = centroU + (w / 2.0) / escala;
  float vMin = centroV - (h / 2.0) / escala, vMax = centroV + (h / 2.0) / escala;
  strokeWeight(1);
  for (float u = floor(uMin / paso) * paso; u <= uMax; u += paso) {
    stroke(round(u / paso) % 5 == 0 ? color(200, 196, 185) : color(232, 229, 220));
    line(pantallaX(u, cx), y0, pantallaX(u, cx), y0 + h);
  }
  for (float v = floor(vMin / paso) * paso; v <= vMax; v += paso) {
    stroke(round(v / paso) % 5 == 0 ? color(200, 196, 185) : color(232, 229, 220));
    line(x0, pantallaY(v, cy), x0 + w, pantallaY(v, cy));
  }
  // origen (posicion en la referencia)
  stroke(90);
  strokeWeight(1.5);
  float ox = pantallaX(0, cx), oy = pantallaY(0, cy);
  line(ox - 10, oy, ox + 10, oy);
  line(ox, oy - 10, ox, oy + 10);

  clip(x0, y0, w, h);
  if (verB) dibujarTray(3, cx, cy, color(40, 90, 200));
  if (verA) dibujarTray(1, cx, cy, color(200, 50, 50));
  noStroke();
  if (verB) { fill(40, 90, 200); ellipse(pantallaX(Bu, cx), pantallaY(Bv, cy), 10, 10); }
  if (verA) { fill(200, 50, 50); ellipse(pantallaX(Au, cx), pantallaY(Av, cy), 10, 10); }
  noClip();

  // barra de escala y leyenda
  fill(60);
  textFont(fuenteChica);
  textAlign(LEFT, BASELINE);
  stroke(60);
  strokeWeight(2);
  float barra = paso * 5;
  line(x0 + 15, y0 + h - 15, x0 + 15 + barra * escala, y0 + h - 15);
  noStroke();
  text(nf(barra, 0, 0) + " cm", x0 + 20 + barra * escala, y0 + h - 11);
  fill(200, 50, 50);
  text("A con Madgwick", x0 + 15, y0 + 20);
  fill(40, 90, 200);
  text("B sin Madgwick (gravedad fija)", x0 + 15, y0 + 38);
  if (incl > INCL_MAX) {
    fill(210, 120, 0);
    textAlign(RIGHT, BASELINE);
    textFont(fuente);
    text(String.format("inclinado %.1f grados: B no es valido", incl), x0 + w - 15, y0 + 22);
  }
  popStyle();
}

// col: 1 = A (Au, Av), 3 = B (Bu, Bv). Se saltean puntos a menos de 1 px.
void dibujarTray(int col, float cx, float cy, color c) {
  noFill();
  stroke(c);
  strokeWeight(2.2);
  beginShape();
  float px = -1e9, py = -1e9;
  for (int i = 0; i < hist.size(); i++) {
    float[] p = hist.get(i);
    float x = pantallaX(p[col], cx), y = pantallaY(p[col + 1], cy);
    if (abs(x - px) < 1 && abs(y - py) < 1 && i < hist.size() - 1) continue;
    vertex(x, y);
    px = x; py = y;
  }
  endShape();
}

void dibujarPanel() {
  int x0 = width - PANEL - 10, y = 80;
  pushStyle();
  textAlign(LEFT, BASELINE);
  fill(240);
  textFont(fuente);
  text(String.format("Posicion [cm]   t = %.1f s desde la referencia", tSeg), x0, y);
  textFont(fuenteMono);
  fill(150);
  text("        u        v   vertical   |plano|", x0, y + 20);
  fill(230, 110, 110);
  text(String.format("A %8.2f %8.2f %9.2f %9.2f", Au, Av, An, sqrt(Au * Au + Av * Av)), x0, y + 38);
  fill(120, 160, 240);
  text(String.format("B %8.2f %8.2f %9.2f %9.2f", Bu, Bv, Bn, sqrt(Bu * Bu + Bv * Bv)), x0, y + 56);
  fill(incl > INCL_MAX ? color(230, 140, 50) : color(185));
  text(String.format("inclinacion %4.1f grados (max %4.1f)", incl, inclMax), x0, y + 76);

  y += 112;
  fill(240);
  textFont(fuente);
  text("Corrida", x0, y);
  textFont(fuenteMono);
  fill(215);
  text("proxima etiqueta: " + ETIQUETAS[etiquetaIdx] + " (tecla E)", x0, y + 20);
  fill(corrida != null ? color(110, 200, 130) : color(150));
  text(corrida != null ? "grabando " + nombreCorrida.substring(9) : "sin grabar", x0, y + 38);
  fill(185);
  text(String.format("%d muestras, %d marcas (tecla M)", muestras, marcas), x0, y + 56);

  y += 90;
  fill(240);
  textFont(fuente);
  text("ESP32 / MPU6050", x0, y);
  textFont(fuenteMono);
  fill(215);
  float cicloMed = muestras > 0 ? cicloSuma / muestras : 0;
  text(String.format("tasa %5.1f Hz  ciclo %4.0f / %.0f us", hz, cicloMed, cicloMax), x0, y + 20);
  fill(perdidas > 0 || satTotal > 0 ? color(230, 120, 60) : color(215));
  text(String.format("perdidas %d  saturadas %d", perdidas, satTotal), x0, y + 38);
  popStyle();
}

// Distancia al origen en el plano vs tiempo, con curvas de error constante
// de aceleracion: d = 1/2 * b * g * t^2 (1 mg y 10 mg)
void dibujarGrafico() {
  int x0 = width - PANEL - 10, y0 = 380, w = PANEL - 10, h = height - y0 - 60;
  pushStyle();
  fill(240);
  textFont(fuente);
  textAlign(LEFT, BASELINE);
  text("Distancia al origen en el plano [cm] vs t [s]", x0, y0 - 8);
  fill(34, 37, 43);
  noStroke();
  rect(x0, y0, w, h);

  float tMax = max(2, tSeg);
  float dMax = 1;
  for (float[] p : hist) {
    if (verA) dMax = max(dMax, sqrt(p[1] * p[1] + p[2] * p[2]));
    if (verB) dMax = max(dMax, sqrt(p[3] * p[3] + p[4] * p[4]));
  }
  dMax *= 1.1;

  // curvas de referencia
  float[] mg = {1, 10};
  for (int k = 0; k < mg.length; k++) {
    stroke(110);
    strokeWeight(1);
    float ax = -1, ay = -1;
    for (int i = 0; i <= 60; i++) {
      float t = tMax * i / 60.0;
      float d = 0.5 * mg[k] * 0.001 * 9.81 * t * t * 100;   // cm
      if (d > dMax) break;
      float x = x0 + w * t / tMax, y = y0 + h - h * d / dMax;
      if (ax >= 0 && i % 2 == 0) line(ax, ay, x, y);
      ax = x; ay = y;
    }
    fill(140);
    textFont(fuenteChica);
    float tEtiq = min(tMax, sqrt(2 * dMax * 0.85 / (mg[k] * 0.001 * 9.81 * 100)));
    float dEtiq = 0.5 * mg[k] * 0.001 * 9.81 * tEtiq * tEtiq * 100;
    text(nf(mg[k], 0, 0) + " mg", x0 + w * tEtiq / tMax - 34, y0 + h - h * dEtiq / dMax);
  }

  int paso = max(1, hist.size() / w);
  if (verB) dibujarCurva(3, x0, y0, w, h, tMax, dMax, paso, color(90, 140, 240));
  if (verA) dibujarCurva(1, x0, y0, w, h, tMax, dMax, paso, color(230, 90, 90));

  fill(150);
  textFont(fuenteChica);
  text(nf(dMax, 0, 1) + " cm", x0 + 4, y0 + 14);
  textAlign(RIGHT, BASELINE);
  text(nf(tMax, 0, 1) + " s", x0 + w - 4, y0 + h - 4);
  popStyle();
}

void dibujarCurva(int col, int x0, int y0, int w, int h, float tMax, float dMax, int paso, color c) {
  noFill();
  stroke(c);
  strokeWeight(1.8);
  beginShape();
  for (int i = 0; i < hist.size(); i += paso) {
    float[] p = hist.get(i);
    float d = sqrt(p[col] * p[col] + p[col + 1] * p[col + 1]);
    vertex(x0 + w * p[0] / tMax, y0 + h - h * d / dMax);
  }
  endShape();
}

void dibujarEncabezado() {
  pushStyle();
  textAlign(LEFT, BASELINE);
  fill(240);
  textFont(fuenteTitulo);
  text("J2 - Pressure Sensitive Stylus  |  Desplazamiento por integracion pura (sin correcciones)", 20, 30);
  textFont(fuenteChica);
  fill(150);
  text("Fuente: " + modo + (ultimoMsg.length() > 0 ? "   |   firmware: " + ultimoMsg : ""), 20, 52);
  text("E etiqueta   C nueva referencia/corrida (quieto)   M marca   1 2 mostrar A B   F encuadrar   +/- zoom   S captura", 20, 70);
  text("Vista desde arriba: plano perpendicular a la gravedad medida en la referencia", 20, height - 18);
  popStyle();
}

// Encuadra todas las trayectorias visibles
void encuadrar() {
  int w = width - PANEL - 40, h = height - 80 - 50;
  float uMin = 0, uMax = 0, vMin = 0, vMax = 0;
  synchronized (lock) {
    for (float[] p : hist) {
      if (verA) { uMin = min(uMin, p[1]); uMax = max(uMax, p[1]); vMin = min(vMin, p[2]); vMax = max(vMax, p[2]); }
      if (verB) { uMin = min(uMin, p[3]); uMax = max(uMax, p[3]); vMin = min(vMin, p[4]); vMax = max(vMax, p[4]); }
    }
  }
  centroU = (uMin + uMax) / 2;
  centroV = (vMin + vMax) / 2;
  float ancho = max(uMax - uMin, 1), alto = max(vMax - vMin, 1);
  escala = constrain(0.85 * min(w / ancho, h / alto), 0.01, 80);
}

// ---------------- teclado ----------------
void keyPressed() {
  if (key == 'c' || key == 'C') {
    if (esp != null) esp.write("c");
    modo = "tomando referencia (sensor QUIETO 1 s)...";
  } else if (key == 'e' || key == 'E') {
    etiquetaIdx = (etiquetaIdx + 1) % ETIQUETAS.length;
  } else if (key == 'm' || key == 'M') {
    synchronized (lock) {
      if (corrida != null) {
        corrida.println("#marca t_ms=" + ultimoT);
        marcas++;
      }
    }
  } else if (key == '1') {
    verA = !verA;
  } else if (key == '2') {
    verB = !verB;
  } else if (key == 'f' || key == 'F') {
    encuadrar();
  } else if (key == '+' || key == '=') {
    escala = min(escala * 1.25, 80);
  } else if (key == '-' || key == '_') {
    escala = max(escala / 1.25, 0.01);
  } else if (key == 's' || key == 'S') {
    String sello = nf(year(), 4) + nf(month(), 2) + nf(day(), 2) + "_" + nf(hour(), 2) + nf(minute(), 2) + nf(second(), 2);
    saveFrame("corridas/captura_" + sello + ".png");
    println("Captura guardada en corridas/captura_" + sello + ".png");
  }
}
