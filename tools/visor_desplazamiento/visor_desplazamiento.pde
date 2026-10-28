// =============================================================================
// J2 - Pressure Sensitive Stylus  |  Visor de DESPLAZAMIENTO (sin rotacion)
// Taller de Proyecto II - UNLP
//
// Se usa con el firmware firmware/ensayo_desplazamiento. Muestra, vista desde
// arriba (plano de la mesa), dos trayectorias calculadas con las mismas muestras:
//   A (rojo)  CON ORIENTACION: como madgwick_filter (Madgwick + resta de 1 g).
//   B (azul)  SIN ROTACION: resta una gravedad fija medida en reposo.
//   C (verde) SIN ROTACION y SIN CORRECCIONES: como B pero sin zona muerta ni
//             atenuacion de velocidad (solo ZUPT).
//
// Procedimiento: apoyar el sensor sobre la mesa, apretar C con el sensor quieto
// y DESLIZARLO SIN GIRARLO contra una regla (un trazo, quieto, otro trazo...).
// Cada trazo (de reposo a reposo) se mide y se compara con el largo real (L).
//   - B bien y A mal  -> el problema es la orientacion (gravedad mal restada).
//   - A y B igual mal -> el problema es la doble integracion / el sensor.
//   - C mucho mejor que B -> las correcciones (zona muerta, atenuacion) acortan el trazo.
// La columna "dur" permite comparar trazos rapidos y lentos.
// Si el sensor se INCLINA mas de 3 grados durante un trazo, B deja de ser valido.
// (Girarlo alrededor de la vertical no afecta: no cambia la direccion de la gravedad.)
//
// Teclas:
//   C        nueva referencia (sensor QUIETO) y borrar todo
//   L        largo real del trazo (10 / 15 / 20 / 30 cm)
//   U        umbral del detector de reposo (0,030 / 0,020 / 0,015 g)
//   1 2 3    mostrar / ocultar A, B, C
//   + / -    zoom                       S  guardar captura PNG + trazos CSV
//
// Cerrar el monitor serie del Arduino IDE antes de abrir este sketch.
// =============================================================================

import processing.serial.*;

// ---------------- configuracion ----------------
final String PUERTO = "";       // "" = ultimo puerto de la lista; o por ejemplo "COM5"
final int    BAUD   = 921600;
final float  GIRO_MAX = 3.0;    // [grados] de inclinacion: por encima B no es valido

// ---------------- datos (los escribe el hilo serie) ----------------
Serial esp;
final Object lock = new Object();
ArrayList<float[]> trayA = new ArrayList<float[]>();   // {u, v} en cm
ArrayList<float[]> trayB = new ArrayList<float[]>();
ArrayList<float[]> trayC = new ArrayList<float[]>();
ArrayList<float[]> trazos = new ArrayList<float[]>();  // ver cerrarTrazo()
final int MAX_PUNTOS = 6000;
float Au, Av, An, Bu, Bv, Bn, Cu, Cv, Cn, giro;
boolean verA = true, verB = true, verC = true;
String umbral = "?";
long tInicioTrazo = 0, tActual = 0;
boolean quieto = true, quietoAnt = true;
float[] inicioTrazo = new float[9];
float giroMaxTrazo = 0;
boolean satTrazo = false;

// estadisticas del firmware
long muestras = 0, satTotal = 0, perdidas = 0;
long ultimoT = -1;
float cicloMax = 0, cicloSuma = 0;
int muestrasVentana = 0, tVentana = 0;
float hz = 0;

String modo = "conectando...";
String ultimoMsg = "";
boolean simulacion = false;

// ---------------- vista ----------------
float escala = 12;              // px por cm
final float[] LARGOS = {10, 15, 20, 30};
int largoIdx = 2;
PFont fuente, fuenteChica, fuenteTitulo, fuenteMono;
final int PANEL = 430;

void settings() {
  size(1280, 760);
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
    simulacion = true;
    modo = "SIN ESP32: no hay puertos serie";
    return;
  }
  try {
    esp = new Serial(this, nombre, BAUD);
    esp.bufferUntil('\n');
    modo = "conectado a " + nombre + ", esperando calibracion (sensor QUIETO)...";
  }
  catch (Exception e) {
    simulacion = true;
    modo = "no se pudo abrir " + nombre + ": " + e.getMessage();
  }
}

// ---------------- recepcion serie ----------------
void serialEvent(Serial p) {
  String s = p.readStringUntil('\n');
  if (s == null) return;
  s = trim(s);
  if (s.length() == 0) return;
  if (s.startsWith("#")) {
    if (!s.startsWith("#cols")) ultimoMsg = s.substring(1).trim();
    if (s.indexOf("Calibracion OK") >= 0) modo = "recibiendo datos";
    if (s.indexOf("Referencia tomada") >= 0) borrarTodo();
    if (s.startsWith("#cfg umbral_reposo=")) umbral = s.substring(19).trim();
    return;
  }
  if (!s.startsWith("D,")) return;
  String[] f = split(s, ',');
  if (f.length != 15) return;
  long t = Long.parseLong(f[1].trim());
  float[] v = new float[13];
  for (int i = 0; i < 13; i++) {
    v[i] = float(f[i + 2]);
    if (Float.isNaN(v[i])) return;
  }
  synchronized (lock) {
    Au = v[0]; Av = v[1]; An = v[2];
    Bu = v[3]; Bv = v[4]; Bn = v[5];
    Cu = v[6]; Cv = v[7]; Cn = v[8];
    quieto = v[9] > 0.5;
    giro = v[10];
    boolean sat = v[11] > 0.5;
    float ciclo = v[12];
    tActual = t;

    muestras++; muestrasVentana++;
    if (sat) satTotal++;
    cicloMax = max(cicloMax, ciclo);
    cicloSuma += ciclo;
    if (ultimoT >= 0 && t - ultimoT > 15) perdidas += round((t - ultimoT) / 10.0) - 1;
    ultimoT = t;

    agregar(trayA, Au, Av);
    agregar(trayB, Bu, Bv);
    agregar(trayC, Cu, Cv);

    // trazos: de reposo a reposo
    if (quietoAnt && !quieto) {
      inicioTrazo = new float[] {Au, Av, An, Bu, Bv, Bn, Cu, Cv, Cn};
      tInicioTrazo = t;
      giroMaxTrazo = giro;
      satTrazo = false;
    }
    if (!quieto) {
      giroMaxTrazo = max(giroMaxTrazo, giro);
      if (sat) satTrazo = true;
    }
    if (!quietoAnt && quieto) cerrarTrazo();
    quietoAnt = quieto;
  }
}

void agregar(ArrayList<float[]> tr, float u, float v) {
  if (tr.size() > 0) {
    float[] ult = tr.get(tr.size() - 1);
    if (abs(ult[0] - u) < 0.01 && abs(ult[1] - v) < 0.01) return;   // no repetir puntos en reposo
  }
  tr.add(new float[] {u, v});
  if (tr.size() > MAX_PUNTOS) tr.remove(0);
}

// trazo = {largoA, largoB, verticalA, verticalB, inclMax, saturo, largoReal, anguloAB, largoC, duracion_s, umbral_g}
void cerrarTrazo() {
  float dAu = Au - inicioTrazo[0], dAv = Av - inicioTrazo[1], dAn = An - inicioTrazo[2];
  float dBu = Bu - inicioTrazo[3], dBv = Bv - inicioTrazo[4], dBn = Bn - inicioTrazo[5];
  float dCu = Cu - inicioTrazo[6], dCv = Cv - inicioTrazo[7];
  float lA = sqrt(dAu * dAu + dAv * dAv), lB = sqrt(dBu * dBu + dBv * dBv), lC = sqrt(dCu * dCu + dCv * dCv);
  if (max(lA, max(lB, lC)) < 0.5) return;   // movimiento minimo: no es un trazo
  float dur = (tActual - tInicioTrazo) / 1000.0;
  float ang = 0;
  if (lA > 0.5 && lB > 0.5) {
    ang = degrees(acos(constrain((dAu * dBu + dAv * dBv) / (lA * lB), -1, 1)));
  }
  float u = float(split(umbral, ' ')[0]);
  trazos.add(new float[] {lA, lB, dAn, dBn, giroMaxTrazo, satTrazo ? 1 : 0, LARGOS[largoIdx], ang, lC, dur, u});
  println(String.format("Trazo %d: real %.0f cm en %.1f s | A %.1f cm | B %.1f cm | C %.1f cm | inclinacion max %.1f grados | umbral %s",
    trazos.size(), LARGOS[largoIdx], dur, lA, lB, lC, giroMaxTrazo, umbral));
}

void borrarTodo() {
  synchronized (lock) {
    trayA.clear(); trayB.clear(); trayC.clear(); trazos.clear();
    muestras = 0; satTotal = 0; perdidas = 0; ultimoT = -1;
    cicloMax = 0; cicloSuma = 0;
    quieto = true; quietoAnt = true;
  }
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
  }
  dibujarEncabezado();
}

void dibujarLienzo() {
  int x0 = 20, y0 = 80, w = width - PANEL - 40, h = height - y0 - 50;
  pushStyle();
  fill(250, 248, 242);
  noStroke();
  rect(x0, y0, w, h);

  float cx = x0 + w / 2.0, cy = y0 + h / 2.0;
  // grilla: 1 cm (fina) y 5 cm (marcada)
  for (int k = -200; k <= 200; k++) {
    float px = cx + k * escala, py = cy + k * escala;
    stroke(k % 5 == 0 ? color(200, 196, 185) : color(232, 229, 220));
    strokeWeight(1);
    if (escala >= 4 || k % 5 == 0) {
      if (px > x0 && px < x0 + w) line(px, y0, px, y0 + h);
      if (py > y0 && py < y0 + h) line(x0, py, x0 + w, py);
    }
  }
  // origen
  stroke(120);
  line(cx - 8, cy, cx + 8, cy);
  line(cx, cy - 8, cx, cy + 8);

  clip(x0, y0, w, h);
  if (verC) dibujarTray(trayC, cx, cy, color(30, 150, 70));
  if (verB) dibujarTray(trayB, cx, cy, color(40, 90, 200));
  if (verA) dibujarTray(trayA, cx, cy, color(200, 50, 50));
  // posicion actual
  noStroke();
  if (verC) { fill(30, 150, 70); ellipse(cx + Cu * escala, cy - Cv * escala, 10, 10); }
  if (verB) { fill(40, 90, 200); ellipse(cx + Bu * escala, cy - Bv * escala, 10, 10); }
  if (verA) { fill(200, 50, 50); ellipse(cx + Au * escala, cy - Av * escala, 10, 10); }
  noClip();

  // barra de escala y leyenda
  fill(60);
  textFont(fuenteChica);
  textAlign(LEFT, BASELINE);
  stroke(60);
  strokeWeight(2);
  line(x0 + 15, y0 + h - 15, x0 + 15 + 5 * escala, y0 + h - 15);
  noStroke();
  text("5 cm", x0 + 20 + 5 * escala, y0 + h - 11);
  fill(200, 50, 50);
  text("A con orientacion (Madgwick)", x0 + 15, y0 + 20);
  fill(40, 90, 200);
  text("B sin rotacion (gravedad fija)", x0 + 15, y0 + 38);
  fill(30, 150, 70);
  text("C sin rotacion, sin zona muerta ni atenuacion", x0 + 15, y0 + 56);
  fill(quieto ? color(110) : color(30, 140, 60));
  textAlign(RIGHT, BASELINE);
  textFont(fuente);
  text(quieto ? "en reposo" : "EN MOVIMIENTO", x0 + w - 15, y0 + 22);
  if (giro > GIRO_MAX) {
    fill(210, 120, 0);
    text(String.format("el sensor se inclino %.1f grados desde la referencia: B no es valido", giro), x0 + w - 15, y0 + 42);
  }
  popStyle();
}

void dibujarTray(ArrayList<float[]> tr, float cx, float cy, color c) {
  noFill();
  stroke(c);
  strokeWeight(2.5);
  beginShape();
  for (float[] p : tr) vertex(cx + p[0] * escala, cy - p[1] * escala);
  endShape();
}

void dibujarPanel() {
  int x0 = width - PANEL - 10, y = 80;
  pushStyle();
  textAlign(LEFT, BASELINE);
  fill(240);
  textFont(fuente);
  text("Posicion actual [cm]", x0, y);
  textFont(fuenteMono);
  fill(230, 110, 110);
  text(String.format("A  u %7.1f  v %7.1f  vertical %6.1f", Au, Av, An), x0, y + 20);
  fill(120, 160, 240);
  text(String.format("B  u %7.1f  v %7.1f  vertical %6.1f", Bu, Bv, Bn), x0, y + 38);
  fill(110, 200, 130);
  text(String.format("C  u %7.1f  v %7.1f  vertical %6.1f", Cu, Cv, Cn), x0, y + 56);
  fill(185);
  text(String.format("inclinacion desde la referencia %4.1f grados", giro), x0, y + 74);
  text("umbral de reposo " + umbral + " (tecla U)", x0, y + 92);

  y += 124;
  fill(240);
  textFont(fuente);
  text("Trazos (largo real L = " + nf(LARGOS[largoIdx], 0, 0) + " cm, tecla L)", x0, y);
  textFont(fuenteMono);
  fill(150);
  text("  #  real  dur     A     B     C  incl", x0, y + 20);
  int desde = max(0, trazos.size() - 12);
  for (int i = desde; i < trazos.size(); i++) {
    float[] t = trazos.get(i);
    int fila = i - desde;
    boolean invalido = t[4] > GIRO_MAX || t[5] > 0.5;
    fill(invalido ? color(210, 140, 40) : color(215));
    text(String.format("%3d %5.0f %4.1fs %4.0f%% %4.0f%% %4.0f%% %5.1f%s",
      i + 1, t[6], t[9], 100 * t[0] / t[6], 100 * t[1] / t[6], 100 * t[8] / t[6], t[4],
      t[5] > 0.5 ? " SAT" : (t[4] > GIRO_MAX ? " incl" : "")), x0, y + 38 + fila * 17);
  }
  if (trazos.size() == 0) {
    fill(130);
    textFont(fuenteChica);
    text("Apretar C con el sensor quieto y deslizarlo sin inclinarlo.", x0, y + 40);
  }

  // promedios de los trazos validos
  float sA = 0, sB = 0, sC = 0; int nv = 0;
  for (float[] t : trazos) {
    if (t[4] > GIRO_MAX || t[5] > 0.5) continue;
    sA += t[0] / t[6]; sB += t[1] / t[6]; sC += t[8] / t[6]; nv++;
  }
  y = height - 240;
  fill(240);
  textFont(fuente);
  text("Resumen de trazos validos", x0, y);
  textFont(fuenteMono);
  fill(215);
  if (nv > 0) {
    text(String.format("A mide el %3.0f%% del largo real (%d trazos)", 100 * sA / nv, nv), x0, y + 20);
    text(String.format("B mide el %3.0f%% del largo real", 100 * sB / nv), x0, y + 38);
    text(String.format("C mide el %3.0f%% del largo real", 100 * sC / nv), x0, y + 56);
  } else {
    text("(todavia no hay trazos validos)", x0, y + 20);
  }

  y += 84;
  fill(240);
  textFont(fuente);
  text("ESP32 / MPU6050 (descarta saturacion)", x0, y);
  textFont(fuenteMono);
  fill(215);
  float cicloMed = muestras > 0 ? cicloSuma / muestras : 0;
  text(String.format("tasa recibida      %6.1f Hz (objetivo 100)", hz), x0, y + 20);
  text(String.format("ciclo medio / max  %6.0f / %.0f us (de 10000)", cicloMed, cicloMax), x0, y + 38);
  text(String.format("muestras perdidas  %6d", perdidas), x0, y + 56);
  fill(satTotal > 0 ? color(230, 120, 60) : color(215));
  text(String.format("muestras saturadas %6d", satTotal), x0, y + 74);
  popStyle();
}

void dibujarEncabezado() {
  pushStyle();
  textAlign(LEFT, BASELINE);
  fill(240);
  textFont(fuenteTitulo);
  text("J2 - Pressure Sensitive Stylus  |  Ensayo de desplazamiento (sin rotacion)", 20, 30);
  textFont(fuenteChica);
  fill(150);
  text("Fuente: " + modo + (ultimoMsg.length() > 0 ? "   |   firmware: " + ultimoMsg : ""), 20, 52);
  text("C nueva referencia (quieto)   L largo real   U umbral de reposo   1 2 3 mostrar A B C   +/- zoom   S guardar", 20, 70);
  fill(150);
  text("Vista desde arriba: plano perpendicular a la gravedad medida", 20, height - 18);
  popStyle();
}

// ---------------- teclado ----------------
void keyPressed() {
  if (key == 'c' || key == 'C') {
    if (esp != null) esp.write("c");
    borrarTodo();
    modo = "tomando referencia (sensor QUIETO)...";
  } else if (key == 'u' || key == 'U') {
    if (esp != null) esp.write("u");
  } else if (key == '1') {
    verA = !verA;
  } else if (key == '2') {
    verB = !verB;
  } else if (key == '3') {
    verC = !verC;
  } else if (key == 'l' || key == 'L') {
    largoIdx = (largoIdx + 1) % LARGOS.length;
  } else if (key == '+' || key == '=') {
    escala = min(escala * 1.25, 80);
  } else if (key == '-' || key == '_') {
    escala = max(escala / 1.25, 1);
  } else if (key == 's' || key == 'S') {
    guardar();
  }
}

void guardar() {
  String sello = nf(year(), 4) + nf(month(), 2) + nf(day(), 2) + "_" + nf(hour(), 2) + nf(minute(), 2) + nf(second(), 2);
  saveFrame("desplazamiento_" + sello + ".png");
  synchronized (lock) {
    PrintWriter out = createWriter("desplazamiento_" + sello + "_trazos.csv");
    out.println("trazo,largo_real_cm,duracion_s,largo_A_cm,largo_B_cm,largo_C_cm,vertical_A_cm,vertical_B_cm,angulo_AB_deg,inclinacion_max_deg,saturo,umbral_reposo");
    for (int i = 0; i < trazos.size(); i++) {
      float[] t = trazos.get(i);
      out.println((i + 1) + "," + t[6] + "," + t[9] + "," + t[0] + "," + t[1] + "," + t[8] + "," + t[2] + "," + t[3] + "," + t[7] + "," + t[4] + "," + int(t[5]) + "," + t[10]);
    }
    out.flush();
    out.close();
  }
  println("Guardado desplazamiento_" + sello + " (.png y _trazos.csv) en la carpeta del sketch");
}
