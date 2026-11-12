// =============================================================================
// J2 - Pressure Sensitive Stylus  |  Visor JOYSTICK con IMU + Hall
// Taller de Proyecto II - UNLP
//
// Prueba de la alternativa "solo joystick": el cursor se mueve con la
// INCLINACION del lapiz (no con la doble integracion) y el Hall decide cuando
// hay tinta y con que grosor. NO es la version final: sirve para compararla con
// visor_dibujo_hall sobre las mismas figuras. Se usa con
// firmware/ensayo_desplazamiento_hall (el mismo firmware, no hay que reflashear).
//
// El calculo esta en MotorJoystick.java (misma carpeta). Resumen:
//   - Desplazamiento de la punta = palanca [cm] x giro [rad] alrededor de los
//     ejes horizontales. El giro alrededor de la vertical no cuenta.
//   - EMBRAGUE: al apoyar se toma la referencia; en el aire el cursor queda
//     quieto (como levantar el mouse). Con E se pasa a modo ABSOLUTO.
//   - Lapiz quieto: el cursor no se mueve (el ruido no se acumula).
//
// Para comparar: T muestra una plantilla (linea de 10 cm, cuadrado de 5 cm,
// circulo de 5 cm) que arranca en el origen. L deja el cursor en el origen.
// Cada trazo informa largo, ancho x alto y cuanto le falta para cerrar.
// G graba la corrida cruda en corridas/ (se puede reprocesar en la PC).
//
// Teclas:
//   L  limpiar (cursor al origen)       R  "arriba" de la hoja = hacia donde apunta el lapiz ahora
//   E  embrague / absoluto              B  beta en movimiento 0 / 0,033
//   + / -  palanca (cm por radian)      M  eje del lapiz: auto / x / y / z
//   X / Y  invertir horizontal / vertical      T  plantilla
//   G  grabar / parar corrida           N  etiqueta de la proxima corrida
//   H  nueva base del Hall (SIN iman)   F  encuadrar     I / O  zoom     S  captura PNG
//
// Si existe tools/calibracion_imu.json (calibrar_imu.py) se aplica. Si existe
// tools/calibracion_presion_hall.json (calibrar_presion_hall.py) se muestra la
// fuerza en gramos.
// Cerrar el monitor serie del Arduino IDE antes de abrir este sketch.
// =============================================================================

import processing.serial.*;

final String PUERTO = "COM3";       // "" = ultimo puerto de la lista; o por ejemplo "COM5"
final int    BAUD   = 921600;
final int    PANEL  = 420;

Serial esp;
final Object lock = new Object();
MotorJoystick motor = new MotorJoystick();
float hallBase = 0, hallMV = 0;
boolean hayHall = false, calibracionCargada = false;
float[] presMv = null, presG = null;  // calibracion de presion (dif mV -> g)
String modo = "conectando...", ultimoMsg = "";
long muestras = 0;

final String[] PLANTILLAS = {"ninguna", "linea 10 cm", "cuadrado 5 cm", "circulo 5 cm"};
int plantilla = 0;
final String[] ETIQUETAS = {"libre", "linea_10cm", "cuadrado_5cm", "circulo_5cm", "letras", "firma"};
int etiquetaIdx = 0;
PrintWriter corrida = null;
String nombreCorrida = "";
ArrayList<String> cabecera = new ArrayList<String>();   // ultimas lineas #cfg, para cada corrida

float escala = 30;
float centroX = 0, centroY = 0;
PFont fuente, fuenteMono, fuenteTitulo;

void settings() {
  size(1300, 780);
  smooth(4);
}

void setup() {
  surface.setTitle("J2 - Pressure Sensitive Stylus | Joystick con IMU + Hall");
  fuente = createFont("SansSerif", 14);
  fuenteTitulo = createFont("SansSerif.bold", 18);
  fuenteMono = createFont("Monospaced", 13);
  cargarCalibracion();
  cargarPresion();
  String[] lista = Serial.list();
  println("Puertos: " + join(lista, ", "));
  String nombre = PUERTO;
  if (nombre.equals("") && lista.length > 0) nombre = lista[lista.length - 1];
  if (nombre.equals("")) { modo = "no hay puertos serie"; return; }
  try {
    esp = new Serial(this, nombre, BAUD);
    esp.bufferUntil('\n');
    modo = "conectado a " + nombre + ", esperando al firmware (sensor QUIETO)...";
  }
  catch (Exception e) {
    modo = "no se pudo abrir " + nombre + ": " + e.getMessage();
  }
}

void cargarCalibracion() {
  String ruta = sketchPath("../calibracion_imu.json");
  if (!new File(ruta).exists()) return;
  try {
    JSONObject j = loadJSONObject(ruta);
    if (j.hasKey("acel_offset_g")) {
      JSONArray o = j.getJSONArray("acel_offset_g"), s = j.getJSONArray("acel_escala");
      for (int i = 0; i < 3; i++) { motor.accOff[i] = o.getFloat(i); motor.accEsc[i] = s.getFloat(i); }
    }
    if (j.hasKey("giro_escala")) {
      JSONArray g = j.getJSONArray("giro_escala");
      for (int i = 0; i < 3; i++) motor.giroEsc[i] = g.getFloat(i);
    }
    calibracionCargada = true;
  }
  catch (Exception e) {
    println("No se pudo leer la calibracion: " + e.getMessage());
  }
}

void cargarPresion() {
  String ruta = sketchPath("../calibracion_presion_hall.json");
  if (!new File(ruta).exists()) return;
  try {
    JSONArray t = loadJSONObject(ruta).getJSONArray("tabla");   // [[dif_mV, g], ...] ordenada
    presMv = new float[t.size()];
    presG = new float[t.size()];
    for (int i = 0; i < t.size(); i++) { presMv[i] = t.getJSONArray(i).getFloat(0); presG[i] = t.getJSONArray(i).getFloat(1); }
  }
  catch (Exception e) {
    println("No se pudo leer la calibracion de presion: " + e.getMessage());
    presMv = null;
  }
}

float gramos(float dif) {
  if (presMv == null || presMv.length < 2) return -1;
  if (dif <= presMv[0]) return presG[0];
  for (int i = 1; i < presMv.length; i++)
    if (dif <= presMv[i]) return map(dif, presMv[i - 1], presMv[i], presG[i - 1], presG[i]);
  return presG[presG.length - 1];   // saturado: no se extrapola
}

// ---------------- recepcion serie ----------------
void serialEvent(Serial p) {
  String s = p.readStringUntil('\n');
  if (s == null) return;
  s = trim(s);
  if (s.length() == 0) return;
  synchronized (lock) {
    if (corrida != null) corrida.println(s);
    if (s.startsWith("#")) {
      if (s.startsWith("#cfg") || s.startsWith("#cols")) {
        cabecera.add(s);
        if (cabecera.size() > 20) cabecera.remove(0);
      }
      int ib = s.indexOf("hall_base_mV=");
      if (ib >= 0) {
        String[] pz = splitTokens(s.substring(ib + 13), " ");
        if (pz.length > 0 && !Float.isNaN(float(pz[0]))) hallBase = float(pz[0]);
      }
      if (s.indexOf("Calibracion OK") >= 0) modo = "recibiendo datos";
      if (!s.startsWith("#cols") && !s.startsWith("#cfg") && !s.startsWith("#hall")) ultimoMsg = s.substring(1).trim();
      return;
    }
    if (!s.startsWith("D,")) return;
    String[] f = split(s, ',');
    if (f.length != 19) {
      if (f.length == 17) modo = "el firmware no tiene Hall: cargar ensayo_desplazamiento_hall";
      return;
    }
    float[] v = new float[18];
    for (int i = 0; i < 18; i++) {
      v[i] = float(f[i + 1]);
      if (Float.isNaN(v[i])) return;
    }
    hayHall = true;
    hallMV = v[16];
    motor.agregar(v[10], v[11], v[12], v[13], v[14], v[15], abs(v[16] - hallBase), v[17] > 0.5);
    muestras++;
  }
}

// ---------------- dibujo ----------------
void draw() {
  background(32, 33, 36);
  synchronized (lock) {
    dibujarHoja();
    dibujarPanel();
  }
  fill(235);
  textFont(fuenteTitulo);
  textAlign(LEFT, BASELINE);
  text("J2 - Pressure Sensitive Stylus  |  Joystick con IMU + Hall (prueba)", 20, 30);
  textFont(fuente);
  fill(160);
  text("Fuente: " + modo + (ultimoMsg.length() > 0 ? "   |   firmware: " + ultimoMsg : ""), 20, 52);
  text("L limpiar  R arriba  E embrague  B beta  +/- palanca  M eje  X/Y invertir  T plantilla  G grabar  N etiqueta  H Hall  F  I/O  S", 20, 72);
}

int hojaX0() { return 20; }
int hojaY0() { return 88; }
int hojaW() { return width - PANEL - 40; }
int hojaH() { return height - 108; }

float[] aPantalla(float x, float y) {
  return new float[] {hojaX0() + hojaW() / 2.0 + (x - centroX) * escala, hojaY0() + hojaH() / 2.0 - (y - centroY) * escala};
}

void dibujarHoja() {
  pushStyle();
  noStroke();
  fill(250, 248, 242);
  rect(hojaX0(), hojaY0(), hojaW(), hojaH());
  clip(hojaX0(), hojaY0(), hojaW(), hojaH());

  // cuadricula de 1 cm (5 cm mas marcada)
  float span = max(hojaW(), hojaH()) / (2 * escala) + 5;
  for (int k = floor(-span); k <= ceil(span); k++) {
    stroke(k % 5 == 0 ? color(205, 210, 220) : color(232, 234, 238));
    strokeWeight(1);
    if (escala >= 6 || k % 5 == 0) {
      float[] a = aPantalla(round(centroX) + k, centroY - span), b = aPantalla(round(centroX) + k, centroY + span);
      float[] c = aPantalla(centroX - span, round(centroY) + k), d = aPantalla(centroX + span, round(centroY) + k);
      line(a[0], a[1], b[0], b[1]);
      line(c[0], c[1], d[0], d[1]);
    }
  }

  dibujarPlantilla();

  // trazos
  stroke(25, 45, 110);
  for (ArrayList<float[]> t : motor.trazos) {
    for (int i = 1; i < t.size(); i++) {
      float[] a = aPantalla(t.get(i - 1)[0], t.get(i - 1)[1]), b = aPantalla(t.get(i)[0], t.get(i)[1]);
      strokeWeight(t.get(i)[2]);
      line(a[0], a[1], b[0], b[1]);
    }
  }

  // cursor
  float[] c = aPantalla(motor.curX, motor.curY);
  strokeWeight(2);
  stroke(motor.mueve ? color(230, 120, 40) : color(60, 160, 90));
  if (motor.contacto) fill(25, 45, 110); else noFill();
  ellipse(c[0], c[1], 14, 14);

  noClip();
  stroke(80);
  strokeWeight(2);
  float x0 = hojaX0() + 20, y0 = hojaY0() + hojaH() - 20;
  line(x0, y0, x0 + 5 * escala, y0);
  noStroke();
  fill(80);
  textFont(fuente);
  text("5 cm", x0 + 5 * escala + 8, y0 + 5);
  popStyle();
}

void dibujarPlantilla() {
  if (plantilla == 0) return;
  stroke(230, 150, 150);
  strokeWeight(2);
  noFill();
  if (plantilla == 1) {
    float[] a = aPantalla(0, 0), b = aPantalla(10, 0);
    line(a[0], a[1], b[0], b[1]);
  } else if (plantilla == 2) {
    float[] a = aPantalla(0, 0), b = aPantalla(5, 5);
    rect(min(a[0], b[0]), min(a[1], b[1]), abs(b[0] - a[0]), abs(b[1] - a[1]));
  } else if (plantilla == 3) {
    float[] m = aPantalla(2.5, 0);
    ellipse(m[0], m[1], 5 * escala, 5 * escala);
  }
  float[] o = aPantalla(0, 0);
  fill(230, 150, 150);
  noStroke();
  ellipse(o[0], o[1], 6, 6);
}

void dibujarPanel() {
  int x0 = width - PANEL - 10, y = 110;
  pushStyle();
  textAlign(LEFT, BASELINE);

  fill(240); textFont(fuente);
  text("Estado", x0, y);
  textFont(fuenteMono);
  fill(motor.contacto ? color(120, 210, 140) : color(200));
  text(motor.estado() + String.format("   giro %.1f grados", motor.anguloDeg), x0, y + 22);

  y += 55;
  fill(240); textFont(fuente);
  text("Hall SS49E", x0, y);
  textFont(fuenteMono);
  if (!hayHall) { fill(160); text("sin datos del Hall", x0, y + 22); }
  else {
    float dif = abs(hallMV - hallBase);
    float g = gramos(dif);
    fill(motor.contacto ? color(120, 210, 140) : color(185));
    text(String.format("%4.0f mV (base %4.0f)  %s", hallMV, hallBase, motor.contacto ? "CONTACTO" : "sin contacto")
         + (g >= 0 ? String.format("  ~%.0f g", g) : ""), x0, y + 22);
    float pr = constrain((dif - motor.hallOnMv) / (motor.hallMaxMv - motor.hallOnMv), 0, 1);
    fill(185); text("presion", x0, y + 44);
    noStroke(); fill(70); rect(x0 + 70, y + 34, PANEL - 100, 10);
    fill(25 + 100 * pr, 45 + 60 * pr, 160); rect(x0 + 70, y + 34, (PANEL - 100) * (motor.contacto ? pr : 0), 10);
  }

  y += 80;
  fill(240); textFont(fuente);
  text("Configuracion", x0, y);
  textFont(fuenteMono); fill(205);
  text("modo: " + (motor.embrague ? "EMBRAGUE (referencia al apoyar)" : "ABSOLUTO (referencia con R)"), x0, y + 22);
  text(String.format("palanca: %.0f cm/rad  (%.2f cm por grado)", motor.palancaCm, motor.palancaCm * PI / 180), x0, y + 42);
  text(String.format("beta en movimiento: %.3f   en reposo: %.2f", motor.betaMovimiento, motor.betaReposo), x0, y + 62);
  String[] ejes = {"x", "y", "z"};
  text("eje del lapiz: " + (motor.ejeLapiz < 0 ? "auto (" + ejes[motor.ejeUsado] + ")" : ejes[motor.ejeLapiz])
       + "   invertir: " + (motor.invX ? "X " : "") + (motor.invY ? "Y" : "") + (!motor.invX && !motor.invY ? "no" : ""), x0, y + 82);
  text("calibracion IMU: " + (calibracionCargada ? "cargada" : "no") + "   presion: " + (presMv != null ? "cargada" : "no"), x0, y + 102);
  text("plantilla: " + PLANTILLAS[plantilla], x0, y + 122);

  y += 155;
  fill(240); textFont(fuente);
  text("Trazos (ultimos 6)", x0, y);
  textFont(fuenteMono); fill(205);
  text("  #   largo   ancho x alto   cierre   dur", x0, y + 22);
  int n = motor.stats.size();
  for (int i = max(0, n - 6); i < n; i++) {
    float[] s = motor.stats.get(i);
    text(String.format("%3d  %5.1f   %4.1f x %4.1f    %5.1f  %4.1f s", i + 1, s[0], s[1], s[2], s[3], s[4]),
         x0, y + 42 + 20 * (i - max(0, n - 6)));
  }
  fill(150);
  text("(cm)", x0 + PANEL - 60, y);

  y += 175;
  fill(240); textFont(fuente);
  text("Corrida", x0, y);
  textFont(fuenteMono);
  if (corrida != null) { fill(240, 110, 90); text("GRABANDO " + nombreCorrida, x0, y + 22); }
  else { fill(185); text("proxima etiqueta: " + ETIQUETAS[etiquetaIdx] + " (N)   G para grabar", x0, y + 22); }
  fill(150);
  text(String.format("muestras recibidas: %d", muestras), x0, y + 42);
  popStyle();
}

void encuadrar() {
  float mnx = 1e9, mny = 1e9, mxx = -1e9, mxy = -1e9;
  int n = 0;
  for (ArrayList<float[]> t : motor.trazos) for (float[] p : t) {
    mnx = min(mnx, p[0]); mxx = max(mxx, p[0]); mny = min(mny, p[1]); mxy = max(mxy, p[1]); n++;
  }
  if (n == 0) { centroX = motor.curX; centroY = motor.curY; return; }
  centroX = (mnx + mxx) / 2; centroY = (mny + mxy) / 2;
  float w = max(mxx - mnx, mxy - mny) + 4;
  escala = constrain(min(hojaW(), hojaH()) / w, 2, 80);
}

String sello() {
  return nf(year(), 4) + nf(month(), 2) + nf(day(), 2) + "_" + nf(hour(), 2) + nf(minute(), 2) + nf(second(), 2);
}

void parametrosEnCorrida() {
  if (corrida == null) return;
  corrida.println(String.format("#param modo=%s palanca_cm=%.1f beta_mov=%.3f eje=%d invX=%d invY=%d",
    motor.embrague ? "embrague" : "absoluto", motor.palancaCm, motor.betaMovimiento, motor.ejeLapiz,
    motor.invX ? 1 : 0, motor.invY ? 1 : 0));
}

void keyPressed() {
  synchronized (lock) {
    char k = Character.toLowerCase(key);
    if (k == 'l') { motor.limpiar(); centroX = centroY = 0; }
    else if (k == 'r') motor.referencia(true);
    else if (k == 'e') { motor.embrague = !motor.embrague; motor.referencia(false); }
    else if (k == 'b') motor.betaMovimiento = motor.betaMovimiento > 0 ? 0 : 0.033;
    else if (key == '+' || key == '=') motor.palancaCm = min(motor.palancaCm + 1, 40);
    else if (key == '-' || key == '_') motor.palancaCm = max(motor.palancaCm - 1, 1);
    else if (k == 'm') { motor.ejeLapiz = motor.ejeLapiz >= 2 ? -1 : motor.ejeLapiz + 1; motor.referencia(true); }
    else if (k == 'x') motor.invX = !motor.invX;
    else if (k == 'y') motor.invY = !motor.invY;
    else if (k == 't') plantilla = (plantilla + 1) % PLANTILLAS.length;
    else if (k == 'n') etiquetaIdx = (etiquetaIdx + 1) % ETIQUETAS.length;
    else if (k == 'h') { if (esp != null) esp.write("h"); }
    else if (k == 'f') encuadrar();
    else if (k == 'i') escala = min(escala * 1.25, 80);
    else if (k == 'o') escala = max(escala / 1.25, 2);
    else if (k == 's') {
      String s = sello();
      saveFrame("capturas/joystick_" + s + ".png");
      println("Captura guardada en capturas/joystick_" + s + ".png");
    }
    else if (k == 'g') {
      if (corrida == null) {
        nombreCorrida = "corridas/" + sello() + "_joy_" + ETIQUETAS[etiquetaIdx] + ".csv";
        corrida = createWriter(nombreCorrida);
        corrida.println("#etiqueta=joy_" + ETIQUETAS[etiquetaIdx]);
        for (String c : cabecera) corrida.println(c);
      } else {
        corrida.flush();
        corrida.close();
        println("Corrida guardada: " + nombreCorrida);
        corrida = null;
        return;
      }
    }
    if ("lrebmxy+=-_g".indexOf(k) >= 0 || key == '+' || key == '-') parametrosEnCorrida();
  }
}

void exit() {
  synchronized (lock) {
    if (corrida != null) { corrida.flush(); corrida.close(); corrida = null; }
  }
  super.exit();
}
