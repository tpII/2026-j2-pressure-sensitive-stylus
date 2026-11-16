import processing.serial.*;

final String PUERTO = "COM3";       // "" = ultimo puerto de la lista; o por ejemplo "COM5"
final int    BAUD   = 921600;
final int    PANEL  = 400;

Serial esp;
final Object lock = new Object();
MotorLapiz motor = new MotorLapiz();
float hallBase = 0, hallMV = 0;
boolean hayHall = false, verProvisorio = true, calibracionCargada = false;
boolean hallActivo = true;                       // false: se ignora el contacto del Hall
volatile boolean tintaMouse = false, tintaTecla = false;   // tinta manual
final String[] BOTONES = {"Centrar (C)", "Limpiar (L)", "Reiniciar (R)", "Alinear (A)", "Hall (H)", "TINTA (espacio)"};
final int B_ALINEAR = 3, B_HALL = 4, B_TINTA = 5;
final int BOT_Y = 58, BOT_H = 24, BOT_W = 130, BOT_SEP = 8;
boolean rumboTomado = false;   // "arriba" de la pantalla = direccion de la punta al arrancar
String modo = "conectando...", ultimoMsg = "";
long muestras = 0;

float escala = 25;              // px por cm
float centroX = 0, centroY = 0; // cm en el centro de la hoja
float rotAng = 0;               // giro de la vista [rad]
boolean espejo = false;
PFont fuente, fuenteMono, fuenteTitulo;

void settings() {
  size(1300, 780);
  smooth(4);
}

void setup() {
  surface.setTitle("J2 - Pressure Sensitive Stylus | Lapiz armado: trazos cortos + Hall");
  fuente = createFont("SansSerif", 14);
  fuenteTitulo = createFont("SansSerif.bold", 18);
  fuenteMono = createFont("Monospaced", 13);
  cargarCalibracion();
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
  if (!new File(ruta).exists()) { println("Sin calibracion (" + ruta + ")"); return; }
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
    println("Calibracion cargada: " + ruta);
  }
  catch (Exception e) {
    println("No se pudo leer la calibracion: " + e.getMessage());
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
      if (!s.startsWith("#cols") && !s.startsWith("#cfg") && !s.startsWith("#hall")) ultimoMsg = s.substring(1).trim();
      return;
    }
    if (!s.startsWith("D,")) return;
    String[] f = split(s, ',');
    if (f.length != 19 && f.length != 17) return;   // 17: firmware sin Hall
    float[] v = new float[f.length - 1];
    for (int i = 0; i < v.length; i++) {
      v[i] = float(f[i + 1]);
      if (Float.isNaN(v[i])) return;
    }
    hayHall = f.length == 19;
    boolean contHall = hayHall && hallActivo && v[17] > 0.5;
    boolean manual = tintaMouse || tintaTecla;
    float dif = 0;
    if (hayHall) { hallMV = v[16]; dif = abs(v[16] - hallBase); }
    if (manual && !contHall) dif = (motor.hallOnMv + motor.hallMaxMv) / 2;   // grosor medio
    motor.agregar(v[10], v[11], v[12], v[13], v[14], v[15], dif, contHall || manual);
    muestras++;
  }
}

// ---------------- dibujo ----------------
// "arriba" de la pantalla = direccion de la punta (lapiz acostado). La vista se
// gira para que esa direccion quede hacia arriba; derecha e izquierda salen solas
// (el marco del mundo tiene z hacia arriba y se mira desde arriba).
boolean tomarRumbo() {
  if (!motor.estaListo()) return false;
  float h = motor.rumboPunta();
  if (Float.isNaN(h)) return false;
  rotAng = HALF_PI - h;
  espejo = false;
  rumboTomado = true;
  centroX = motor.curX; centroY = motor.curY;
  return true;
}

void draw() {
  background(32, 33, 36);
  synchronized (lock) {
    if (!rumboTomado && motor.estaListo()) tomarRumbo();
    dibujarHoja();
    dibujarPanel();
  }
  fill(235);
  textFont(fuenteTitulo);
  textAlign(LEFT, BASELINE);
  text("J2 - Pressure Sensitive Stylus  |  Lapiz armado: trazos cortos + Hall", 20, 30);
  textFont(fuente);
  fill(160);
  text("Fuente: " + modo + (ultimoMsg.length() > 0 ? "   |   firmware: " + ultimoMsg : ""), 20, 52);
  dibujarBotones();
}

float botonX(int i) { return 20 + i * (BOT_W + BOT_SEP); }

void dibujarBotones() {
  pushStyle();
  textFont(fuente);
  textAlign(CENTER, CENTER);
  for (int i = 0; i < BOTONES.length; i++) {
    String t = BOTONES[i];
    boolean activo = (i == B_HALL && hallActivo) || (i == B_TINTA && (tintaMouse || tintaTecla));
    if (i == B_HALL) t = hallActivo ? "Hall: usar (H)" : "Hall: ignorar (H)";
    boolean sobre = mouseX >= botonX(i) && mouseX <= botonX(i) + BOT_W && mouseY >= BOT_Y && mouseY <= BOT_Y + BOT_H;
    stroke(90);
    fill(activo ? color(60, 120, 80) : (sobre ? color(70, 72, 78) : color(52, 54, 60)));
    if (i == B_TINTA && activo) fill(25, 45, 110);
    rect(botonX(i), BOT_Y, BOT_W, BOT_H, 5);
    fill(235);
    text(t, botonX(i) + BOT_W / 2, BOT_Y + BOT_H / 2 - 1);
  }
  popStyle();
}

int botonEn(int x, int y) {
  for (int i = 0; i < BOTONES.length; i++)
    if (x >= botonX(i) && x <= botonX(i) + BOT_W && y >= BOT_Y && y <= BOT_Y + BOT_H) return i;
  return -1;
}

void accion(int i) {
  synchronized (lock) {
    if (i == 0) { centroX = motor.curX; centroY = motor.curY; }
    else if (i == 1) { motor.limpiar(); centroX = centroY = 0; }
    else if (i == 2) {
      motor = new MotorLapiz();
      cargarCalibracion();
      centroX = centroY = 0;
      muestras = 0;
      rumboTomado = false;
      modo = "reiniciado: lapiz ACOSTADO, punta hacia adelante, quieto 1 s";
    }
    else if (i == B_ALINEAR) {
      if (!tomarRumbo()) modo = "para alinear: acostar el lapiz con la punta hacia adelante y repetir";
    }
    else if (i == B_HALL) hallActivo = !hallActivo;
  }
}

void mousePressed() {
  int i = botonEn(mouseX, mouseY);
  if (i == B_TINTA) tintaMouse = true;
  else if (i >= 0) accion(i);
}

void mouseReleased() { tintaMouse = false; }

int hojaX0() { return 20; }
int hojaY0() { return 88; }
int hojaW() { return width - PANEL - 40; }
int hojaH() { return height - 108; }

// cm -> pantalla (y hacia arriba), con la rotacion de la vista
float[] aPantalla(float x, float y) {
  float u0 = x - centroX, w0 = y - centroY;
  if (espejo) u0 = -u0;
  float u = u0 * cos(rotAng) - w0 * sin(rotAng), w = u0 * sin(rotAng) + w0 * cos(rotAng);
  return new float[] {hojaX0() + hojaW() / 2.0 + u * escala, hojaY0() + hojaH() / 2.0 - w * escala};
}

void dibujarHoja() {
  pushStyle();
  noStroke();
  fill(250, 248, 242);
  rect(hojaX0(), hojaY0(), hojaW(), hojaH());
  clip(hojaX0(), hojaY0(), hojaW(), hojaH());

  // cuadricula de 1 cm (y 5 cm mas marcada)
  float cmMin = -hojaW() / (2 * escala) - 2, cmMax = hojaW() / (2 * escala) + 2;
  float span = max(hojaW(), hojaH()) / (2 * escala) + 5;
  for (int k = floor(-span); k <= ceil(span); k++) {
    stroke(k % 5 == 0 ? color(205, 210, 220) : color(232, 234, 238));
    strokeWeight(1);
    float[] a = aPantalla(centroX + k, centroY - span), b = aPantalla(centroX + k, centroY + span);
    float[] c = aPantalla(centroX - span, centroY + k), d = aPantalla(centroX + span, centroY + k);
    if (escala * 1 >= 6 || k % 5 == 0) { line(a[0], a[1], b[0], b[1]); line(c[0], c[1], d[0], d[1]); }
  }

  // trazos definitivos
  stroke(25, 45, 110);
  for (ArrayList<float[]> t : motor.trazos) {
    for (int i = 1; i < t.size(); i++) {
      float[] a = aPantalla(t.get(i - 1)[0], t.get(i - 1)[1]), b = aPantalla(t.get(i)[0], t.get(i)[1]);
      strokeWeight(t.get(i)[2]);
      line(a[0], a[1], b[0], b[1]);
    }
    if (t.size() == 1) {
      float[] a = aPantalla(t.get(0)[0], t.get(0)[1]);
      strokeWeight(t.get(0)[2] + 1);
      point(a[0], a[1]);
    }
  }

  // trazo provisorio de la pasada en curso
  if (verProvisorio) {
    ArrayList<float[]> pv = motor.provisorio;
    for (int i = 1; i < pv.size(); i++) {
      float[] a = aPantalla(pv.get(i - 1)[0], pv.get(i - 1)[1]), b = aPantalla(pv.get(i)[0], pv.get(i)[1]);
      if (pv.get(i)[2] > 0.5) { stroke(120, 150, 215); strokeWeight(max(1, pv.get(i)[3])); }
      else { stroke(175); strokeWeight(1); }
      line(a[0], a[1], b[0], b[1]);
    }
  }

  // cursor (posicion del lapiz)
  float[] c = aPantalla(motor.curX, motor.curY);
  strokeWeight(2);
  stroke(motor.enPasada ? color(230, 120, 40) : color(60, 160, 90));
  if (motor.contacto) fill(25, 45, 110); else noFill();
  ellipse(c[0], c[1], 14, 14);

  noClip();
  // escala
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

void dibujarPanel() {
  int x0 = width - PANEL - 10, y = 110;
  pushStyle();
  textAlign(LEFT, BASELINE);

  fill(240); textFont(fuente);
  text("Estado", x0, y);
  textFont(fuenteMono);
  fill(motor.enPasada ? color(240, 160, 80) : color(120, 210, 140));
  text(motor.estado(), x0, y + 22);
  if (motor.aviso.length() > 0) { fill(240, 130, 60); text(motor.aviso, x0, y + 42); }
  if (motor.enPasada) {
    float f = constrain(motor.durPasada() / motor.durMaxPasada, 0, 1);
    noStroke(); fill(70); rect(x0, y + 52, PANEL - 30, 8);
    fill(f < 0.6 ? color(120, 210, 140) : (f < 1 ? color(240, 190, 80) : color(240, 110, 60)));
    rect(x0, y + 52, (PANEL - 30) * f, 8);
  }

  y += 95;
  fill(240); textFont(fuente);
  text("Hall SS49E", x0, y);
  textFont(fuenteMono);
  if (!hayHall) { fill(160); text("sin Hall: tinta solo con el boton TINTA", x0, y + 22); }
  else if (!hallActivo) { fill(240, 160, 80); text(String.format("%4.0f mV - IGNORADO (tinta con el boton)", hallMV), x0, y + 22); }
  else {
    fill(motor.contacto ? color(120, 210, 140) : color(185));
    text(String.format("%4.0f mV (base %4.0f)  %s", hallMV, hallBase, motor.contacto ? "CONTACTO" : "sin contacto"), x0, y + 22);
    float pr = constrain((abs(hallMV - hallBase) - motor.hallOnMv) / (motor.hallMaxMv - motor.hallOnMv), 0, 1);
    fill(185); text("presion", x0, y + 44);
    noStroke(); fill(70); rect(x0 + 70, y + 34, PANEL - 100, 10);
    fill(25 + 100 * pr, 45 + 60 * pr, 160); rect(x0 + 70, y + 34, (PANEL - 100) * (motor.contacto ? pr : 0), 10);
  }

  y += 80;
  fill(240); textFont(fuente);
  text("Dibujo", x0, y);
  textFont(fuenteMono); fill(205);
  text(String.format("%d pasadas, %d trazos", motor.pasadas, motor.trazos.size()), x0, y + 22);
  if (motor.pasadas > 0)
    text(String.format("ultima pasada: %.1f s, correccion %.1f cm/s", motor.ultimaDur, motor.ultimaVelSobrante), x0, y + 42);
  text(String.format("lapiz en (%.1f, %.1f) cm", motor.curX, motor.curY), x0, y + 62);

  y += 100;
  fill(240); textFont(fuente);
  text("Configuracion", x0, y);
  textFont(fuenteMono); fill(185);
  text("calibracion IMU: " + (calibracionCargada ? "cargada" : "no (tools/calibracion_imu.json)"), x0, y + 22);
  text(String.format("muestras recibidas: %d", muestras), x0, y + 42);
  float[] b = motor.biasDps();
  text(String.format("bias giro: %.2f %.2f %.2f grados/s", b[0], b[1], b[2]), x0, y + 62);
  text(String.format("pasadas descartadas (giro grande): %d", motor.descartadas), x0, y + 82);

  y += 120;
  textFont(fuente); fill(150);
  text("Gris: pasada en curso (provisorio).", x0, y);
  text("Azul oscuro: trazo definitivo, al frenar 0,3 s.", x0, y + 20);
  text("Trazos de menos de 2 s, frenando con la punta apoyada.", x0, y + 48);
  text("Arrancar y recalibrar con el lapiz acostado en la mesa.", x0, y + 68);
  popStyle();
}

void keyPressed() {
  if (key == ' ') { tintaTecla = true; return; }
  char k = Character.toLowerCase(key);
  if (k == 'c') accion(0);
  else if (k == 'l') accion(1);
  else if (k == 'r') accion(2);
  else if (k == 'a') accion(B_ALINEAR);
  else if (k == 'h') accion(B_HALL);
}

void keyReleased() {
  if (key == ' ') tintaTecla = false;
}
