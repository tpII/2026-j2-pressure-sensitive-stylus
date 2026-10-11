import processing.serial.*;

final int PORT_INDEX = 0;
final int BAUD_RATE  = 921600;  // debe coincidir con SERIAL_BAUD del firmware

Serial puerto;
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
float ref0 = 1, ref1 = 0, ref2 = 0, ref3 = 0;

float posX = 0, posY = 0, posZ = 0;
boolean quieto = false;

ArrayList<float[]> trayectoria = new ArrayList<float[]>();
final int MAX_PUNTOS = 2000;
float escalaTray = 500.0;  // px por metro (teclas + / - para cambiar)

void setup() {
    size(800, 750, P3D);
    pixelDensity(1);

    String[] puertos = Serial.list();
    println("=== Puertos disponibles ===");
    printArray(puertos);

    if (puertos.length == 0) {
        println("ERROR: no hay puertos serie. Conecta el ESP32.");
        return;
    }

    println("Usando: " + puertos[PORT_INDEX]);
    puerto = new Serial(this, puertos[PORT_INDEX], BAUD_RATE);
    puerto.bufferUntil('\n');
}

void draw() {
    background(25);

    ambientLight(100, 100, 100);
    directionalLight(200, 200, 200, 0, 1, -1);

    pushMatrix();
    translate(width / 2, height * 0.32, 0);

    float rq0 = ref0*q0 + ref1*q1 + ref2*q2 + ref3*q3;
    float rq1 = ref0*q1 - ref1*q0 - ref2*q3 + ref3*q2;
    float rq2 = ref0*q2 + ref1*q3 - ref2*q0 - ref3*q1;
    float rq3 = ref0*q3 - ref1*q2 + ref2*q1 - ref3*q0;
    float[] R = quaternionToMatrix(rq0, rq1, rq2, rq3);
    applyMatrix(
        R[0], R[1], R[2], 0,
        R[3], R[4], R[5], 0,
        R[6], R[7], R[8], 0,
           0,    0,    0, 1
    );

    noStroke();
    fill(70, 130, 220);
    box(35, 220, 35);

    pushMatrix();
    translate(0, 125, 0);
    fill(220, 180, 60);
    box(35, 30, 35);
    popMatrix();

    strokeWeight(3);
    stroke(255, 60, 60);  line(0, 0, 0,  130,    0,   0);
    stroke(60, 220, 60);  line(0, 0, 0,    0, -130,   0);
    stroke(60, 100, 255); line(0, 0, 0,    0,    0, 130);

    popMatrix();

    dibujarTrayectoria();

    hint(DISABLE_DEPTH_TEST);
    noStroke();
    fill(200);
    textSize(14);
    text("q0: " + nf(q0, 1, 4), 15, 25);
    text("q1: " + nf(q1, 1, 4), 15, 45);
    text("q2: " + nf(q2, 1, 4), 15, 65);
    text("q3: " + nf(q3, 1, 4), 15, 85);

    fill(quieto ? color(60, 220, 60) : color(220, 80, 60));
    ellipse(width - 110, 22, 12, 12);
    fill(180);
    text(quieto ? "ZUPT activo" : "integrando", width - 95, 27);

    fill(150);
    text("[R] referencia   [Z] reiniciar orientacion   [C] limpiar trayectoria   [+/-] escala (" + int(escalaTray) + " px/m)", 15, height - 15);
    hint(ENABLE_DEPTH_TEST);
}

void dibujarTrayectoria() {
    hint(DISABLE_DEPTH_TEST);

    int panelY = (int)(height * 0.62);
    int panelH = (int)(height * 0.33);
    int panelW = width - 40;
    int cx = width / 2;
    int cy = panelY + panelH / 2;

    noFill();
    stroke(60);
    strokeWeight(1);
    rect(20, panelY, panelW, panelH);

    stroke(50);
    line(cx, panelY, cx, panelY + panelH);
    line(20, cy, 20 + panelW, cy);

    fill(150);
    noStroke();
    textSize(12);
    text("Trayectoria XY (m)", 25, panelY + 15);
    text("X", 20 + panelW - 15, cy - 5);
    text("Y", cx + 5, panelY + 15);

    synchronized (trayectoria) {
    if (trayectoria.size() > 1) {
        strokeWeight(2);
        noFill();
        for (int i = 1; i < trayectoria.size(); i++) {
            float[] p0 = trayectoria.get(i - 1);
            float[] p1 = trayectoria.get(i);
            float t = map(i, 0, trayectoria.size(), 0, 1);
            stroke(lerpColor(color(60, 100, 255), color(255, 60, 60), t));
            line(
                cx  + p0[0] * escalaTray, cy  - p0[1] * escalaTray,
                cx  + p1[0] * escalaTray, cy  - p1[1] * escalaTray
            );
        }
    }

    }

    fill(255, 220, 60);
    noStroke();
    ellipse(cx + posX * escalaTray, cy - posY * escalaTray, 8, 8);

    fill(160);
    noStroke();
    textSize(11);
    text("px=" + nf(posX, 1, 3) + "m  py=" + nf(posY, 1, 3) + "m  pz=" + nf(posZ, 1, 3) + "m",
         25, panelY + panelH - 8);

    hint(ENABLE_DEPTH_TEST);
}

void keyPressed() {
    if (key == 'r' || key == 'R') {
        ref0 = q0; ref1 = q1; ref2 = q2; ref3 = q3;
    }
    if (key == 'z' || key == 'Z') {   // reinicia la orientacion en la placa (sensor quieto)
        if (puerto != null) puerto.write("z");
        ref0 = 1; ref1 = 0; ref2 = 0; ref3 = 0;
    }
    if (key == '+') escalaTray *= 2;
    if (key == '-') escalaTray /= 2;
    if (key == 'c' || key == 'C') {
        synchronized (trayectoria) { trayectoria.clear(); }
        posX = 0; posY = 0; posZ = 0;
        if (puerto != null) puerto.write("c");
    }
}

void serialEvent(Serial p) {
  try {
    String linea = p.readStringUntil('\n');
    if (linea == null) return;
    linea = trim(linea);
    if (linea.startsWith("q") || linea.startsWith("C") || linea.startsWith("B")) return;

    String[] partes = split(linea, ',');
    if (partes.length >= 11) {
        float _q0 = float(partes[0]);
        float _q1 = float(partes[1]);
        float _q2 = float(partes[2]);
        float _q3 = float(partes[3]);
        float len = sqrt(_q0*_q0 + _q1*_q1 + _q2*_q2 + _q3*_q3);
        if (!Float.isNaN(_q0) && !Float.isInfinite(_q0) && len > 0.5) {
            q0 = _q0; q1 = _q1; q2 = _q2; q3 = _q3;
        }
        posX = float(partes[7]);
        posY = float(partes[8]);
        posZ = float(partes[9]);
        quieto = (int(partes[10]) == 1);

        synchronized (trayectoria) {
            trayectoria.add(new float[]{posX, posY});
            if (trayectoria.size() > MAX_PUNTOS) {
                trayectoria.remove(0);
            }
        }
    }
  } catch (Exception e) {
    // Una linea corrupta no debe matar el hilo serie (antes congelaba la ventana)
    println("Linea serie descartada: " + e);
  }
}

float[] quaternionToMatrix(float w, float x, float y, float z) {
    float[] m = new float[9];
    m[0] = 1 - 2*(y*y + z*z);   m[1] = 2*(x*y - w*z);         m[2] = 2*(x*z + w*y);
    m[3] = 2*(x*y + w*z);        m[4] = 1 - 2*(x*x + z*z);     m[5] = 2*(y*z - w*x);
    m[6] = 2*(x*z - w*y);        m[7] = 2*(y*z + w*x);         m[8] = 1 - 2*(x*x + y*y);
    return m;
}
