import processing.serial.*;

final int PORT_INDEX = 0;
final int BAUD_RATE  = 115200;

Serial puerto;
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;

float ref0 = 1, ref1 = 0, ref2 = 0, ref3 = 0;

void setup() {
    size(800, 600, P3D);
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
    translate(width / 2, height / 2, 0);

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

    hint(DISABLE_DEPTH_TEST);
    noStroke();
    fill(200);
    textSize(14);
    text("q0: " + nf(q0, 1, 4), 15, 25);
    text("q1: " + nf(q1, 1, 4), 15, 45);
    text("q2: " + nf(q2, 1, 4), 15, 65);
    text("q3: " + nf(q3, 1, 4), 15, 85);
    fill(150);
    text("[R] resetear orientacion", 15, height - 15);
    hint(ENABLE_DEPTH_TEST);
}

void keyPressed() {
    if (key == 'r' || key == 'R') {
        ref0 = q0; ref1 = q1; ref2 = q2; ref3 = q3;
    }
}

void serialEvent(Serial p) {
    String linea = p.readStringUntil('\n');
    if (linea == null) return;
    linea = trim(linea);
    if (linea.startsWith("q")) return;

    String[] partes = split(linea, ',');
    if (partes.length >= 4) {
        float _q0 = float(partes[0]);
        float _q1 = float(partes[1]);
        float _q2 = float(partes[2]);
        float _q3 = float(partes[3]);
        float len = sqrt(_q0*_q0 + _q1*_q1 + _q2*_q2 + _q3*_q3);
        if (!Float.isNaN(_q0) && !Float.isInfinite(_q0) && len > 0.5) {
            q0 = _q0; q1 = _q1; q2 = _q2; q3 = _q3;
        }
    }
}

float[] quaternionToMatrix(float w, float x, float y, float z) {
    float[] m = new float[9];
    m[0] = 1 - 2*(y*y + z*z);   m[1] = 2*(x*y - w*z);         m[2] = 2*(x*z + w*y);
    m[3] = 2*(x*y + w*z);        m[4] = 1 - 2*(x*x + z*z);     m[5] = 2*(y*z - w*x);
    m[6] = 2*(x*z - w*y);        m[7] = 2*(y*z + w*x);         m[8] = 1 - 2*(x*x + y*y);
    return m;
}
