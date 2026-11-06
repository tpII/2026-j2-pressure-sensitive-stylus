// =============================================================================
// MotorTrazos - calculo del dibujo a partir de la IMU y del Hall
// J2 - Pressure Sensitive Stylus | Taller de Proyecto II - UNLP
//
// Clase en Java puro (sin Processing) para poder probarla fuera del visor con
// corridas grabadas. Recibe cada muestra cruda del firmware
// ensayo_desplazamiento_hall (100 Hz) y arma los trazos en cm.
//
// Metodo (validado con las pruebas 3, 5, H2 y H3 del 05/11/2026):
//  - REPOSO: el sensor esta quieto. Madgwick con beta (corrige la inclinacion con
//    la gravedad), se actualizan el bias del giroscopo y el modulo de la gravedad,
//    y la velocidad es 0. La posicion del lapiz no cambia.
//  - PASADA: desde que se detecta movimiento hasta el siguiente reposo. La
//    orientacion se integra SOLO con el giroscopo (beta = 0, para que la
//    aceleracion del trazo no tuerza la inclinacion), la aceleracion se rota al
//    marco de la hoja, se le resta la gravedad y se integra dos veces. Mientras
//    dura, el trazo se muestra PROVISORIO.
//  - Al frenar (0,3 s quieto) se cierra la pasada: la velocidad real al final es 0,
//    asi que la que sobra es error y se reparte linealmente en toda la pasada. Se
//    recalcula la posicion y los tramos con contacto del Hall quedan como trazos
//    DEFINITIVOS. La posicion final pasa a ser el origen de la pasada siguiente.
//  - El Hall solo decide que partes de la pasada dejan tinta (contacto) y su
//    grosor (cuanto se aparta de la base: presion). No reinicia la integracion,
//    porque al apoyar o levantar el lapiz puede estar en movimiento.
// =============================================================================
import java.util.ArrayList;

public class MotorTrazos {
    // --- parametros ---
    public static final float DT = 0.01f;
    public static final float G = 9.81f;
    public float betaReposo = 0.1f;
    public float umbralAccG = 0.006f;     // variacion del acelerometro en 0,1 s y | |a| - g |
    public float umbralGiroDps = 1.5f;    // |giro| medio de 0,1 s
    public int muestrasParaReposo = 30;   // 0,3 s quieto cierra la pasada
    public int preroll = 20;              // la pasada arranca 0,2 s antes de detectarla
    public float hallOnMv = 150, hallMaxMv = 900;   // para el grosor
    public float anchoMin = 1.0f, anchoMax = 7.0f;
    public float durMaxPasada = 5.0f;     // mas que esto: aviso de deriva

    // calibracion (calibrar_imu.py): a = (a_cruda - off) / esc ; w = w_cruda * gesc
    public float[] accOff = {0, 0, 0}, accEsc = {1, 1, 1}, giroEsc = {1, 1, 1};

    // --- estado ---
    float[] q = {1, 0, 0, 0};
    float[] bias = new float[3];          // rad/s
    float gMag = 1;
    boolean listo = false;
    int nInit = 0;
    double[] sInitA = new double[3], sInitG = new double[3];

    // historial reciente (para el preroll): cuaternion antes de la muestra y la muestra
    ArrayList<float[]> anillo = new ArrayList<float[]>();
    float[] promLin = new float[3], promGiro = new float[3];
    ArrayList<float[]> ventLin = new ArrayList<float[]>(), ventGiro = new ArrayList<float[]>();   // ventLin: aceleracion cruda [g]

    public boolean enPasada = false;
    ArrayList<float[]> pas = new ArrayList<float[]>();   // {linx, liny, contacto, ancho}
    float vx, vy, px, py;                                  // integracion provisoria
    int quietas = 0, ultimaMovil = 0;

    // --- salida ---
    public float ox = 0, oy = 0;                           // posicion del lapiz (cm) en reposo
    public ArrayList<ArrayList<float[]>> trazos = new ArrayList<ArrayList<float[]>>();  // {x, y, ancho}
    public ArrayList<float[]> provisorio = new ArrayList<float[]>();                    // {x, y, contacto, ancho}
    public float curX = 0, curY = 0;
    public boolean contacto = false;
    public int pasadas = 0;
    public float ultimaDur = 0, ultimaVelSobrante = 0;     // s, cm/s
    public String aviso = "";

    public String estado() {
        if (!listo) return "calibrando: dejar QUIETO";
        if (enPasada) return String.format("pasada en curso (%.1f s)", pas.size() * DT);
        return "reposo";
    }

    public float durPasada() { return enPasada ? pas.size() * DT : 0; }

    public void limpiar() {
        trazos.clear(); provisorio.clear(); ox = oy = 0; curX = curY = 0; pasadas = 0; aviso = "";
    }

    // ax..az en g, gx..gz en grados/s (lectura cruda); hallDifMv = |hall - base|
    public void agregar(float rax, float ray, float raz, float rgx, float rgy, float rgz,
                        float hallDifMv, boolean cont) {
        float[] a = {(rax - accOff[0]) / accEsc[0], (ray - accOff[1]) / accEsc[1], (raz - accOff[2]) / accEsc[2]};
        float k = (float) (Math.PI / 180.0);
        float[] w = {rgx * giroEsc[0] * k, rgy * giroEsc[1] * k, rgz * giroEsc[2] * k};
        contacto = cont;
        float ancho = cont ? anchoMin + (anchoMax - anchoMin) * clamp((hallDifMv - hallOnMv) / (hallMaxMv - hallOnMv)) : 0;

        if (!listo) {                     // 1 s quieto para arrancar
            for (int i = 0; i < 3; i++) { sInitA[i] += a[i]; sInitG[i] += w[i]; }
            if (++nInit >= 100) {
                float[] g = {(float) (sInitA[0] / nInit), (float) (sInitA[1] / nInit), (float) (sInitA[2] / nInit)};
                for (int i = 0; i < 3; i++) bias[i] = (float) (sInitG[i] / nInit);
                gMag = norma(g);
                desdeGravedad(g);
                listo = true;
            }
            return;
        }

        float[] wc = {w[0] - bias[0], w[1] - bias[1], w[2] - bias[2]};
        float[] qAntes = q.clone();
        madgwick(wc, a, enPasada ? 0 : betaReposo);
        float[] lin = lineal(a);

        // deteccion de movimiento, en ventanas de 0,1 s y SIN usar la orientacion
        // (durante la pasada la orientacion sale solo del giroscopo y puede tener un
        // error chico; si se usara la aceleracion lineal, ese error se veria como
        // movimiento y la pasada no se cerraria nunca):
        //  - el acelerometro cambia (desvio de sus lecturas en la ventana),
        //  - o su modulo se aparta de la gravedad,
        //  - o el giroscopo gira.
        empujar(ventLin, promLin, a);
        empujar(ventGiro, promGiro, new float[] {Math.abs(wc[0]), Math.abs(wc[1]), Math.abs(wc[2])});
        float var = 0;
        for (float[] v : ventLin)
            for (int i = 0; i < 3; i++) var += (v[i] - promLin[i]) * (v[i] - promLin[i]);
        float desvio = (float) Math.sqrt(var / Math.max(1, ventLin.size()));
        boolean mueve = ventLin.size() < 10 ? enPasada
            : (desvio > umbralAccG || Math.abs(norma(promLin) - gMag) > umbralAccG || norma(promGiro) / k > umbralGiroDps);

        float[] reg = {qAntes[0], qAntes[1], qAntes[2], qAntes[3], a[0], a[1], a[2], wc[0], wc[1], wc[2],
                       cont ? 1 : 0, ancho};
        anillo.add(reg);
        if (anillo.size() > preroll + 1) anillo.remove(0);

        if (!enPasada) {
            if (mueve) {
                // la pasada arranca 0,2 s antes: se rehace la orientacion sin beta desde ahi
                enPasada = true; aviso = "";
                pas.clear(); provisorio.clear();
                vx = vy = px = py = 0; quietas = 0;
                float[] r0 = anillo.get(0);
                q = new float[] {r0[0], r0[1], r0[2], r0[3]};
                for (float[] r : anillo) {
                    madgwick(new float[] {r[7], r[8], r[9]}, new float[] {r[4], r[5], r[6]}, 0);
                    paso(lineal(new float[] {r[4], r[5], r[6]}), r[10] > 0.5f, r[11]);
                }
                ultimaMovil = pas.size();
            } else {
                // reposo: se actualizan el bias del giroscopo y |g|; la posicion no cambia
                for (int i = 0; i < 3; i++) bias[i] += 0.01f * wc[i];
                gMag += 0.01f * (norma(a) - gMag);
                curX = ox; curY = oy;
            }
            return;
        }

        paso(lin, cont, ancho);
        if (mueve) { quietas = 0; ultimaMovil = pas.size(); }
        else if (++quietas >= muestrasParaReposo) cerrarPasada();
        if (pas.size() * DT > durMaxPasada && aviso.length() == 0)
            aviso = "pasada larga: frenar mas seguido (la deriva crece con t^2)";
    }

    void paso(float[] lin, boolean cont, float ancho) {
        pas.add(new float[] {lin[0], lin[1], cont ? 1 : 0, ancho});
        vx += lin[0] * DT; vy += lin[1] * DT;
        px += vx * DT; py += vy * DT;
        curX = ox + px * 100; curY = oy + py * 100;
        provisorio.add(new float[] {curX, curY, cont ? 1 : 0, ancho});
    }

    void cerrarPasada() {
        int n = Math.min(pas.size(), ultimaMovil + preroll);   // hasta 0,2 s despues del ultimo movimiento
        // velocidad sin corregir al final
        float ux = 0, uy = 0;
        for (int i = 0; i < n; i++) { ux += pas.get(i)[0] * DT; uy += pas.get(i)[1] * DT; }
        float x = 0, y = 0, wx = 0, wy = 0;
        ArrayList<float[]> trazo = null;
        for (int i = 0; i < n; i++) {
            float[] s = pas.get(i);
            wx += s[0] * DT; wy += s[1] * DT;
            float f = (i + 1) / (float) n;
            x += (wx - f * ux) * DT; y += (wy - f * uy) * DT;
            float X = ox + x * 100, Y = oy + y * 100;
            if (s[2] > 0.5f) {
                if (trazo == null) { trazo = new ArrayList<float[]>(); trazos.add(trazo); }
                trazo.add(new float[] {X, Y, s[3]});
            } else trazo = null;
        }
        ox += x * 100; oy += y * 100;
        curX = ox; curY = oy;
        ultimaDur = n * DT;
        ultimaVelSobrante = (float) Math.hypot(ux, uy) * 100;
        pasadas++;
        enPasada = false;
        pas.clear(); provisorio.clear();
        ventLin.clear(); ventGiro.clear(); promLin = new float[3]; promGiro = new float[3];
    }

    // ---------------------------------------------------------------- filtro --
    void desdeGravedad(float[] g) {
        float r = (float) Math.atan2(g[1], g[2]);
        float p = (float) Math.atan2(-g[0], Math.sqrt(g[1] * g[1] + g[2] * g[2]));
        float cr = (float) Math.cos(r / 2), sr = (float) Math.sin(r / 2);
        float cp = (float) Math.cos(p / 2), sp = (float) Math.sin(p / 2);
        q = new float[] {cr * cp, sr * cp, cr * sp, -sr * sp};
    }

    void madgwick(float[] w, float[] a, float beta) {
        float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
        float d0 = 0.5f * (-q1 * w[0] - q2 * w[1] - q3 * w[2]);
        float d1 = 0.5f * (q0 * w[0] + q2 * w[2] - q3 * w[1]);
        float d2 = 0.5f * (q0 * w[1] - q1 * w[2] + q3 * w[0]);
        float d3 = 0.5f * (q0 * w[2] + q1 * w[1] - q2 * w[0]);
        if (beta > 0) {
            float na = norma(a);
            if (na > 0) {
                float ax = a[0] / na, ay = a[1] / na, az = a[2] / na;
                float s0 = 4 * q0 * q2 * q2 + 2 * q2 * ax + 4 * q0 * q1 * q1 - 2 * q1 * ay;
                float s1 = 4 * q1 * q3 * q3 - 2 * q3 * ax + 4 * q0 * q0 * q1 - 2 * q0 * ay - 4 * q1 + 8 * q1 * q1 * q1 + 8 * q1 * q2 * q2 + 4 * q1 * az;
                float s2 = 4 * q0 * q0 * q2 + 2 * q0 * ax + 4 * q2 * q3 * q3 - 2 * q3 * ay - 4 * q2 + 8 * q2 * q1 * q1 + 8 * q2 * q2 * q2 + 4 * q2 * az;
                float s3 = 4 * q1 * q1 * q3 - 2 * q1 * ax + 4 * q2 * q2 * q3 - 2 * q2 * ay;
                float ns = (float) Math.sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
                if (ns > 0) { d0 -= beta * s0 / ns; d1 -= beta * s1 / ns; d2 -= beta * s2 / ns; d3 -= beta * s3 / ns; }
            }
        }
        q0 += d0 * DT; q1 += d1 * DT; q2 += d2 * DT; q3 += d3 * DT;
        float nq = (float) Math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q = new float[] {q0 / nq, q1 / nq, q2 / nq, q3 / nq};
    }

    // aceleracion lineal en el marco de la hoja [m/s2]
    float[] lineal(float[] a) {
        float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
        float x = (1 - 2 * (q2 * q2 + q3 * q3)) * a[0] + 2 * (q1 * q2 - q0 * q3) * a[1] + 2 * (q1 * q3 + q0 * q2) * a[2];
        float y = 2 * (q1 * q2 + q0 * q3) * a[0] + (1 - 2 * (q1 * q1 + q3 * q3)) * a[1] + 2 * (q2 * q3 - q0 * q1) * a[2];
        float z = 2 * (q1 * q3 - q0 * q2) * a[0] + 2 * (q2 * q3 + q0 * q1) * a[1] + (1 - 2 * (q1 * q1 + q2 * q2)) * a[2];
        return new float[] {x * G, y * G, (z - gMag) * G};
    }

    void empujar(ArrayList<float[]> vent, float[] prom, float[] v) {
        vent.add(v);
        for (int i = 0; i < 3; i++) prom[i] += v[i] / 10f;
        if (vent.size() > 10) {
            float[] o = vent.remove(0);
            for (int i = 0; i < 3; i++) prom[i] -= o[i] / 10f;
        }
    }

    static float norma(float[] v) { return (float) Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); }
    static float clamp(float x) { return x < 0 ? 0 : (x > 1 ? 1 : x); }

    // largo (cm) de un trazo definitivo
    public static float largo(ArrayList<float[]> t) {
        float L = 0;
        for (int i = 1; i < t.size(); i++) L += (float) Math.hypot(t.get(i)[0] - t.get(i - 1)[0], t.get(i)[1] - t.get(i - 1)[1]);
        return L;
    }
}
