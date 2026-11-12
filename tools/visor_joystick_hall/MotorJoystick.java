// =============================================================================
// MotorJoystick - dibujo por INCLINACION con el Hall (version "joystick")
// J2 - Pressure Sensitive Stylus | Taller de Proyecto II - UNLP
//
// Alternativa a MotorTrazos para el caso en que se descarte el desplazamiento
// por doble integracion. NO es la version final: es para probarla y compararla.
//
// Clase en Java puro (sin Processing), para poder probarla con corridas grabadas.
// Recibe cada muestra cruda del firmware ensayo_desplazamiento_hall (100 Hz).
//
// Metodo:
//  - Orientacion: Madgwick. En reposo beta = 0,1 (corrige la inclinacion con la
//    gravedad y se recalibra el bias del giroscopo). En movimiento beta =
//    betaMovimiento (0 por defecto: solo giroscopo, para que la aceleracion del
//    trazo no tuerza la inclinacion; con B se cambia a 0,033 para comparar).
//  - Cursor: se toma la rotacion del lapiz desde una pose de referencia y se usan
//    solo sus componentes alrededor de los ejes HORIZONTALES (el giro alrededor
//    de la vertical, que deriva sin magnetometro, no mueve el cursor).
//    Desplazamiento = palanca [cm] x angulo [rad] (modelo de pivote: la punta se
//    mueve palanca x angulo cuando el lapiz gira alrededor de los dedos/muneca).
//  - EMBRAGUE (por defecto): al apoyar (Hall) se toma la referencia y el cursor
//    parte de donde quedo; al levantar el cursor queda quieto. Es como levantar
//    el mouse: la hoja no tiene limite y el error no se acumula entre trazos.
//    Con el lapiz quieto se re-toma la referencia en cada muestra, asi el ruido
//    en reposo no se acumula. Sin embrague (modo ABSOLUTO): la referencia se
//    toma solo con R.
//  - Tinta solo con contacto del Hall; el grosor sale de la presion.
// =============================================================================
import java.util.ArrayList;

public class MotorJoystick {
    // --- parametros (se cambian desde el visor) ---
    public static final float DT = 0.01f;
    public float betaReposo = 0.1f;
    public float betaMovimiento = 0.0f;
    public float palancaCm = 8.0f;          // cm de punta por radian de giro
    public boolean embrague = true;
    public int ejeLapiz = -1;               // -1 = automatico; 0, 1, 2 = x, y, z del sensor
    public boolean invX = false, invY = false;
    public float suavizado = 0.5f;          // EMA del cursor (1 = sin suavizado)
    public float umbralAccG = 0.006f, umbralGiroDps = 1.5f;
    public float hallOnMv = 150, hallMaxMv = 900;
    public float anchoMin = 1.0f, anchoMax = 7.0f;

    // calibracion (calibrar_imu.py): a = (a_cruda - off) / esc ; w = w_cruda * gesc
    public float[] accOff = {0, 0, 0}, accEsc = {1, 1, 1}, giroEsc = {1, 1, 1};

    // --- estado ---
    float[] q = {1, 0, 0, 0};
    float[] bias = new float[3];
    float gMag = 1;
    boolean listo = false;
    int nInit = 0;
    double[] sInitA = new double[3], sInitG = new double[3];
    float[] promLin = new float[3], promGiro = new float[3];
    ArrayList<float[]> ventLin = new ArrayList<float[]>(), ventGiro = new ArrayList<float[]>();
    public boolean mueve = false;

    float[][] Rref = null;                  // pose de referencia (sensor -> mundo)
    float ux = 0, uy = 1;                   // "arriba" de la hoja en el plano horizontal del mundo
    float anclaX = 0, anclaY = 0;           // cursor en el momento de la referencia
    float objX = 0, objY = 0;               // cursor sin suavizar
    boolean contactoAntes = false;
    long nMuestra = 0, inicioTrazo = 0;
    ArrayList<float[]> histQ = new ArrayList<float[]>();   // cuaterniones de los ultimos 0,1 s

    // --- salida ---
    public float curX = 0, curY = 0;        // cm
    public boolean contacto = false;
    public float anguloDeg = 0;             // inclinacion respecto de la referencia (sin el giro vertical)
    public int ejeUsado = 2;
    public ArrayList<ArrayList<float[]>> trazos = new ArrayList<ArrayList<float[]>>();   // {x, y, ancho}
    public ArrayList<float[]> stats = new ArrayList<float[]>();  // {largo, ancho bbox, alto bbox, cierre, dur}
    ArrayList<float[]> actual = null;

    public String estado() {
        if (!listo) return "calibrando: dejar QUIETO";
        if (contacto) return "dibujando";
        return mueve ? "en el aire (moviendo)" : "en el aire (quieto)";
    }

    public void limpiar() {
        trazos.clear(); stats.clear(); actual = null;
        curX = curY = objX = objY = anclaX = anclaY = 0;
        if (listo) referencia(true);
    }

    // Toma la pose actual como referencia. conRumbo = true redefine tambien el
    // "arriba" de la hoja (tecla R, con el lapiz en la posicion de escribir).
    public void referencia(boolean conRumbo) { referencia(conRumbo, q); }

    void referencia(boolean conRumbo, float[] qr) {
        float[][] R = matriz(qr);
        Rref = R;
        anclaX = objX; anclaY = objY;
        if (!conRumbo) return;
        // eje del lapiz: el eje del sensor mas alineado con la vertical (automatico)
        int e = ejeLapiz;
        if (e < 0) {
            e = 0;
            for (int i = 1; i < 3; i++) if (Math.abs(R[2][i]) > Math.abs(R[2][e])) e = i;
        }
        ejeUsado = e;
        float hx = R[0][e], hy = R[1][e];
        if (Math.hypot(hx, hy) < 0.3) {     // lapiz casi vertical (o placa plana): se usa otro eje
            int o = (e == 0) ? 1 : 0;
            hx = R[0][o]; hy = R[1][o];
        }
        float n = (float) Math.hypot(hx, hy);
        ux = hx / n; uy = hy / n;
    }

    // ax..az en g, gx..gz en grados/s (lectura cruda); hallDifMv = |hall - base|
    public void agregar(float rax, float ray, float raz, float rgx, float rgy, float rgz,
                        float hallDifMv, boolean cont) {
        nMuestra++;
        float[] a = {(rax - accOff[0]) / accEsc[0], (ray - accOff[1]) / accEsc[1], (raz - accOff[2]) / accEsc[2]};
        float k = (float) (Math.PI / 180.0);
        float[] w = {rgx * giroEsc[0] * k, rgy * giroEsc[1] * k, rgz * giroEsc[2] * k};
        float ancho = cont ? anchoMin + (anchoMax - anchoMin) * clamp((hallDifMv - hallOnMv) / (hallMaxMv - hallOnMv)) : 0;

        if (!listo) {
            for (int i = 0; i < 3; i++) { sInitA[i] += a[i]; sInitG[i] += w[i]; }
            if (++nInit >= 100) {
                float[] g = {(float) (sInitA[0] / nInit), (float) (sInitA[1] / nInit), (float) (sInitA[2] / nInit)};
                for (int i = 0; i < 3; i++) bias[i] = (float) (sInitG[i] / nInit);
                gMag = norma(g);
                desdeGravedad(g);
                listo = true;
                referencia(true);
            }
            return;
        }

        float[] wc = {w[0] - bias[0], w[1] - bias[1], w[2] - bias[2]};

        // deteccion de movimiento (ventanas de 0,1 s, sin usar la orientacion)
        empujar(ventLin, promLin, a);
        empujar(ventGiro, promGiro, new float[] {Math.abs(wc[0]), Math.abs(wc[1]), Math.abs(wc[2])});
        float var = 0;
        for (float[] v : ventLin)
            for (int i = 0; i < 3; i++) var += (v[i] - promLin[i]) * (v[i] - promLin[i]);
        float desvio = (float) Math.sqrt(var / Math.max(1, ventLin.size()));
        mueve = ventLin.size() < 10 ? true
            : (desvio > umbralAccG || Math.abs(norma(promLin) - gMag) > umbralAccG || norma(promGiro) / k > umbralGiroDps);

        madgwick(wc, a, mueve ? betaMovimiento : betaReposo);
        if (!mueve) {
            for (int i = 0; i < 3; i++) bias[i] += 0.01f * wc[i];
            gMag += 0.01f * (norma(a) - gMag);
        }

        // embrague: al apoyar se toma la referencia desde donde quedo el cursor
        contacto = cont;
        if (embrague && cont && !contactoAntes) referencia(false);

        // rotacion desde la referencia, en el marco del mundo: dR = R * Rref^T
        float[][] R = matriz(q);
        float[][] d = new float[3][3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                d[i][j] = R[i][0] * Rref[j][0] + R[i][1] * Rref[j][1] + R[i][2] * Rref[j][2];
        float tr = d[0][0] + d[1][1] + d[2][2];
        float th = (float) Math.acos(Math.max(-1, Math.min(1, (tr - 1) / 2)));
        float f = th < 1e-6 ? 0.5f : (float) (th / (2 * Math.sin(th)));
        float rx = (d[2][1] - d[1][2]) * f, ry = (d[0][2] - d[2][0]) * f;   // rz (vertical) se ignora
        anguloDeg = (float) Math.toDegrees(Math.hypot(rx, ry));

        // la parte de arriba del lapiz se mueve r x z = (ry, -rx); se expresa en la hoja
        float mx = ry * palancaCm, my = -rx * palancaCm;
        float dx = mx * uy - my * ux;       // "derecha" = arriba girado -90 grados
        float dy = mx * ux + my * uy;
        if (invX) dx = -dx;
        if (invY) dy = -dy;

        boolean mover = !embrague || cont;  // con embrague, en el aire el cursor queda quieto
        // con embrague y el lapiz QUIETO el cursor no se mueve y se vuelve a tomar
        // la referencia en cada muestra: el ruido y la correccion del beta en
        // reposo no se acumulan en el dibujo
        // (se usa la pose de hasta 0,1 s atras, pero no anterior al fin del ultimo
        // movimiento: la deteccion de movimiento tarda eso, y asi el giro del
        // comienzo del trazo no se pierde ni se cuenta dos veces el final)
        if (mueve) histQ.clear();
        histQ.add(q.clone());
        if (histQ.size() > 10) histQ.remove(0);
        if (embrague && !mueve) referencia(false, histQ.get(0));
        else if (mover) { objX = anclaX + dx; objY = anclaY + dy; }
        curX += suavizado * (objX - curX);
        curY += suavizado * (objY - curY);

        // trazos y estadisticas
        if (cont) {
            if (actual == null) { actual = new ArrayList<float[]>(); trazos.add(actual); inicioTrazo = nMuestra; }
            actual.add(new float[] {curX, curY, ancho});
        } else if (actual != null) {
            cerrarTrazo();
        }
        contactoAntes = cont;
    }

    void cerrarTrazo() {
        ArrayList<float[]> t = actual;
        actual = null;
        if (t.size() < 2) return;
        float L = largo(t);
        float mnx = 1e9f, mny = 1e9f, mxx = -1e9f, mxy = -1e9f;
        for (float[] p : t) { mnx = Math.min(mnx, p[0]); mxx = Math.max(mxx, p[0]); mny = Math.min(mny, p[1]); mxy = Math.max(mxy, p[1]); }
        float cierre = (float) Math.hypot(t.get(t.size() - 1)[0] - t.get(0)[0], t.get(t.size() - 1)[1] - t.get(0)[1]);
        stats.add(new float[] {L, mxx - mnx, mxy - mny, cierre, (nMuestra - inicioTrazo) * DT});
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

    // matriz sensor -> mundo (z del mundo hacia arriba)
    static float[][] matriz(float[] q) {
        float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
        return new float[][] {
            {1 - 2 * (q2 * q2 + q3 * q3), 2 * (q1 * q2 - q0 * q3), 2 * (q1 * q3 + q0 * q2)},
            {2 * (q1 * q2 + q0 * q3), 1 - 2 * (q1 * q1 + q3 * q3), 2 * (q2 * q3 - q0 * q1)},
            {2 * (q1 * q3 - q0 * q2), 2 * (q2 * q3 + q0 * q1), 1 - 2 * (q1 * q1 + q2 * q2)}};
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

    public static float largo(ArrayList<float[]> t) {
        float L = 0;
        for (int i = 1; i < t.size(); i++) L += (float) Math.hypot(t.get(i)[0] - t.get(i - 1)[0], t.get(i)[1] - t.get(i - 1)[1]);
        return L;
    }
}
