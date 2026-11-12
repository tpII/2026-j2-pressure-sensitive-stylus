// =============================================================================
// MotorLapiz - trazos cortos con IMU + Hall, adaptado al LAPIZ ARMADO
// J2 - Pressure Sensitive Stylus | Taller de Proyecto II - UNLP
//
// Es MotorTrazos (visor_dibujo_hall) con los cambios que salieron de las
// pruebas del lapiz armado del 11/11:
//  1. BIAS DEL GIROSCOPO: con el lapiz en la mano el giroscopo varia 1-2 grados/s
//     por el pulso (apoyado, 0,05). Si el bias se recalibra en las pausas en la
//     mano, toma el pulso como bias. Ahora se mide al arrancar con el lapiz
//     ACOSTADO en la mesa y solo se actualiza cuando vuelve a estar apoyado
//     (giroscopo con desvio < 0,2 grados/s). Tambien se puede fijar (biasFijoDps).
//  2. PAUSAS EN LA MANO: los umbrales de "quieto" de MotorTrazos (6 mg, 1,5
//     grados/s) no se cumplen nunca con el lapiz en la mano, y la pasada no se
//     cerraba. Ahora hay umbrales de pausa en la mano (20 mg, 6 grados/s).
//  3. PUNTA: la IMU esta a 12,5 cm de la punta, sobre el eje Y del sensor (Y
//     apunta hacia arriba). Se dibuja la posicion de la PUNTA:
//       punta = posicion de la IMU + R(q) * vectorPunta  (vectorPunta = 0, -12,5, 0 cm)
//  4. Si una pasada gira mas de giroMaxPasada (por ejemplo, levantar el lapiz de
//     la mesa), no se dibuja: con giros grandes el error de orientacion es de
//     varios grados y el trazo sale cualquier cosa.
// El resto es igual: orientacion solo con el giroscopo durante la pasada,
// correccion de velocidad al frenar, tinta y grosor por el Hall.
// =============================================================================
import java.util.ArrayList;

public class MotorLapiz {
    // --- parametros ---
    public static final float DT = 0.01f;
    public static final float G = 9.81f;
    public float betaReposo = 0.1f;
    public float umbralAccG = 0.02f;       // pausa en la mano: desvio del acelerometro en 0,1 s y | |a| - g |
    public float umbralGiroDps = 6.0f;     // pausa en la mano: |giro| medio de 0,1 s
    public float apoyadoGiroStdDps = 0.2f; // apoyado en la mesa: desvio del giroscopo en 0,5 s
    public int muestrasParaReposo = 30;    // 0,3 s en pausa cierra la pasada
    public int preroll = 20;
    public float giroMaxPasada = 120;      // grados: mas que esto, la pasada no se dibuja
    public float hallOnMv = 150, hallMaxMv = 900;
    public float anchoMin = 1.0f, anchoMax = 7.0f;
    public float durMaxPasada = 3.0f;
    public float[] vectorPunta = {0, -12.5f, 0};   // cm, ejes del sensor (IMU -> punta)
    public float[] biasFijoDps = null;             // si no es null, bias fijo (no se estima)

    public float[] accOff = {0, 0, 0}, accEsc = {1, 1, 1}, giroEsc = {1, 1, 1};

    // --- estado ---
    float[] q = {1, 0, 0, 0};
    float[] bias = new float[3];
    float gMag = 1;
    boolean listo = false;
    int nInit = 0;
    double[] sInitA = new double[3], sInitG = new double[3];
    ArrayList<float[]> anillo = new ArrayList<float[]>();
    float[] promLin = new float[3], promGiro = new float[3];
    ArrayList<float[]> ventLin = new ArrayList<float[]>(), ventGiro = new ArrayList<float[]>();
    ArrayList<float[]> ventApoyo = new ArrayList<float[]>();   // giroscopo con signo, 0,5 s

    public boolean enPasada = false, apoyado = false;
    ArrayList<float[]> pas = new ArrayList<float[]>();   // {linx, liny, contacto, ancho, q0, q1, q2, q3, giro}
    float vx, vy, px, py, giroPasada;
    float[] puntaIni = new float[3];
    int quietas = 0, ultimaMovil = 0;

    // --- salida ---
    public float ox = 0, oy = 0;
    public ArrayList<ArrayList<float[]>> trazos = new ArrayList<ArrayList<float[]>>();
    public ArrayList<float[]> provisorio = new ArrayList<float[]>();
    public float curX = 0, curY = 0;
    public boolean contacto = false;
    public int pasadas = 0, descartadas = 0;
    public float ultimaDur = 0, ultimaVelSobrante = 0, ultimoGiro = 0;
    public float[] ultimoTrazoIni = null, ultimoTrazoFin = null;   // para alinear la vista
    public String aviso = "";

    public String estado() {
        if (!listo) return "calibrando: lapiz ACOSTADO en la mesa, quieto";
        if (enPasada) return String.format("pasada en curso (%.1f s)", pas.size() * DT);
        return apoyado ? "apoyado (recalibrando bias)" : "pausa";
    }

    public float durPasada() { return enPasada ? pas.size() * DT : 0; }
    public float[] biasDps() { float k = (float) (180 / Math.PI); return new float[] {bias[0] * k, bias[1] * k, bias[2] * k}; }

    public void limpiar() {
        trazos.clear(); provisorio.clear(); ox = oy = 0; curX = curY = 0; pasadas = 0; descartadas = 0; aviso = "";
        ultimoTrazoIni = ultimoTrazoFin = null;
    }

    public void agregar(float rax, float ray, float raz, float rgx, float rgy, float rgz,
                        float hallDifMv, boolean cont) {
        float[] a = {(rax - accOff[0]) / accEsc[0], (ray - accOff[1]) / accEsc[1], (raz - accOff[2]) / accEsc[2]};
        float k = (float) (Math.PI / 180.0);
        float[] w = {rgx * giroEsc[0] * k, rgy * giroEsc[1] * k, rgz * giroEsc[2] * k};
        contacto = cont;
        float ancho = cont ? anchoMin + (anchoMax - anchoMin) * clamp((hallDifMv - hallOnMv) / (hallMaxMv - hallOnMv)) : 0;

        if (!listo) {
            for (int i = 0; i < 3; i++) { sInitA[i] += a[i]; sInitG[i] += w[i]; }
            if (++nInit >= 100) {
                float[] g = {(float) (sInitA[0] / nInit), (float) (sInitA[1] / nInit), (float) (sInitA[2] / nInit)};
                for (int i = 0; i < 3; i++) bias[i] = (float) (sInitG[i] / nInit);
                if (biasFijoDps != null) for (int i = 0; i < 3; i++) bias[i] = biasFijoDps[i] * k;
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

        empujar(ventLin, promLin, a, 10);
        empujar(ventGiro, promGiro, new float[] {Math.abs(wc[0]), Math.abs(wc[1]), Math.abs(wc[2])}, 10);
        float var = 0;
        for (float[] v : ventLin)
            for (int i = 0; i < 3; i++) var += (v[i] - promLin[i]) * (v[i] - promLin[i]);
        float desvio = (float) Math.sqrt(var / Math.max(1, ventLin.size()));
        boolean mueve = ventLin.size() < 10 ? enPasada
            : (desvio > umbralAccG || Math.abs(norma(promLin) - gMag) > umbralAccG || norma(promGiro) / k > umbralGiroDps);

        // apoyado en la mesa: giroscopo casi sin variacion durante 0,5 s
        ventApoyo.add(w.clone());
        if (ventApoyo.size() > 50) ventApoyo.remove(0);
        apoyado = false;
        if (ventApoyo.size() == 50) {
            float[] m = new float[3], s = new float[3];
            for (float[] v : ventApoyo) for (int i = 0; i < 3; i++) m[i] += v[i] / 50;
            for (float[] v : ventApoyo) for (int i = 0; i < 3; i++) s[i] += (v[i] - m[i]) * (v[i] - m[i]) / 50;
            apoyado = Math.sqrt(s[0] + s[1] + s[2]) / k < apoyadoGiroStdDps;
        }

        float giro = norma(wc) * DT;
        float[] reg = {qAntes[0], qAntes[1], qAntes[2], qAntes[3], a[0], a[1], a[2], wc[0], wc[1], wc[2],
                       cont ? 1 : 0, ancho};
        anillo.add(reg);
        if (anillo.size() > preroll + 1) anillo.remove(0);

        if (!enPasada) {
            if (mueve) {
                enPasada = true; aviso = "";
                pas.clear(); provisorio.clear();
                vx = vy = px = py = 0; quietas = 0; giroPasada = 0;
                float[] r0 = anillo.get(0);
                q = new float[] {r0[0], r0[1], r0[2], r0[3]};
                puntaIni = rotar(q, vectorPunta);
                for (float[] r : anillo) {
                    float[] qr = q.clone();
                    madgwick(new float[] {r[7], r[8], r[9]}, new float[] {r[4], r[5], r[6]}, 0);
                    paso(lineal(new float[] {r[4], r[5], r[6]}), r[10] > 0.5f, r[11], qr,
                         norma(new float[] {r[7], r[8], r[9]}) * DT);
                }
                ultimaMovil = pas.size();
            } else {
                if (apoyado && biasFijoDps == null) {        // solo apoyado: el pulso no entra como bias
                    for (int i = 0; i < 3; i++) bias[i] += 0.01f * wc[i];
                    gMag += 0.01f * (norma(a) - gMag);
                }
                curX = ox; curY = oy;
            }
            return;
        }

        paso(lin, cont, ancho, qAntes, giro);
        if (mueve) { quietas = 0; ultimaMovil = pas.size(); }
        else if (++quietas >= muestrasParaReposo) cerrarPasada();
        if (pas.size() * DT > durMaxPasada && aviso.length() == 0)
            aviso = "pasada larga: frenar mas seguido (la deriva crece con t^2)";
    }

    void paso(float[] lin, boolean cont, float ancho, float[] qm, float giro) {
        pas.add(new float[] {lin[0], lin[1], cont ? 1 : 0, ancho, qm[0], qm[1], qm[2], qm[3], giro});
        giroPasada += giro;
        vx += lin[0] * DT; vy += lin[1] * DT;
        px += vx * DT; py += vy * DT;
        float[] pt = rotar(qm, vectorPunta);
        curX = ox + px * 100 + (pt[0] - puntaIni[0]);
        curY = oy + py * 100 + (pt[1] - puntaIni[1]);
        provisorio.add(new float[] {curX, curY, cont ? 1 : 0, ancho});
    }

    void cerrarPasada() {
        int n = Math.min(pas.size(), ultimaMovil + preroll);
        ultimoGiro = (float) Math.toDegrees(giroPasada);
        float ux = 0, uy = 0;
        for (int i = 0; i < n; i++) { ux += pas.get(i)[0] * DT; uy += pas.get(i)[1] * DT; }
        ultimaDur = n * DT;
        ultimaVelSobrante = (float) Math.hypot(ux, uy) * 100;
        pasadas++;
        enPasada = false;
        if (ultimoGiro > giroMaxPasada) {      // levantar o acostar el lapiz: no se dibuja
            descartadas++;
            aviso = String.format("pasada descartada: giro %.0f grados (levantar/apoyar el lapiz)", ultimoGiro);
            // la posicion no cambia
        } else {
            float x = 0, y = 0, wx = 0, wy = 0;
            ArrayList<float[]> trazo = null;
            float[] ini = null, fin = null;
            for (int i = 0; i < n; i++) {
                float[] s = pas.get(i);
                wx += s[0] * DT; wy += s[1] * DT;
                float f = (i + 1) / (float) n;
                x += (wx - f * ux) * DT; y += (wy - f * uy) * DT;
                float[] pt = rotar(new float[] {s[4], s[5], s[6], s[7]}, vectorPunta);
                float X = ox + x * 100 + (pt[0] - puntaIni[0]), Y = oy + y * 100 + (pt[1] - puntaIni[1]);
                if (s[2] > 0.5f) {
                    if (trazo == null) { trazo = new ArrayList<float[]>(); trazos.add(trazo); }
                    trazo.add(new float[] {X, Y, s[3]});
                    if (ini == null) ini = new float[] {X, Y};
                    fin = new float[] {X, Y};
                } else trazo = null;
                if (i == n - 1) { curX = X; curY = Y; }
            }
            ox = curX; oy = curY;
            if (ini != null) { ultimoTrazoIni = ini; ultimoTrazoFin = fin; }
        }
        curX = ox; curY = oy;
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

    static float[] rotar(float[] q, float[] v) {   // R(q) * v
        float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
        return new float[] {
            (1 - 2 * (q2 * q2 + q3 * q3)) * v[0] + 2 * (q1 * q2 - q0 * q3) * v[1] + 2 * (q1 * q3 + q0 * q2) * v[2],
            2 * (q1 * q2 + q0 * q3) * v[0] + (1 - 2 * (q1 * q1 + q3 * q3)) * v[1] + 2 * (q2 * q3 - q0 * q1) * v[2],
            2 * (q1 * q3 - q0 * q2) * v[0] + 2 * (q2 * q3 + q0 * q1) * v[1] + (1 - 2 * (q1 * q1 + q2 * q2)) * v[2]};
    }

    float[] lineal(float[] a) {
        float[] r = rotar(q, a);
        return new float[] {r[0] * G, r[1] * G, (r[2] - gMag) * G};
    }

    void empujar(ArrayList<float[]> vent, float[] prom, float[] v, int n) {
        vent.add(v);
        for (int i = 0; i < 3; i++) prom[i] += v[i] / n;
        if (vent.size() > n) {
            float[] o = vent.remove(0);
            for (int i = 0; i < 3; i++) prom[i] -= o[i] / n;
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
