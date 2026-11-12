"""
comparar_dmp_madgwick.py - DMP del MPU6050 contra el Madgwick propio
J2 - Pressure Sensitive Stylus | Taller de Proyecto II - UNLP

Se usa con firmware/ensayo_dmp_vs_madgwick, que manda en cada muestra el
cuaternion del DMP y la lectura cruda de la IMU. Las variantes del Madgwick se
calculan aca sobre esa misma lectura, asi todas ven los mismos datos:

  DMP        fusion de InvenSense dentro del MPU6050
  M 0,1      Madgwick beta fijo 0,1 (el mejor del ensayo del 01/10)
  M 0,033    Madgwick beta fijo 0,033
  M adapt.   beta 0,033 en movimiento / 0,1 en reposo (madgwick_filter)
  M giro     beta 0 en movimiento / 0,1 en reposo (lo de MotorTrazos y las pruebas del 05/11)

Uso:
  pip install pyserial                     (una vez)
  python tools/comparar_dmp_madgwick.py grabar --puerto COM3 --etiqueta reposo --segundos 70
  python tools/comparar_dmp_madgwick.py calibrar corridas_dmp/<corrida>_caras.csv
                                          (offset y escala por eje del acelerometro -> calibracion_acc_dmp.json)
  python tools/comparar_dmp_madgwick.py analizar          (todas las corridas de corridas_dmp/)

Correccion del acelerometro: CalibrateAccel solo deja bien la pose plana; en las otras caras el
modulo de la gravedad se va hasta ~2 % de 1 g, y el detector de movimiento (tolerancia 6 mg,
la de MotorTrazos) toma esas caras como movimiento. Si existe calibracion_acc_dmp.json, analizar
corrige la lectura cruda (a - offset) / escala antes de todo, como hace el sistema real con
calibracion_imu.json. Al DMP no se le puede aplicar: usa la lectura interna del chip.
  python tools/comparar_dmp_madgwick.py simular           (corridas sinteticas para probar el script)

Etiquetas (el analisis depende de la etiqueta; ver PRUEBAS_PENDIENTES.md):
  reposo         quieto >= 60 s: ruido y deriva de la inclinacion y del yaw
  caras          6 caras, 5 s quieto en cada una: error y convergencia despues de girar
  deslizar_10cm  trazos de 10 cm SIN inclinar, contra una regla, 1 s quieto entre trazos:
                 cuanto se tuerce la inclinacion durante el trazo y error del desplazamiento
  giros          quieto contra un tope, 20-30 s girandolo a mano, y de vuelta al MISMO tope:
                 cuanto no vuelve (inclinacion y yaw) y retardo del DMP
"""
import argparse
import csv
import glob
import json
import math
import os
import sys
import time
from datetime import datetime

import numpy as np

AQUI = os.path.dirname(os.path.abspath(__file__))
CORRIDAS = os.path.join(AQUI, "corridas_dmp")
RESULTADOS = os.path.join(AQUI, "resultados_dmp")
CALIB = os.path.join(AQUI, "calibracion_acc_dmp.json")
G = 9.81
METODOS = ["DMP", "M 0,1", "M 0,033", "M adapt.", "M giro"]
BETAS = {"M 0,1": (0.1, 0.1), "M 0,033": (0.033, 0.033), "M adapt.": (0.033, 0.1), "M giro": (0.0, 0.1)}


# ============================================================ cuaterniones ==
def q_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1*w2 - x1*x2 - y1*y2 - z1*z2, w1*x2 + x1*w2 + y1*z2 - z1*y2,
                     w1*y2 - x1*z2 + y1*w2 + z1*x2, w1*z2 + x1*y2 - y1*x2 + z1*w2])


def q_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def matriz(q):
    w, x, y, z = q
    return np.array([[1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
                     [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
                     [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]])


def gravedad_sensor(Q):
    """Direccion 'arriba' del mundo vista en ejes del sensor (lo que mide el acelerometro quieto)."""
    w, x, y, z = Q[:, 0], Q[:, 1], Q[:, 2], Q[:, 3]
    return np.stack([2*(x*z - w*y), 2*(w*x + y*z), w*w - x*x - y*y + z*z], axis=1)


def angulo(u, v):
    u = u / np.linalg.norm(u, axis=-1, keepdims=True)
    v = v / np.linalg.norm(v, axis=-1, keepdims=True)
    return np.degrees(np.arccos(np.clip(np.sum(u * v, axis=-1), -1, 1)))


def yaw_relativo(Q, q0):
    """Giro alrededor de la vertical del mundo desde q0 (parte 'twist' de q * q0^-1), en grados."""
    out = np.empty(len(Q))
    c0 = q_conj(q0)
    for i, q in enumerate(Q):
        r = q_mul(q, c0)
        out[i] = math.degrees(2 * math.atan2(r[3], r[0]))
    return (out + 180) % 360 - 180


def desde_gravedad(g):
    r = math.atan2(g[1], g[2])
    p = math.atan2(-g[0], math.hypot(g[1], g[2]))
    cr, sr, cp, sp = math.cos(r/2), math.sin(r/2), math.cos(p/2), math.sin(p/2)
    return np.array([cr*cp, sr*cp, cr*sp, -sr*sp])


def madgwick_paso(q, w, a, beta, dt):
    q0, q1, q2, q3 = q
    d = 0.5 * np.array([-q1*w[0] - q2*w[1] - q3*w[2], q0*w[0] + q2*w[2] - q3*w[1],
                        q0*w[1] - q1*w[2] + q3*w[0], q0*w[2] + q1*w[1] - q2*w[0]])
    na = math.sqrt(a[0]**2 + a[1]**2 + a[2]**2)
    if beta > 0 and na > 0:
        ax, ay, az = a / na
        s = np.array([
            4*q0*q2*q2 + 2*q2*ax + 4*q0*q1*q1 - 2*q1*ay,
            4*q1*q3*q3 - 2*q3*ax + 4*q0*q0*q1 - 2*q0*ay - 4*q1 + 8*q1*q1*q1 + 8*q1*q2*q2 + 4*q1*az,
            4*q0*q0*q2 + 2*q0*ax + 4*q2*q3*q3 - 2*q3*ay - 4*q2 + 8*q2*q1*q1 + 8*q2*q2*q2 + 4*q2*az,
            4*q1*q1*q3 - 2*q1*ax + 4*q2*q2*q3 - 2*q2*ay])
        ns = np.linalg.norm(s)
        if ns > 0:
            d -= beta * s / ns
    q = q + d * dt
    return q / np.linalg.norm(q)


# ================================================================== lectura ==
def leer(ruta):
    filas, meta = [], {}
    with open(ruta, encoding="utf-8", errors="ignore") as f:
        for ln in f:
            ln = ln.strip()
            if ln.startswith("#etiqueta="):
                meta["etiqueta"] = ln.split("=", 1)[1]
            elif ln.startswith("M,"):
                p = ln.split(",")
                if len(p) == 15:
                    try:
                        filas.append([float(x) for x in p[1:]])
                    except ValueError:
                        pass
    if not filas:
        return None
    d = np.array(filas)
    return {"t": d[:, 0] / 1000, "dt": np.clip(d[:, 1] / 1e6, 0.002, 0.05), "qd": d[:, 2:6],
            "a": d[:, 6:9], "w": d[:, 9:12], "sat": d[:, 12], "perdidos": d[-1, 13],
            "etiqueta": meta.get("etiqueta", os.path.basename(ruta).split("_", 2)[-1][:-4])}


def detectar_movimiento(a, wdps, umbral_a=0.006, umbral_w=1.5, n=10, con_gref=False):
    """Como MotorTrazos: ventanas de 0,1 s sobre la lectura cruda. Los umbrales se
    suben si el ruido del primer segundo (quieto) es mayor que el de los ensayos
    del 05/11, y un tramo quieto tiene que durar 0,2 s (evita parpadeos)."""
    N = len(a)
    mueve = np.zeros(N, bool)
    gmag = np.median(np.linalg.norm(a[:100], axis=1))
    ruido_a = math.sqrt(np.mean(np.sum((a[:100] - a[:100].mean(0))**2, axis=1)))
    ruido_w = np.linalg.norm(np.std(wdps[:100], axis=0))
    umbral_a = max(umbral_a, 2.0 * ruido_a)
    umbral_w = max(umbral_w, 1.0 * ruido_w)
    bias_w = wdps[:100].mean(0)
    gref = a[:100].mean(0)
    quietas = 0
    for i in range(N):
        j = max(0, i - n + 1)
        va, vw = a[j:i+1], wdps[j:i+1] - bias_w
        if len(va) < n:
            mueve[i] = True
            continue
        desvio = math.sqrt(np.mean(np.sum((va - va.mean(0))**2, axis=1)))
        # con_gref (solo para cortar trazos): ademas, el promedio se aparta de la
        # gravedad del ultimo reposo (detecta deslizamientos suaves, que casi no
        # cambian el modulo de a). Para el filtro se usa el criterio de MotorTrazos.
        m_a = va.mean(0)
        estable = not (desvio > umbral_a or abs(np.linalg.norm(m_a) - gmag) > umbral_a
                       or np.linalg.norm(vw.mean(0)) > umbral_w)
        mueve[i] = not estable
        if con_gref:   # solo para cortar los trazos de 'deslizar'
            quietas = quietas + 1 if estable else 0
            if quietas >= 50:             # 0,5 s estable: nueva gravedad de referencia
                gref = m_a
            mueve[i] = mueve[i] or np.linalg.norm(m_a - gref) > umbral_a
    # tramos quietos de menos de 0,2 s se toman como movimiento
    for (i, j) in tramos(~mueve, 1):
        if j - i < 20 and i > 0:
            mueve[i:j] = True
    return mueve


def calcular(d):
    """Cuaterniones de todos los metodos, muestra a muestra."""
    a, wdps, dt = d["a"], d["w"], d["dt"]
    N = len(a)
    n0 = min(100, N)
    bias = np.radians(wdps[:n0].mean(0))
    q_ini = desde_gravedad(a[:n0].mean(0))
    mueve = detectar_movimiento(a, wdps)
    out = {"DMP": d["qd"] / np.linalg.norm(d["qd"], axis=1, keepdims=True)}
    for nombre, (b_mov, b_rep) in BETAS.items():
        q = q_ini.copy()
        Q = np.empty((N, 4))
        bi = bias.copy()
        for i in range(N):
            w = np.radians(wdps[i]) - bi
            if i >= n0:
                q = madgwick_paso(q, w, a[i], b_mov if mueve[i] else b_rep, dt[i])
                if not mueve[i] and b_mov != b_rep:      # recalibracion del bias en reposo (MotorTrazos)
                    bi += 0.01 * w
            Q[i] = q
        out[nombre] = Q
    return out, mueve


def tramos(mask, minimo):
    """Indices [ini, fin) de los tramos True de al menos 'minimo' muestras."""
    res, i, N = [], 0, len(mask)
    while i < N:
        if mask[i]:
            j = i
            while j < N and mask[j]:
                j += 1
            if j - i >= minimo:
                res.append((i, j))
            i = j
        else:
            i += 1
    return res


# ================================================================ analisis ==
def analizar_reposo(d, Q, mueve):
    ini = int(np.searchsorted(d["t"], d["t"][0] + 5))     # se saltean 5 s
    res = {}
    t = d["t"][ini:] - d["t"][ini]
    for m in METODOS:
        g = gravedad_sensor(Q[m][ini:])
        incl = angulo(g, np.repeat(g[:50].mean(0, keepdims=True), len(g), 0))
        ruido = float(np.std(incl - np.polyval(np.polyfit(t, incl, 1), t)))
        deriva_i = float(np.polyfit(t, incl, 1)[0] * 60)
        yaw = yaw_relativo(Q[m][ini:], Q[m][ini])
        deriva_y = float(np.polyfit(t, np.unwrap(np.radians(yaw)), 1)[0] * 60 * 180 / math.pi)
        res[m] = {"ruido incl. [grados]": ruido, "deriva incl. [grados/min]": deriva_i,
                  "deriva yaw [grados/min]": deriva_y}
    return res


def analizar_caras(d, Q, mueve):
    quietos = tramos(~mueve, 200)                   # >= 2 s quieto
    res = {m: {"err 0,5 s [grados]": [], "err final [grados]": []} for m in METODOS}
    for (i, j) in quietos:
        if i < 100:                                  # el arranque no cuenta
            continue
        verdad = d["a"][j-100:j].mean(0)             # acelerometro quieto: la gravedad
        for m in METODOS:
            g = gravedad_sensor(Q[m])
            res[m]["err 0,5 s [grados]"].append(float(angulo(g[min(i+50, j-1)], verdad)))
            res[m]["err final [grados]"].append(float(angulo(g[j-100:j].mean(0), verdad)))
    return {m: {"poses": len(v["err final [grados]"]),
                "err 0,5 s medio [grados]": np.mean(v["err 0,5 s [grados]"]) if v["err final [grados]"] else float("nan"),
                "err 0,5 s max [grados]": np.max(v["err 0,5 s [grados]"]) if v["err final [grados]"] else float("nan"),
                "err final medio [grados]": np.mean(v["err final [grados]"]) if v["err final [grados]"] else float("nan")}
            for m, v in res.items()}


def analizar_deslizar(d, Q, mueve, largo_real):
    pas = tramos(detectar_movimiento(d["a"], d["w"], con_gref=True), 20)
    res = {m: {"incl": [], "largo": []} for m in METODOS}
    for (i, j) in pas:
        i0, j0 = max(0, i - 10), min(len(mueve), j + 10)
        if i0 < 100:
            continue
        ref = d["a"][i0-50:i0].mean(0)              # gravedad medida justo antes del trazo
        gmag = np.linalg.norm(ref)
        dt = d["dt"][i0:j0]
        for m in METODOS:
            g = gravedad_sensor(Q[m][i0:j0])
            res[m]["incl"].append(float(np.max(angulo(g, np.repeat(ref[None], len(g), 0)))))
            # doble integracion con correccion lineal de velocidad (v = 0 al frenar)
            lin = np.array([matriz(q) @ a for q, a in zip(Q[m][i0:j0], d["a"][i0:j0])])
            lin[:, 2] -= gmag
            v = np.cumsum(lin * G * dt[:, None], axis=0)
            v -= np.outer(np.cumsum(dt) / dt.sum(), v[-1])
            p = np.cumsum(v * dt[:, None], axis=0)
            res[m]["largo"].append(float(np.hypot(p[-1, 0], p[-1, 1]) * 100))
    out = {}
    for m, v in res.items():
        if not v["largo"]:
            continue
        L = np.array(v["largo"])
        r = {"trazos": len(L), "incl. max media [grados]": float(np.mean(v["incl"])), "largo medio [cm]": float(L.mean())}
        if largo_real:
            r["error medio [%]"] = float(np.mean(np.abs(L - largo_real)) / largo_real * 100)
        out[m] = r
    return out


def analizar_giros(d, Q, mueve):
    quietos = tramos(~mueve, 100)
    if len(quietos) < 2:
        return {}
    # inicio y final: los tramos quietos a los dos lados del movimiento mas largo
    # (el primer tramo de la corrida cae en la estabilizacion del DMP)
    k = max(range(len(quietos) - 1), key=lambda k: quietos[k+1][0] - quietos[k][1])
    (i1, j1), (i2, j2) = quietos[k], quietos[k+1]
    a1, a2 = max(i1, j1 - 100), max(i2, j2 - 100)
    verdad = float(angulo(d["a"][a1:j1].mean(0), d["a"][a2:j2].mean(0)))   # cuanto no volvio de verdad
    res = {}
    mov = slice(j1, i2)
    gm = gravedad_sensor(Q["M giro"])[mov]
    X = gm - gm.mean(0)
    for m in METODOS:
        q1 = Q[m][a1:j1].mean(0); q1 /= np.linalg.norm(q1)
        q2 = Q[m][a2:j2].mean(0); q2 /= np.linalg.norm(q2)
        g = gravedad_sensor(np.array([q1, q2]))
        r = q_mul(q2, q_conj(q1))
        # contra un tope en angulo el sensor puede volver girado 90 o 180 grados (rectangulo
        # en un rincon): el error es la distancia al multiplo de 90 mas cercano
        yaw = math.degrees(2 * math.atan2(r[3], r[0])) % 360
        res[m] = {"no vuelve: incl. [grados]": float(angulo(g[0], g[1])),
                  "giro neto yaw [grados]": yaw,
                  "no vuelve: yaw [grados]": abs((yaw + 45) % 90 - 45),
                  "duracion [s]": float(d["t"][i2] - d["t"][j1])}
        # retardo respecto de M giro: correlacion de las tres componentes de la
        # gravedad estimada durante el movimiento, corriendo una serie k muestras
        gy = gravedad_sensor(Q[m])[mov]
        Y = gy - gy.mean(0)
        mejor, lag = -2, 0
        for k in range(-15, 16):
            xa, ya = (X[:len(X)-k], Y[k:]) if k >= 0 else (X[-k:], Y[:len(Y)+k])
            if len(xa) > 10 and xa.std() > 0 and ya.std() > 0:
                c = np.corrcoef(xa.ravel(), ya.ravel())[0, 1]
                if c > mejor:
                    mejor, lag = c, k
        res[m]["retardo vs M giro [ms]"] = lag * float(np.median(d["dt"])) * 1000
    res["_verdad"] = verdad
    return res


def graficar(d, Q, ruta, titulo):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return
    fig, ax = plt.subplots(figsize=(9, 3.6), dpi=120)
    g_ref = d["a"][:100].mean(0)
    for m in METODOS:
        g = gravedad_sensor(Q[m])
        ax.plot(d["t"] - d["t"][0], angulo(g, np.repeat(g_ref[None], len(g), 0)), lw=1, label=m)
    ax.plot(d["t"] - d["t"][0], angulo(d["a"], np.repeat(g_ref[None], len(g), 0)), lw=0.6, color="0.7",
            label="acelerometro", zorder=0)
    ax.set_xlabel("t [s]")
    ax.set_ylabel("inclinacion desde el inicio [grados]")
    ax.set_title(titulo)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, ncol=6)
    fig.tight_layout()
    fig.savefig(ruta)
    plt.close(fig)


def tabla_md(titulo, res, columnas=None):
    filas = {m: v for m, v in res.items() if not m.startswith("_")}
    if not filas:
        return f"### {titulo}\n\nSin datos suficientes.\n"
    cols = columnas or list(next(iter(filas.values())).keys())
    s = f"### {titulo}\n\n| Metodo | " + " | ".join(cols) + " |\n|---|" + "---|" * len(cols) + "\n"
    for m, v in filas.items():
        s += f"| {m} | " + " | ".join(
            (f"{v[c]:.2f}" if isinstance(v[c], float) else str(v[c])) for c in cols) + " |\n"
    return s + "\n"


def cmd_analizar(args):
    rutas = sorted(args.archivos or glob.glob(os.path.join(args.carpeta, "*.csv")))
    if not rutas:
        sys.exit(f"No hay corridas en {args.carpeta}")
    os.makedirs(args.salida, exist_ok=True)
    md = ["# DMP del MPU6050 contra Madgwick\n",
          "Todas las variantes se calcularon sobre la misma lectura cruda de cada corrida "
          "(firmware/ensayo_dmp_vs_madgwick). Menor es mejor en todas las columnas.\n"]
    calib = None
    if os.path.exists(args.calib):
        with open(args.calib, encoding="utf-8") as f:
            calib = json.load(f)
        md.append(f"Lectura del acelerometro corregida con `{os.path.basename(args.calib)}` "
                  f"(de {calib['origen']}); el DMP no se corrige.\n")
    else:
        md.append("Lectura del acelerometro **sin** correccion por eje (no hay calibracion_acc_dmp.json).\n")
    for ruta in rutas:
        d = leer(ruta)
        if d is None or len(d["t"]) < 300:
            print(f"{os.path.basename(ruta)}: muy corta, se saltea")
            continue
        if calib:
            d["a"] = (d["a"] - np.array(calib["acel_offset_g"])) / np.array(calib["acel_escala"])
        et = d["etiqueta"]
        Q, mueve = calcular(d)
        nombre = os.path.basename(ruta)
        md.append(f"## {nombre}\n")
        md.append(f"Etiqueta: `{et}` · {d['t'][-1] - d['t'][0]:.0f} s · paquetes perdidos: {int(d['perdidos'])} · "
                  f"muestras saturadas: {int(d['sat'].sum())}" + ("" if d["t"][0] > 20 else
                  " · **ojo: la corrida arranco antes de los 20 s del DMP**") + "\n")
        if et.startswith("reposo"):
            md.append(tabla_md("Reposo", analizar_reposo(d, Q, mueve)))
        elif et.startswith("caras"):
            md.append(tabla_md("Caras: error de la inclinacion contra el acelerometro quieto", analizar_caras(d, Q, mueve)))
        elif et.startswith("deslizar"):
            num = "".join(c for c in et.split("_")[-1] if c.isdigit())
            md.append(tabla_md("Deslizar sin inclinar: inclinacion falsa y desplazamiento",
                               analizar_deslizar(d, Q, mueve, float(num) if num else None)))
        elif et.startswith("giros"):
            r = analizar_giros(d, Q, mueve)
            md.append(tabla_md("Giros y vuelta al mismo tope", r))
            if "_verdad" in r:
                md.append(f"El acelerometro indica que la pose final difiere {r['_verdad']:.2f} grados de la "
                          "inicial (cuanto no se volvio exactamente al tope).\n")
        else:
            md.append("Etiqueta sin analisis especifico; solo grafico.\n")
        png = os.path.join(args.salida, nombre[:-4] + ".png")
        graficar(d, Q, png, f"{nombre} ({et})")
        md.append(f"![{nombre}]({os.path.basename(png)})\n")
        print(f"{nombre}: listo")
    salida = os.path.join(args.salida, "resumen_dmp_vs_madgwick.md")
    with open(salida, "w", encoding="utf-8") as f:
        f.write("\n".join(md))
    print(f"\nResumen: {salida}")


# ================================================================ calibrar ==
def cmd_calibrar(args):
    from calibrar_imu import ajustar_acel, caras_cubiertas, tramos_quietos
    d = leer(args.archivo)
    if d is None:
        sys.exit(f"No se pudo leer {args.archivo}")
    medias = tramos_quietos(d["a"], d["w"])
    caras = caras_cubiertas(medias)
    print(f"Posiciones quietas: {len(medias)}  caras: {' '.join(caras)}")
    if len(caras) < 6:
        sys.exit("Faltan caras: hacen falta las 6 para ajustar offset y escala de cada eje")
    o, s, res = ajustar_acel(medias)
    antes = np.abs(np.linalg.norm(medias, axis=1) - 1)
    print(f"offset [g] = {np.round(o, 4)}   escala = {np.round(s, 4)}")
    print(f"error del modulo: antes max {antes.max()*1000:.1f} mg -> despues max {np.abs(res).max()*1000:.1f} mg")
    with open(args.salida, "w", encoding="utf-8") as f:
        json.dump({"acel_offset_g": o.tolist(), "acel_escala": s.tolist(), "acel_posiciones": len(medias),
                   "acel_caras": caras, "origen": os.path.basename(args.archivo)}, f, indent=2)
    print(f"Guardado: {args.salida}")


# ================================================================== grabar ==
def cmd_grabar(args):
    try:
        import serial
    except ImportError:
        sys.exit("Falta pyserial: pip install pyserial")
    os.makedirs(args.carpeta, exist_ok=True)
    s = serial.Serial(args.puerto, args.baud, timeout=0.2)
    # Reinicio explicito del ESP32 (EN por RTS, como esptool): al abrir el puerto Windows a veces
    # lo reinicia y a veces no. Asi cada corrida recalibra y el t del firmware arranca en 0.
    s.dtr = False
    s.rts = True
    time.sleep(0.1)
    s.rts = False
    sello = datetime.now().strftime("%Y%m%d_%H%M%S")
    ruta = os.path.join(args.carpeta, f"{sello}_{args.etiqueta}.csv")
    print(f"Grabando {ruta} ({'Ctrl+C para parar' if not args.segundos else f'{args.segundos} s'})")
    buf, n, t_ini, ult = b"", 0, time.time(), 0
    with open(ruta, "w", encoding="utf-8") as f:
        f.write(f"#etiqueta={args.etiqueta}\n")
        try:
            while not args.segundos or time.time() - t_ini < args.segundos:
                if n == 0 and time.time() - t_ini > 20:
                    print("\nNo llegan muestras en 20 s: revisar el cable y la conexion del MPU6050")
                    break
                buf += s.read(4096)
                *lineas, buf = buf.split(b"\n")
                for ln in lineas:
                    ln = ln.decode(errors="ignore").strip()
                    if not ln:
                        continue
                    f.write(ln + "\n")
                    if ln.startswith("#"):
                        print(ln)
                    elif ln.startswith("M,"):
                        n += 1
                        if time.time() - ult > 0.5:
                            ult = time.time()
                            p = ln.split(",")
                            if len(p) == 15:
                                print(f"\r t={float(p[1])/1000:6.1f} s  muestras={n:6d}  perdidos={p[14]}  "
                                      f"{'(DMP estabilizando)' if float(p[1]) < 20000 else '':20s}", end="")
        except KeyboardInterrupt:
            pass
    print(f"\nGuardado: {ruta} ({n} muestras)")


# ================================================================ simular ==
def cmd_simular(args):
    """Corridas sinteticas con un 'DMP' falso (Madgwick beta 0,05 con 20 ms de retardo)."""
    rng = np.random.default_rng(1)
    os.makedirs(args.carpeta, exist_ok=True)

    def rot(eje, ang):
        e = np.array(eje, float) / np.linalg.norm(eje)
        return np.concatenate([[math.cos(ang/2)], e * math.sin(ang/2)])

    def generar(etiqueta, guion):
        dt, q, p_prev, v_prev = 0.01, np.array([1.0, 0, 0, 0]), np.zeros(3), np.zeros(3)
        filas, t = [], 0.0
        for (dur, eje, ang, despl) in guion:
            n = int(dur / dt)
            for k in range(n):
                s0, s1 = k / n, (k + 1) / n
                mj = lambda s: 10*s**3 - 15*s**4 + 6*s**5
                dang = ang * (mj(s1) - mj(s0)) if eje else 0
                w_s = np.array(eje, float) / np.linalg.norm(eje) * dang / dt if eje else np.zeros(3)
                if eje:
                    q = q_mul(q, rot(eje, dang)); q /= np.linalg.norm(q)
                pos = np.array(despl) * mj(s1) if despl else np.zeros(3)
                pos0 = np.array(despl) * mj(s0) if despl else np.zeros(3)
                v = (pos - pos0) / dt
                acc_w = (v - v_prev) / dt / G if despl else np.zeros(3)
                v_prev = v if despl else np.zeros(3)
                a_s = matriz(q).T @ (acc_w + np.array([0, 0, 1]))
                vib = 0.006 if despl else 0.0                  # vibracion del roce al deslizar
                filas.append((t, a_s + rng.normal(0, 0.0017 + vib, 3), np.degrees(w_s) + rng.normal(0, 0.05, 3) + [0.02, -0.03, 0.01]))
                t += dt
            v_prev = np.zeros(3)
        A = np.array([f[1] for f in filas]); W = np.array([f[2] for f in filas])
        qd, QD = desde_gravedad(A[:100].mean(0)), []
        for i in range(len(A)):
            qd = madgwick_paso(qd, np.radians(W[i]), A[i], 0.05, dt)
            QD.append(qd)
        QD = np.array(QD); QD = np.vstack([np.repeat(QD[:1], 2, 0), QD[:-2]])    # 20 ms de retardo
        ruta = os.path.join(args.carpeta, f"sim_{etiqueta}.csv")
        with open(ruta, "w") as f:
            f.write(f"#etiqueta={etiqueta}\n")
            for i, (t, a, w) in enumerate(filas):
                qq = QD[i]
                f.write(f"M,{int((t + 25) * 1000)},10000,{qq[0]:.6f},{qq[1]:.6f},{qq[2]:.6f},{qq[3]:.6f},"
                        f"{a[0]:.5f},{a[1]:.5f},{a[2]:.5f},{w[0]:.3f},{w[1]:.3f},{w[2]:.3f},0,0\n")
        print(ruta)

    quieto = (2, None, 0, None)
    generar("reposo", [(70, None, 0, None)])
    generar("caras", [quieto] + [x for e in ([1, 0, 0], [1, 0, 0], [0, 1, 0], [0, 1, 0], [1, 0, 0])
                                 for x in ((1.2, e, math.pi / 2, None), (5, None, 0, None))])
    generar("deslizar_10cm", [quieto] + [x for k in range(5) for x in ((1.5, None, 0, [0.1 if k % 2 == 0 else -0.1, 0, 0]),
                                                                   (1.5, None, 0, None))])
    ida = [(2.5, [1, 0.3, 0], 0.8, None), (2.5, [0, 1, 0.5], -1.2, None), (2.5, [0.3, 0.2, 1], 1.5, None)]
    vuelta = [(d, e, -a, None) for (d, e, a, _) in reversed(ida)]
    generar("giros", [(3, None, 0, None)] + ida + vuelta + [(3, None, 0, None)])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    g = sub.add_parser("grabar")
    g.add_argument("--puerto", default="COM3")
    g.add_argument("--baud", type=int, default=921600)
    g.add_argument("--etiqueta", required=True)
    g.add_argument("--segundos", type=float, default=0)
    g.add_argument("--carpeta", default=CORRIDAS)
    a = sub.add_parser("analizar")
    a.add_argument("archivos", nargs="*")
    a.add_argument("--carpeta", default=CORRIDAS)
    a.add_argument("--salida", default=RESULTADOS)
    a.add_argument("--calib", default=CALIB)
    c = sub.add_parser("calibrar")
    c.add_argument("archivo")
    c.add_argument("--salida", default=CALIB)
    s = sub.add_parser("simular")
    s.add_argument("--carpeta", default=os.path.join(CORRIDAS, "simuladas"))
    args = ap.parse_args()
    {"grabar": cmd_grabar, "analizar": cmd_analizar, "calibrar": cmd_calibrar,
     "simular": cmd_simular}[args.cmd](args)


if __name__ == "__main__":
    main()
