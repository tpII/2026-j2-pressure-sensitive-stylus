"""
Reprocesa en la PC la lectura cruda de las corridas de visor_desplazamiento_integracion_pura
y compara variantes del calculo del desplazamiento sobre las MISMAS muestras.
No hace falta reflashear: el firmware ensayo_desplazamiento_integracion_pura ya graba ax..gz.

Corridas que usa (etiqueta, tecla E en el visor):
  trazo_<L>cm   trazo recto de L cm deslizando el sensor apoyado, sin inclinarlo
  lapiz_<L>cm   trazo recto de L cm con el sensor agarrado como un lapiz
  circulo_<D>cm tres vueltas seguidas sobre un circulo de D cm de diametro
Cada corrida: C (quieto 1 s) -> esperar 1 s -> un solo movimiento -> quieto 1-2 s.

Todas las variantes arrancan el trazo con velocidad 0 y la orientacion tomada de la
gravedad medida justo antes de moverse (como si el Hall marcara el apoyo):
  V0  Madgwick con beta 0,033 durante el trazo (lo que hace el firmware hoy)
  V1  beta = 0 durante el trazo: orientacion solo con el giroscopo
  V2  V1 + correccion de velocidad: al frenar la velocidad real es 0; la velocidad que
      sobra se reparte linealmente en el trazo
  V3  V2 + posicion de la PUNTA en vez de la IMU (solo con --punta)
Ademas se informa "FW": lo que integro el firmware (A) entre el inicio y el fin del trazo.

Corridas con el Hall (firmware ensayo_desplazamiento_hall, columnas hall_mV y contacto):
  hall_trazo_<L>cm     un trazo recto de L cm; el contacto (iman) dura lo que dura el trazo
  encadenado_<N>x<L>cm N tramos de L cm en una sola pasada, sin frenar: el contacto se
                       apoya y se levanta con el sensor EN MOVIMIENTO
Para esas se comparan dos formas de cortar los trazos (orientacion solo con giroscopo):
  H-contacto  cada tramo se integra solo mientras hay contacto: velocidad 0 al apoyar y
              correccion de velocidad al levantar (supone que en esos instantes esta quieto)
  H-corrida   se integra la pasada completa, de reposo a reposo, con la correccion de
              velocidad al final, y el largo de cada tramo sale de la posicion al apoyar y
              al levantar (el resultado esta listo cuando el lapiz se frena, no al levantar)

Si existe tools/calibracion_imu.json (de calibrar_imu.py) se aplica a la lectura cruda.

Uso:
    python tools/reprocesar_desplazamiento.py [corridas...] [--punta 0,0,-12] [--sin-calibracion]
  --punta x,y,z   vector desde la IMU hasta la punta del lapiz [cm], en ejes del sensor
                  (los de la serigrafia del GY-521). Medirlo con una regla.

Genera <corrida>_variantes.png al lado de cada CSV y resumen_variantes.md en la carpeta.
"""
import argparse
import glob
import json
import os
import re
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, AQUI)
from analizar_desplazamiento import detectar_movimiento, leer_corrida  # noqa: E402

G = 9.81
DT = 0.01
BETA_MOV = 0.033
D2R = np.pi / 180
VARIANTES = ["V0", "V1", "V2", "V3"]
COLORES = {"FW": "#999999", "V0": "#c83232", "V1": "#e08a1e", "V2": "#2a9d4b", "V3": "#285ac8"}


# ------------------------------------------------------------------ filtro --
def q_desde_gravedad(g):
    r = np.arctan2(g[1], g[2])
    p = np.arctan2(-g[0], np.hypot(g[1], g[2]))
    cr, sr, cp, sp = np.cos(r / 2), np.sin(r / 2), np.cos(p / 2), np.sin(p / 2)
    return np.array([cr * cp, sr * cp, cr * sp, -sr * sp])


def madgwick(q, w, a, beta):
    q0, q1, q2, q3 = q
    gx, gy, gz = w
    qd = 0.5 * np.array([-q1 * gx - q2 * gy - q3 * gz, q0 * gx + q2 * gz - q3 * gy,
                         q0 * gy - q1 * gz + q3 * gx, q0 * gz + q1 * gy - q2 * gx])
    if beta > 0:
        ax, ay, az = a / np.linalg.norm(a)
        s = np.array([
            4 * q0 * q2 * q2 + 2 * q2 * ax + 4 * q0 * q1 * q1 - 2 * q1 * ay,
            4 * q1 * q3 * q3 - 2 * q3 * ax + 4 * q0 * q0 * q1 - 2 * q0 * ay - 4 * q1 + 8 * q1 * q1 * q1 + 8 * q1 * q2 * q2 + 4 * q1 * az,
            4 * q0 * q0 * q2 + 2 * q0 * ax + 4 * q2 * q3 * q3 - 2 * q3 * ay - 4 * q2 + 8 * q2 * q1 * q1 + 8 * q2 * q2 * q2 + 4 * q2 * az,
            4 * q1 * q1 * q3 - 2 * q1 * ax + 4 * q2 * q2 * q3 - 2 * q2 * ay])
        ns = np.linalg.norm(s)
        if ns > 0:
            qd -= beta * s / ns
    q = q + qd * DT
    return q / np.linalg.norm(q)


def matriz(q):
    q0, q1, q2, q3 = q
    return np.array([[1 - 2 * (q2 * q2 + q3 * q3), 2 * (q1 * q2 - q0 * q3), 2 * (q1 * q3 + q0 * q2)],
                     [2 * (q1 * q2 + q0 * q3), 1 - 2 * (q1 * q1 + q3 * q3), 2 * (q2 * q3 - q0 * q1)],
                     [2 * (q1 * q3 - q0 * q2), 2 * (q2 * q3 + q0 * q1), 1 - 2 * (q1 * q1 + q2 * q2)]])


# ------------------------------------------------------------- variantes --
def calcular(acc, gyr, i0, i1, punta):
    """acc [g], gyr [rad/s] ya sin bias. Integra de i0 a i1 (incluido). Devuelve
    {variante: posiciones Nx3 [cm] en el marco del mundo, z vertical}."""
    # gravedad de referencia: quieto antes del trazo, dejando 0,1 s de margen por si el
    # movimiento empezo un poco antes de lo detectado
    g_ref = acc[max(0, i0 - 60):i0 - 10].mean(axis=0)
    gmag = np.linalg.norm(g_ref)
    q_ini = q_desde_gravedad(g_ref)
    out = {}
    for beta, nombre in [(BETA_MOV, "V0"), (0.0, "V1")]:
        q = q_ini.copy()
        R0 = matriz(q)
        pos, vel, Rs = [np.zeros(3)], np.zeros(3), [R0]
        vels = [np.zeros(3)]
        for i in range(i0, i1):
            q = madgwick(q, gyr[i], acc[i], beta)
            R = matriz(q)
            lin = R @ acc[i] - np.array([0, 0, gmag])
            vel = vel + lin * G * DT
            pos.append(pos[-1] + vel * DT)
            vels.append(vel.copy())
            Rs.append(R)
        out[nombre] = np.array(pos) * 100
        if nombre == "V1":
            vels = np.array(vels)
            n = len(vels)
            frac = np.linspace(0, 1, n)[:, None]
            vc = vels - frac * vels[-1]
            p2 = np.vstack([np.zeros(3), np.cumsum(vc[1:] * DT, axis=0)])
            out["V2"] = p2 * 100
            if punta is not None:
                r = np.array(punta) / 100.0
                brazo = np.array([R @ r - R0 @ r for R in Rs])
                out["V3"] = (p2 + brazo) * 100
    return out


# ------------------------------------------------------------------ Hall --
def lin_mundo(acc, gyr, i0, i1):
    """Aceleracion lineal en el mundo [m/s2] de i0 a i1 con orientacion solo del
    giroscopo desde la gravedad quieta antes de i0. Fila k = muestra i0 + k."""
    g_ref = acc[max(0, i0 - 60):i0 - 10].mean(axis=0)
    gmag = np.linalg.norm(g_ref)
    q = q_desde_gravedad(g_ref)
    lin = [np.zeros(3)]
    for i in range(i0, i1):
        q = madgwick(q, gyr[i], acc[i], 0.0)
        lin.append((matriz(q) @ acc[i] - np.array([0, 0, gmag])) * G)
    return np.array(lin)


def integrar_corregido(lin):
    """Velocidad 0 al principio, correccion lineal para que de 0 al final. Posicion [cm]."""
    v = np.vstack([np.zeros(3), np.cumsum(lin[1:] * DT, axis=0)])
    v = v - np.linspace(0, 1, len(v))[:, None] * v[-1]
    return np.vstack([np.zeros(3), np.cumsum(v[1:] * DT, axis=0)]) * 100


def tramos_contacto(cont, i0, i1, min_muestras=10):
    tr, k = [], i0
    while k < i1:
        if cont[k] > 0.5:
            j = k
            while j < i1 and cont[j] > 0.5:
                j += 1
            if j - k >= min_muestras:
                tr.append((k, j))
            k = j
        else:
            k += 1
    return tr


def pasadas(mov, min_quieto=50, min_mov=20):
    """Tramos de movimiento separados por al menos min_quieto muestras quietas."""
    segs, i, n = [], 0, len(mov)
    while i < n:
        if mov[i]:
            j = i
            while j < n and (mov[j] or mov[j:j + min_quieto].any()):
                j += 1
            if j - i >= min_mov:
                segs.append((i, j))
            i = j
        else:
            i += 1
    return segs


def procesar_hall(c, cal, m):
    """Una corrida puede tener varias pasadas (cada una: movimiento entre dos reposos).
    Cada pasada se analiza por separado, con la gravedad y el bias del giroscopo
    tomados en el reposo justo antes de ella."""
    d, t = c["d"], c["d"]["t"]
    nombre = os.path.basename(c["ruta"])
    if "contacto" not in d:
        print(f"{nombre}: no tiene columnas del Hall (firmware ensayo_desplazamiento_hall), se omite")
        return None
    n_esp = int(m.group(1)) if m.group(1) else 1
    L = float(m.group(2))
    cont = d["contacto"]
    tr_todos = tramos_contacto(cont, 0, len(t))
    if not tr_todos:
        print(f"{nombre}: no hubo contacto, se omite")
        return None
    acc = np.column_stack([d["ax"], d["ay"], d["az"]])
    gyr_c = np.column_stack([d["gx"], d["gy"], d["gz"]])
    acc, gyr_c = aplicar_calibracion(acc, gyr_c, cal)
    mov = mascara_movimiento(d)
    n = len(t)
    # a velocidad constante el IMU no "ve" movimiento: un hueco sin movimiento detectado
    # pero con contacto, entre dos tramos en movimiento, se considera parte de la pasada
    idx = np.flatnonzero(mov)
    if idx.size:
        k = idx[0]
        for j in idx[1:]:
            if j - k > 1 and cont[k + 1:j].min() > 0.5:
                mov[k + 1:j] = True
            k = j
    res = {"archivo": nombre, "tipo": "hall", "n_esp": n_esp, "medida": L, "pasadas": []}
    fig, ax = plt.subplots(figsize=(7, 6.5))
    desplaz = 0.0
    for k, (s0, s1) in enumerate(pasadas(mov), 1):
        i0, i1 = max(0, s0 - 20), min(n - 1, s1 + 20)
        tr = [(max(a, i0), min(b, i1)) for a, b in tr_todos if b > i0 and a < i1]
        tr = [(a, b) for a, b in tr if b - a >= 10]
        if not tr:
            continue                                   # movimiento sin contacto (acomodar)
        pas = {"n": k, "t0": float(t[i0]), "dur": (i1 - i0) * DT, "tramos": [], "avisos": []}
        if i0 < 70 or mov[max(0, i0 - 60):i0 - 10].any():
            pas["avisos"].append("sin reposo antes")
        if s1 >= n - 5:
            pas["avisos"].append("la corrida termina en movimiento")
        if pas["dur"] > 5:
            pas["avisos"].append("lenta")
        if len(tr) != n_esp:
            pas["avisos"].append(f"{len(tr)} tramos")
        g_q = gyr_c[max(0, i0 - 60):max(1, i0 - 10)]
        gyr = (gyr_c - g_q.mean(axis=0)) * D2R
        lin = lin_mundo(acc, gyr, i0, i1)
        p_corr = integrar_corregido(lin)
        for a, b in tr:
            pc = p_corr[b - i0, :2] - p_corr[a - i0, :2]
            ph = integrar_corregido(lin[a - i0:b - i0 + 1])
            pas["tramos"].append({"dur": (b - a) * DT, "corrida": float(np.linalg.norm(pc)),
                                  "contacto": float(np.linalg.norm(ph[-1, :2]))})
        for key in ("corrida", "contacto"):
            pas["err_" + key] = float(np.mean([abs(x[key] - L) / L * 100 for x in pas["tramos"]]))
        pas["valida"] = not any(a in pas["avisos"] for a in ("sin reposo antes", "la corrida termina en movimiento")) \
            and len(tr) == n_esp and pas["dur"] <= 5
        res["pasadas"].append(pas)
        # grafico: cada pasada desplazada para que no se superpongan
        off = np.array([desplaz, 0])
        xy = p_corr[:, :2] + off
        ax.plot(xy[:, 0], xy[:, 1], color="#bbbbbb", lw=1)
        for n_t, (a, b) in enumerate(tr):
            seg = xy[a - i0:b - i0 + 1]
            ax.plot(seg[:, 0], seg[:, 1], color="#285ac8" if pas["valida"] else "#c88a28", lw=2.5)
        ax.text(xy[0, 0], xy[0, 1], f"P{k}", fontsize=8)
        desplaz = xy[:, 0].max() + 5
    plt.close(fig) if not res["pasadas"] else None
    if not res["pasadas"]:
        print(f"{nombre}: ninguna pasada con contacto, se omite")
        return None
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(alpha=0.3)
    ax.set_xlabel("x [cm] (pasadas corridas a la derecha)")
    ax.set_ylabel("y [cm]")
    ax.set_title(f"{nombre}: H-corrida, tramos con contacto (azul = pasada valida)")
    fig.tight_layout()
    fig.savefig(os.path.splitext(c["ruta"])[0] + "_variantes.png", dpi=120)
    plt.close(fig)
    val = [x for p_ in res["pasadas"] if p_["valida"] for x in p_["tramos"]]
    for key in ("corrida", "contacto"):
        res["err_" + key] = float(np.mean([abs(x[key] - L) / L * 100 for x in val])) if val else float("nan")
    return res


def diametro(p):
    p = p[:: max(1, len(p) // 400), :2]
    dif = p[:, None, :] - p[None, :, :]
    return float(np.sqrt((dif ** 2).sum(-1)).max())


def desvio_lateral(p):
    xy = p[:, :2]
    cuerda = xy[-1] - xy[0]
    L = np.linalg.norm(cuerda)
    if L < 1e-6:
        return float("nan")
    n = np.array([-cuerda[1], cuerda[0]]) / L
    return float(np.abs((xy - xy[0]) @ n).max())


def tipo_y_medida(etiqueta):
    m = re.match(r"(trazo|lapiz|circulo)_(\d+(?:\.\d+)?)cm", etiqueta)
    return (m.group(1), float(m.group(2))) if m else (None, None)


def aplicar_calibracion(acc, gyr, cal):
    if cal is None:
        return acc, gyr
    if "acel_offset_g" in cal:
        acc = (acc - np.array(cal["acel_offset_g"])) / np.array(cal["acel_escala"])
    if "giro_escala" in cal:
        gyr = gyr * np.array(cal["giro_escala"])
    return acc, gyr


def mascara_movimiento(d, t_quieto=0.5):
    """True donde el sensor se mueve por encima del ruido del tramo quieto inicial."""
    t = d["t"]
    q = t < t_quieto
    acc = np.column_stack([d["ax"], d["ay"], d["az"]])
    gyr = np.column_stack([d["gx"], d["gy"], d["gz"]])
    k = np.ones(10) / 10
    da = np.convolve(np.linalg.norm(acc - acc[q].mean(0), axis=1), k, "same")
    dg = np.convolve(np.linalg.norm(gyr - gyr[q].mean(0), axis=1), k, "same")
    ua = max(4 * da[q].std() + da[q].mean(), 0.003)
    ug = max(4 * dg[q].std() + dg[q].mean(), 0.5)
    return (da > ua) | (dg > ug)


def ajustar_bordes(d, t0, t1):
    """El detector de analizar_desplazamiento usa un umbral alto y corta los extremos lentos
    del movimiento. Se extienden los bordes hasta que la senal vuelve al nivel de ruido del
    tramo quieto inicial. Corregir la velocidad en un instante en el que el sensor todavia
    se mueve arruina V2, por eso importa."""
    t = d["t"]
    q = t < min(0.5, t0)
    if q.sum() < 10:
        return t0, t1
    acc = np.column_stack([d["ax"], d["ay"], d["az"]])
    gyr = np.column_stack([d["gx"], d["gy"], d["gz"]])
    n = 10
    k = np.ones(n) / n
    da = np.convolve(np.linalg.norm(acc - acc[q].mean(0), axis=1), k, "same")
    dg = np.convolve(np.linalg.norm(gyr - gyr[q].mean(0), axis=1), k, "same")
    ua = max(4 * da[q].std() + da[q].mean(), 0.003)
    ug = max(4 * dg[q].std() + dg[q].mean(), 0.5)
    mov = (da > ua) | (dg > ug)
    i0, i1 = int(np.searchsorted(t, t0)), min(len(t) - 1, int(np.searchsorted(t, t1)))
    while i0 > 0 and mov[i0 - 1]:
        i0 -= 1
    while i1 < len(t) - 1 and mov[i1 + 1]:
        i1 += 1
    return t[max(0, i0 - 20)], t[min(len(t) - 1, i1 + 20)]   # 0,2 s de margen


def procesar(c, cal, punta):
    mh = re.match(r"(?:hall_trazo|encadenado_(\d+)x)_?(\d+(?:\.\d+)?)cm", c["etiqueta"])
    if mh:
        return procesar_hall(c, cal, mh)
    tipo, medida = tipo_y_medida(c["etiqueta"])
    if tipo is None:
        return None
    d, t = c["d"], c["d"]["t"]
    mov = detectar_movimiento(d, c["marcas"])
    if mov is None:
        print(f"{os.path.basename(c['ruta'])}: no se detecto movimiento, se omite")
        return None
    t0, t1 = mov if len(c["marcas"]) >= 2 else ajustar_bordes(d, *mov)
    i0, i1 = int(np.searchsorted(t, t0)), int(np.searchsorted(t, t1))
    if i0 < 40:
        print(f"{os.path.basename(c['ruta'])}: menos de 0,4 s quieto antes del movimiento, se omite")
        return None
    acc = np.column_stack([d["ax"], d["ay"], d["az"]])
    gyr = np.column_stack([d["gx"], d["gy"], d["gz"]])
    acc, gyr = aplicar_calibracion(acc, gyr, cal)
    gyr = (gyr - gyr[:i0].mean(axis=0)) * D2R           # bias del tramo quieto previo
    pos = calcular(acc, gyr, i0, i1, punta)
    fw = np.column_stack([d["Au"][i0:i1 + 1] - d["Au"][i0], d["Av"][i0:i1 + 1] - d["Av"][i0], np.zeros(i1 + 1 - i0)])
    pos = {"FW": fw, **pos}
    res = {"archivo": os.path.basename(c["ruta"]), "tipo": tipo, "medida": medida, "dur": t1 - t0,
           "incl": float(d["incl"][i0:i1].max())}
    for k, p in pos.items():
        if tipo == "circulo":
            res["med_" + k] = diametro(p)
            res["cierre_" + k] = float(np.linalg.norm(p[-1, :2] - p[0, :2]))
        else:
            res["med_" + k] = float(np.linalg.norm(p[-1, :2] - p[0, :2]))
            res["lat_" + k] = desvio_lateral(p)
        res["err_" + k] = 100 * (res["med_" + k] - medida) / medida
    graficar(c, res, pos)
    return res


def graficar(c, res, pos):
    fig, ax = plt.subplots(figsize=(7, 6.5))
    for k, p in pos.items():
        ax.plot(p[:, 0], p[:, 1], color=COLORES[k], lw=1.6 if k != "FW" else 1,
                label=f"{k}: {res['med_' + k]:.1f} cm ({res['err_' + k]:+.0f} %)")
    ax.plot(0, 0, "k+", ms=12)
    if res["tipo"] == "circulo":
        ref = pos.get("V3", pos["V2"])
        cen = ref[:, :2].mean(axis=0)
        a = np.linspace(0, 2 * np.pi, 200)
        ax.plot(cen[0] + res["medida"] / 2 * np.cos(a), cen[1] + res["medida"] / 2 * np.sin(a), "k--", lw=0.8, label=f"circulo real D = {res['medida']:g} cm")
    else:
        ref = pos.get("V3", pos["V2"])
        u = ref[-1, :2] / (np.linalg.norm(ref[-1, :2]) + 1e-9)
        ax.plot([0, u[0] * res["medida"]], [0, u[1] * res["medida"]], "k--", lw=0.8, label=f"largo real {res['medida']:g} cm (en la direccion de la mejor variante)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(alpha=0.3)
    ax.set_xlabel("x [cm]")
    ax.set_ylabel("y [cm]")
    ax.legend(fontsize=8)
    ax.set_title(f"{res['archivo']}  ({res['dur']:.1f} s de movimiento)")
    fig.tight_layout()
    fig.savefig(os.path.splitext(c["ruta"])[0] + "_variantes.png", dpi=120)
    plt.close(fig)


def f(x, dec=1):
    return "-" if x is None or (isinstance(x, float) and np.isnan(x)) else f"{x:.{dec}f}"


def resumen(resultados, carpeta, cal, punta):
    ks = ["FW"] + [v for v in VARIANTES if any(("med_" + v) in r for r in resultados)]
    L = ["# Reprocesamiento del desplazamiento - comparacion de variantes", "",
         "FW = lo que integro el firmware; V0 = Madgwick beta 0,033; V1 = beta 0 durante el trazo;",
         "V2 = V1 + correccion de velocidad al frenar; V3 = V2 + posicion de la punta.",
         f"Calibracion: {'si (' + ', '.join(k for k in cal if not k.startswith(('acel_pos', 'acel_caras', 'giro_ejes'))) + ')' if cal else 'no'}. "
         f"Punta: {punta if punta is not None else 'no se uso (sin V3)'}.", ""]
    for tipo, titulo, extra, nom_extra in [("trazo", "Trazos deslizando sin inclinar", "lat", "desvio lateral"),
                                           ("lapiz", "Trazos agarrando el sensor como un lapiz", "lat", "desvio lateral"),
                                           ("circulo", "Circulos (3 vueltas)", "cierre", "no cierra por")]:
        rs = [r for r in resultados if r["tipo"] == tipo]
        if not rs:
            continue
        med = "diametro" if tipo == "circulo" else "largo"
        L += [f"## {titulo}", "",
              f"Celdas: {med} medido [cm] / error [%] / {nom_extra} [cm].", "",
              "| Corrida | Real [cm] | Dur. [s] | Incl. max [grados] | " + " | ".join(ks) + " |",
              "|---|---|---|---|" + "---|" * len(ks)]
        for r in rs:
            celdas = [f"{f(r['med_' + k])} / {r['err_' + k]:+.0f} / {f(r[extra + '_' + k])}" for k in ks]
            L.append(f"| {r['archivo']} | {r['medida']:g} | {f(r['dur'])} | {f(r['incl'])} | " + " | ".join(celdas) + " |")
        L.append("| **Error medio en valor absoluto** | | | | " + " | ".join(f"**{np.mean([abs(r['err_' + k]) for r in rs]):.0f} %**" for k in ks) + " |")
        L.append("")
    rs = [r for r in resultados if r["tipo"] == "hall"]
    if rs:
        L += ["## Con el Hall: cortar los trazos por contacto", "",
              "Cada pasada = movimiento entre dos reposos. H-contacto = cada tramo integrado solo",
              "mientras hay contacto; H-corrida = pasada completa de reposo a reposo y largo de cada",
              "tramo entre el apoyo y la levantada. Largos en cm. Valida = reposo antes y despues,",
              "la cantidad de tramos esperada y 5 s o menos.", "",
              "| Corrida | Pasada | Dur. [s] | Tramos (esp.) | Real [cm] | H-contacto | H-corrida | Err. H-contacto | Err. H-corrida | Valida |",
              "|---|---|---|---|---|---|---|---|---|---|"]
        for r in rs:
            for p_ in r["pasadas"]:
                tr = p_["tramos"]
                L.append(f"| {r['archivo']} | {p_['n']} | {p_['dur']:.1f} | {len(tr)} ({r['n_esp']}) | {r['medida']:g} | "
                         + ", ".join(f"{x['contacto']:.1f}" for x in tr) + " | "
                         + ", ".join(f"{x['corrida']:.1f}" for x in tr)
                         + f" | {p_['err_contacto']:.0f} % | {p_['err_corrida']:.0f} % | "
                         + ("si" if p_["valida"] else "no (" + ", ".join(p_["avisos"]) + ")") + " |")
        ok = [x for r in rs for p_ in r["pasadas"] if p_["valida"] for x in p_["tramos"]]
        if ok:
            L.append(f"| **Pasadas validas** | | | {len(ok)} tramos | | | | **{np.mean([abs(x['contacto'] - r['medida']) / r['medida'] * 100 for r in rs for p_ in r['pasadas'] if p_['valida'] for x in p_['tramos']]):.0f} %** | "
                     f"**{np.mean([abs(x['corrida'] - r['medida']) / r['medida'] * 100 for r in rs for p_ in r['pasadas'] if p_['valida'] for x in p_['tramos']]):.0f} %** | |")
        L.append("")
    ruta = os.path.join(carpeta, "resumen_variantes.md")
    with open(ruta, "w", encoding="utf-8") as fh:
        fh.write("\n".join(L) + "\n")
    return ruta


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("rutas", nargs="*")
    p.add_argument("--punta", help="vector IMU -> punta en cm, ejes del sensor, por ejemplo 0,0,-12")
    p.add_argument("--calibracion", default=os.path.join(AQUI, "calibracion_imu.json"))
    p.add_argument("--sin-calibracion", action="store_true")
    args = p.parse_args()
    punta = [float(x) for x in args.punta.split(",")] if args.punta else None
    cal = None
    if not args.sin_calibracion and os.path.exists(args.calibracion):
        with open(args.calibracion, encoding="utf-8") as fh:
            cal = json.load(fh)
        print("Usando calibracion:", args.calibracion)
    rutas = args.rutas or [os.path.join(AQUI, "visor_desplazamiento_integracion_pura", "corridas")]
    archivos = []
    for r in rutas:
        archivos += sorted(glob.glob(os.path.join(r, "*.csv"))) if os.path.isdir(r) else [r]
    resultados = []
    for a in archivos:
        c = leer_corrida(a)
        if c is None:
            continue
        r = procesar(c, cal, punta)
        if r is None:
            continue
        resultados.append(r)
        if r["tipo"] == "hall":
            for p_ in r["pasadas"]:
                print(f"{r['archivo']} pasada {p_['n']} (t={p_['t0']:.1f} s, {p_['dur']:.1f} s, {len(p_['tramos'])} tramos): "
                      "H-contacto " + ", ".join(f"{x['contacto']:.1f}" for x in p_["tramos"]) +
                      " | H-corrida " + ", ".join(f"{x['corrida']:.1f}" for x in p_["tramos"]) +
                      f" cm (real {r['medida']:g})" + ("" if p_["valida"] else "  [no valida: " + ", ".join(p_["avisos"]) + "]"))
            continue
        ks = ["FW"] + [v for v in VARIANTES if ("med_" + v) in r]
        print(f"{r['archivo']}: real {r['medida']:g} cm | " + "  ".join(f"{k} {r['med_' + k]:.1f} ({r['err_' + k]:+.0f} %)" for k in ks))
    if not resultados:
        sys.exit("No hay corridas trazo_*, lapiz_*, circulo_*, hall_trazo_* ni encadenado_* con movimiento detectado.")
    print("Resumen:", resumen(resultados, os.path.dirname(os.path.abspath(archivos[0])), cal, punta))


if __name__ == "__main__":
    main()
