"""
Analisis de las corridas del ensayo de desplazamiento (integracion pura, sin correcciones).

Lee los CSV que graba tools/visor_desplazamiento_integracion_pura (una corrida por referencia, tecla C)
y calcula, para A (con Madgwick) y B (sin Madgwick):

  - Corridas "reposo": deriva de la posicion con el sensor quieto. Se ajusta
    d(t) = 1/2 * b * g * t^2 y se informa b [mg], el error de aceleracion constante
    equivalente. Con b se estima el error de un trazo de cualquier duracion.
  - Corridas "trazo_<L>cm": se detecta el movimiento en la lectura cruda (o se usan
    las dos primeras marcas, tecla M) y se compara el desplazamiento medido entre el
    inicio y el fin del movimiento con el largo real L. Tambien se informa cuanto
    sigue moviendose la posicion 1 s despues de frenar (velocidad residual).
  - Corridas "libre": solo graficos.

Uso:
    python tools/analizar_desplazamiento.py                      # todas las de visor_desplazamiento_integracion_pura/corridas
    python tools/analizar_desplazamiento.py ruta/a/corrida.csv ...
    python tools/analizar_desplazamiento.py ruta/a/carpeta

Genera, al lado de cada CSV, un PNG con la trayectoria y la distancia vs tiempo,
y en la carpeta de la primera corrida un resumen_desplazamiento.md con las tablas.
"""
import argparse
import glob
import os
import re
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

G = 9.81
T_QUIETO_INI = 0.5      # [s] al inicio de cada corrida el sensor esta quieto (recien tomada la referencia)
UMBRAL_ACC_G = 0.03     # [g] desvio de la aceleracion para considerar movimiento
UMBRAL_GYRO_DPS = 5.0   # [grados/s]
VENTANA_S = 0.1         # media movil para detectar movimiento
MARGEN_S = 0.1          # se agrega antes y despues del movimiento detectado (el umbral corta los bordes)
T_DERIVA = [0.5, 1, 2, 5, 10]
INCL_MAX = 3.0

COLS = ["t_ms", "Au", "Av", "An", "Bu", "Bv", "Bn", "incl", "sat", "ciclo", "ax", "ay", "az", "gx", "gy", "gz"]


def leer_corrida(ruta):
    etiqueta, marcas, filas = "libre", [], []
    with open(ruta, encoding="utf-8", errors="replace") as f:
        for linea in f:
            linea = linea.strip()
            if linea.startswith("#etiqueta="):
                etiqueta = linea.split("=", 1)[1]
            elif linea.startswith("#marca"):
                m = re.search(r"t_ms=(-?\d+)", linea)
                if m:
                    marcas.append(int(m.group(1)) / 1000.0)
            elif linea.startswith("D,"):
                partes = linea.split(",")
                if len(partes) == 17:
                    try:
                        filas.append([float(x) for x in partes[1:]])
                    except ValueError:
                        pass
    if not filas:
        return None
    a = np.array(filas)
    d = {c: a[:, i] for i, c in enumerate(COLS)}
    d["t"] = d["t_ms"] / 1000.0
    d["dA"] = np.hypot(d["Au"], d["Av"])
    d["dB"] = np.hypot(d["Bu"], d["Bv"])
    return {"ruta": ruta, "etiqueta": etiqueta, "marcas": marcas, "d": d}


def largo_real(etiqueta):
    m = re.match(r"trazo_(\d+(?:\.\d+)?)cm", etiqueta)
    return float(m.group(1)) if m else None


def detectar_movimiento(d, marcas):
    """Devuelve (t_ini, t_fin) del movimiento o None. Usa las marcas si hay dos."""
    if len(marcas) >= 2:
        return marcas[0], marcas[1]
    t = d["t"]
    quieto = t < T_QUIETO_INI
    if quieto.sum() < 10:
        return None
    acc = np.column_stack([d["ax"], d["ay"], d["az"]])
    gyr = np.column_stack([d["gx"], d["gy"], d["gz"]])
    desv_acc = np.linalg.norm(acc - acc[quieto].mean(axis=0), axis=1)
    desv_gyr = np.linalg.norm(gyr - gyr[quieto].mean(axis=0), axis=1)
    n = max(1, int(VENTANA_S * 100))
    k = np.ones(n) / n
    mov = (np.convolve(desv_acc, k, "same") > UMBRAL_ACC_G) | (np.convolve(desv_gyr, k, "same") > UMBRAL_GYRO_DPS)
    idx = np.flatnonzero(mov)
    if idx.size == 0:
        return None
    return max(0.0, t[idx[0]] - MARGEN_S), min(t[-1], t[idx[-1]] + MARGEN_S)


def trazo_desde_reposo(d, t0, t1):
    """Recalcula B solo dentro del movimiento, con velocidad 0 al empezar: la gravedad
    se toma de los 0,3 s anteriores a t0. Separa el error del trazo en si de la velocidad
    acumulada antes por la deriva. Devuelve el largo en el plano [cm]."""
    t = d["t"]
    acc = np.column_stack([d["ax"], d["ay"], d["az"]])
    antes = (t >= t0 - 0.3) & (t < t0)
    if antes.sum() < 5:
        return float("nan")
    g = acc[antes].mean(axis=0)
    e3 = g / np.linalg.norm(g)
    dentro = (t >= t0) & (t <= t1)
    lin = (acc[dentro] - g) * G
    vel = np.cumsum(lin, axis=0) * 0.01
    pos = np.cumsum(vel, axis=0) * 0.01
    p = pos[-1] - np.dot(pos[-1], e3) * e3      # componente en el plano de la mesa
    return float(np.linalg.norm(p) * 100)


def ajustar_b_mg(t, dist_cm):
    """Ajuste por minimos cuadrados de d = 1/2 b g t^2 (sin ordenada). Devuelve b en mg."""
    x = 0.5 * G * t ** 2 * 100          # cm por cada g de error
    den = np.dot(x, x)
    return 1000.0 * np.dot(x, dist_cm) / den if den > 0 else float("nan")


def valor_en(t, y, t0):
    if t0 > t[-1]:
        return float("nan")
    return float(np.interp(t0, t, y))


def analizar(c):
    d, t = c["d"], c["d"]["t"]
    res = {"archivo": os.path.basename(c["ruta"]), "etiqueta": c["etiqueta"], "dur_total": t[-1],
           "incl_max": float(d["incl"].max()), "sat": int(d["sat"].sum())}
    mov = detectar_movimiento(d, c["marcas"])
    res["mov"] = mov

    # Deriva en reposo: toda la corrida si es "reposo", o hasta que empieza el movimiento
    t_fin_quieto = t[-1] if c["etiqueta"] == "reposo" or mov is None else mov[0]
    q = t <= t_fin_quieto
    res["t_quieto"] = t_fin_quieto
    for s in "AB":
        res["b_" + s] = ajustar_b_mg(t[q], d["d" + s][q]) if q.sum() > 20 else float("nan")
        res["deriva_" + s] = [valor_en(t[q], d["d" + s][q], x) if x <= t_fin_quieto else float("nan") for x in T_DERIVA]

    L = largo_real(c["etiqueta"])
    res["L"] = L
    if L is not None and mov is not None:
        t0, t1 = mov
        res["t_ini"], res["dur_mov"] = t0, t1 - t0
        vec = {}
        for s in "AB":
            u0, v0 = np.interp(t0, t, d[s + "u"]), np.interp(t0, t, d[s + "v"])
            u1, v1 = np.interp(t1, t, d[s + "u"]), np.interp(t1, t, d[s + "v"])
            vec[s] = np.array([u1 - u0, v1 - v0])
            res["largo_" + s] = float(np.hypot(*vec[s]))
            res["err_" + s] = res["largo_" + s] - L
            if t1 + 1.0 <= t[-1]:
                u2, v2 = np.interp(t1 + 1.0, t, d[s + "u"]), np.interp(t1 + 1.0, t, d[s + "v"])
                res["post_" + s] = float(np.hypot(u2 - u1, v2 - v1))
            else:
                res["post_" + s] = float("nan")
        na, nb = np.linalg.norm(vec["A"]), np.linalg.norm(vec["B"])
        res["ang_AB"] = float(np.degrees(np.arccos(np.clip(vec["A"] @ vec["B"] / (na * nb), -1, 1)))) if na > 0 and nb > 0 else float("nan")
        res["incl_mov"] = float(d["incl"][(t >= t0) & (t <= t1)].max())
        res["largo_B0"] = trazo_desde_reposo(d, t0, t1)
    return res


def graficar(c, res):
    d, t = c["d"], c["d"]["t"]
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5.5))
    ax1.plot(d["Au"], d["Av"], color="#c83232", lw=1.4, label="A con Madgwick")
    ax1.plot(d["Bu"], d["Bv"], color="#285ac8", lw=1.4, label="B sin Madgwick")
    ax1.plot(0, 0, "k+", ms=12)
    ax1.set_aspect("equal", adjustable="datalim")
    ax1.set_xlabel("u [cm]")
    ax1.set_ylabel("v [cm]")
    ax1.set_title("Trayectoria vista desde arriba")
    ax1.grid(alpha=0.3)
    ax1.legend()

    ax2.plot(t, d["dA"], color="#c83232", lw=1.4, label="A con Madgwick")
    ax2.plot(t, d["dB"], color="#285ac8", lw=1.4, label="B sin Madgwick")
    tt = np.linspace(0, t[-1], 200)
    ymax = max(d["dA"].max(), d["dB"].max(), 1) * 1.1
    for mg, est in [(1, ":"), (10, "--")]:
        ax2.plot(tt, 0.5 * mg * 1e-3 * G * tt ** 2 * 100, "k", ls=est, lw=0.8, label=f"error constante {mg} mg")
    if res["mov"] is not None:
        ax2.axvspan(res["mov"][0], res["mov"][1], color="0.85", label="movimiento")
    if res["L"] is not None:
        ax2.axhline(res["L"], color="0.4", lw=0.8, ls="-.", label=f"L = {res['L']:g} cm")
    ax2.set_ylim(0, ymax)
    ax2.set_xlabel("t desde la referencia [s]")
    ax2.set_ylabel("distancia al origen en el plano [cm]")
    ax2.set_title("Distancia vs tiempo")
    ax2.grid(alpha=0.3)
    ax2.legend(fontsize=8)
    fig.suptitle(f"{res['archivo']}  ({res['etiqueta']})")
    fig.tight_layout()
    salida = os.path.splitext(c["ruta"])[0] + ".png"
    fig.savefig(salida, dpi=120)
    plt.close(fig)
    return salida


def f(x, dec=2):
    return "-" if x is None or (isinstance(x, float) and np.isnan(x)) else f"{x:.{dec}f}"


def resumen(resultados, carpeta):
    lineas = ["# Ensayo de desplazamiento - integracion pura (sin correcciones)", ""]
    lineas += ["A = con Madgwick, B = sin Madgwick (gravedad fija). Distancias en el plano de la mesa.",
               "b = error de aceleracion constante equivalente, ajustando d = 1/2 b g t^2 con el sensor quieto.",
               "Error esperado en un trazo de T segundos: 1/2 b g T^2 (1 mg -> 0,49 cm en 1 s; 0,12 cm en 0,5 s).",
               "B desde reposo: B recalculado en la PC solo durante el trazo, con velocidad 0 al empezar (como si el",
               "lapiz se apoyara quieto antes de cada trazo). No es lo que integra el firmware.", ""]

    lineas += ["## Deriva con el sensor quieto", "",
               "| Corrida | Quieto [s] | b A [mg] | b B [mg] | A en 0,5 / 1 / 2 / 5 s [cm] | B en 0,5 / 1 / 2 / 5 s [cm] | Incl. max [grados] |",
               "|---|---|---|---|---|---|---|"]
    for r in resultados:
        lineas.append(f"| {r['archivo']} | {f(r['t_quieto'], 1)} | {f(r['b_A'])} | {f(r['b_B'])} | "
                      + " / ".join(f(x) for x in r["deriva_A"][:4]) + " | "
                      + " / ".join(f(x) for x in r["deriva_B"][:4]) + f" | {f(r['incl_max'], 1)} |")

    trazos = [r for r in resultados if r.get("largo_A") is not None]
    if trazos:
        lineas += ["", "## Trazos de largo conocido", "",
                   "| Corrida | L [cm] | Duracion [s] | A [cm] | B [cm] | Error A [%] | Error B [%] | B desde reposo [cm] | Angulo A-B [grados] | Sigue 1 s despues A / B [cm] | Incl. [grados] |",
                   "|---|---|---|---|---|---|---|---|---|---|---|"]
        for r in trazos:
            aviso = " (B no valido)" if r["incl_mov"] > INCL_MAX else ""
            lineas.append(f"| {r['archivo']} | {r['L']:g} | {f(r['dur_mov'])} | {f(r['largo_A'])} | {f(r['largo_B'])} | "
                          f"{f(100 * r['err_A'] / r['L'], 0)} | {f(100 * r['err_B'] / r['L'], 0)} | {f(r['largo_B0'])} | {f(r['ang_AB'], 0)} | "
                          f"{f(r['post_A'])} / {f(r['post_B'])} | {f(r['incl_mov'], 1)}{aviso} |")

    bA = [r["b_A"] for r in resultados if r["etiqueta"] == "reposo" and not np.isnan(r["b_A"])]
    bB = [r["b_B"] for r in resultados if r["etiqueta"] == "reposo" and not np.isnan(r["b_B"])]
    if bA or bB:
        lineas += ["", "## Error esperado por deriva segun la duracion del trazo (corridas de reposo)", "",
                   "| | b medio [mg] | 0,25 s | 0,5 s | 1 s | 2 s |", "|---|---|---|---|---|---|"]
        for nombre, bs in [("A con Madgwick", bA), ("B sin Madgwick", bB)]:
            if bs:
                b = float(np.mean(np.abs(bs)))
                errs = [0.5 * b * 1e-3 * G * T ** 2 * 1000 for T in (0.25, 0.5, 1, 2)]   # mm
                lineas.append(f"| {nombre} | {b:.2f} | " + " | ".join(f"{e:.1f} mm" for e in errs) + " |")

    ruta = os.path.join(carpeta, "resumen_desplazamiento.md")
    with open(ruta, "w", encoding="utf-8") as fh:
        fh.write("\n".join(lineas) + "\n")
    return ruta


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("rutas", nargs="*", help="CSV de corridas o carpetas (por defecto visor_desplazamiento_integracion_pura/corridas)")
    args = p.parse_args()
    rutas = args.rutas or [os.path.join(os.path.dirname(os.path.abspath(__file__)), "visor_desplazamiento_integracion_pura", "corridas")]
    archivos = []
    for r in rutas:
        archivos += sorted(glob.glob(os.path.join(r, "*.csv"))) if os.path.isdir(r) else [r]
    if not archivos:
        sys.exit("No se encontraron corridas (.csv).")

    resultados = []
    for a in archivos:
        c = leer_corrida(a)
        if c is None:
            print(f"{os.path.basename(a)}: sin muestras, se omite")
            continue
        r = analizar(c)
        png = graficar(c, r)
        resultados.append(r)
        print(f"{r['archivo']}: {r['etiqueta']}, {r['dur_total']:.1f} s, b A = {f(r['b_A'])} mg, b B = {f(r['b_B'])} mg"
              + (f", trazo A {f(r['largo_A'])} cm / B {f(r['largo_B'])} cm (L = {r['L']:g})" if r.get("largo_A") is not None else "")
              + f"  -> {os.path.basename(png)}")
    if resultados:
        print("Resumen:", resumen(resultados, os.path.dirname(os.path.abspath(archivos[0]))))


if __name__ == "__main__":
    main()
