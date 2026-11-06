"""
Calibracion del MPU6050 a partir de corridas grabadas con visor_desplazamiento_integracion_pura
(firmware ensayo_desplazamiento_integracion_pura, que manda la lectura cruda ax..gz).

1) Acelerometro (corrida con etiqueta "calib_6caras"):
   se apoya el sensor quieto ~5 s sobre cada una de sus 6 caras (o mas posiciones).
   El script busca los tramos quietos y ajusta, por minimos cuadrados, un offset y una
   escala por eje para que el modulo de la gravedad de 1 g en todas las posiciones:
       a_corregida = (a_cruda - offset) / escala

2) Giroscopo (corridas con etiqueta "giro_360"):
   se gira el sensor una vuelta completa alrededor de un eje y se lo deja quieto en la misma
   pose. Para el eje vertical, apoyado sobre la mesa desde un tope hasta el mismo tope; para
   un eje horizontal se lo da vuelta "rodando" sobre ese eje (la gravedad fija la pose final).
   En una corrida puede haber varios giros, separados por pausas; los movimientos que no son
   una vuelta completa (reacomodar el sensor) se descartan. Se compara con 360*N grados:
       w_corregida = w_cruda * escala_giro   (el bias se sigue midiendo en cada referencia)
   Para calibrar los tres ejes hay que hacer al menos un giro con cada eje vertical.

Uso:
    python tools/calibrar_imu.py                          # corridas de visor_desplazamiento_integracion_pura/corridas
    python tools/calibrar_imu.py ruta/a/carpeta_o_csv ... [--vueltas 1]

Genera tools/calibracion_imu.json, que usa tools/reprocesar_desplazamiento.py.
"""
import argparse
import glob
import json
import os
import sys

import numpy as np

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, AQUI)
from analizar_desplazamiento import leer_corrida  # noqa: E402

DT = 0.01
VENT_QUIETO_S = 1.5        # ventana para considerar una posicion quieta
STD_ACC_MAX_G = 0.015      # desvio maximo de cada eje dentro de la ventana
GYRO_MAX_DPS = 3.0         # giro maximo (respecto del bias) dentro de la ventana
UMBRAL_GIRO_DPS = 5.0


def tramos_quietos(acc, gyr):
    """Medias de aceleracion de los tramos quietos (uno por posicion)."""
    n = int(VENT_QUIETO_S / DT)
    bias = np.median(gyr, axis=0)
    quieto = np.zeros(len(acc), bool)
    for i in range(0, len(acc) - n, n // 3):
        a, g = acc[i:i + n], gyr[i:i + n] - bias
        if a.std(axis=0).max() < STD_ACC_MAX_G and np.abs(g).max() < GYRO_MAX_DPS * 3 and np.linalg.norm(g.mean(0)) < GYRO_MAX_DPS:
            quieto[i:i + n] = True
    tramos, i = [], 0
    while i < len(quieto):
        if quieto[i]:
            j = i
            while j < len(quieto) and quieto[j]:
                j += 1
            seg = acc[i:j]
            if j - i >= n:
                k = (j - i) // 6             # se descartan los bordes
                tramos.append(seg[k:len(seg) - k].mean(axis=0))
            i = j
        else:
            i += 1
    # se unen tramos de la misma posicion (si el corte quedo en dos pedazos)
    unidos = []
    for m in tramos:
        if unidos and np.linalg.norm(m - unidos[-1][0]) < 0.05:
            c, k = unidos[-1]
            unidos[-1] = ((c * k + m) / (k + 1), k + 1)
        else:
            unidos.append((m, 1))
    return np.array([m for m, _ in unidos])


def ajustar_acel(medias):
    """Minimos cuadrados (Gauss-Newton): | (a - o) / s | = 1 para todas las posiciones."""
    o, s = np.zeros(3), np.ones(3)
    for _ in range(50):
        u = (medias - o) / s
        nu = np.linalg.norm(u, axis=1)
        r = nu - 1
        J = np.hstack([-(u / s) / nu[:, None], -(u * u / s) / nu[:, None]])
        paso, *_ = np.linalg.lstsq(J, -r, rcond=None)
        o, s = o + paso[:3], s + paso[3:]
        if np.abs(paso).max() < 1e-9:
            break
    res = np.linalg.norm((medias - o) / s, axis=1) - 1
    return o, s, res


def caras_cubiertas(medias):
    caras = set()
    for m in medias:
        k = int(np.argmax(np.abs(m)))
        if abs(m[k]) > 0.8:
            caras.add(("+" if m[k] > 0 else "-") + "xyz"[k])
    return sorted(caras)


def giros_(d, vueltas):
    """Separa la corrida en movimientos (con al menos 0,5 s quieto entre uno y otro) y
    devuelve uno por cada movimiento que sea una vuelta completa (o N vueltas) alrededor
    de un eje. Los movimientos que no lo son (reacomodar el sensor) se descartan.
    El bias se mide en el tramo quieto justo antes de cada movimiento."""
    t = d["t"]
    gyr = np.column_stack([d["gx"], d["gy"], d["gz"]])
    b0 = np.median(gyr[t < 0.5], axis=0) if (t < 0.5).sum() > 10 else np.median(gyr, axis=0)
    k = np.ones(10) / 10
    mov = np.convolve(np.linalg.norm(gyr - b0, axis=1), k, "same") > UMBRAL_GIRO_DPS
    segs, i, n = [], 0, len(t)
    while i < n:
        if mov[i]:
            j = i
            while j < n and (mov[j] or mov[j:j + 50].any()):
                j += 1
            segs.append((i, j))
            i = j
        else:
            i += 1
    out = []
    for i0, i1 in segs:
        a0, a1 = max(0, i0 - 120), max(0, i0 - 20)
        bias = gyr[a0:a1].mean(axis=0) if a1 - a0 >= 30 else b0
        i0, i1 = max(0, i0 - 20), min(n, i1 + 20)
        ang = (gyr[i0:i1] - bias).sum(axis=0) * DT
        k = int(np.argmax(np.abs(ang)))
        # angulo total girado (modulo): no depende de que el eje del giro quede perfectamente
        # alineado con el eje del sensor
        medido = float(np.linalg.norm(ang))
        if medido < 30:
            continue                      # movimiento chico (acomodar), ni se informa
        nv = round(medido / 360.0)
        if nv < 1 or abs(medido - 360.0 * nv) > 0.15 * 360.0 * nv or (vueltas and nv != vueltas):
            out.append({"eje": "xyz"[k], "medido": medido, "descartado": True, "t": float(t[i0])})
            continue
        out.append({"eje": "xyz"[k], "medido": medido, "vueltas": nv, "escala": 360.0 * nv / medido,
                    "otros": float(np.linalg.norm(np.delete(ang, k))), "dur": float(t[i1 - 1] - t[i0]),
                    "t": float(t[i0]), "descartado": False})
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("rutas", nargs="*")
    p.add_argument("--vueltas", type=int, default=0, help="exigir N vueltas por giro (default: cualquier numero entero)")
    p.add_argument("--salida", default=os.path.join(AQUI, "calibracion_imu.json"))
    args = p.parse_args()
    rutas = args.rutas or [os.path.join(AQUI, "visor_desplazamiento_integracion_pura", "corridas")]
    archivos = []
    for r in rutas:
        archivos += sorted(glob.glob(os.path.join(r, "*.csv"))) if os.path.isdir(r) else [r]
    corridas = [c for c in (leer_corrida(a) for a in archivos) if c is not None]

    calib = {}
    if os.path.exists(args.salida):
        with open(args.salida, encoding="utf-8") as fh:
            calib = json.load(fh)

    # --- acelerometro ---
    medias = []
    for c in corridas:
        if c["etiqueta"] == "calib_6caras":
            d = c["d"]
            m = tramos_quietos(np.column_stack([d["ax"], d["ay"], d["az"]]), np.column_stack([d["gx"], d["gy"], d["gz"]]))
            print(f"{os.path.basename(c['ruta'])}: {len(m)} posiciones quietas")
            medias += list(m)
    if len(medias) > 0:
        medias = np.array(medias)
        caras = caras_cubiertas(medias)
        if len(medias) < 6:
            print(f"AVISO: solo {len(medias)} posiciones; hacen falta al menos 6. No se calibra el acelerometro.")
        else:
            o, s, res = ajustar_acel(medias)
            antes = np.abs(np.linalg.norm(medias, axis=1) - 1)
            print("Acelerometro:")
            print(f"  caras cubiertas: {', '.join(caras)}" + ("" if len(caras) == 6 else "  (AVISO: faltan caras, el ajuste es menos confiable)"))
            print(f"  offset [mg]   = {np.round(o * 1000, 2)}")
            print(f"  escala        = {np.round(s, 5)}")
            print(f"  error de |g| antes: max {antes.max() * 1000:.1f} mg | despues: max {np.abs(res).max() * 1000:.2f} mg")
            calib["acel_offset_g"] = o.tolist()
            calib["acel_escala"] = s.tolist()
            calib["acel_posiciones"] = len(medias)
            calib["acel_caras"] = caras

    # --- giroscopo ---
    giros = {}
    for c in corridas:
        if c["etiqueta"] == "giro_360":
            lista = giros_(c["d"], args.vueltas)
            if not lista:
                print(f"{os.path.basename(c['ruta'])}: no se detecto giro")
            for g in lista:
                if g["descartado"]:
                    print(f"{os.path.basename(c['ruta'])} t={g['t']:.1f} s: eje {g['eje']}, {g['medido']:.0f} grados -> no es una vuelta completa, se descarta")
                    continue
                print(f"{os.path.basename(c['ruta'])} t={g['t']:.1f} s: eje {g['eje']}, {g['medido']:.1f} grados medidos en {g['dur']:.1f} s "
                      f"(esperado {360 * g['vueltas']:.0f}) -> escala {g['escala']:.4f}; giro en los otros ejes {g['otros']:.1f} grados")
                giros.setdefault(g["eje"], []).append(g["escala"])
    if giros:
        esc = calib.get("giro_escala", [1.0, 1.0, 1.0])
        for k, eje in enumerate("xyz"):
            if eje in giros:
                esc[k] = float(np.mean(giros[eje]))
        calib["giro_escala"] = esc
        faltan = [e for e in "xyz" if e not in giros and e not in calib.get("giro_ejes", [])]
        calib["giro_ejes"] = sorted(set(calib.get("giro_ejes", [])) | set(giros))
        print(f"Giroscopo: escala = {np.round(esc, 4)}" + (f"  (sin calibrar: {', '.join(faltan)}; quedan en 1)" if faltan else ""))

    if len(medias) == 0 and not giros:
        sys.exit("No hay corridas calib_6caras ni giro_360.")
    with open(args.salida, "w", encoding="utf-8") as fh:
        json.dump(calib, fh, indent=2)
    print("Guardado:", args.salida)


if __name__ == "__main__":
    main()
