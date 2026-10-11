"""
Prueba del punto 0: roll y pitch con el sensor apoyado plano.

Hace 3 repeticiones. En cada una:
  1. 6 s para apoyar el protoboard PLANO (levantarlo y volver a apoyarlo entre repeticiones).
  2. 5 s de registro con el sensor quieto.
Asi se mide el cero y tambien su repetibilidad.

Uso (con t4_madgwick cargado, y el monitor de PlatformIO y Teleplot SIN conectar):
    python tools/prueba_cero.py COM3
    python tools/prueba_cero.py COM3 --reps 5

Genera en resultados/:
    <fecha>_t4_prueba_cero.csv           todas las muestras
    <fecha>_t4_prueba_cero_resumen.md    media y desvio de roll/pitch por repeticion
"""
import argparse
import os
import socket
import statistics
import time
from datetime import datetime

import serial

from validacion_angulos import Lector, cuenta  # mismo lector y cuenta regresiva

T_APOYAR = 6.0
T_REGISTRO = 5.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("puerto", help="Puerto serie, ej. COM3")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--reps", type=int, default=3, help="cantidad de repeticiones")
    ap.add_argument("--sin-teleplot", action="store_true")
    args = ap.parse_args()

    carpeta = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "resultados")
    os.makedirs(carpeta, exist_ok=True)
    base = os.path.join(carpeta, f"{datetime.now():%Y-%m-%d_%H%M%S}_t4_prueba_cero")
    udp = None if args.sin_teleplot else socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    filas = []
    with serial.Serial(args.puerto, args.baud, timeout=0) as ser, \
            open(base + ".csv", "w", newline="") as f:
        f.write("rep,t_s,variable,valor\n")
        lector = Lector(ser, udp)
        print("Esperando arranque y calibracion del ESP32 (dejar QUIETO y PLANO)...")
        cuenta(lector, 5.0, "Calibrando, no mover:")
        if "roll" not in lector.ultimo:
            print("No llegan datos de roll/pitch. Esta cargado t4_madgwick? Teleplot/monitor desconectados?")
            return

        t0 = time.time()
        for rep in range(1, args.reps + 1):
            print(f"\n>>> Repeticion {rep}/{args.reps}: levantar el protoboard y volver a apoyarlo PLANO")
            print("    (que el cable USB no lo levante; chequear con el nivel del celular que marque 0)")
            cuenta(lector, T_APOYAR, "Apoyar plano:")
            muestras = {"roll": [], "pitch": []}

            def registrar(nombre, v, rep=rep, muestras=muestras):
                if nombre in muestras:
                    muestras[nombre].append(v)
                f.write(f"{rep},{time.time() - t0:.3f},{nombre},{v}\n")

            cuenta(lector, T_REGISTRO, "REGISTRANDO, no tocar:", registrar)
            if len(muestras["roll"]) < 10:
                print("  Pocas muestras, se omite.")
                continue
            r, p = muestras["roll"], muestras["pitch"]
            fila = (rep, statistics.mean(r), statistics.pstdev(r), statistics.mean(p), statistics.pstdev(p), len(r))
            filas.append(fila)
            print(f"  roll {fila[1]:+6.2f} (desvio {fila[2]:.2f})   pitch {fila[3]:+6.2f} (desvio {fila[4]:.2f})")

    lineas = [f"# Prueba del punto 0 - t4_madgwick ({datetime.now():%Y-%m-%d %H:%M})", "",
              f"{args.reps} repeticiones: {T_APOYAR:.0f} s para apoyar plano y {T_REGISTRO:.0f} s de registro.",
              "Valores en la convencion del filtro (sin invertir signo).", "",
              "| Repeticion | Roll medio [deg] | Desvio roll [deg] | Pitch medio [deg] | Desvio pitch [deg] | Muestras |",
              "|---|---|---|---|---|---|"]
    for rep, rm, rs, pm, ps, n in filas:
        lineas.append(f"| {rep} | {rm:+.2f} | {rs:.2f} | {pm:+.2f} | {ps:.2f} | {n} |")
    if len(filas) >= 2:
        rr = [x[1] for x in filas]; pp = [x[3] for x in filas]
        lineas += ["",
                   f"Roll: media {statistics.mean(rr):+.2f} deg, dispersion entre repeticiones {max(rr) - min(rr):.2f} deg.",
                   f"Pitch: media {statistics.mean(pp):+.2f} deg, dispersion entre repeticiones {max(pp) - min(pp):.2f} deg."]
    texto = "\n".join(lineas) + "\n"
    with open(base + "_resumen.md", "w", encoding="utf-8") as fo:
        fo.write(texto)
    print("\n" + texto)
    print(f"Guardado: {base}.csv y {base}_resumen.md")


if __name__ == "__main__":
    main()
