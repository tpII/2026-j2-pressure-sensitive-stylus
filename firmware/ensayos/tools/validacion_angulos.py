"""
Validacion de roll/pitch del filtro de Madgwick contra angulos de referencia.

Secuencia guiada: para cada eje (X = roll, Y = pitch) y cada angulo
(0, -45, -90, 45, 90 grados):
  1. 6 s para llevar el sensor al angulo y alinearlo con el inclinometro.
  2. 5 s de registro con el sensor QUIETO (se promedian ~500 muestras).

Uso (con t4_madgwick cargado, y el monitor de PlatformIO y Teleplot SIN conectar):
    python tools/validacion_angulos.py COM3
    python tools/validacion_angulos.py COM3 --pedir-ref   # pide el valor del celular tras cada angulo
    python tools/validacion_angulos.py COM3 --pedir-ref --invertir-signo --nombre con_offset
      --invertir-signo: invierte roll/pitch del filtro (convencion: eje X del dibujo del GY-521)

Mientras corre, reenvia los datos a Teleplot por UDP (verlos en vivo sin abrir el puerto).

Genera en resultados/:
    <fecha>_t4_validacion_angulos.csv           todas las muestras registradas
    <fecha>_t4_validacion_angulos_resumen.md    tabla: objetivo, referencia, media, desvio, error
"""
import argparse
import os
import socket
import statistics
import sys
import time
from datetime import datetime

import serial

ANGULOS = [0, -45, -90, 45, 90]
EJES = [("X", "roll"), ("Y", "pitch")]
T_ALINEAR = 6.0
T_REGISTRO = 5.0
SIGNO = 1.0  # -1 con --invertir-signo (solo para roll/pitch)


class Lector:
    """Lee el puerto serie sin bloquear y guarda el ultimo roll/pitch/yaw."""

    def __init__(self, ser, udp):
        self.ser, self.udp = ser, udp
        self.ultimo = {}
        self.buf = b""

    def leer(self):
        """Procesa todo lo que haya llegado. Devuelve lista de (nombre, valor)."""
        nuevos = []
        n = self.ser.in_waiting
        if n:
            self.buf += self.ser.read(n)
        while b"\n" in self.buf:
            linea, self.buf = self.buf.split(b"\n", 1)
            linea = linea.decode(errors="ignore").strip()
            if not linea.startswith(">") or ":" not in linea:
                continue
            nombre, _, valor = linea[1:].partition(":")
            try:
                v = float(valor)
            except ValueError:
                continue
            if self.udp:
                try:
                    self.udp.sendto(linea[1:].encode(), ("127.0.0.1", 47269))
                except OSError:
                    pass
            self.ultimo[nombre] = v
            nuevos.append((nombre, v))
        return nuevos


def cuenta(lector, segundos, texto, registrar=None):
    """Cuenta regresiva en pantalla mientras sigue leyendo el puerto."""
    fin = time.time() + segundos
    while True:
        resto = fin - time.time()
        if resto <= 0:
            break
        for nombre, v in lector.leer():
            if registrar is not None and nombre in ("roll", "pitch", "yaw"):
                registrar(nombre, v)
        r = SIGNO * lector.ultimo.get("roll", float("nan"))
        p = SIGNO * lector.ultimo.get("pitch", float("nan"))
        sys.stdout.write(f"\r  {texto} {resto:4.1f} s   | roll {r:7.2f}   pitch {p:7.2f}   ")
        sys.stdout.flush()
        time.sleep(0.02)
    sys.stdout.write("\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("puerto", help="Puerto serie, ej. COM3")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--pedir-ref", action="store_true",
                    help="pedir el angulo que marca el inclinometro despues de cada registro")
    ap.add_argument("--sin-teleplot", action="store_true")
    ap.add_argument("--invertir-signo", action="store_true",
                    help="invierte el signo de roll y pitch para que coincida con la convencion del inclinometro")
    ap.add_argument("--nombre", default="", help="sufijo para el archivo, ej. con_offset")
    args = ap.parse_args()
    global SIGNO
    SIGNO = -1.0 if args.invertir_signo else 1.0

    carpeta = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "resultados")
    os.makedirs(carpeta, exist_ok=True)
    base = os.path.join(carpeta, f"{datetime.now():%Y-%m-%d_%H%M%S}_t4_validacion_angulos" + (f"_{args.nombre}" if args.nombre else ""))
    udp = None if args.sin_teleplot else socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    filas = []
    with serial.Serial(args.puerto, args.baud, timeout=0) as ser, \
            open(base + ".csv", "w", newline="") as f:
        f.write("eje,objetivo_deg,t_s,variable,valor\n")
        lector = Lector(ser, udp)
        print("Esperando arranque y calibracion del ESP32 (dejar QUIETO y plano)...")
        cuenta(lector, 5.0, "Calibrando, no mover:")
        if "roll" not in lector.ultimo:
            print("No llegan datos de roll/pitch. Esta cargado t4_madgwick? Teleplot/monitor desconectados?")
            return

        t0 = time.time()
        for eje, var in EJES:
            print(f"\n================ EJE {eje} ({var}) ================")
            for ang in ANGULOS:
                print(f"\n>>> Llevar a {ang:+d} grados en el eje {eje}")
                cuenta(lector, T_ALINEAR, "Alinear:")
                muestras = {"roll": [], "pitch": [], "yaw": []}

                def registrar(nombre, v, eje=eje, ang=ang, muestras=muestras):
                    muestras[nombre].append(v)
                    f.write(f"{eje},{ang},{time.time() - t0:.3f},{nombre},{v}\n")

                cuenta(lector, T_REGISTRO, "REGISTRANDO, no mover:", registrar)
                sg = -1.0 if args.invertir_signo else 1.0
                for k in ("roll", "pitch"):
                    muestras[k] = [sg * v for v in muestras[k]]
                vals = muestras[var]
                if len(vals) < 10:
                    print("  Pocas muestras, se omite este angulo.")
                    continue
                media, desvio = statistics.mean(vals), statistics.pstdev(vals)
                otro = "pitch" if var == "roll" else "roll"
                media_otro = statistics.mean(muestras[otro]) if muestras[otro] else float("nan")
                ref = float(ang)
                if args.pedir_ref:
                    resp = input(f"  Angulo que marca el inclinometro [Enter = {ang}]: ").strip().replace(",", ".")
                    if resp:
                        try:
                            ref = float(resp)
                        except ValueError:
                            print("  Valor invalido, se usa el objetivo.")
                    ser.reset_input_buffer()
                    lector.buf = b""
                err = media - ref
                print(f"  {var}: media {media:+7.2f}  desvio {desvio:5.2f}  ref {ref:+7.2f}  "
                      f"error {err:+6.2f}  ({len(vals)} muestras) | {otro} medio {media_otro:+7.2f}")
                filas.append((eje, var, ang, ref, media, desvio, err, len(vals), otro, media_otro))

    lineas = [f"# Validacion de roll/pitch - t4_madgwick ({datetime.now():%Y-%m-%d %H:%M})", "",
              f"Por angulo: {T_ALINEAR:.0f} s para alinear y {T_REGISTRO:.0f} s de registro con el sensor quieto.",
              ("Signo de roll/pitch invertido (convencion del inclinometro).\n" if args.invertir_signo else "") +
              "Referencia: " + ("valor leido en el inclinometro." if args.pedir_ref else
                                "angulo objetivo nominal (sin lectura de inclinometro)."), "",
              "| Eje | Variable | Objetivo [deg] | Referencia [deg] | Media filtro [deg] | Desvio [deg] | Error [deg] | Muestras | Otro eje (media) [deg] |",
              "|---|---|---|---|---|---|---|---|---|"]
    for eje, var, ang, ref, media, desvio, err, n, otro, mo in filas:
        lineas.append(f"| {eje} | {var} | {ang:+d} | {ref:+.1f} | {media:+.2f} | {desvio:.2f} | {err:+.2f} | {n} | {otro} {mo:+.1f} |")
    for eje, var in EJES:
        errs = [abs(fl[6]) for fl in filas if fl[0] == eje and abs(fl[2]) < 90]
        if errs:
            lineas.append(f"\nEje {eje}: error absoluto medio (sin +-90) {statistics.mean(errs):.2f} deg, maximo {max(errs):.2f} deg.")
    lineas.append("\nNota: en pitch = +-90 deg los angulos de Euler son singulares (gimbal lock); esos valores se informan aparte.")
    texto = "\n".join(lineas) + "\n"
    with open(base + "_resumen.md", "w", encoding="utf-8") as fo:
        fo.write(texto)
    print("\n" + texto)
    print(f"Guardado: {base}.csv y {base}_resumen.md")


if __name__ == "__main__":
    main()
