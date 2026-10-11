"""
Graba la salida en formato Teleplot (">nombre:valor") de cualquier test y
resume las metricas de temporizacion. Pensado para el TEST 2 (lectura cruda).

Uso (con el ESP32 corriendo t2_lectura_cruda y el monitor de PlatformIO CERRADO):
    python tools/logger_teleplot.py COM3 --seg 120 --nombre t2_lectura_cruda

Mientras graba, reenvia todo a Teleplot por UDP (127.0.0.1:47269): los graficos
aparecen solos en Teleplot SIN abrir el puerto serie ahi (no apretar "Open").
Usar --sin-teleplot para no reenviar.

Pruebas de beta (con t4_madgwick cargado):
    python tools/logger_teleplot.py COM3 --seg 40 --beta 0.1 --nombre t4_beta_0.1

Genera:
    resultados/<fecha>_<nombre>.csv          (t_s, variable, valor)  -> todo lo recibido
    resultados/<fecha>_<nombre>_resumen.md   (tabla de temporizacion para el informe)
"""
import argparse
import os
import socket
import statistics
import time
from collections import defaultdict
from datetime import datetime

import serial

METRICAS = ["tasa_hz", "dt_min_us", "dt_max_us", "lectura_i2c_max_us", "fallos_i2c", "temp_c",
            "filtro_max_us", "ciclo_max_us"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("puerto", help="Puerto serie, ej. COM5")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--seg", type=float, default=120, help="duracion de la grabacion [s]")
    ap.add_argument("--nombre", default="t2_lectura_cruda")
    ap.add_argument("--beta", type=float, default=None, help="(t4) fija beta antes de grabar")
    ap.add_argument("--sin-teleplot", action="store_true", help="no reenviar a Teleplot por UDP")
    args = ap.parse_args()

    carpeta = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "resultados")
    os.makedirs(carpeta, exist_ok=True)
    base = os.path.join(carpeta, f"{datetime.now():%Y-%m-%d_%H%M%S}_{args.nombre}")

    datos = defaultdict(list)
    udp = None if args.sin_teleplot else socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    destino = ("127.0.0.1", 47269)  # puerto UDP por defecto de Teleplot
    with serial.Serial(args.puerto, args.baud, timeout=1) as ser, \
            open(base + ".csv", "w", newline="") as f:
        time.sleep(2.5)  # abrir el puerto resetea el ESP32
        if args.beta is not None:
            time.sleep(2.0)  # t4 calibra el giro al arrancar: esperar antes de mandar comandos
            ser.write(f"b{args.beta}\n".encode())
            time.sleep(0.3)
            print(f"beta = {args.beta}")
        ser.reset_input_buffer()
        f.write("t_s,variable,valor\n")
        t0 = time.time()
        print(f"Grabando {args.seg:.0f} s en {base}.csv  (Ctrl+C para cortar antes)")
        if udp:
            print("Reenviando a Teleplot por UDP 47269 (en Teleplot NO abrir el puerto serie)")
        try:
            while time.time() - t0 < args.seg:
                linea = ser.readline().decode(errors="ignore").strip()
                if not linea.startswith(">") or ":" not in linea:
                    continue
                if udp:
                    try:
                        udp.sendto(linea[1:].encode(), destino)  # Teleplot UDP: "nombre:valor"
                    except OSError:
                        pass
                nombre, _, valor = linea[1:].partition(":")
                try:
                    v = float(valor)
                except ValueError:
                    continue
                t = time.time() - t0
                f.write(f"{t:.4f},{nombre},{v}\n")
                datos[nombre].append(v)
                if nombre == "tasa_hz":
                    print(f"\r t={t:5.0f} s  tasa={v:6.1f} Hz", end="")
        except KeyboardInterrupt:
            pass

    print("\n")
    if not datos.get("tasa_hz"):
        print("No llegaron metricas. Esta cargado t2_lectura_cruda? Teleplot/monitor cerrados?")
        return

    # Se descarta la primera ventana de 1 s (arranque)
    filas = []
    for m in METRICAS:
        vals = datos.get(m, [])[1:]
        if not vals:
            continue
        filas.append((m, min(vals), statistics.mean(vals), max(vals), len(vals)))

    periodo_nom = 10000
    resumen = [f"# Temporizacion - {args.nombre} ({datetime.now():%Y-%m-%d %H:%M})", "",
               f"Duracion: {args.seg:.0f} s - ventanas de 1 s (se descarta la primera).", "",
               "| Metrica | Minimo | Promedio | Maximo | Ventanas |", "|---|---|---|---|---|"]
    for m, lo, me, hi, n in filas:
        resumen.append(f"| {m} | {lo:.1f} | {me:.1f} | {hi:.1f} | {n} |")
    if datos.get("dt_min_us") and datos.get("dt_max_us"):
        jit = max(max(datos["dt_max_us"][1:]) - periodo_nom, periodo_nom - min(datos["dt_min_us"][1:]))
        resumen += ["", f"Jitter maximo respecto del periodo nominal de {periodo_nom} us: "
                        f"{jit:.0f} us ({100 * jit / periodo_nom:.2f} %)."]
    texto = "\n".join(resumen) + "\n"
    with open(base + "_resumen.md", "w", encoding="utf-8") as f:
        f.write(texto)
    print(texto)
    print(f"Guardado: {base}.csv y {base}_resumen.md")


if __name__ == "__main__":
    main()
