"""
Logger CSV para el TEST 4 (Madgwick).

Uso (con el ESP32 corriendo t4_madgwick y el monitor de PlatformIO CERRADO):
    pip install -r tools/requirements.txt
    python tools/logger.py COM5                 # graba hasta Ctrl+C
    python tools/logger.py COM5 --seg 30 --beta 0.05 --nombre giro_lento

Genera datos/<fecha>_<nombre>.csv con:
t_us,ax,ay,az,gx,gy,gz,q0,q1,q2,q3,roll,pitch,yaw,beta
"""
import argparse
import os
import time
from datetime import datetime

import serial

CABECERA = "t_us,ax,ay,az,gx,gy,gz,q0,q1,q2,q3,roll,pitch,yaw,beta"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("puerto", help="Puerto serie, ej. COM5")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--seg", type=float, default=0, help="duracion (0 = hasta Ctrl+C)")
    ap.add_argument("--beta", type=float, default=None, help="fijar beta antes de grabar")
    ap.add_argument("--nombre", default="prueba")
    args = ap.parse_args()

    carpeta = os.path.join(os.path.dirname(__file__), "..", "datos")
    os.makedirs(carpeta, exist_ok=True)
    archivo = os.path.join(
        carpeta, f"{datetime.now():%Y%m%d_%H%M%S}_{args.nombre}.csv")

    with serial.Serial(args.puerto, args.baud, timeout=1) as ser:
        time.sleep(2.5)  # abrir el puerto resetea el ESP32: esperar calibracion
        ser.reset_input_buffer()
        if args.beta is not None:
            ser.write(f"b{args.beta}\n".encode())
            time.sleep(0.2)
        ser.write(b"l\n")  # pasar a modo CSV

        # Esperar la cabecera CSV
        t_lim = time.time() + 10
        while time.time() < t_lim:
            linea = ser.readline().decode(errors="ignore").strip()
            if linea == CABECERA:
                break
        else:
            print("No llego la cabecera CSV. Esta cargado t4_madgwick?")
            return

        n = 0
        t_ini = time.time()
        print(f"Grabando en {archivo}  (Ctrl+C para cortar)")
        with open(archivo, "w", newline="") as f:
            f.write(CABECERA + "\n")
            try:
                while args.seg <= 0 or time.time() - t_ini < args.seg:
                    linea = ser.readline().decode(errors="ignore").strip()
                    if not linea or linea.startswith("#") or linea.count(",") != 14:
                        continue
                    f.write(linea + "\n")
                    n += 1
                    if n % 100 == 0:
                        print(f"\r{n} muestras  ({n / (time.time() - t_ini):.1f} Hz)", end="")
            except KeyboardInterrupt:
                pass
        ser.write(b"l\n")  # volver a modo Teleplot
    print(f"\nListo: {n} muestras en {archivo}")


if __name__ == "__main__":
    main()
