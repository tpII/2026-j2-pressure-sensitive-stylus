"""
calibrar_presion_hall.py - curva de presion del Hall SS49E en la punta del lapiz
J2 - Pressure Sensitive Stylus | Taller de Proyecto II - UNLP

Para que sirve:
  - Pasar de "mV del Hall" a una fuerza en gramos (el grosor del trazo hoy es
    lineal en mV, pero el campo del iman no es lineal con la distancia).
  - Medir los limites del sensor de contacto que pide el objetivo 5 del plan:
    fuerza minima que detecta el contacto, resolucion en gramos (ruido / pendiente),
    saturacion e histeresis (subida contra bajada).

Como se hace (10-15 min):
  1. Cargar firmware/ensayo_desplazamiento_hall en el ESP32 y cerrar cualquier
     monitor serie o visor que tenga el puerto abierto.
  2. Una balanza de cocina en gramos, TARADA sin nada encima. El lapiz (con el
     iman y el resorte ya armados) se apoya vertical con la punta sobre la
     balanza; el primer punto es el lapiz apoyado sin apretar (su peso propio).
  3. Correr:  python tools/calibrar_presion_hall.py --puerto COM3
     El script toma la base del Hall con el lapiz en el aire (manda 'h').
  4. Para cada punto: apretar hasta el valor que se quiera (por ejemplo 0, 20, 50,
     100, 150, 200, 300, 500 g), MANTENER, escribir lo que marca la balanza y
     Enter. Se graban 2 s. Primero subiendo y despues bajando (para la histeresis).
     Enter vacio pasa de la subida a la bajada, y otro Enter vacio termina.

Salida (en tools/):
  calibracion_presion_hall.json   la tabla dif_mV -> g que usan los visores
  presion_hall_AAAAMMDD_HHMMSS.csv  todos los puntos
  presion_hall_AAAAMMDD_HHMMSS.png  curva (si hay matplotlib)

Requiere:  pip install pyserial   (y matplotlib para el grafico, opcional)
Sin hardware:  python tools/calibrar_presion_hall.py --simular   (prueba el script)
"""
import argparse
import csv
import json
import math
import os
import random
import statistics
import sys
import time
from datetime import datetime

AQUI = os.path.dirname(os.path.abspath(__file__))
HALL_ON_MV = 150.0       # umbral de contacto del firmware (HALL_ON_MV)
SATURA_MV = 3050.0       # cerca del tope del ADC con atenuacion de 11 dB
SEGUNDOS = 2.0


# ------------------------------------------------------------------ fuentes --
class FuenteSerie:
    def __init__(self, puerto, baud):
        try:
            import serial
        except ImportError:
            sys.exit("Falta pyserial: pip install pyserial")
        self.s = serial.Serial(puerto, baud, timeout=0.2)
        time.sleep(0.3)

    def enviar(self, c):
        self.s.write(c.encode())

    def lineas(self, segundos):
        fin = time.time() + segundos
        buf = b""
        while time.time() < fin:
            buf += self.s.read(4096)
            *partes, buf = buf.split(b"\n")
            for p in partes:
                yield p.decode(errors="ignore").strip()

    def vaciar(self):
        self.s.reset_input_buffer()


class FuenteSimulada:
    """Hall con base 1708 mV, ruido 8 mV y respuesta no lineal con la fuerza."""
    def __init__(self):
        self.g = 0.0
        self.fase = "subida"

    def enviar(self, c):
        pass

    def poner(self, g):
        self.g = g

    def lineas(self, segundos):
        n = int(segundos * 100)
        yield "#cfg hall_base_mV=1708.4 ruido_mV=8.1 on_mV=150 off_mV=100"
        histeresis = -15.0 if self.fase == "bajada" else 0.0
        for _ in range(n):
            dif = 1100 * (1 - math.exp(-self.g / 260.0)) + (histeresis if self.g > 0 else 0)
            mv = 1708 + dif + random.gauss(0, 8)
            c = 1 if dif > HALL_ON_MV else 0
            yield f"D,0,0,0,0,0,0,0,0,0,1800,0,0,1,0,0,0,{mv:.0f},{c}"

    def vaciar(self):
        pass


# ---------------------------------------------------------------- medicion --
def leer_hall(fuente, segundos):
    """Devuelve (lista de hall_mV, base informada por el firmware o None)."""
    vals, base = [], None
    for ln in fuente.lineas(segundos):
        if ln.startswith("#") and "hall_base_mV=" in ln:
            try:
                base = float(ln.split("hall_base_mV=")[1].split()[0])
            except ValueError:
                pass
        elif ln.startswith("D,"):
            f = ln.split(",")
            if len(f) == 19:
                try:
                    vals.append(float(f[17]))
                except ValueError:
                    pass
    return vals, base


def medir_base(fuente):
    print("\nLevantar el lapiz (punta en el aire, sin tocar nada) y Enter...")
    input()
    fuente.vaciar()
    fuente.enviar("h")
    vals, base = leer_hall(fuente, 2.5)
    if base is None:
        if not vals:
            sys.exit("No llegan datos: revisar el puerto y que este cargado ensayo_desplazamiento_hall.")
        base = statistics.median(vals)
        print(f"(el firmware no informo la base; se usa la mediana: {base:.1f} mV)")
    print(f"Base del Hall: {base:.1f} mV")
    return base


def pedir_puntos(fuente, base, simulada):
    puntos = []
    for fase in ("subida", "bajada"):
        if simulada:
            fuente.fase = fase
            serie = [0, 20, 50, 100, 150, 200, 300, 500, 750] if fase == "subida" else [500, 300, 200, 100, 50, 20]
        print(f"\n=== {fase.upper()} ===  (Enter vacio para {'pasar a la bajada' if fase == 'subida' else 'terminar'})")
        while True:
            if simulada:
                if not serie:
                    break
                g = serie.pop(0)
                fuente.poner(g)
                print(f"[simulado] {g} g")
            else:
                r = input("Apretar, MANTENER, escribir los gramos de la balanza y Enter: ").strip().replace(",", ".")
                if r == "":
                    break
                try:
                    g = float(r)
                except ValueError:
                    print("  numero no valido")
                    continue
            fuente.vaciar()
            vals, _ = leer_hall(fuente, SEGUNDOS)
            if len(vals) < 20:
                print("  muy pocas muestras, repetir")
                continue
            # se usa la diferencia en valor absoluto, como el firmware (no importa el polo)
            difs = [abs(v - base) for v in vals]
            med = statistics.median(difs)
            sd = statistics.pstdev(difs)
            sat = max(vals) > SATURA_MV or min(vals) < 100
            puntos.append({"fase": fase, "g": g, "dif_mV": round(med, 1), "desvio_mV": round(sd, 1),
                           "n": len(vals), "saturado": sat})
            print(f"  {g:7.1f} g -> {med:7.1f} mV (desvio {sd:4.1f} mV, {len(vals)} muestras)" + ("  SATURADO" if sat else ""))
    return puntos


# ------------------------------------------------------------------ analisis --
def analizar(puntos, base):
    sub = sorted([p for p in puntos if p["fase"] == "subida" and not p["saturado"]], key=lambda p: p["g"])
    if len(sub) < 2:
        sys.exit("Hacen falta al menos 2 puntos de subida sin saturar.")
    # tabla monotona (dif_mV creciente): si un punto baja, se descarta
    tabla = []
    for p in sub:
        if not tabla or p["dif_mV"] > tabla[-1][0]:
            tabla.append([p["dif_mV"], p["g"]])

    def g_de(dif):
        if dif <= tabla[0][0]:
            return tabla[0][1]
        for (m0, g0), (m1, g1) in zip(tabla, tabla[1:]):
            if dif <= m1:
                return g0 + (g1 - g0) * (dif - m0) / (m1 - m0)
        return None   # fuera de la tabla

    res = {"base_mV": round(base, 1), "tabla": tabla}
    res["umbral_contacto_g"] = None if g_de(HALL_ON_MV) is None else round(g_de(HALL_ON_MV), 1)
    # resolucion por tramo: desvio / pendiente
    tramos = []
    for (m0, g0), (m1, g1) in zip(tabla, tabla[1:]):
        pend = (m1 - m0) / (g1 - g0) if g1 > g0 else float("nan")   # mV por gramo
        sd = statistics.mean([p["desvio_mV"] for p in sub if g0 <= p["g"] <= g1])
        tramos.append({"de_g": g0, "a_g": g1, "mV_por_g": round(pend, 2),
                       "resolucion_g": round(sd / pend, 1) if pend > 0 else None})
    res["tramos"] = tramos
    # histeresis: misma fuerza en subida y bajada
    baj = [p for p in puntos if p["fase"] == "bajada" and not p["saturado"]]
    hist = []
    for p in baj:
        g_sub = g_de(p["dif_mV"])
        if g_sub is not None:
            hist.append(round(g_sub - p["g"], 1))
    res["histeresis_g"] = {"max_abs": max(map(abs, hist)) if hist else None, "puntos": hist}
    res["saturacion"] = [p["g"] for p in puntos if p["saturado"]]
    return res


def graficar(puntos, res, ruta):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return False
    fig, ax = plt.subplots(figsize=(7, 4.2), dpi=130)
    for fase, mk in (("subida", "o-"), ("bajada", "s--")):
        pp = sorted([p for p in puntos if p["fase"] == fase], key=lambda p: p["g"])
        if pp:
            ax.errorbar([p["g"] for p in pp], [p["dif_mV"] for p in pp], yerr=[p["desvio_mV"] for p in pp],
                        fmt=mk, capsize=3, label=fase)
    ax.axhline(HALL_ON_MV, color="gray", lw=0.8, ls=":")
    ax.text(0, HALL_ON_MV, " umbral de contacto", va="bottom", fontsize=8, color="gray")
    ax.set_xlabel("Fuerza en la punta [g]")
    ax.set_ylabel("|Hall - base| [mV]")
    ax.set_title("Curva de presion del Hall SS49E")
    ax.grid(alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(ruta)
    return True


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--puerto", default="COM3")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--simular", action="store_true", help="sin hardware, para probar el script")
    ap.add_argument("--salida", default=AQUI, help="carpeta de salida (por defecto tools/)")
    a = ap.parse_args()

    if a.simular:
        random.seed(1)
        fuente = FuenteSimulada()
        import builtins
        builtins.input = lambda *x: ""
    else:
        fuente = FuenteSerie(a.puerto, a.baud)
    base = medir_base(fuente)
    puntos = pedir_puntos(fuente, base, a.simular)
    res = analizar(puntos, base)

    sello = datetime.now().strftime("%Y%m%d_%H%M%S")
    os.makedirs(a.salida, exist_ok=True)
    with open(os.path.join(a.salida, f"presion_hall_{sello}.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=["fase", "g", "dif_mV", "desvio_mV", "n", "saturado"])
        w.writeheader()
        w.writerows(puntos)
    res["fecha"] = sello
    with open(os.path.join(a.salida, "calibracion_presion_hall.json"), "w", encoding="utf-8") as f:
        json.dump(res, f, indent=2, ensure_ascii=False)
    png = os.path.join(a.salida, f"presion_hall_{sello}.png")
    hay_png = graficar(puntos, res, png)

    print("\n=== Resultado ===")
    print(f"Base: {res['base_mV']} mV")
    print(f"Contacto (|dif| > {HALL_ON_MV:.0f} mV) a partir de: "
          + (f"{res['umbral_contacto_g']} g" if res["umbral_contacto_g"] is not None else "fuera de la tabla"))
    for t in res["tramos"]:
        print(f"  {t['de_g']:6.0f}-{t['a_g']:<6.0f} g: {t['mV_por_g']:6.2f} mV/g, resolucion ~{t['resolucion_g']} g")
    if res["histeresis_g"]["max_abs"] is not None:
        print(f"Histeresis maxima: {res['histeresis_g']['max_abs']} g")
    if res["saturacion"]:
        print(f"Saturado en: {res['saturacion']} g (bajar el iman o usar un resorte mas duro)")
    print(f"\nGuardado en {a.salida}: calibracion_presion_hall.json, presion_hall_{sello}.csv"
          + (f", presion_hall_{sello}.png" if hay_png else ""))


if __name__ == "__main__":
    main()
