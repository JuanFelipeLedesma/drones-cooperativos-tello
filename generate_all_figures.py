"""
Genera TODAS las gráficas faltantes para el informe de tesis.
Procesa los CSVs/JSON de las pruebas 2.2, 2.3, 2.4, 3.1, 3.2, 3.3, 3.4,
4.1, 4.2 y 5.2. Guarda los PNG en presentation_assets/.

Las gráficas de 1.1, 1.2, 1.3 y 2.1 ya fueron generadas por
analyze_for_presentation.py.
"""
import csv
import json
import math
import statistics
from pathlib import Path
from collections import Counter, defaultdict

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# Rutas relativas al propio script → el repositorio es autocontenido y portable.
BASE = Path(__file__).resolve().parent
LOGS = BASE / "logs"
DL = LOGS   # los CSV del SLAVE se incluyen en logs/ (antes vivían fuera del repo)
OUT = BASE / "presentation_assets"
OUT.mkdir(exist_ok=True)

COLOR_X, COLOR_Y, COLOR_Z = "#E63946", "#2A9D8F", "#264653"
ACCENT = "#E76F51"
NAVY = "#1E2761"


def fnum(x):
    try:
        return float(x)
    except (ValueError, TypeError):
        return None


def load(path):
    with open(path) as f:
        return list(csv.DictReader(f))


# ============================================================
# 2.2 — Formación estática (con filtro vs sin filtro)
# ============================================================
def fig_2_2():
    with_filter = load(DL / "test_2_2_slave_20260506_094144.csv")
    without = load(DL / "test_2_2_slave_20260506_084913.csv")

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5),
                                   gridspec_kw={"width_ratios": [2, 1]})

    # Serie temporal con filtro
    ok = [r for r in with_filter if r.get("feedback") == "ok"]
    t = [fnum(r["elapsed"]) for r in ok if fnum(r["err_3d_cm"]) is not None]
    e = [fnum(r["err_3d_cm"]) for r in ok if fnum(r["err_3d_cm"]) is not None]
    ax1.plot(t, e, color=ACCENT, linewidth=1.0)
    ax1.axhline(15, color="red", linestyle="--", linewidth=1, label="Umbral 15 cm (plan)")
    cut = len(e) // 4
    mean_steady = statistics.mean(e[cut:])
    ax1.axhline(mean_steady, color=NAVY, linestyle=":", linewidth=1.5,
                label=f"Media estac. = {mean_steady:.1f} cm")
    ax1.set_xlabel("Tiempo [s]"); ax1.set_ylabel("Error de formación 3D [cm]")
    ax1.set_title("Error de formación con filtro promedio móvil (N=8)",
                  fontsize=11, fontweight="bold")
    ax1.grid(alpha=0.3); ax1.legend(loc="upper right", fontsize=9)
    ax1.set_ylim(0, 60)

    # Barras comparación
    labels = ["Sin filtro", "Con filtro (N=8)"]
    vals = [27.4, 12.9]
    colors = ["#999999", ACCENT]
    bars = ax2.bar(labels, vals, color=colors, edgecolor="black")
    ax2.axhline(15, color="red", linestyle="--", linewidth=1)
    ax2.set_ylabel("Error 3D estacionario [cm]")
    ax2.set_title("Efecto del filtro\n(−53%)", fontsize=11, fontweight="bold")
    for b, v in zip(bars, vals):
        ax2.text(b.get_x() + b.get_width()/2, v + 0.8, f"{v:.1f} cm",
                 ha="center", fontweight="bold")
    ax2.grid(axis="y", alpha=0.3)

    fig.suptitle("Prueba 2.2 — Formación estática líder-seguidor",
                 fontsize=13, fontweight="bold")
    plt.tight_layout()
    plt.savefig(OUT / "2_2_formacion_estatica.png", dpi=150, bbox_inches="tight")
    plt.close()
    print("  OK 2_2_formacion_estatica.png")


# ============================================================
# 2.3 — Formación dinámica (trayectoria + error)
# ============================================================
def fig_2_3():
    rows = load(DL / "test_2_3_slave_20260506_111120.csv")
    ok = [r for r in rows if r.get("feedback") == "ok"]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5.2))

    # Trayectoria en plano X-Z: master, slave-target (master+offset), slave real
    mx = [fnum(r["master_pos_x"]) for r in ok if fnum(r["master_pos_x"]) is not None]
    mz = [fnum(r["master_pos_z"]) for r in ok if fnum(r["master_pos_z"]) is not None]
    sx = [fnum(r["pos_x"]) for r in ok if fnum(r["pos_x"]) is not None]
    sz = [fnum(r["pos_z"]) for r in ok if fnum(r["pos_z"]) is not None]
    ax1.plot(mx, mz, color=COLOR_X, linewidth=1.2, alpha=0.8, label="MASTER (líder)")
    ax1.plot(sx, sz, color=COLOR_Y, linewidth=1.2, alpha=0.8, label="SLAVE (seguidor)")
    ax1.set_xlabel("X [m] (a lo largo de la pared)")
    ax1.set_ylabel("Z [m] (distancia a la pared)")
    ax1.set_title("Trayectorias en el plano horizontal",
                  fontsize=11, fontweight="bold")
    ax1.grid(alpha=0.3); ax1.legend(loc="best", fontsize=9)
    ax1.set_aspect("equal", adjustable="datalim")

    # Error 3D vs tiempo
    t = [fnum(r["elapsed"]) for r in ok if fnum(r["err_3d_cm"]) is not None]
    e = [fnum(r["err_3d_cm"]) for r in ok if fnum(r["err_3d_cm"]) is not None]
    ax2.plot(t, e, color=ACCENT, linewidth=1.0)
    ax2.axhline(30, color="red", linestyle="--", linewidth=1, label="Umbral 30 cm (mov.)")
    ax2.set_xlabel("Tiempo [s]"); ax2.set_ylabel("Error de formación 3D [cm]")
    ax2.set_title("Error de formación durante la trayectoria",
                  fontsize=11, fontweight="bold")
    ax2.grid(alpha=0.3); ax2.legend(loc="upper right", fontsize=9)

    fig.suptitle("Prueba 2.3 — Formación dinámica (trayectoria cuadrada 0,6×0,6 m)",
                 fontsize=13, fontweight="bold")
    plt.tight_layout()
    plt.savefig(OUT / "2_3_formacion_dinamica.png", dpi=150, bbox_inches="tight")
    plt.close()
    print("  OK 2_3_formacion_dinamica.png")


# ============================================================
# 2.4 — Consenso (convergencia de la distancia inter-dron)
# ============================================================
def fig_2_4():
    id1 = load(LOGS / "test_2_4_consensus_id1_20260506_170624.csv")
    ok = [r for r in id1 if r.get("feedback") == "ok"]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))

    # pos_x propio y del peer convergiendo
    t = [fnum(r["elapsed"]) for r in ok]
    mine_x = [fnum(r["pos_x"]) for r in ok]
    peer_x = [fnum(r["peer_pos_x"]) for r in ok]
    valid = [(ti, mi, pe) for ti, mi, pe in zip(t, mine_x, peer_x)
             if mi is not None and pe is not None]
    if valid:
        tv, miv, pev = zip(*valid)
        ax1.plot(tv, miv, color=COLOR_X, linewidth=1.3, label="Drone A (id=1)")
        ax1.plot(tv, pev, color=COLOR_Y, linewidth=1.3, label="Drone B (id=2)")
        ax1.set_xlabel("Tiempo [s]"); ax1.set_ylabel("Posición X [m]")
        ax1.set_title("Convergencia de posición X de ambos drones",
                      fontsize=11, fontweight="bold")
        ax1.grid(alpha=0.3); ax1.legend(loc="best", fontsize=9)

    # distancia inter-dron
    dist = [fnum(r["dist_to_peer_cm"]) for r in ok if fnum(r["dist_to_peer_cm"]) is not None]
    td = [fnum(r["elapsed"]) for r in ok if fnum(r["dist_to_peer_cm"]) is not None]
    ax2.plot(td, dist, color=ACCENT, linewidth=1.2)
    ax2.axhline(60, color=NAVY, linestyle="--", linewidth=1.5,
                label="Separación diseñada = 60 cm")
    ax2.set_xlabel("Tiempo [s]"); ax2.set_ylabel("Distancia inter-dron (X-Z) [cm]")
    ax2.set_title("Convergencia al consenso con separación mínima",
                  fontsize=11, fontweight="bold")
    ax2.grid(alpha=0.3); ax2.legend(loc="upper right", fontsize=9)

    fig.suptitle("Prueba 2.4 — Consenso distribuido con separación mínima de seguridad",
                 fontsize=13, fontweight="bold")
    plt.tight_layout()
    plt.savefig(OUT / "2_4_consenso.png", dpi=150, bbox_inches="tight")
    plt.close()
    print("  OK 2_4_consenso.png")


# ============================================================
# 3.1 — Benchmark Ethernet (del JSON)
# ============================================================
def fig_3_1():
    with open(LOGS / "test_3_1_ethernet_20260505_172224.json") as f:
        data = json.load(f)
    coop = data.get("coop_protocol", {})
    freqs = ["10Hz", "25Hz", "50Hz"]
    rtt = [coop[f]["rtt_avg_ms"] for f in freqs if f in coop]
    jitter = [coop[f]["jitter_ms"] for f in freqs if f in coop]
    loss = [coop[f]["loss_pct"] for f in freqs if f in coop]
    labels = [f.replace("Hz", " Hz") for f in freqs if f in coop]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))
    x = np.arange(len(labels))
    w = 0.35
    ax1.bar(x - w/2, rtt, w, label="Latencia media [ms]", color=ACCENT, edgecolor="black")
    ax1.bar(x + w/2, jitter, w, label="Jitter [ms]", color=NAVY, edgecolor="black")
    ax1.set_xticks(x); ax1.set_xticklabels(labels)
    ax1.set_ylabel("Tiempo [ms]")
    ax1.set_title("Latencia y jitter del protocolo UDP por frecuencia",
                  fontsize=11, fontweight="bold")
    ax1.set_ylim(0, max(rtt + jitter) * 1.35)
    ax1.legend(loc="upper right"); ax1.grid(axis="y", alpha=0.3)
    for i, v in enumerate(rtt):
        ax1.text(i - w/2, v + 0.05, f"{v:.2f}", ha="center", fontsize=8)

    # Callout con ping + throughput
    ping = data.get("ping", {})
    ip3 = data.get("iperf3", {})
    ax2.axis("off")
    txt = (f"Ping RTT promedio:\n  {ping.get('rtt_avg_ms', '?'):.2f} ms\n\n"
           f"Pérdida de paquetes:\n  {ping.get('loss_pct', 0):.1f} %\n\n"
           f"Throughput (iperf3):\n  {ip3.get('throughput_mbps', 0):.1f} Mbps\n\n"
           f"Pérdida UDP cooperación:\n  0,0 % en 10/25/50 Hz")
    ax2.text(0.1, 0.5, txt, fontsize=14, va="center", family="monospace",
             bbox=dict(boxstyle="round,pad=0.8", facecolor="#F2F2F2", edgecolor=NAVY))
    ax2.set_title("Rendimiento base del enlace", fontsize=11, fontweight="bold")

    fig.suptitle("Prueba 3.1 — Benchmarking del enlace Ethernet",
                 fontsize=13, fontweight="bold")
    plt.tight_layout()
    plt.savefig(OUT / "3_1_benchmark_ethernet.png", dpi=150, bbox_inches="tight")
    plt.close()
    print("  OK 3_1_benchmark_ethernet.png")


# ============================================================
# 3.2 — Protocolo (binario vs JSON)
# ============================================================
def fig_3_2():
    with open(LOGS / "test_3_2_protocol_20260505_174112.json") as f:
        data = json.load(f)
    b = data["binary"]; j = data["json"]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))

    # Tamaño
    ax1.bar(["Binario", "JSON"], [b["msg_size_bytes"], j["msg_size_bytes"]],
            color=[ACCENT, "#999999"], edgecolor="black")
    ax1.set_ylabel("Tamaño del mensaje [bytes]")
    ax1.set_title(f"Tamaño: binario {b['msg_size_bytes']} B vs JSON {j['msg_size_bytes']} B (4×)",
                  fontsize=11, fontweight="bold")
    for i, v in enumerate([b["msg_size_bytes"], j["msg_size_bytes"]]):
        ax1.text(i, v + 3, f"{v} B", ha="center", fontweight="bold")
    ax1.grid(axis="y", alpha=0.3)

    # Encode/decode
    x = np.arange(2); w = 0.35
    ax2.bar(x - w/2, [b["encode_us_mean"], b["decode_us_mean"]], w,
            label="Binario", color=ACCENT, edgecolor="black")
    ax2.bar(x + w/2, [j["encode_us_mean"], j["decode_us_mean"]], w,
            label="JSON", color="#999999", edgecolor="black")
    ax2.set_xticks(x); ax2.set_xticklabels(["Encode", "Decode"])
    ax2.set_ylabel("Tiempo [µs]")
    ax2.set_title("Overhead de serialización (binario 3-5× más rápido)",
                  fontsize=11, fontweight="bold")
    ax2.legend(); ax2.grid(axis="y", alpha=0.3)

    fig.suptitle("Prueba 3.2 — Protocolo de mensajes: binario vs JSON",
                 fontsize=13, fontweight="bold")
    plt.tight_layout()
    plt.savefig(OUT / "3_2_protocolo.png", dpi=150, bbox_inches="tight")
    plt.close()
    print("  OK 3_2_protocolo.png")


# ============================================================
# 3.3 — Degradación (error vs delay, error vs loss)
# ============================================================
def fig_3_3():
    rows = load(DL / "test_3_3_slave_20260507_165017.csv")
    by_cond = defaultdict(list)
    for r in rows:
        if r.get("feedback") == "ok" and fnum(r.get("err_3d_cm")) is not None:
            by_cond[r.get("network_condition")].append(fnum(r["err_3d_cm"]))

    def mean_std(cond):
        v = by_cond.get(cond, [])
        if not v: return 0, 0
        return statistics.mean(v), (statistics.stdev(v) if len(v) > 1 else 0)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))

    # Error vs delay
    delay_conds = ["baseline", "delay_50ms", "delay_100ms", "delay_200ms"]
    delay_x = [0, 50, 100, 200]
    delay_m = [mean_std(c)[0] for c in delay_conds]
    delay_s = [mean_std(c)[1] for c in delay_conds]
    ax1.errorbar(delay_x, delay_m, yerr=delay_s, marker="o", markersize=8,
                 color=ACCENT, capsize=5, linewidth=2)
    ax1.set_xlabel("Retardo inyectado [ms]")
    ax1.set_ylabel("Error de formación 3D [cm]")
    ax1.set_title("Error vs retardo de red", fontsize=11, fontweight="bold")
    ax1.grid(alpha=0.3)
    for xi, mi in zip(delay_x, delay_m):
        ax1.text(xi, mi + 2, f"{mi:.0f}", ha="center", fontsize=9)

    # Error vs loss
    loss_conds = ["baseline", "loss_5pct", "loss_10pct", "loss_20pct"]
    loss_x = [0, 5, 10, 20]
    loss_m = [mean_std(c)[0] for c in loss_conds]
    loss_s = [mean_std(c)[1] for c in loss_conds]
    ax2.errorbar(loss_x, loss_m, yerr=loss_s, marker="s", markersize=8,
                 color=NAVY, capsize=5, linewidth=2)
    ax2.set_xlabel("Pérdida de paquetes [%]")
    ax2.set_ylabel("Error de formación 3D [cm]")
    ax2.set_title("Error vs pérdida de paquetes", fontsize=11, fontweight="bold")
    ax2.grid(alpha=0.3)
    for xi, mi in zip(loss_x, loss_m):
        ax2.text(xi, mi + 2, f"{mi:.0f}", ha="center", fontsize=9)

    fig.suptitle("Prueba 3.3 — Degradación controlada de la red",
                 fontsize=13, fontweight="bold")
    plt.tight_layout()
    plt.savefig(OUT / "3_3_degradacion.png", dpi=150, bbox_inches="tight")
    plt.close()
    print("  OK 3_3_degradacion.png")


# ============================================================
# 3.4 — Tolerancia a fallas (error vs tiempo con desconexiones)
# ============================================================
def fig_3_4():
    rows = load(DL / "test_3_4_slave_20260506_183809.csv")
    t = [fnum(r["elapsed"]) for r in rows if fnum(r["elapsed"]) is not None]
    age = [fnum(r["master_age_s"]) for r in rows if fnum(r["elapsed"]) is not None]

    fig, ax = plt.subplots(figsize=(13, 5))
    # master_age como proxy del estado de conexión
    ax.plot(t, [a*1000 if a is not None and a >= 0 else None for a in age],
            color=ACCENT, linewidth=0.8, label="Edad del último mensaje [ms]")
    ax.axhline(5000, color="red", linestyle="--", linewidth=1,
               label="Umbral hover seguro (5 s)")
    ax.set_xlabel("Tiempo [s]"); ax.set_ylabel("Edad del mensaje del MASTER [ms]")
    ax.set_title("Prueba 3.4 — Tolerancia a desconexión: edad de mensajes y eventos de pérdida",
                 fontsize=12, fontweight="bold")
    ax.grid(alpha=0.3); ax.legend(loc="upper left", fontsize=9)
    ax.set_yscale("symlog")

    # Anotar las 3 desconexiones
    events = [(18.0, "10 s\n52 cm"), (49.2, "18 s\n100 cm"), (86.4, "8 s\n39 cm")]
    for et, lbl in events:
        ax.axvline(et, color="gray", alpha=0.4, linestyle=":")
        ax.text(et, ax.get_ylim()[1]*0.5, lbl, fontsize=8, color="gray", ha="center")

    plt.tight_layout()
    plt.savefig(OUT / "3_4_tolerancia.png", dpi=150, bbox_inches="tight")
    plt.close()
    print("  OK 3_4_tolerancia.png")


# ============================================================
# 4.1 — Simulación (trayectoria + error)
# ============================================================
def fig_4_1():
    rows = load(LOGS / "test_4_1_simulation.csv")
    mx = [fnum(r["master_x"]) for r in rows]
    mz = [fnum(r["master_z"]) for r in rows]
    sx = [fnum(r["slave_x"]) for r in rows]
    sz = [fnum(r["slave_z"]) for r in rows]
    t = [fnum(r["t"]) for r in rows]
    e = [fnum(r["err_3d_cm"]) for r in rows]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5.2))
    ax1.plot(mx, mz, color=COLOR_X, linewidth=1.2, label="MASTER (sim)")
    ax1.plot(sx, sz, color=COLOR_Y, linewidth=1.2, label="SLAVE (sim)")
    ax1.set_xlabel("X [m]"); ax1.set_ylabel("Z [m]")
    ax1.set_title("Trayectorias simuladas", fontsize=11, fontweight="bold")
    ax1.grid(alpha=0.3); ax1.legend(loc="best", fontsize=9)
    ax1.set_aspect("equal", adjustable="datalim")

    ax2.plot(t, e, color=ACCENT, linewidth=1.0)
    cut = len(e) // 5
    mean_steady = statistics.mean([x for x in e[cut:] if x is not None])
    ax2.axhline(mean_steady, color=NAVY, linestyle=":", linewidth=1.5,
                label=f"Media sim = {mean_steady:.1f} cm")
    ax2.axhline(23, color="red", linestyle="--", linewidth=1,
                label="Real 2.3 ≈ 23 cm")
    ax2.set_xlabel("Tiempo [s]"); ax2.set_ylabel("Error de formación 3D [cm]")
    ax2.set_title("Error simulado vs real (sim-to-real gap)",
                  fontsize=11, fontweight="bold")
    ax2.grid(alpha=0.3); ax2.legend(loc="upper right", fontsize=9)

    fig.suptitle("Prueba 4.1 — Simulación de la formación dinámica",
                 fontsize=13, fontweight="bold")
    plt.tight_layout()
    plt.savefig(OUT / "4_1_simulacion.png", dpi=150, bbox_inches="tight")
    plt.close()
    print("  OK 4_1_simulacion.png")


# ============================================================
# 4.2 — Sim vs real bajo degradación
# ============================================================
def fig_4_2():
    conds = ["baseline", "delay\n50ms", "delay\n100ms", "delay\n200ms",
             "loss\n5%", "loss\n10%", "loss\n20%", "combo"]
    sim = [11.8, 11.0, 12.8, 7.9, 11.7, 9.0, 9.5, 8.6]
    real = [21.5, 25.2, 26.4, 41.5, 33.7, 30.3, 36.1, 43.4]

    fig, ax = plt.subplots(figsize=(13, 5.5))
    x = np.arange(len(conds)); w = 0.38
    ax.bar(x - w/2, sim, w, label="Simulación 4.2", color=ACCENT, edgecolor="black")
    ax.bar(x + w/2, real, w, label="Real 3.3", color=NAVY, edgecolor="black")
    ax.set_xticks(x); ax.set_xticklabels(conds, fontsize=9)
    ax.set_ylabel("Error de formación 3D [cm]")
    ax.set_title("Prueba 4.2 — Error de formación: simulación vs realidad por condición de red",
                 fontsize=12, fontweight="bold")
    ax.legend(loc="upper left"); ax.grid(axis="y", alpha=0.3)
    ax.text(0.5, 0.95, "Hallazgo: la simulación se mantiene plana → el delay NO\n"
                       "desestabiliza el lazo líder-seguidor (feedback propio del seguidor)",
            transform=ax.transAxes, fontsize=9, va="top", ha="left",
            bbox=dict(boxstyle="round,pad=0.5", facecolor="#FFF3E0", edgecolor=ACCENT))
    plt.tight_layout()
    plt.savefig(OUT / "4_2_sim_vs_real.png", dpi=150, bbox_inches="tight")
    plt.close()
    print("  OK 4_2_sim_vs_real.png")


# ============================================================
# 5.2 — Repetibilidad (8 reps, 7 válidas)
# ============================================================
def fig_5_2():
    # Datos de las 8 reps (rep 1 = fallida por IMU)
    reps = list(range(1, 9))
    master_err = [23.73, 13.09, 12.57, 11.72, 11.39, 10.14, 11.72, 10.71]
    slave_err  = [100.54, 20.48, 18.42, 22.49, 17.01, 18.78, 19.86, 12.90]
    valid = [False, True, True, True, True, True, True, True]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))

    colors_m = [ACCENT if v else "#CCCCCC" for v in valid]
    colors_s = [NAVY if v else "#CCCCCC" for v in valid]

    ax1.bar([r - 0.2 for r in reps], master_err, 0.4, color=colors_m,
            edgecolor="black", label="MASTER")
    ax1.bar([r + 0.2 for r in reps], slave_err, 0.4, color=colors_s,
            edgecolor="black", label="SLAVE")
    ax1.set_xlabel("Repetición"); ax1.set_ylabel("Error 3D [cm]")
    ax1.set_title("Error por repetición (gris = rep fallida por IMU)",
                  fontsize=11, fontweight="bold")
    ax1.set_xticks(reps); ax1.legend(); ax1.grid(axis="y", alpha=0.3)

    # Métricas resumen (7 reps válidas) con barras de error
    valid_master = [master_err[i] for i in range(8) if valid[i]]
    valid_slave = [slave_err[i] for i in range(8) if valid[i]]
    durations = [73.9, 71.7, 67.8, 69.2, 71.1, 71.1, 74.0]
    metrics = ["MASTER\nerr 3D", "SLAVE\nerr 3D", "Duración\nmisión [s]"]
    means = [statistics.mean(valid_master), statistics.mean(valid_slave),
             statistics.mean(durations)]
    stds = [statistics.stdev(valid_master), statistics.stdev(valid_slave),
            statistics.stdev(durations)]
    xb = np.arange(len(metrics))
    ax2.bar(xb, means, yerr=stds, capsize=8, color=[ACCENT, NAVY, COLOR_Y],
            edgecolor="black")
    ax2.set_xticks(xb); ax2.set_xticklabels(metrics)
    ax2.set_ylabel("Valor (media ± std)")
    ax2.set_title("Métricas de repetibilidad (7 reps válidas)",
                  fontsize=11, fontweight="bold")
    ax2.grid(axis="y", alpha=0.3)
    for i, (m, s) in enumerate(zip(means, stds)):
        ax2.text(i, m + s + 1, f"{m:.1f}±{s:.1f}", ha="center", fontsize=9)

    fig.suptitle("Prueba 5.2 — Repetibilidad de la misión cooperativa (8 repeticiones)",
                 fontsize=13, fontweight="bold")
    plt.tight_layout()
    plt.savefig(OUT / "5_2_repetibilidad.png", dpi=150, bbox_inches="tight")
    plt.close()
    print("  OK 5_2_repetibilidad.png")


def main():
    print("Generando gráficas faltantes...")
    for fn in [fig_2_2, fig_2_3, fig_2_4, fig_3_1, fig_3_2,
               fig_3_3, fig_3_4, fig_4_1, fig_4_2, fig_5_2]:
        try:
            fn()
        except Exception as e:
            print(f"  [ERROR] {fn.__name__}: {e}")
    print(f"\nGráficas en: {OUT}")


if __name__ == "__main__":
    main()
