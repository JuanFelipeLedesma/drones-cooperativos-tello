"""
Análisis completo + generación de gráficas para la presentación de avances.
Procesa los CSVs de las pruebas 1.1, 1.2, 1.3 y 2.1.
Genera PNGs en presentation_assets/.
"""
import csv
import statistics
import math
import json
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")  # sin display
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

BASE = Path(__file__).resolve().parent
LOGS = BASE / "logs"
OUT = BASE / "presentation_assets"
OUT.mkdir(exist_ok=True)

CSV_11 = LOGS / "test_1_1_step_response_20260430_170517.csv"
CSV_12 = LOGS / "test_1_2_latency_20260430_194507.csv"
CSV_13 = LOGS / "test_1_3_hover_20260430_191701.csv"
CSV_21 = LOGS / "test_2_1_closed_loop_20260430_190555.csv"

# Colores consistentes
COLOR_X, COLOR_Y, COLOR_Z = "#E63946", "#2A9D8F", "#264653"
ACCENT = "#E76F51"

# Resultados acumulados para guardar a JSON
results = {}


def load_csv(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def fnum(x):
    if x in (None, "", "None"):
        return None
    try:
        return float(x)
    except (ValueError, TypeError):
        return None


# ============================================================
# PRUEBA 1.1 — Step response
# ============================================================
def analyze_test_1_1():
    print("\n=== ANALIZANDO 1.1 (step response) ===")
    rows = load_csv(CSV_11)
    print(f"Filas: {len(rows)}")

    CMD = {"right": ("x", 0.30), "up": ("y", 0.30),
           "left": ("x", -0.30), "down": ("y", -0.30)}

    # Métricas por (cmd, rep)
    stats_per_run = defaultdict(list)
    trajectories = defaultdict(list)  # (cmd) → list of (rep, t_array, pos_array, p0)

    for cmd, (axis, dx) in CMD.items():
        for rep in range(1, 4):
            sub = [r for r in rows
                   if r["command"] == cmd and r["repeat"] == str(rep)
                   and r["phase"] == "response"
                   and fnum(r[f"pos_{axis}"]) is not None]
            if len(sub) < 10:
                continue
            t_send = fnum(sub[0]["t_send"])
            ts = np.array([fnum(r["t_since_cmd"]) for r in sub])
            xs = np.array([fnum(r[f"pos_{axis}"]) for r in sub])
            p0 = float(np.mean(xs[:5]))
            target = p0 + dx
            sign = 1 if dx > 0 else -1

            # Métricas
            real_motion_cm = (xs[-1] - xs[0]) * 100
            extreme = float(np.max(xs)) if dx > 0 else float(np.min(xs))
            overshoot_cm = max(0.0, sign * (extreme - target)) * 100
            tail = xs[len(xs) * 2 // 3:]
            final_pos = float(np.mean(tail))
            err_fin_cm = (final_pos - target) * 100

            # Settling time al 5 % (banda min 4 cm)
            band = max(0.04, 0.05 * abs(dx))
            settling_t = None
            for i in range(len(xs) - 1, -1, -1):
                if abs(xs[i] - target) > band:
                    settling_t = float(ts[i])
                    break

            # Tiempo de respuesta: cruce de 2 cm en sentido del comando
            response_t = None
            for i in range(len(xs)):
                if sign * (xs[i] - p0) > 0.02:
                    response_t = float(ts[i])
                    break

            stats_per_run[cmd].append({
                "rep": rep, "p0": p0, "target": target,
                "real_motion_cm": real_motion_cm,
                "overshoot_cm": overshoot_cm,
                "err_fin_cm": err_fin_cm,
                "settling_s": settling_t,
                "response_s": response_t,
            })
            trajectories[cmd].append({
                "rep": rep, "t": ts, "pos": xs, "p0": p0, "target": target,
            })

    # Resumen estadístico por comando
    summary = {}
    for cmd, runs in stats_per_run.items():
        motions = [r["real_motion_cm"] for r in runs]
        oversh = [r["overshoot_cm"] for r in runs if r["overshoot_cm"] is not None]
        errs = [r["err_fin_cm"] for r in runs]
        settles = [r["settling_s"] for r in runs if r["settling_s"] is not None]
        responses = [r["response_s"] for r in runs if r["response_s"] is not None]
        summary[cmd] = {
            "n_reps": len(runs),
            "motion_mean_cm": float(np.mean(motions)),
            "motion_std_cm": float(np.std(motions, ddof=1)) if len(motions) > 1 else 0.0,
            "overshoot_mean_cm": float(np.mean(oversh)) if oversh else 0.0,
            "err_fin_mean_cm": float(np.mean(errs)),
            "err_fin_std_cm": float(np.std(errs, ddof=1)) if len(errs) > 1 else 0.0,
            "settling_mean_s": float(np.mean(settles)) if settles else None,
            "response_mean_s": float(np.mean(responses)) if responses else None,
        }
    results["1.1"] = {"summary": summary, "n_reps_total": sum(s["n_reps"] for s in summary.values())}

    print("Resumen 1.1:")
    for cmd, s in summary.items():
        print(f"  {cmd}: {s['n_reps']} reps  motion={s['motion_mean_cm']:+.1f}±{s['motion_std_cm']:.1f}cm  "
              f"oversh={s['overshoot_mean_cm']:.1f}cm  err_fin={s['err_fin_mean_cm']:+.1f}±{s['err_fin_std_cm']:.1f}cm")

    # ----- GRÁFICA 1: 4 subplots (uno por comando) con las 3 reps de cada uno -----
    fig, axes = plt.subplots(2, 2, figsize=(13, 8))
    cmd_titles = {
        "right": ("X (right +30 cm)", "x"),
        "up":    ("Y (up +30 cm)",    "y"),
        "left":  ("X (left −30 cm)",  "x"),
        "down":  ("Y (down −30 cm)",  "y"),
    }
    for ax, (cmd, (title, axislabel)) in zip(axes.flat, cmd_titles.items()):
        for tr in trajectories[cmd]:
            ax.plot(tr["t"], (tr["pos"] - tr["p0"]) * 100,
                    label=f"rep {tr['rep']}", linewidth=1.5, alpha=0.85)
        # Target line (delta = ±30)
        target_delta = 30 if cmd in ("right", "up") else -30
        ax.axhline(target_delta, color="black", linestyle="--",
                   linewidth=1, label=f"target ({target_delta:+d} cm)")
        ax.axhline(0, color="gray", linestyle=":", linewidth=0.5)
        ax.set_title(f"Comando '{cmd}' — desplazamiento eje {axislabel}",
                     fontsize=11, fontweight="bold")
        ax.set_xlabel("Tiempo desde envío del comando [s]")
        ax.set_ylabel("Δ posición [cm]")
        ax.grid(alpha=0.3)
        ax.legend(loc="best", fontsize=9)
        ax.set_xlim(0, 10)
    fig.suptitle("Prueba 1.1 — Respuesta escalón del Tello (12 step responses, 3 reps × 4 ejes)",
                 fontsize=13, fontweight="bold")
    plt.tight_layout()
    plt.savefig(OUT / "1_1_step_responses.png", dpi=150, bbox_inches="tight")
    plt.close()
    print(f"  ✓ {OUT / '1_1_step_responses.png'}")

    # ----- GRÁFICA 2: barras con bias y std del desplazamiento real -----
    cmds = list(summary.keys())
    targets = [30 if c in ("right", "up") else -30 for c in cmds]
    means = [summary[c]["motion_mean_cm"] for c in cmds]
    stds = [summary[c]["motion_std_cm"] for c in cmds]
    bias = [m - t for m, t in zip(means, targets)]

    fig, ax = plt.subplots(figsize=(10, 6.5))
    x = np.arange(len(cmds))
    width = 0.35
    ax.bar(x - width/2, targets, width, label="Target", color="#cccccc", edgecolor="black")
    ax.bar(x + width/2, means, width, yerr=stds, capsize=8, label="Real (mean ± std)",
           color=ACCENT, edgecolor="black")
    ax.axhline(0, color="black", linewidth=0.5)
    ax.set_xticks(x); ax.set_xticklabels(cmds, fontsize=12)
    ax.set_ylabel("Desplazamiento [cm]", fontsize=11)
    ax.set_title("Prueba 1.1 — Desplazamiento real vs comandado (target ±30 cm)",
                 fontsize=12, fontweight="bold", pad=15)
    ax.legend(loc="upper right"); ax.grid(axis="y", alpha=0.3)
    # Margen vertical extra para que los labels no choquen con el título
    ymin, ymax = ax.get_ylim()
    ax.set_ylim(ymin - 8, ymax + 12)
    # Labels colocados abajo de la barra correspondiente (fuera del área central)
    for i, (m, s, b) in enumerate(zip(means, stds, bias)):
        # Texto cerca del eje horizontal, sin chocar con el título
        ax.text(i + width/2, ymin - 5,
                f"{m:+.1f}±{s:.1f} cm\nbias: {b:+.1f}",
                ha="center", va="top", fontsize=9,
                bbox=dict(boxstyle="round,pad=0.3", facecolor="white",
                          edgecolor="gray", alpha=0.9))
    plt.tight_layout()
    plt.savefig(OUT / "1_1_bias_std.png", dpi=150, bbox_inches="tight")
    plt.close()
    print(f"  ✓ {OUT / '1_1_bias_std.png'}")

    # ----- GRÁFICA 3: ajuste de modelo de 2do orden a la respuesta promedio -----
    # Usamos right rep 2 o 3 (las más limpias) como ejemplo
    fig, ax = plt.subplots(figsize=(10, 6))
    cmd_demo = "right"
    tr_demo = trajectories[cmd_demo][1]   # rep 2
    ts = tr_demo["t"]
    pos_delta = (tr_demo["pos"] - tr_demo["p0"]) * 100  # cm
    target_cm = 30

    # Sistema de 2do orden subamortiguado: y(t) = K·(1 - exp(-ζωt)·(cos(ωd·t) + ζ/√(1−ζ²) sin(ωd·t)))
    def second_order(t, K, zeta, wn, t0):
        t_eff = np.maximum(t - t0, 0)
        wd = wn * np.sqrt(max(1e-6, 1 - zeta**2))
        env = np.exp(-zeta * wn * t_eff)
        if zeta < 1:
            response = K * (1 - env * (np.cos(wd * t_eff) +
                                        (zeta / np.sqrt(1 - zeta**2)) * np.sin(wd * t_eff)))
        else:
            response = K * (1 - env)
        return response

    try:
        popt, _ = curve_fit(second_order, ts, pos_delta,
                            p0=[30, 0.5, 2.0, 0.2],
                            bounds=([10, 0.05, 0.3, 0], [60, 2.0, 10.0, 1.5]),
                            maxfev=5000)
        t_fit = np.linspace(0, ts[-1], 500)
        y_fit = second_order(t_fit, *popt)
        # RMSE
        y_at_data = second_order(ts, *popt)
        rmse_cm = float(np.sqrt(np.mean((pos_delta - y_at_data) ** 2)))
        K_fit, zeta_fit, wn_fit, t0_fit = popt
        results["1.1"]["model_fit"] = {
            "cmd_used": cmd_demo, "rep_used": tr_demo["rep"],
            "K": float(K_fit), "zeta": float(zeta_fit), "wn": float(wn_fit),
            "t0_s": float(t0_fit), "rmse_cm": rmse_cm,
        }
        ax.plot(ts, pos_delta, "o-", color=COLOR_X, alpha=0.6,
                label=f"Datos ({cmd_demo} rep {tr_demo['rep']})", markersize=3)
        ax.plot(t_fit, y_fit, "-", color="black", linewidth=2,
                label=f"Modelo 2°: K={K_fit:.1f}cm, ζ={zeta_fit:.2f}, ωn={wn_fit:.2f} rad/s\n"
                      f"   τ_d={t0_fit*1000:.0f} ms,  RMSE={rmse_cm:.2f} cm")
        ax.axhline(target_cm, color="gray", linestyle="--", label="Target 30 cm")
        ax.set_xlabel("Tiempo [s]"); ax.set_ylabel("Δ posición [cm]")
        ax.set_title("Prueba 1.1 — Ajuste de modelo dinámico de 2° orden",
                     fontsize=12, fontweight="bold")
        ax.grid(alpha=0.3); ax.legend(loc="best", fontsize=10)
        plt.tight_layout()
        plt.savefig(OUT / "1_1_model_fit.png", dpi=150, bbox_inches="tight")
        plt.close()
        print(f"  ✓ {OUT / '1_1_model_fit.png'} (K={K_fit:.1f}, ζ={zeta_fit:.2f}, ωn={wn_fit:.2f})")
    except Exception as e:
        print(f"  [WARN] Ajuste de modelo falló: {e}")
        plt.close()


# ============================================================
# PRUEBA 1.2 — Latencia
# ============================================================
def analyze_test_1_2():
    print("\n=== ANALIZANDO 1.2 (latencia) ===")
    rows = load_csv(CSV_12)
    # Encontrar UN único valor de latency_ms por command_idx (el primero != None)
    seen_idx = set()
    latencies = []
    for r in rows:
        if r.get("latency_ms") and r["command_idx"] not in seen_idx:
            try:
                lat = float(r["latency_ms"])
                if 100 < lat < 5000:  # filtro de rango razonable
                    latencies.append({"idx": r["command_idx"],
                                      "cmd": r["sdk_cmd"], "lat_ms": lat})
                    seen_idx.add(r["command_idx"])
            except (ValueError, TypeError):
                continue
    print(f"  Comandos con latencia válida: {len(latencies)}")
    lat_arr = np.array([l["lat_ms"] for l in latencies])

    summary = {
        "n": len(latencies),
        "mean_ms": float(np.mean(lat_arr)),
        "std_ms": float(np.std(lat_arr, ddof=1)) if len(lat_arr) > 1 else 0,
        "median_ms": float(np.median(lat_arr)),
        "min_ms": float(np.min(lat_arr)),
        "max_ms": float(np.max(lat_arr)),
        "p25_ms": float(np.percentile(lat_arr, 25)),
        "p75_ms": float(np.percentile(lat_arr, 75)),
        "p90_ms": float(np.percentile(lat_arr, 90)),
        "p95_ms": float(np.percentile(lat_arr, 95)),
    }
    results["1.2"] = summary

    print(f"  Mean: {summary['mean_ms']:.0f} ms  Median: {summary['median_ms']:.0f} ms  "
          f"Range: [{summary['min_ms']:.0f}, {summary['max_ms']:.0f}] ms")

    # GRÁFICA: histograma + boxplot lado a lado
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5),
                                    gridspec_kw={"width_ratios": [2, 1]})
    ax1.hist(lat_arr, bins=8, color=ACCENT, edgecolor="black", alpha=0.85)
    ax1.axvline(summary["mean_ms"], color="black", linestyle="-",
                linewidth=2, label=f"Mean = {summary['mean_ms']:.0f} ms")
    ax1.axvline(summary["median_ms"], color=COLOR_Y, linestyle="--",
                linewidth=2, label=f"Median = {summary['median_ms']:.0f} ms")
    ax1.set_xlabel("Tiempo de respuesta [ms]")
    ax1.set_ylabel("Frecuencia")
    ax1.set_title("Distribución del tiempo de respuesta (comando → cruce de 10 cm)",
                  fontsize=11, fontweight="bold")
    ax1.grid(alpha=0.3); ax1.legend()

    bp = ax2.boxplot(lat_arr, vert=True, patch_artist=True,
                     boxprops=dict(facecolor=ACCENT, alpha=0.7),
                     medianprops=dict(color="black", linewidth=2))
    ax2.set_ylabel("Tiempo de respuesta [ms]")
    ax2.set_title(f"Boxplot (n={summary['n']})", fontsize=11, fontweight="bold")
    ax2.set_xticklabels(["latencia"])
    ax2.grid(axis="y", alpha=0.3)
    fig.suptitle("Prueba 1.2 — Tiempo de respuesta a comandos discretos `move_X`",
                 fontsize=12, fontweight="bold")
    plt.tight_layout()
    plt.savefig(OUT / "1_2_latency.png", dpi=150, bbox_inches="tight")
    plt.close()
    print(f"  ✓ {OUT / '1_2_latency.png'}")


# ============================================================
# PRUEBA 1.3 — Hover
# ============================================================
def analyze_test_1_3():
    print("\n=== ANALIZANDO 1.3 (hover) ===")
    rows = load_csv(CSV_13)
    samples = [r for r in rows if fnum(r.get("pos_x")) is not None]
    print(f"  Muestras con pose: {len(samples)}")

    elapsed = np.array([fnum(r["elapsed"]) for r in samples])
    xs = np.array([fnum(r["pos_x"]) for r in samples])
    ys = np.array([fnum(r["pos_y"]) for r in samples])
    zs = np.array([fnum(r["pos_z"]) for r in samples])
    height_cm = np.array([fnum(r.get("tello_height", r.get("tello_height_cm", 0)))
                          or 0 for r in samples])

    # Centrar
    xs_c = (xs - np.mean(xs)) * 100
    ys_c = (ys - np.mean(ys)) * 100
    zs_c = (zs - np.mean(zs)) * 100

    summary = {
        "n_samples": len(samples),
        "duration_s": float(elapsed[-1] - elapsed[0]),
        "std_x_cm": float(np.std(xs, ddof=1) * 100),
        "std_y_cm": float(np.std(ys, ddof=1) * 100),
        "std_z_cm": float(np.std(zs, ddof=1) * 100),
        "range_x_cm": float((np.max(xs) - np.min(xs)) * 100),
        "range_y_cm": float((np.max(ys) - np.min(ys)) * 100),
        "range_z_cm": float((np.max(zs) - np.min(zs)) * 100),
        "drift_xy_max_cm": float(np.max(np.sqrt((xs - xs[0])**2 + (ys - ys[0])**2)) * 100),
    }
    if len(height_cm) > 0:
        summary["tof_mean_cm"] = float(np.mean(height_cm))
        summary["tof_std_cm"] = float(np.std(height_cm, ddof=1))
    results["1.3"] = summary
    print(f"  std (x,y,z) = ({summary['std_x_cm']:.1f}, {summary['std_y_cm']:.1f}, "
          f"{summary['std_z_cm']:.1f}) cm")
    print(f"  drift max XY: {summary['drift_xy_max_cm']:.1f} cm en {summary['duration_s']:.0f} s")

    # GRÁFICA: 4 subplots (XY plano, X(t), Y(t), Z(t))
    fig = plt.figure(figsize=(14, 9))
    # XY top-down
    ax_xy = plt.subplot2grid((3, 2), (0, 0), rowspan=3)
    sc = ax_xy.scatter(xs_c, ys_c, c=elapsed, cmap="viridis", s=12, alpha=0.7)
    ax_xy.plot(xs_c[0], ys_c[0], "go", markersize=15, label="Inicio")
    ax_xy.plot(xs_c[-1], ys_c[-1], "rs", markersize=15, label="Final")
    cb = plt.colorbar(sc, ax=ax_xy, label="Tiempo [s]")
    ax_xy.set_aspect("equal")
    ax_xy.set_xlabel("Δ X [cm] (a lo largo de la pared)")
    ax_xy.set_ylabel("Δ Y [cm] (vertical)")
    ax_xy.set_title("Trayectoria 2D del hover (vista frontal)", fontweight="bold")
    ax_xy.grid(alpha=0.3); ax_xy.legend(loc="best")

    # X vs t
    ax_x = plt.subplot2grid((3, 2), (0, 1))
    ax_x.plot(elapsed, xs_c, color=COLOR_X, linewidth=1)
    ax_x.set_ylabel("Δ X [cm]"); ax_x.set_title(f"σ_x = {summary['std_x_cm']:.2f} cm",
                                                  fontweight="bold")
    ax_x.axhline(0, color="gray", linewidth=0.5); ax_x.grid(alpha=0.3)
    # Y vs t
    ax_y = plt.subplot2grid((3, 2), (1, 1))
    ax_y.plot(elapsed, ys_c, color=COLOR_Y, linewidth=1)
    ax_y.set_ylabel("Δ Y [cm]"); ax_y.set_title(f"σ_y = {summary['std_y_cm']:.2f} cm",
                                                  fontweight="bold")
    ax_y.axhline(0, color="gray", linewidth=0.5); ax_y.grid(alpha=0.3)
    # Z vs t
    ax_z = plt.subplot2grid((3, 2), (2, 1))
    ax_z.plot(elapsed, zs_c, color=COLOR_Z, linewidth=1)
    ax_z.set_ylabel("Δ Z [cm]"); ax_z.set_xlabel("Tiempo [s]")
    ax_z.set_title(f"σ_z = {summary['std_z_cm']:.2f} cm", fontweight="bold")
    ax_z.axhline(0, color="gray", linewidth=0.5); ax_z.grid(alpha=0.3)

    fig.suptitle(f"Prueba 1.3 — Hover natural del Tello, {summary['duration_s']:.0f} s "
                 f"({summary['n_samples']} muestras)", fontsize=13, fontweight="bold")
    plt.tight_layout()
    plt.savefig(OUT / "1_3_hover.png", dpi=150, bbox_inches="tight")
    plt.close()
    print(f"  ✓ {OUT / '1_3_hover.png'}")


# ============================================================
# PRUEBA 2.1 — Closed-loop
# ============================================================
def analyze_test_2_1():
    print("\n=== ANALIZANDO 2.1 (lazo cerrado) ===")
    rows = load_csv(CSV_21)
    samples = [r for r in rows if fnum(r.get("err_3d_cm")) is not None]
    print(f"  Muestras con error válido: {len(samples)}")

    elapsed = np.array([fnum(r["elapsed"]) for r in samples])
    err_3d = np.array([fnum(r["err_3d_cm"]) for r in samples])
    err_x = np.array([fnum(r["err_x_cm"]) for r in samples])
    err_y = np.array([fnum(r["err_y_cm"]) for r in samples])
    err_z = np.array([fnum(r["err_z_cm"]) for r in samples])

    # Estado estacionario: ignorar primer 25 %
    cut = len(samples) // 4
    summary = {
        "n_samples": len(samples),
        "duration_s": float(elapsed[-1] - elapsed[0]),
        "loop_hz": len(samples) / float(elapsed[-1] - elapsed[0]),
        "err_3d_mean_cm": float(np.mean(err_3d)),
        "err_3d_steady_mean_cm": float(np.mean(err_3d[cut:])),
        "err_3d_max_cm": float(np.max(err_3d)),
        "err_x_mean_cm": float(np.mean(err_x[cut:])),
        "err_x_std_cm": float(np.std(err_x[cut:], ddof=1)),
        "err_y_mean_cm": float(np.mean(err_y[cut:])),
        "err_y_std_cm": float(np.std(err_y[cut:], ddof=1)),
        "err_z_mean_cm": float(np.mean(err_z[cut:])),
        "err_z_std_cm": float(np.std(err_z[cut:], ddof=1)),
    }

    # Detectar perturbaciones (picos > 15 cm separados ≥ 1.5 s)
    peaks = []
    last_peak = -10
    for i in range(2, len(err_3d) - 2):
        e = err_3d[i]
        t = elapsed[i]
        if (e > 15 and e > err_3d[i-1] and e > err_3d[i+1]
                and (t - last_peak > 1.5)):
            # Tiempo de recuperación a 8 cm sostenido
            recov = None
            for j in range(i, len(err_3d)):
                if elapsed[j] - t > 8: break
                if err_3d[j] <= 8.0:
                    ok = True
                    for k in range(j, min(j+8, len(err_3d))):
                        if err_3d[k] > 8: ok = False; break
                    if ok:
                        recov = float(elapsed[j] - t); break
            peaks.append({"t": float(t), "peak_cm": float(e), "recov_s": recov})
            last_peak = t

    big_peaks = [p for p in peaks if p["peak_cm"] > 25]
    summary["n_perturbations"] = len(big_peaks)
    if big_peaks:
        recs = [p["recov_s"] for p in big_peaks if p["recov_s"]]
        summary["perturbation_peak_max_cm"] = float(max(p["peak_cm"] for p in big_peaks))
        summary["perturbation_recovery_mean_s"] = float(np.mean(recs)) if recs else None
    results["2.1"] = summary
    print(f"  Lazo: {summary['loop_hz']:.1f} Hz")
    print(f"  Estado estacionario: err 3D = {summary['err_3d_steady_mean_cm']:.2f} cm")
    print(f"  Por eje: x={summary['err_x_mean_cm']:+.2f}±{summary['err_x_std_cm']:.2f}, "
          f"y={summary['err_y_mean_cm']:+.2f}±{summary['err_y_std_cm']:.2f}, "
          f"z={summary['err_z_mean_cm']:+.2f}±{summary['err_z_std_cm']:.2f}")
    print(f"  Perturbaciones grandes (>25cm): {len(big_peaks)}")

    # GRÁFICA: error 3D vs tiempo + perturbaciones marcadas
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(13, 8), sharex=True,
                                    gridspec_kw={"height_ratios": [2, 1.2]})

    ax1.plot(elapsed, err_3d, color=ACCENT, linewidth=1.2, label="Error 3D")
    ax1.axhline(8, color="gray", linestyle=":", linewidth=1, label="Banda 8 cm (recuperación)")
    ax1.axhline(15, color="red", linestyle=":", linewidth=1, label="Umbral pert. 15 cm")
    # Marcar perturbaciones
    for k, p in enumerate(big_peaks):
        ax1.axvline(p["t"], color="red", alpha=0.3)
        recov_label = f"{p['recov_s']:.1f}s" if p["recov_s"] else "no recup"
        ax1.annotate(f"P{k+1}\n{p['peak_cm']:.0f}cm\n{recov_label}",
                     xy=(p["t"], p["peak_cm"]), xytext=(p["t"], p["peak_cm"] + 8),
                     ha="center", fontsize=8, color="red",
                     arrowprops=dict(arrowstyle="->", color="red", alpha=0.5))
    ax1.set_ylabel("Error 3D [cm]"); ax1.grid(alpha=0.3); ax1.legend(loc="upper right")
    ax1.set_title("Prueba 2.1 — Error de seguimiento del lazo cerrado",
                  fontsize=12, fontweight="bold")
    ax1.set_ylim(0, max(err_3d) * 1.15)

    ax2.plot(elapsed, err_x, color=COLOR_X, linewidth=1, label=f"err_x  ({summary['err_x_mean_cm']:+.1f}±{summary['err_x_std_cm']:.1f} cm)")
    ax2.plot(elapsed, err_y, color=COLOR_Y, linewidth=1, label=f"err_y  ({summary['err_y_mean_cm']:+.1f}±{summary['err_y_std_cm']:.1f} cm)")
    ax2.plot(elapsed, err_z, color=COLOR_Z, linewidth=1, label=f"err_z  ({summary['err_z_mean_cm']:+.1f}±{summary['err_z_std_cm']:.1f} cm)")
    ax2.axhline(0, color="gray", linewidth=0.5)
    ax2.set_ylabel("Error por eje [cm]"); ax2.set_xlabel("Tiempo [s]")
    ax2.grid(alpha=0.3); ax2.legend(loc="upper right", fontsize=9)
    plt.tight_layout()
    plt.savefig(OUT / "2_1_closed_loop.png", dpi=150, bbox_inches="tight")
    plt.close()
    print(f"  ✓ {OUT / '2_1_closed_loop.png'}")


def main():
    analyze_test_1_1()
    analyze_test_1_2()
    analyze_test_1_3()
    analyze_test_2_1()
    # Guardar JSON con todos los números
    with open(OUT / "results.json", "w") as f:
        json.dump(results, f, indent=2)
    print(f"\n✓ Resultados numéricos: {OUT / 'results.json'}")
    print(f"\n{'='*60}\nGráficas generadas en: {OUT}\n{'='*60}")


if __name__ == "__main__":
    main()
