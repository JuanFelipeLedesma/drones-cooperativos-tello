"""
═══════════════════════════════════════════════════════════════
PRUEBA 3.1 — Benchmarking del enlace Ethernet
═══════════════════════════════════════════════════════════════
Drones: 0  |  Complejidad: Baja  |  Tiempo: ~20 min

Objetivo (plan de pruebas, OE3):
    Medir el rendimiento base del canal Ethernet entre los dos
    computadores que conforman la red Ad-Hoc del sistema cooperativo.

Métricas medidas:
    - Latencia round-trip (ping del sistema, 1000 paquetes)
    - Throughput máximo (iperf3, 30 s)
    - Latencia + jitter + packet loss del protocolo UDP de cooperación
      a frecuencias 10 Hz, 25 Hz y 50 Hz

USO — Hay dos modos. Se corre PRIMERO el RECEIVER y LUEGO el SENDER:

    En el Ubuntu (RECEIVER, 192.168.1.2):
        python test_3_1_ethernet.py receiver

    En el Mac (SENDER, 192.168.1.1):
        python test_3_1_ethernet.py sender

El SENDER controla todo (frecuencias, duración, etc.) y guarda el log.
El RECEIVER solo escucha y devuelve cada paquete (echo) para medir RTT.
"""
import sys
import os
import time
import socket
import json
import statistics
import subprocess
import argparse
from pathlib import Path

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import config

# ----------------------------------------------------------------
# Parámetros del experimento
# ----------------------------------------------------------------
RECEIVER_IP   = config.SLAVE_IP    # Ubuntu
SENDER_IP     = config.MASTER_IP   # Mac
PORT          = config.COMMS_PORT  # 5005
PORT_ECHO     = config.COMMS_PORT + 1   # 5006 (canal de echo)

# Frecuencias a evaluar para el protocolo de cooperación
TEST_FREQS_HZ = [10, 25, 50]
TEST_DURATION_S = 30          # segundos por frecuencia
PING_COUNT = 1000             # paquetes de ping

# Tamaño de payload (mensaje de cooperación realista, ver 3.2)
PAYLOAD_SIZE_BYTES = 128


# ============================================================
# RECEIVER MODE (Ubuntu)
# ============================================================
def run_receiver():
    """Hace echo de cada paquete recibido en PORT_ECHO."""
    print("=" * 60)
    print(f"RECEIVER  —  escuchando en {RECEIVER_IP}:{PORT_ECHO}")
    print("=" * 60)
    print("Espera que el SENDER en el Mac arranque la prueba.")
    print("Ctrl+C para terminar.\n")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", PORT_ECHO))
    sock.settimeout(1.0)

    n_received = 0
    t_last_print = time.time()
    try:
        while True:
            try:
                data, addr = sock.recvfrom(4096)
                # Echo: devolver el paquete al SENDER en el mismo puerto
                sock.sendto(data, addr)
                n_received += 1
                if time.time() - t_last_print > 5.0:
                    print(f"  [RX] {n_received} paquetes recibidos (echo OK)")
                    t_last_print = time.time()
            except socket.timeout:
                continue
    except KeyboardInterrupt:
        print(f"\n[INFO] Receiver terminado. Total paquetes echoed: {n_received}")
    finally:
        sock.close()


# ============================================================
# SENDER MODE (Mac)
# ============================================================
def run_sender():
    """Corre las 3 partes del benchmark y guarda resultados."""
    print("=" * 60)
    print(f"SENDER  —  Mac ({SENDER_IP}) → Ubuntu ({RECEIVER_IP})")
    print("=" * 60)

    # Verificar conectividad básica primero
    print(f"\n[STEP 0/3] Verificando conectividad con {RECEIVER_IP}...")
    try:
        result = subprocess.run(["ping", "-c", "3", "-t", "2", RECEIVER_IP],
                                capture_output=True, text=True, timeout=10)
        if result.returncode != 0:
            print("[ERROR] No hay conectividad con el receiver. Aborto.")
            print(result.stdout)
            return
        print("  ✓ Conectividad OK")
    except Exception as e:
        print(f"[ERROR] {e}")
        return

    results = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "sender_ip": SENDER_IP,
        "receiver_ip": RECEIVER_IP,
    }

    # ---------- Parte 1: ping del sistema ----------
    print(f"\n[STEP 1/3] Ping del sistema, {PING_COUNT} paquetes...")
    print(f"  (esto toma ~{PING_COUNT//1000 + 1} segundos)")
    try:
        # En Mac/Linux: ping -c COUNT -i 0.001 (intervalo 1 ms)
        # macOS no permite intervalo <0.1s sin sudo; usamos 0.01
        result = subprocess.run(
            ["ping", "-c", str(PING_COUNT), "-i", "0.01", RECEIVER_IP],
            capture_output=True, text=True, timeout=PING_COUNT * 0.02 + 30,
        )
        out = result.stdout
        # Parsear líneas finales: "round-trip min/avg/max/stddev = ..."
        ping_stats = {}
        for line in out.splitlines():
            if "min/avg/max" in line or "min/avg/max/mdev" in line:
                parts = line.split("=")[1].strip().split(" ")[0]
                vals = [float(v) for v in parts.split("/")]
                ping_stats = {
                    "rtt_min_ms": vals[0],
                    "rtt_avg_ms": vals[1],
                    "rtt_max_ms": vals[2],
                    "rtt_stddev_ms": vals[3],
                }
            elif "packet loss" in line:
                # "5 packets transmitted, 5 packets received, 0.0% packet loss"
                pct = line.split(",")[2].strip().split("%")[0]
                ping_stats["loss_pct"] = float(pct)
        results["ping"] = ping_stats
        print(f"  ✓ RTT avg = {ping_stats.get('rtt_avg_ms', '?'):.3f} ms  "
              f"(min={ping_stats.get('rtt_min_ms', '?')}, max={ping_stats.get('rtt_max_ms', '?')})")
        print(f"  ✓ packet loss = {ping_stats.get('loss_pct', '?')}%")
    except Exception as e:
        print(f"  [WARN] ping falló: {e}")
        results["ping"] = {"error": str(e)}

    # ---------- Parte 2: throughput con iperf3 ----------
    print(f"\n[STEP 2/3] Throughput con iperf3 (30 s)...")
    print(f"  Necesitas correr en el Ubuntu PRIMERO:  iperf3 -s")
    print(f"  Y dejarlo corriendo. Pulsa ENTER cuando esté listo, o 's' para SKIP.")
    inp = input("  > ").strip().lower()
    if inp == "s":
        print("  [SKIP] iperf3 saltado.")
        results["iperf3"] = {"skipped": True}
    else:
        try:
            result = subprocess.run(
                ["iperf3", "-c", RECEIVER_IP, "-t", "30", "-J"],
                capture_output=True, text=True, timeout=45,
            )
            if result.returncode == 0:
                data = json.loads(result.stdout)
                summary = data["end"]["sum_received"]
                throughput_mbps = summary["bits_per_second"] / 1e6
                results["iperf3"] = {
                    "throughput_mbps": throughput_mbps,
                    "bytes_total": summary["bytes"],
                    "duration_s": summary["seconds"],
                }
                print(f"  ✓ Throughput = {throughput_mbps:.1f} Mbps")
            else:
                print(f"  [WARN] iperf3 falló: {result.stderr[:200]}")
                results["iperf3"] = {"error": result.stderr[:500]}
        except FileNotFoundError:
            print("  [WARN] iperf3 no instalado. Saltando.")
            print("         Para instalar: brew install iperf3 (Mac), sudo apt install iperf3 (Ubuntu)")
            results["iperf3"] = {"error": "not installed"}
        except Exception as e:
            print(f"  [WARN] iperf3 error: {e}")
            results["iperf3"] = {"error": str(e)}

    # ---------- Parte 3: latencia del protocolo de cooperación ----------
    print(f"\n[STEP 3/3] Latencia del protocolo UDP a {TEST_FREQS_HZ} Hz, "
          f"{TEST_DURATION_S} s cada uno.")
    print(f"  (asegúrate que el RECEIVER en Ubuntu esté corriendo)")
    print(f"  Pulsa ENTER para arrancar.")
    input("  > ")

    coop_results = {}
    for freq_hz in TEST_FREQS_HZ:
        print(f"\n  → Frecuencia {freq_hz} Hz...")
        coop_results[f"{freq_hz}Hz"] = measure_coop_latency(freq_hz, TEST_DURATION_S)
        print(f"    avg latency = {coop_results[f'{freq_hz}Hz']['rtt_avg_ms']:.3f} ms")
        print(f"    jitter (std)= {coop_results[f'{freq_hz}Hz']['jitter_ms']:.3f} ms")
        print(f"    pkt loss    = {coop_results[f'{freq_hz}Hz']['loss_pct']:.2f} %")

    results["coop_protocol"] = coop_results

    # ---------- Guardar resultados ----------
    log_dir = Path(os.path.dirname(os.path.abspath(__file__))) / "logs"
    log_dir.mkdir(exist_ok=True)
    out_path = log_dir / f"test_3_1_ethernet_{time.strftime('%Y%m%d_%H%M%S')}.json"
    with open(out_path, "w") as f:
        json.dump(results, f, indent=2)
    print(f"\n[LOG] Resultados guardados en: {out_path}")
    print_summary(results)


def measure_coop_latency(freq_hz: int, duration_s: float) -> dict:
    """Manda paquetes a freq_hz al receiver y mide RTT de cada uno."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0.5)   # timeout corto para no bloquear si se pierden

    interval_s = 1.0 / freq_hz
    payload_padding = b"x" * (PAYLOAD_SIZE_BYTES - 24)  # 24 bytes para "{seq:08d}{t_send:.6f}"

    sent = 0
    received = 0
    rtts_ms = []

    t_start = time.time()
    next_send_t = t_start
    pending = {}   # seq → t_send

    while time.time() - t_start < duration_s:
        # Enviar si ya toca
        if time.time() >= next_send_t:
            t_send = time.time()
            seq = sent
            header = f"{seq:08d}{t_send:.6f}".encode("ascii")
            payload = header + payload_padding
            try:
                sock.sendto(payload, (RECEIVER_IP, PORT_ECHO))
                pending[seq] = t_send
                sent += 1
            except Exception:
                pass
            next_send_t += interval_s

        # Intentar recibir lo que haya disponible (no bloquea)
        try:
            sock.settimeout(0.001)
            while True:
                data, _ = sock.recvfrom(4096)
                if len(data) >= 24:
                    seq_back = int(data[:8])
                    if seq_back in pending:
                        rtt_ms = (time.time() - pending.pop(seq_back)) * 1000
                        rtts_ms.append(rtt_ms)
                        received += 1
        except socket.timeout:
            pass
        except Exception:
            pass

    # Drenar paquetes que aún están en el buffer
    sock.settimeout(0.2)
    drain_until = time.time() + 0.3
    while time.time() < drain_until:
        try:
            data, _ = sock.recvfrom(4096)
            if len(data) >= 24:
                seq_back = int(data[:8])
                if seq_back in pending:
                    rtt_ms = (time.time() - pending.pop(seq_back)) * 1000
                    rtts_ms.append(rtt_ms)
                    received += 1
        except socket.timeout:
            break
        except Exception:
            break

    sock.close()

    if not rtts_ms:
        return {"sent": sent, "received": 0, "rtt_avg_ms": 0,
                "jitter_ms": 0, "loss_pct": 100.0,
                "rtt_min_ms": 0, "rtt_max_ms": 0}

    return {
        "sent": sent,
        "received": received,
        "loss_pct": (sent - received) / sent * 100 if sent else 0,
        "rtt_avg_ms": float(statistics.mean(rtts_ms)),
        "rtt_min_ms": float(min(rtts_ms)),
        "rtt_max_ms": float(max(rtts_ms)),
        "jitter_ms": float(statistics.stdev(rtts_ms)) if len(rtts_ms) > 1 else 0,
        "rtt_p95_ms": float(sorted(rtts_ms)[int(len(rtts_ms) * 0.95)]),
    }


def print_summary(results):
    print("\n" + "=" * 60)
    print("RESUMEN — Prueba 3.1 (Benchmark Ethernet)")
    print("=" * 60)
    p = results.get("ping", {})
    if "rtt_avg_ms" in p:
        print(f"  PING       avg = {p['rtt_avg_ms']:.3f} ms     "
              f"loss = {p.get('loss_pct', 0):.1f} %")
    ip3 = results.get("iperf3", {})
    if "throughput_mbps" in ip3:
        print(f"  iperf3     throughput = {ip3['throughput_mbps']:.1f} Mbps")
    print(f"\n  Protocolo de cooperación (UDP echo):")
    print(f"  {'freq':>6} {'avg(ms)':>10} {'p95(ms)':>10} {'jitter(ms)':>12} {'loss(%)':>10}")
    for k, v in results.get("coop_protocol", {}).items():
        print(f"  {k:>6} {v['rtt_avg_ms']:>10.3f} {v.get('rtt_p95_ms', 0):>10.3f} "
              f"{v['jitter_ms']:>12.3f} {v['loss_pct']:>10.2f}")
    print("=" * 60)


# ============================================================
# Main
# ============================================================
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", choices=["sender", "receiver"],
                        help="sender (Mac) o receiver (Ubuntu)")
    args = parser.parse_args()
    if args.mode == "receiver":
        run_receiver()
    else:
        run_sender()
