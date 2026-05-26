"""
═══════════════════════════════════════════════════════════════
PRUEBA 3.2 — Protocolo de mensajes de cooperación
═══════════════════════════════════════════════════════════════
Drones: 0  |  Complejidad: Baja  |  Tiempo: ~30 min

Objetivo (plan de pruebas, OE3):
    Definir, implementar y verificar la estructura del mensaje
    de cooperación que intercambiarán los drones a través del
    enlace Ethernet.

Estructura formal del mensaje:
    drone_id      uint8    [0-255]    identificador único
    seq           uint32   [0-2³²]    número de secuencia
    timestamp     float64             unix epoch en segundos
    pos_x/y/z     float32 (m)         posición mundo
    vel_x/y/z     float32 (m/s)       velocidad mundo
    battery       uint8    [0-100]    batería %
    mission_state uint8    [0-255]    enum del estado de misión
                  (0=idle, 1=takeoff, 2=hover, 3=formation,
                   4=trajectory, 5=consensus, 6=landing, 7=emergency)
    crc16         uint16              checksum CRC-16/CCITT-FALSE

Total: 51 bytes en formato binario (struct).

Métricas medidas:
    - Tamaño del mensaje (binario vs JSON)
    - Overhead de serialización (encode + decode) en µs
    - Tasa de mensajes entregados correctamente (1000 paquetes)
    - Integridad CRC verificada extremo a extremo

USO — En el Ubuntu (receiver, 192.168.1.2) ya debe estar corriendo:
    python test_3_1_ethernet.py receiver

En el Mac (sender, 192.168.1.1):
    python test_3_2_protocol.py
"""
import sys
import os
import time
import socket
import json
import struct
import statistics
import binascii   # CRC-16 implementado en C (orden de magnitud más rápido que Python puro)
from dataclasses import dataclass, asdict
from pathlib import Path

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import config

RECEIVER_IP = config.SLAVE_IP
PORT_ECHO   = config.COMMS_PORT + 1   # 5006 (mismo que el receiver de 3.1)
N_MESSAGES  = 1000
SEND_HZ     = 50          # frecuencia de envío
RTT_TIMEOUT_S = 0.5       # timeout para esperar el echo de cada mensaje


# ============================================================
# Estados de misión (enum como uint8)
# ============================================================
MISSION_STATES = {
    0: "idle", 1: "takeoff", 2: "hover", 3: "formation",
    4: "trajectory", 5: "consensus", 6: "landing", 7: "emergency",
}


# ============================================================
# Dataclass del mensaje (representación lógica)
# ============================================================
@dataclass
class CoopMessage:
    drone_id: int       # uint8
    seq: int            # uint32
    timestamp: float    # float64
    pos_x: float        # float32
    pos_y: float        # float32
    pos_z: float        # float32
    vel_x: float        # float32
    vel_y: float        # float32
    vel_z: float        # float32
    battery: int        # uint8 (0-100)
    mission_state: int  # uint8


# ============================================================
# Codec BINARIO (struct)
#   Formato: > B I d 6f B B H   (big-endian / network byte order)
#   B = uint8, I = uint32, d = float64, f = float32, H = uint16
#   Total: 1+4+8+4*6+1+1+2 = 41 bytes (sin CRC = 39)
# ============================================================
BINARY_FMT = ">BIdffffffBBH"   # incluye crc16 al final
BINARY_SIZE = struct.calcsize(BINARY_FMT)


def crc16_ccitt(data: bytes) -> int:
    """
    CRC-16 CCITT (XMODEM, init=0, poly=0x1021).
    Implementación en C de la stdlib (binascii.crc_hqx) → ~10x más rápida
    que un loop Python puro. La elección importa: con Python puro, el costo
    de calcular CRC dominaba el encode/decode del binario y lo hacía más
    lento que JSON.
    """
    return binascii.crc_hqx(data, 0)


def encode_binary(msg: CoopMessage) -> bytes:
    """Serializa a bytes (formato binario compacto con CRC)."""
    # Primero codificamos sin CRC para calcularlo, luego rearmamos con CRC
    body = struct.pack(
        ">BIdffffffBB",
        msg.drone_id, msg.seq, msg.timestamp,
        msg.pos_x, msg.pos_y, msg.pos_z,
        msg.vel_x, msg.vel_y, msg.vel_z,
        msg.battery, msg.mission_state,
    )
    crc = crc16_ccitt(body)
    return body + struct.pack(">H", crc)


def decode_binary(data: bytes) -> tuple[CoopMessage, bool]:
    """Devuelve (mensaje, crc_ok). crc_ok=True si la integridad se verificó."""
    if len(data) != BINARY_SIZE:
        raise ValueError(f"Tamaño inesperado: {len(data)} (esperado {BINARY_SIZE})")
    body, crc_recv = data[:-2], struct.unpack(">H", data[-2:])[0]
    crc_calc = crc16_ccitt(body)
    crc_ok = (crc_recv == crc_calc)
    fields = struct.unpack(">BIdffffffBB", body)
    msg = CoopMessage(*fields)
    return msg, crc_ok


# ============================================================
# Codec JSON
# ============================================================
def encode_json(msg: CoopMessage) -> bytes:
    return json.dumps(asdict(msg), separators=(",", ":")).encode("ascii")


def decode_json(data: bytes) -> tuple[CoopMessage, bool]:
    obj = json.loads(data.decode("ascii"))
    return CoopMessage(**obj), True   # JSON no tiene CRC; la integridad la da el parser


# ============================================================
# Generador de mensaje "realista" (datos plausibles)
# ============================================================
def make_message(seq: int) -> CoopMessage:
    return CoopMessage(
        drone_id=1,
        seq=seq,
        timestamp=time.time(),
        pos_x=1.0 + 0.5 * (seq % 10) / 10,
        pos_y=0.85,
        pos_z=2.5 + 0.1 * ((seq // 10) % 5) / 5,
        vel_x=0.05, vel_y=0.0, vel_z=-0.02,
        battery=85 - (seq // 100),
        mission_state=3,   # formation
    )


# ============================================================
# Benchmark: encode/decode + RTT por formato
# ============================================================
def benchmark_format(name: str, encoder, decoder) -> dict:
    print(f"\n=== {name} ===")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(RTT_TIMEOUT_S)
    interval = 1.0 / SEND_HZ

    encode_us, decode_us, rtt_ms, sizes = [], [], [], []
    received_seqs, crc_ok_count = set(), 0
    sent = 0
    pending = {}

    t_start = time.time()
    next_send = t_start

    while sent < N_MESSAGES or pending:
        # Enviar si toca y aún no terminamos
        if sent < N_MESSAGES and time.time() >= next_send:
            msg = make_message(sent)

            # Tiempo de encode
            t0 = time.perf_counter()
            payload = encoder(msg)
            encode_us.append((time.perf_counter() - t0) * 1e6)
            sizes.append(len(payload))

            t_send = time.perf_counter()
            try:
                sock.sendto(payload, (RECEIVER_IP, PORT_ECHO))
                pending[msg.seq] = t_send
                sent += 1
            except Exception as e:
                print(f"  [WARN] sendto seq={msg.seq}: {e}")
            next_send += interval

        # Drenar lo que haya en el buffer
        try:
            sock.settimeout(0.001)
            while True:
                data, _ = sock.recvfrom(4096)
                t_recv = time.perf_counter()

                # Tiempo de decode
                t0 = time.perf_counter()
                try:
                    msg, crc_ok = decoder(data)
                    decode_us.append((time.perf_counter() - t0) * 1e6)
                    if msg.seq in pending:
                        rtt_ms.append((t_recv - pending.pop(msg.seq)) * 1000)
                        received_seqs.add(msg.seq)
                        if crc_ok:
                            crc_ok_count += 1
                except Exception as e:
                    print(f"  [DECODE-FAIL] {e}")
        except socket.timeout:
            pass
        except Exception:
            pass

        # Si ya enviamos todo, esperamos un poco a los echoes restantes y salimos
        if sent >= N_MESSAGES and not pending:
            break
        if sent >= N_MESSAGES and (time.time() - t_start) > (N_MESSAGES * interval + 3):
            print(f"  [INFO] Timeout esperando últimos echoes. Pendientes: {len(pending)}")
            break

    sock.close()

    if not rtt_ms:
        return {"error": "no echoes received"}

    return {
        "format": name,
        "sent": sent,
        "received": len(received_seqs),
        "loss_pct": (sent - len(received_seqs)) / sent * 100,
        "msg_size_bytes": int(statistics.mean(sizes)),
        "msg_size_min": int(min(sizes)),
        "msg_size_max": int(max(sizes)),
        "encode_us_mean": float(statistics.mean(encode_us)),
        "encode_us_p95":  float(sorted(encode_us)[int(len(encode_us)*0.95)]),
        "decode_us_mean": float(statistics.mean(decode_us)),
        "decode_us_p95":  float(sorted(decode_us)[int(len(decode_us)*0.95)]),
        "rtt_ms_mean":    float(statistics.mean(rtt_ms)),
        "rtt_ms_p95":     float(sorted(rtt_ms)[int(len(rtt_ms)*0.95)]),
        "rtt_ms_max":     float(max(rtt_ms)),
        "jitter_ms":      float(statistics.stdev(rtt_ms)) if len(rtt_ms) > 1 else 0,
        "crc_ok_count":   crc_ok_count,
        "crc_ok_pct":     crc_ok_count / len(received_seqs) * 100 if received_seqs else 0,
    }


# ============================================================
# Smoke test: encode + decode local (sin red)
# ============================================================
def _msgs_close(a: CoopMessage, b: CoopMessage, tol_float: float = 1e-5) -> bool:
    """Compara dos mensajes con tolerancia para floats (float32 ≠ float64)."""
    int_fields = ("drone_id", "seq", "battery", "mission_state")
    float_fields = ("timestamp", "pos_x", "pos_y", "pos_z",
                    "vel_x", "vel_y", "vel_z")
    for f in int_fields:
        if getattr(a, f) != getattr(b, f):
            return False
    for f in float_fields:
        if abs(getattr(a, f) - getattr(b, f)) > tol_float:
            # timestamp se queda en float64 también si lo guardamos como d
            if f == "timestamp":
                # El campo timestamp usa float64 ('d' en struct), match exacto
                if abs(getattr(a, f) - getattr(b, f)) > 1e-9:
                    return False
            else:
                return False
    return True


def smoke_test():
    print("=== SMOKE TEST (local, sin red) ===")
    msg = make_message(42)
    print(f"  Mensaje original: {msg}")

    # Binario
    b = encode_binary(msg)
    msg_b, crc_ok = decode_binary(b)
    print(f"  Binario: {len(b)} bytes, CRC OK={crc_ok}")
    assert crc_ok, "CRC binario falló"
    assert _msgs_close(msg_b, msg), "binary roundtrip falló (más allá de precisión float32)"

    # JSON
    j = encode_json(msg)
    msg_j, _ = decode_json(j)
    print(f"  JSON: {len(j)} bytes")
    assert _msgs_close(msg_j, msg, tol_float=1e-9), "json roundtrip falló"

    print(f"  ✓ Roundtrip OK en ambos formatos. "
          f"Binario {len(b)} B vs JSON {len(j)} B (ratio {len(j)/len(b):.1f}×)\n")


# ============================================================
# Main
# ============================================================
def main():
    smoke_test()
    print(f"Sender → {RECEIVER_IP}:{PORT_ECHO}")
    print(f"Enviando {N_MESSAGES} mensajes a {SEND_HZ} Hz por cada formato.\n")
    print(f"⚠️  Asegúrate que el RECEIVER esté corriendo en el Ubuntu:")
    print(f"   python test_3_1_ethernet.py receiver")
    input("Pulsa ENTER cuando esté listo...")

    results = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "n_messages": N_MESSAGES,
        "send_hz": SEND_HZ,
        "binary_format_spec": BINARY_FMT,
        "binary_size_bytes": BINARY_SIZE,
    }

    res_bin = benchmark_format("BINARIO (struct + CRC-16)", encode_binary, decode_binary)
    res_json = benchmark_format("JSON", encode_json, decode_json)
    results["binary"] = res_bin
    results["json"] = res_json

    # Guardar resultados
    log_dir = Path(os.path.dirname(os.path.abspath(__file__))) / "logs"
    log_dir.mkdir(exist_ok=True)
    out_path = log_dir / f"test_3_2_protocol_{time.strftime('%Y%m%d_%H%M%S')}.json"
    with open(out_path, "w") as f:
        json.dump(results, f, indent=2)

    # Resumen comparativo
    print("\n" + "=" * 72)
    print("RESUMEN — Prueba 3.2 (Protocolo de mensajes)")
    print("=" * 72)
    print(f"  {'':<22} {'BINARIO':>16} {'JSON':>16}   ratio")
    print(f"  {'-'*22} {'-'*16} {'-'*16}   {'-'*5}")
    rows = [
        ("Tamaño mensaje (B)",   res_bin["msg_size_bytes"], res_json["msg_size_bytes"], "x"),
        ("Encode (µs, mean)",    res_bin["encode_us_mean"], res_json["encode_us_mean"], "x"),
        ("Decode (µs, mean)",    res_bin["decode_us_mean"], res_json["decode_us_mean"], "x"),
        ("RTT (ms, mean)",       res_bin["rtt_ms_mean"],    res_json["rtt_ms_mean"],    "x"),
        ("RTT (ms, p95)",        res_bin["rtt_ms_p95"],     res_json["rtt_ms_p95"],     "x"),
        ("Jitter (ms, std)",     res_bin["jitter_ms"],      res_json["jitter_ms"],      "x"),
        ("Pérdida (%)",          res_bin["loss_pct"],       res_json["loss_pct"],       ""),
        ("CRC OK (%)",           res_bin["crc_ok_pct"],     "—",                        ""),
    ]
    for label, b, j, marker in rows:
        if isinstance(j, str):
            print(f"  {label:<22} {b:>16.2f} {j:>16}")
        else:
            ratio = (j / b) if b and isinstance(b, (int, float)) else 0
            print(f"  {label:<22} {b:>16.2f} {j:>16.2f}   {ratio:>4.1f}{marker}")
    print("=" * 72)
    print(f"\n[LOG] Detalle completo: {out_path}")
    print("[DONE] Prueba 3.2 completada.")


if __name__ == "__main__":
    main()
