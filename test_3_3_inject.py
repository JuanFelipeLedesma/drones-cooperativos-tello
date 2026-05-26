"""
═══════════════════════════════════════════════════════════════
PRUEBA 3.3 — Inyector de degradación de red (Ubuntu)
═══════════════════════════════════════════════════════════════

Corre EN PARALELO al test_3_3_slave.py, en el mismo Ubuntu.
Sigue un cronograma fijo y aplica reglas de tc netem en la
interfaz Ethernet en los momentos correctos. Escribe la
condición actual al archivo flag /tmp/net_condition.txt
para que el slave la lea y la anote en el CSV de cada fila.

USO (en el Ubuntu, en una segunda terminal):
    python test_3_3_inject.py [interface]

Si no especificas interface, usa enp1s0 por defecto.

REQUISITOS:
    - sudo (te va a pedir password una vez al inicio)
    - tc instalado (`which tc` debe imprimir /usr/sbin/tc)
    - IFB redirect configurado previamente (una sola vez por sesión):
        sudo modprobe ifb
        sudo ip link add ifb0 type ifb || true
        sudo ip link set ifb0 up
        sudo tc qdisc add dev enp1s0 ingress
        sudo tc filter add dev enp1s0 parent ffff: protocol ip u32 \
            match u32 0 0 flowid 1:1 action mirred egress redirect dev ifb0

    El script aplica las reglas netem al dispositivo IFB (no a enp1s0
    directamente), así afectan el INGRESS de enp1s0 — que es el flujo
    crítico MASTER → SLAVE.

CRONOGRAMA (relativo al ARRANQUE de este script):
    El usuario debe arrancar este script JUSTO CUANDO
    el slave imprime "Lazo de formación activo".
    A partir de ese momento:

    t=0     baseline (sin degradación)             — 25 s
    t=25    delay 50ms                              — 25 s
    t=50    delay 100ms                             — 25 s
    t=75    delay 200ms                             — 25 s
    t=100   restaurar (transitorio)                 — 5 s
    t=105   loss 5%                                 — 25 s
    t=130   loss 10%                                — 25 s
    t=155   loss 20%                                — 25 s
    t=180   restaurar (transitorio)                 — 5 s
    t=185   combo: delay 100ms + loss 10%           — 25 s
    t=210   restaurar final                          —

ABORTAR: Ctrl+C. El finally limpia las reglas de tc para que
el cable quede normal (importante!).
"""
import sys
import time
import subprocess
import argparse
from pathlib import Path

# ----------------------------------------------------------------
# Cronograma de degradación (segundos relativos al inicio)
# Cada tupla: (t_absoluto_s, etiqueta_corta, comando_tc o None)
# ----------------------------------------------------------------
SCHEDULE = [
    (  0, "baseline",         None),
    ( 25, "delay_50ms",       "delay 50ms"),
    ( 50, "delay_100ms",      "delay 100ms"),
    ( 75, "delay_200ms",      "delay 200ms"),
    (100, "transitorio",      None),
    (105, "loss_5pct",        "loss 5%"),
    (130, "loss_10pct",       "loss 10%"),
    (155, "loss_20pct",       "loss 20%"),
    (180, "transitorio",      None),
    (185, "combo_100ms_10pct", "delay 100ms loss 10%"),
    (210, "restaurado",       None),
]

NET_CONDITION_FILE = "/tmp/net_condition.txt"

# Las reglas netem se aplican al IFB device, no al enp1s0 directamente,
# para afectar el INGRESS de enp1s0 (que es el tráfico MASTER→SLAVE).
IFB_DEVICE = "ifb0"


def write_condition(label: str):
    """Atomic write para que el slave nunca lea un archivo a medias."""
    tmp = Path(NET_CONDITION_FILE + ".tmp")
    tmp.write_text(label)
    tmp.replace(NET_CONDITION_FILE)


def run_sudo(cmd: list, check: bool = False):
    """Wrapper para sudo. Devuelve True si exit code == 0."""
    res = subprocess.run(["sudo"] + cmd, capture_output=True, text=True)
    if res.returncode != 0 and check:
        raise RuntimeError(f"Comando falló: {' '.join(cmd)}\n  stderr: {res.stderr}")
    return res.returncode == 0


def clear_tc_ifb():
    """Borra netem del dispositivo IFB. NO toca el redirect ingress de enp1s0."""
    run_sudo(["tc", "qdisc", "del", "dev", IFB_DEVICE, "root"], check=False)


def apply_netem_ifb(rule: str):
    """Aplica netem al IFB (afecta ingress de enp1s0)."""
    args = ["tc", "qdisc", "change", "dev", IFB_DEVICE, "root", "netem"] + rule.split()
    if not run_sudo(args, check=False):
        args = ["tc", "qdisc", "add", "dev", IFB_DEVICE, "root", "netem"] + rule.split()
        run_sudo(args, check=True)


def verify_ifb_setup():
    """Verifica que el redirect IFB esté configurado."""
    # ¿Existe el ifb0?
    res = subprocess.run(["ip", "link", "show", IFB_DEVICE],
                         capture_output=True, text=True)
    if res.returncode != 0:
        return False, f"Dispositivo {IFB_DEVICE} no existe"
    if "UP" not in res.stdout:
        return False, f"{IFB_DEVICE} existe pero no está UP"
    return True, "OK"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("interface", nargs="?", default="enp1s0",
                        help="Interfaz Ethernet (default: enp1s0)")
    args = parser.parse_args()
    iface = args.interface

    # Verificar tc
    if subprocess.run(["which", "tc"], capture_output=True).returncode != 0:
        print("[ERROR] 'tc' no está instalado. sudo apt install iproute2")
        return 1

    # Pedir sudo upfront para que después no nos interrumpan los prompts
    print(f"[3.3] Inyector de degradación de red sobre {iface} (vía IFB)")
    print(f"[3.3] Pidiendo sudo (cache la sesión por 15 min)...")
    if subprocess.run(["sudo", "-v"]).returncode != 0:
        print("[ERROR] sudo falló."); return 1

    # Verificar que el redirect IFB está configurado
    ok, msg = verify_ifb_setup()
    if not ok:
        print(f"[ERROR] IFB no configurado: {msg}")
        print(f"        Ejecuta primero (UNA sola vez por sesión):")
        print(f"          sudo modprobe ifb")
        print(f"          sudo ip link add ifb0 type ifb")
        print(f"          sudo ip link set ifb0 up")
        print(f"          sudo tc qdisc add dev {iface} ingress")
        print(f"          sudo tc filter add dev {iface} parent ffff: protocol ip u32 \\")
        print(f"              match u32 0 0 flowid 1:1 action mirred egress redirect dev ifb0")
        return 1
    print(f"[3.3] ✓ IFB redirect verificado")

    # Limpiar netem residual en ifb0 si lo hubiera
    clear_tc_ifb()
    write_condition("baseline")

    print(f"\n[3.3] CRONOGRAMA (~{SCHEDULE[-1][0]} s totales):")
    for t, label, rule in SCHEDULE:
        print(f"  t={t:>4}s  {label:<20}  {rule or '(sin degradación)'}")
    print()
    print("[3.3] ⚠️  ARRANCA AHORA cuando el slave en la otra terminal imprima")
    print("    'Lazo de formación activo'. Pulsa ENTER para empezar el cronograma.")
    input("    > ")

    t_start = time.time()
    try:
        for i, (t_abs, label, rule) in enumerate(SCHEDULE):
            # Esperar al instante absoluto
            sleep_s = t_abs - (time.time() - t_start)
            if sleep_s > 0:
                time.sleep(sleep_s)

            elapsed = time.time() - t_start
            if rule is None:
                clear_tc_ifb()
                write_condition(label)
                print(f"[3.3] t={elapsed:6.1f}s  → {label}  (ifb cleared)")
            else:
                apply_netem_ifb(rule)
                write_condition(label)
                print(f"[3.3] t={elapsed:6.1f}s  → {label}  (ifb netem {rule})")

        # Mantener restaurado hasta que el usuario corte
        print(f"\n[3.3] Cronograma completo. Manteniendo red sin degradación.")
        print(f"[3.3] Cuando el slave aterrice, pulsa Ctrl+C aquí para terminar.")
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n[3.3] Interrumpido.")
    finally:
        # Importantísimo: dejar el cable LIMPIO (sin reglas tc en ifb0)
        # El redirect ingress de enp1s0 → ifb0 se mantiene (no lo borramos
        # porque es setup manual del usuario y querrá reusarlo).
        clear_tc_ifb()
        write_condition("clean")
        print(f"[3.3] netem en ifb0 limpiado. Cable Ethernet en estado normal.")
        print(f"[3.3] El redirect ingress de {iface} → {IFB_DEVICE} se mantiene")
        print(f"      (para borrarlo manualmente: sudo tc qdisc del dev {iface} ingress)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
