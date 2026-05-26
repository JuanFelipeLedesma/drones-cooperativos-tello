#!/bin/bash
# ═══════════════════════════════════════════════════════════════
# PRUEBA 3.3 — Degradación controlada de red
# ═══════════════════════════════════════════════════════════════
# EJECUTAR EN: Ubuntu (SLAVE) — requiere sudo
#
# Este script controla tc (traffic control) para inyectar retardo
# y pérdida de paquetes en la interfaz Ethernet mientras los
# drones vuelan en formación (Prueba 2.3).
#
# USO:
#   1. En Mac:    python test_2_3_master.py
#   2. En Ubuntu: python test_2_3_slave.py  (en una terminal)
#   3. En Ubuntu: sudo bash test_3_3_degradation.sh ethX  (en otra terminal)
#      Reemplazar ethX por el nombre de tu interfaz Ethernet
#      (encontrar con: ip link show | grep -v lo)
#
# ═══════════════════════════════════════════════════════════════

IFACE=${1:-"eth0"}

echo "═══════════════════════════════════════════════════"
echo " PRUEBA 3.3 — Degradación de Red"
echo " Interfaz: $IFACE"
echo "═══════════════════════════════════════════════════"
echo ""

# Función para limpiar tc al salir
cleanup() {
    echo ""
    echo "[CLEAN] Eliminando reglas de tc..."
    sudo tc qdisc del dev $IFACE root 2>/dev/null
    echo "[CLEAN] Red restaurada a estado normal."
}
trap cleanup EXIT

# Función para aplicar condición y esperar
apply_condition() {
    local desc="$1"
    local tc_cmd="$2"
    local duration=${3:-30}

    echo ""
    echo "──────────────────────────────────────────────────"
    echo " $desc"
    echo " Duración: ${duration}s"
    echo "──────────────────────────────────────────────────"

    # Limpiar regla anterior
    sudo tc qdisc del dev $IFACE root 2>/dev/null

    # Aplicar nueva regla
    eval "sudo $tc_cmd"
    echo " [ACTIVO] Condición aplicada."

    # Cuenta regresiva
    for ((i=$duration; i>0; i--)); do
        printf "\r   Tiempo restante: %3ds " $i
        sleep 1
    done
    echo ""
}

# Verificar que la interfaz existe
if ! ip link show $IFACE > /dev/null 2>&1; then
    echo "[ERROR] Interfaz '$IFACE' no encontrada."
    echo "Interfaces disponibles:"
    ip link show | grep -E "^[0-9]" | awk '{print "  " $2}'
    exit 1
fi

echo "Presiona ENTER para iniciar la secuencia de degradación..."
echo "(Los drones deben estar volando en formación)"
read

# ── BASELINE: Sin degradación (30s) ──
echo "── BASELINE: Red normal (30s) ──"
sleep 30

# ── RETARDO: 50ms ──
apply_condition "RETARDO: 50ms" \
    "tc qdisc add dev $IFACE root netem delay 50ms" 30

# ── RETARDO: 100ms ──
apply_condition "RETARDO: 100ms" \
    "tc qdisc add dev $IFACE root netem delay 100ms" 30

# ── RETARDO: 200ms ──
apply_condition "RETARDO: 200ms" \
    "tc qdisc add dev $IFACE root netem delay 200ms" 30

# ── Recuperación: quitar todo ──
echo ""
echo "── RECUPERACIÓN: Red normal (15s) ──"
sudo tc qdisc del dev $IFACE root 2>/dev/null
sleep 15

# ── PÉRDIDA: 5% ──
apply_condition "PÉRDIDA: 5% de paquetes" \
    "tc qdisc add dev $IFACE root netem loss 5%" 30

# ── PÉRDIDA: 10% ──
apply_condition "PÉRDIDA: 10% de paquetes" \
    "tc qdisc add dev $IFACE root netem loss 10%" 30

# ── PÉRDIDA: 20% ──
apply_condition "PÉRDIDA: 20% de paquetes" \
    "tc qdisc add dev $IFACE root netem loss 20%" 30

# ── Recuperación ──
echo ""
echo "── RECUPERACIÓN: Red normal (15s) ──"
sudo tc qdisc del dev $IFACE root 2>/dev/null
sleep 15

# ── COMBINACIÓN: retardo 100ms + pérdida 10% ──
apply_condition "COMBINACIÓN: 100ms retardo + 10% pérdida" \
    "tc qdisc add dev $IFACE root netem delay 100ms loss 10%" 30

echo ""
echo "═══════════════════════════════════════════════════"
echo " PRUEBA 3.3 COMPLETADA"
echo " La red se ha restaurado automáticamente."
echo "═══════════════════════════════════════════════════"
