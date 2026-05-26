"""
Módulo de comunicación UDP entre computadores (backbone Ethernet).
Maneja envío y recepción de mensajes de estado para cooperación.
"""

import socket
import json
import time
import threading
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config

class DroneMessage:
    """Estructura de un mensaje de cooperación."""
    def __init__(self, drone_id="", pos_x=0, pos_y=0, pos_z=0,
                 vel_x=0, vel_y=0, vel_z=0, battery=100,
                 mission_state="idle", seq=0):
        self.drone_id = drone_id
        self.timestamp = time.time()
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.pos_z = pos_z
        self.vel_x = vel_x
        self.vel_y = vel_y
        self.vel_z = vel_z
        self.battery = battery
        self.mission_state = mission_state
        self.seq = seq

    def to_json(self):
        return json.dumps(self.__dict__)

    @classmethod
    def from_json(cls, json_str):
        data = json.loads(json_str)
        msg = cls()
        for k, v in data.items():
            setattr(msg, k, v)
        return msg

    def to_dict(self):
        return self.__dict__.copy()


class CommsSender:
    """Envía mensajes de estado a un destino (IP, puerto) vía UDP."""
    def __init__(self, dest_ip, dest_port=config.COMMS_PORT):
        self.dest = (dest_ip, dest_port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.seq = 0

    def send(self, msg: DroneMessage):
        msg.seq = self.seq
        msg.timestamp = time.time()
        data = msg.to_json().encode('utf-8')
        self.sock.sendto(data, self.dest)
        self.seq += 1

    def close(self):
        self.sock.close()


class CommsReceiver:
    """Recibe mensajes de estado en un puerto UDP (hilo separado)."""
    def __init__(self, listen_port=config.COMMS_PORT):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", listen_port))
        self.sock.settimeout(0.1)

        self.last_message = None
        self.last_recv_time = 0
        self.messages_received = 0
        self.messages_lost = 0
        self._last_seq = -1

        self._running = False
        self._thread = None

    def start(self):
        """Inicia el hilo receptor."""
        self._running = True
        self._thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._thread.start()

    def _recv_loop(self):
        while self._running:
            try:
                data, addr = self.sock.recvfrom(4096)
                msg = DroneMessage.from_json(data.decode('utf-8'))
                recv_time = time.time()

                # Detectar mensajes perdidos
                if self._last_seq >= 0 and msg.seq > self._last_seq + 1:
                    self.messages_lost += (msg.seq - self._last_seq - 1)
                self._last_seq = msg.seq

                self.last_message = msg
                self.last_recv_time = recv_time
                self.messages_received += 1

            except socket.timeout:
                continue
            except Exception as e:
                print(f"[COMMS] Error recibiendo: {e}")

    def get_latest(self):
        """Retorna el último mensaje recibido y la latencia."""
        if self.last_message is None:
            return None, float('inf')
        latency = self.last_recv_time - self.last_message.timestamp
        return self.last_message, latency

    def is_connected(self):
        """True si se recibió un mensaje en los últimos COMMS_TIMEOUT_S segundos."""
        if self.last_recv_time == 0:
            return False
        return (time.time() - self.last_recv_time) < config.COMMS_TIMEOUT_S

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)
        self.sock.close()

    def get_stats(self):
        return {
            "received": self.messages_received,
            "lost": self.messages_lost,
            "loss_rate": self.messages_lost / max(1, self.messages_received + self.messages_lost),
        }
