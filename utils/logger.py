"""
Módulo de logging a CSV para registrar datos de vuelo.
Cada prueba crea su propio archivo de log con timestamp en el nombre.
"""

import csv
import time
import os

class FlightLogger:
    def __init__(self, test_name, log_dir="logs"):
        os.makedirs(log_dir, exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.filename = os.path.join(log_dir, f"{test_name}_{timestamp}.csv")
        self.file = None
        self.writer = None
        self.headers_written = False

    def log(self, data_dict):
        """Escribe una fila de datos. La primera llamada define los headers."""
        if not self.headers_written:
            self.file = open(self.filename, 'w', newline='')
            self.writer = csv.DictWriter(self.file, fieldnames=data_dict.keys())
            self.writer.writeheader()
            self.headers_written = True
        self.writer.writerow(data_dict)
        self.file.flush()

    def close(self):
        if self.file:
            self.file.close()
            print(f"[LOG] Datos guardados en: {self.filename}")

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
