"""
Genera el patrón de chessboard para calibrar la cámara del Tello.

USO:
    python generate_chessboard.py

SALIDA:
    aruco_markers/chessboard_9x6_30mm.pdf
    Imprimir en CARTA / A4 a 100% (tamaño real, NO ajustar a página).

DEFAULT: 9 × 6 esquinas interiores, cuadrados de 30 mm (= 30 cm × 21 cm).
Cada cuadrado debe quedar exactamente del tamaño impreso. Después de
imprimir, MIDE con regla un lado de un cuadrado y mete el valor real
(en metros) en calibrate_camera.py → SQUARE_SIZE_M.
"""

import os

# Patrón estándar para calibración (esquinas INTERIORES)
COLS_INNER = 9          # cuadrados horizontales − 1
ROWS_INNER = 6          # cuadrados verticales − 1
SQUARE_MM  = 30.0       # tamaño físico de cada cuadrado [mm]

OUTPUT_DIR = "aruco_markers"
os.makedirs(OUTPUT_DIR, exist_ok=True)

try:
    from reportlab.pdfgen import canvas
    from reportlab.lib.pagesizes import letter
    from reportlab.lib.units import mm
except ImportError:
    print("[ERROR] Falta reportlab. Instala con:")
    print("    pip install reportlab")
    raise SystemExit(1)

# Tamaño total del tablero
total_cols = COLS_INNER + 1
total_rows = ROWS_INNER + 1
board_w_mm = total_cols * SQUARE_MM
board_h_mm = total_rows * SQUARE_MM

pdf_path = os.path.join(
    OUTPUT_DIR,
    f"chessboard_{COLS_INNER}x{ROWS_INNER}_{int(SQUARE_MM)}mm.pdf"
)

c = canvas.Canvas(pdf_path, pagesize=letter)
page_w, page_h = letter
x0 = (page_w - board_w_mm * mm) / 2
y0 = (page_h - board_h_mm * mm) / 2

# Dibujar el tablero (cuadros alternados negro/blanco)
for r in range(total_rows):
    for col in range(total_cols):
        if (r + col) % 2 == 0:
            c.setFillColorRGB(0, 0, 0)
        else:
            c.setFillColorRGB(1, 1, 1)
        c.rect(
            x0 + col * SQUARE_MM * mm,
            y0 + r * SQUARE_MM * mm,
            SQUARE_MM * mm,
            SQUARE_MM * mm,
            stroke=0, fill=1,
        )

# Borde negro alrededor del tablero
c.setStrokeColorRGB(0, 0, 0)
c.setLineWidth(0.5)
c.rect(x0, y0, board_w_mm * mm, board_h_mm * mm, stroke=1, fill=0)

# Etiquetas debajo del tablero
c.setFillColorRGB(0, 0, 0)
c.setFont("Helvetica", 10)
label_y = y0 - 18
c.drawString(
    x0, label_y,
    f"Chessboard {COLS_INNER}x{ROWS_INNER} esquinas interiores | "
    f"cuadrado = {SQUARE_MM:.0f} mm | imprimir a 100%"
)
c.drawString(
    x0, label_y - 14,
    f"Verifica con regla: cada cuadrado debe medir {SQUARE_MM:.0f} mm. "
    f"Si no, mete el valor real en calibrate_camera.py."
)

c.showPage()
c.save()

print(f"[OK] Generado: {pdf_path}")
print(f"     Dimensiones del tablero: {board_w_mm:.0f} mm × {board_h_mm:.0f} mm")
print(f"     Esquinas interiores:    {COLS_INNER} × {ROWS_INNER}")
print(f"     Tamaño de cuadrado:     {SQUARE_MM:.0f} mm")
print()
print("Para imprimir: 'Tamaño real' / 'Actual size' / 100% en el diálogo.")
print("Pegar sobre cartón/cartulina rígida (que NO se doble) antes de calibrar.")
