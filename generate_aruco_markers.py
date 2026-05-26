"""
Genera imágenes de marcadores ArUco para imprimir.
Salida: PNG y PDF listos para imprimir a tamaño exacto.

USO:
    python generate_aruco_markers.py

Por defecto: 6 markers, 20 cm × 20 cm cada uno (mejor SNR de pose
para vuelos a 1-2 m de la pared con la cámara 720p del Tello).

IMPORTANTE PARA IMPRIMIR:
    En el diálogo de impresión, asegúrate de seleccionar
    "Tamaño real" / "100%" / "Actual size" — NO "Ajustar a página".
    Después de imprimir, mide con regla el lado del cuadrado negro
    para confirmar que mide exactamente MARKER_SIZE_CM y actualiza
    config.py: MARKER_SIZE_M = <medida_real_en_metros>.
"""

import cv2
import os
import numpy as np

OUTPUT_DIR    = "aruco_markers"
DICT_ID       = cv2.aruco.DICT_4X4_50
NUM_MARKERS   = 6
MARKER_SIZE_CM = 20.0          # tamaño físico del lado del cuadrado negro
BORDER_RATIO   = 0.10          # borde blanco = 10% del lado del marker

# Resolución de la imagen PNG (suficiente para impresión a 300 DPI hasta 25 cm).
DPI            = 300
MARKER_PX      = int(round(MARKER_SIZE_CM / 2.54 * DPI))
BORDER_PX      = int(round(MARKER_PX * BORDER_RATIO))
PAGE_PX        = MARKER_PX + 2 * BORDER_PX

os.makedirs(OUTPUT_DIR, exist_ok=True)
aruco_dict = cv2.aruco.getPredefinedDictionary(DICT_ID)

png_paths = []
for marker_id in range(NUM_MARKERS):
    marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, MARKER_PX)

    page = np.ones((PAGE_PX, PAGE_PX), dtype=np.uint8) * 255
    page[BORDER_PX:BORDER_PX + MARKER_PX,
         BORDER_PX:BORDER_PX + MARKER_PX] = marker_img

    # Etiquetas (no afectan la zona del marker)
    cv2.putText(page, f"ID: {marker_id}",
                (BORDER_PX, PAGE_PX - 50),
                cv2.FONT_HERSHEY_SIMPLEX, 1.2, 0, 3)
    cv2.putText(page, f"{MARKER_SIZE_CM:.0f} cm x {MARKER_SIZE_CM:.0f} cm  |  DICT_4X4_50",
                (BORDER_PX, PAGE_PX - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, 128, 2)

    fn_png = os.path.join(OUTPUT_DIR, f"aruco_marker_{marker_id}.png")
    cv2.imwrite(fn_png, page)
    png_paths.append(fn_png)
    print(f"[PNG] {fn_png}")

# Generar un PDF con todos los markers — uno por página, tamaño exacto
# (carta con márgenes amplios, marker centrado al tamaño físico declarado).
try:
    from reportlab.pdfgen import canvas
    from reportlab.lib.pagesizes import letter
    from reportlab.lib.units import cm

    pdf_path = os.path.join(OUTPUT_DIR, f"aruco_markers_{int(MARKER_SIZE_CM)}cm.pdf")
    c = canvas.Canvas(pdf_path, pagesize=letter)
    page_w, page_h = letter
    marker_pt = MARKER_SIZE_CM * cm  # 1 cm = 28.346 pt
    x = (page_w - marker_pt) / 2
    y = (page_h - marker_pt) / 2

    for marker_id, fn_png in enumerate(png_paths):
        c.drawImage(fn_png, x, y, width=marker_pt, height=marker_pt,
                    preserveAspectRatio=True)
        c.setFont("Helvetica", 12)
        c.drawString(x, y - 18,
                     f"ID {marker_id}  |  {MARKER_SIZE_CM:.0f} x {MARKER_SIZE_CM:.0f} cm"
                     "  |  DICT_4X4_50  |  imprimir a 100%")
        c.showPage()

    c.save()
    print(f"\n[PDF] {pdf_path}")
    print("    → Imprimir en CARTA con 'Tamaño real / 100%'")
except ImportError:
    print("\n[WARN] reportlab no instalado, no se generó PDF.")
    print("       pip install reportlab  para obtener el PDF tamaño-exacto.")

print(f"\n{NUM_MARKERS} marcadores ({MARKER_SIZE_CM:.0f} cm) generados en {OUTPUT_DIR}/")
print("Recuerda: mide el lado físico tras imprimir y mete el valor exacto"
      " (en metros) en config.py → MARKER_SIZE_M.")
