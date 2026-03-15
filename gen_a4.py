import cv2, numpy as np
from PIL import Image
import os

DPI = 300
A4_W_IN = 210/25.4
A4_H_IN = 297/25.4
A4_W = int(A4_W_IN * DPI)   # ~2480px
A4_H = int(A4_H_IN * DPI)   # ~3508px

ROWS, COLS = 5, 4
NUM_MARKERS = ROWS * COLS

# EXACT 4cm size at 300 DPI
TARGET_CM = 4.0
GAP_CM = 1.0  # Gap between markers
MARKER_PX = int(TARGET_CM / 2.54 * DPI)
GAP_PX = int(GAP_CM / 2.54 * DPI)

# Calculate centering
TOTAL_W_PX = COLS * MARKER_PX + (COLS - 1) * GAP_PX
TOTAL_H_PX = ROWS * MARKER_PX + (ROWS - 1) * GAP_PX

MARGIN_X = (A4_W - TOTAL_W_PX) // 2
MARGIN_Y = (A4_H - TOTAL_H_PX) // 2

# Use dictionary from phone_balance.py (DICT_4X4_250)
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)

def make_marker(mid):
    return cv2.aruco.generateImageMarker(aruco_dict, mid, MARKER_PX)

# Create white page
page = np.ones((A4_H, A4_W), dtype=np.uint8) * 255

# Fill grid
for r in range(ROWS):
    for c in range(COLS):
        mid = r * COLS + c
        if mid >= NUM_MARKERS: break
        
        marker_img = make_marker(mid)
        
        ox = MARGIN_X + c * (MARKER_PX + GAP_PX)
        oy = MARGIN_Y + r * (MARKER_PX + GAP_PX)
        
        page[oy:oy+MARKER_PX, ox:ox+MARKER_PX] = marker_img
        
        # Add ID label
        cv2.putText(page, f"ID {mid}", (ox + 5, oy + 25), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, 200, 2)

# Header
cv2.putText(page, f"Neurotech: EXACT 4.0cm x 4.0cm Markers with Gaps", 
            (MARGIN_X, MARGIN_Y - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, 0, 2)
cv2.putText(page, "A4 Paper | 300 DPI | Markers are exactly 4cm | IDs 0-19", 
            (MARGIN_X, MARGIN_Y - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, 80, 1)

out_dir = os.getcwd()
out_png = os.path.join(out_dir, "aruco_4cm_grid.png")
out_pdf = os.path.join(out_dir, "aruco_4cm_grid.pdf")

# Save PNG
cv2.imwrite(out_png, page)

# Save PDF using Pillow
img_pil = Image.fromarray(page)
img_pil.save(out_pdf, "PDF", resolution=DPI)

print(f"File saved: {out_png}")
print(f"File saved: {out_pdf}")
print(f"Marker physical size: ~{MARKER_PX/DPI*2.54:.1f} cm")
print(f"Gap physical size: ~{GAP_PX/DPI*2.54:.1f} cm")
