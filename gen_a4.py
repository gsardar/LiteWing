import cv2, numpy as np

DPI = 300
A4_W = int(210/25.4*DPI)   # 2480px
A4_H = int(297/25.4*DPI)   # 3508px

MARKER_CM = 6
MARKER_PX = int(MARKER_CM/2.54*DPI)
BORDER_PX = int(1.0/2.54*DPI)   # 1cm white border each side

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)

def make_marker(mid):
    m = cv2.aruco.generateImageMarker(aruco_dict, mid, MARKER_PX)
    return cv2.copyMakeBorder(m, BORDER_PX, BORDER_PX, BORDER_PX, BORDER_PX,
                               cv2.BORDER_CONSTANT, value=255)

MARKER_IDS = [7, 8, 9, 10]
markers = [make_marker(mid) for mid in MARKER_IDS]
sz = markers[0].shape[0]   # total size per marker including border

COLS, ROWS = 2, 2
MARGIN_X = (A4_W - COLS * sz) // (COLS + 1)
MARGIN_Y = (A4_H - ROWS * sz) // (ROWS + 1)

DASH  = int(0.5/2.54*DPI)
SPACE = int(0.3/2.54*DPI)
GAP   = int(0.3/2.54*DPI)
LT    = 4
F     = cv2.FONT_HERSHEY_SIMPLEX

def dashed(img, p1, p2, c, t, d, s):
    xa,ya = p1; xb,yb = p2
    ln = ((xb-xa)**2+(yb-ya)**2)**0.5
    if ln == 0: return
    dx,dy = (xb-xa)/ln, (yb-ya)/ln
    pos = 0; draw = True
    while pos < ln:
        seg = min(d if draw else s, ln-pos)
        if draw:
            cv2.line(img,
                     (int(xa+dx*pos),       int(ya+dy*pos)),
                     (int(xa+dx*(pos+seg)), int(ya+dy*(pos+seg))), c, t)
        pos += seg; draw = not draw

def draw_cut_box(img, ox, oy, sz, gap, dash, space, lt):
    x1,y1 = ox-gap, oy-gap
    x2,y2 = ox+sz+gap, oy+sz+gap
    dashed(img,(x1,y1),(x2,y1),60,lt,dash,space)
    dashed(img,(x1,y2),(x2,y2),60,lt,dash,space)
    dashed(img,(x1,y1),(x1,y2),60,lt,dash,space)
    dashed(img,(x2,y1),(x2,y2),60,lt,dash,space)
    T = int(0.5/2.54*DPI)
    for cx,cy in [(x1,y1),(x2,y1),(x1,y2),(x2,y2)]:
        cv2.line(img,(cx-T,cy),(cx+T,cy),60,lt)
        cv2.line(img,(cx,cy-T),(cx,cy+T),60,lt)

page = np.ones((A4_H, A4_W), dtype=np.uint8) * 255

for row in range(ROWS):
    for col in range(COLS):
        idx = row * COLS + col
        mid = MARKER_IDS[idx]
        ox  = MARGIN_X + col * (sz + MARGIN_X)
        oy  = MARGIN_Y + row * (sz + MARGIN_Y)
        page[oy:oy+sz, ox:ox+sz] = markers[idx]
        draw_cut_box(page, ox, oy, sz, GAP, DASH, SPACE, LT)
        # Label under each marker
        lx = ox
        ly = oy + sz + GAP + int(0.45/2.54*DPI)
        cv2.putText(page, "ID={}  8x8cm (incl. border)".format(mid), (lx, ly), F, 0.85, 80, 2)

# Page header
cv2.putText(page, "Neurotech Drone Tracker - ArUco DICT_4X4_250  IDs 7-10",
            (MARGIN_X, int(0.7/2.54*DPI)), F, 1.2, 60, 3)

# Page footer
fy = A4_H - int(0.8/2.54*DPI)
cv2.putText(page, "Print at 100% scale - NO fit/shrink to page  |  Cut on dashed line  |  Keep white border",
            (MARGIN_X, fy - int(0.5/2.54*DPI)), F, 0.85, 60, 2)

# 1cm scale bar
bx = MARGIN_X; by = fy
bl = int(1/2.54*DPI)
cv2.rectangle(page,(bx,by-14),(bx+bl,by),0,-1)
cv2.line(page,(bx,by+4),(bx,by+16),0,3)
cv2.line(page,(bx+bl,by+4),(bx+bl,by+16),0,3)
cv2.putText(page,"1 cm",(bx+bl+10,by),F,0.8,70,2)

out = r"F:\Projects\Hackathon\Neurotech\aruco_A4_print.png"
cv2.imwrite(out, page)
print("Saved: " + out)
print("IDs 7, 8, 9, 10 - one marker each, 2x2 grid, A4 300dpi")
