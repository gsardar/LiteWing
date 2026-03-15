import cv2
import sys

img_path = r"g:\Projects\Hackathon\Neurotech\aruco_A4_print_4.png"
img = cv2.imread(img_path)
if img is None:
    print("Could not read image")
    sys.exit(1)

# Check for markers in different dictionaries
dicts_ids = {
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250
}

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
params = cv2.aruco.DetectorParameters()

for name, d_id in dicts_ids.items():
    d = cv2.aruco.getPredefinedDictionary(d_id)
    detector = cv2.aruco.ArucoDetector(d, params)
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is not None:
        print(f"Detected {len(ids)} markers in {name}: IDs {ids.flatten().tolist()}")
    else:
        print(f"No markers found in {name}")

print(f"Image shape: {img.shape}")
