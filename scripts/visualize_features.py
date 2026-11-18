"""
SIFT feature matching visualiser for LROC NAC lunar images.

Loads two PNGs from ./lunar_images, detects SIFT keypoints on both,
matches them with a FLANN-based matcher (Lowe ratio test), and writes
a composite image to ./feature_matches.png.
"""

import glob
import os
import sys

import cv2
import numpy as np

IMG_DIR  = "./lunar_images"
OUT_PATH = "./feature_matches.png"

# Strip height to sample from each image (full strips are 52 000+ px tall;
# crop a representative 2532×2532 window from the vertical centre so SIFT
# doesn't run for hours and the composite is readable).
CROP_H = 2532

# FLANN / ratio-test parameters
FLANN_TREES   = 5
FLANN_CHECKS  = 100
LOWE_RATIO    = 0.65    # stricter than the default 0.75: keeps only clean matches
MAX_DRAW      = 80      # cap drawn lines so the image stays legible

# Pair selection

pngs = sorted(glob.glob(os.path.join(IMG_DIR, "*.png")))
if len(pngs) < 2:
    sys.exit(f"Need at least 2 PNG files in {IMG_DIR}, found {len(pngs)}.")

# Prefer the two images that COLMAP found most matches between:
# M179270116LE (526 SIFT) and M179284423LE (270 SIFT): same orbit family, 2011.
preferred = ["M179270116LE.png", "M179284423LE.png"]
selected  = []
for name in preferred:
    full = os.path.join(IMG_DIR, name)
    if os.path.exists(full):
        selected.append(full)

# Fall back to first two available if preferred pair is missing
if len(selected) < 2:
    selected = pngs[:2]

img_a_path, img_b_path = selected[0], selected[1]
print(f"Image A : {os.path.basename(img_a_path)}")
print(f"Image B : {os.path.basename(img_b_path)}")

# Load & crop

def load_and_crop(path: str, crop_h: int) -> np.ndarray:
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        sys.exit(f"Could not read image: {path}")
    h, w = img.shape
    # Take the vertical centre slice
    mid   = h // 2
    start = max(0, mid - crop_h // 2)
    end   = min(h, start + crop_h)
    cropped = img[start:end, :]
    print(f"  {os.path.basename(path)}: full {w}×{h} → cropped {cropped.shape[1]}×{cropped.shape[0]}")
    return cropped

print("\nLoading images …")
gray_a = load_and_crop(img_a_path, CROP_H)
gray_b = load_and_crop(img_b_path, CROP_H)

# SIFT detection

print("\nDetecting SIFT keypoints …")
sift = cv2.SIFT_create(
    nfeatures=0,           # no hard cap: keep all
    nOctaveLayers=4,       # extra octave layers help on low-contrast lunar texture
    contrastThreshold=0.02,# lower than default (0.04) to catch subtle crater edges
    edgeThreshold=12,
    sigma=1.6,
)

kp_a, des_a = sift.detectAndCompute(gray_a, None)
kp_b, des_b = sift.detectAndCompute(gray_b, None)
print(f"  Image A keypoints: {len(kp_a)}")
print(f"  Image B keypoints: {len(kp_b)}")

if des_a is None or des_b is None or len(kp_a) == 0 or len(kp_b) == 0:
    sys.exit("One or both images yielded zero descriptors: cannot match.")

# FLANN matching + Lowe ratio test

print("\nMatching with FLANN …")
index_params  = dict(algorithm=1, trees=FLANN_TREES)   # algorithm=1 → KDTREE
search_params = dict(checks=FLANN_CHECKS)
flann = cv2.FlannBasedMatcher(index_params, search_params)

# knnMatch returns 2 nearest neighbours; ratio test keeps geometrically consistent pairs
raw = flann.knnMatch(des_a, des_b, k=2)
good = [m for m, n in raw if m.distance < LOWE_RATIO * n.distance]
print(f"  Raw matches      : {len(raw)}")
print(f"  After ratio test : {len(good)}  (Lowe ratio={LOWE_RATIO})")

if len(good) == 0:
    sys.exit("No matches survived the ratio test. "
             "The two images likely cover different ground tracks.")

# Sort by match quality (lowest distance = best) and cap for legibility
good_sorted = sorted(good, key=lambda m: m.distance)
draw_matches = good_sorted[:MAX_DRAW]

# Composite image

print(f"\nRendering composite ({len(draw_matches)} matches drawn) …")

# Scale for display: keep width at 2532 px, downsample height proportionally
DISPLAY_H = 900
scale_a = DISPLAY_H / gray_a.shape[0]
scale_b = DISPLAY_H / gray_b.shape[0]

disp_a = cv2.resize(gray_a, (int(gray_a.shape[1] * scale_a), DISPLAY_H))
disp_b = cv2.resize(gray_b, (int(gray_b.shape[1] * scale_b), DISPLAY_H))

# Scale keypoint coordinates to match resized images
def scale_kp(kps, sx, sy):
    out = []
    for k in kps:
        pt = (k.pt[0] * sx, k.pt[1] * sy)
        out.append(cv2.KeyPoint(pt[0], pt[1], k.size * sx,
                                k.angle, k.response, k.octave, k.class_id))
    return out

kp_a_s = scale_kp(kp_a, scale_a, scale_a)
kp_b_s = scale_kp(kp_b, scale_b, scale_b)

# Convert to BGR for colour drawing
bgr_a = cv2.cvtColor(disp_a, cv2.COLOR_GRAY2BGR)
bgr_b = cv2.cvtColor(disp_b, cv2.COLOR_GRAY2BGR)

# Colour-map matches by rank: best (green) → worst (red)
n = len(draw_matches)
composite = cv2.drawMatches(
    bgr_a, kp_a_s,
    bgr_b, kp_b_s,
    draw_matches, None,
    matchColor=(-1, -1, -1),         # individual colours set below
    singlePointColor=(60, 60, 60),
    flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS,
)

# Redraw match lines with quality gradient (green → yellow → red)
w_a = bgr_a.shape[1]
for i, m in enumerate(draw_matches):
    t    = i / max(n - 1, 1)          # 0 = best, 1 = worst
    r    = int(255 * t)
    g    = int(255 * (1 - t))
    col  = (0, g, r)                  # BGR
    pt1  = (int(kp_a_s[m.queryIdx].pt[0]),
            int(kp_a_s[m.queryIdx].pt[1]))
    pt2  = (int(kp_b_s[m.trainIdx].pt[0]) + w_a,
            int(kp_b_s[m.trainIdx].pt[1]))
    cv2.line(composite, pt1, pt2, col, 1, cv2.LINE_AA)
    cv2.circle(composite, pt1, 4, col, -1, cv2.LINE_AA)
    cv2.circle(composite, pt2, 4, col, -1, cv2.LINE_AA)

# Annotation overlay

def put(img, text, pos, scale=0.55, color=(220, 220, 220), thickness=1):
    cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX,
                scale, (0, 0, 0), thickness + 2, cv2.LINE_AA)
    cv2.putText(img, text, pos, cv2.FONT_HERSHEY_SIMPLEX,
                scale, color, thickness, cv2.LINE_AA)

margin = 10
put(composite, f"A: {os.path.basename(img_a_path)}", (margin, 28), color=(100, 220, 100))
put(composite, f"B: {os.path.basename(img_b_path)}", (w_a + margin, 28), color=(100, 180, 255))
put(composite, f"SIFT keypoints: A:{len(kp_a)}  B:{len(kp_b)}", (margin, 56))
put(composite, f"FLANN matches: {len(raw)} raw  →  {len(good)} (Lowe {LOWE_RATIO})  →  {len(draw_matches)} drawn",
    (margin, 80))
put(composite, "Match quality:  GREEN = best  /  RED = weakest",
    (margin, composite.shape[0] - 12), color=(180, 180, 180))

# Thin dividing line between images
cv2.line(composite, (w_a, 0), (w_a, composite.shape[0]), (80, 80, 80), 1)

cv2.imwrite(OUT_PATH, composite)
print(f"\nSaved: {OUT_PATH}  ({composite.shape[1]}×{composite.shape[0]} px)")
print(f"Best match distance : {draw_matches[0].distance:.2f}")
print(f"Worst drawn distance: {draw_matches[-1].distance:.2f}")
