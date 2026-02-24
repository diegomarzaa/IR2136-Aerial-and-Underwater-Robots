#!/usr/bin/env python3
import argparse, glob, os, re
import numpy as np
import cv2

def read_ocv_yaml(path):
    # 1) Size como lista (no opencv-matrix)
    txt = open(path, "r", errors="ignore").read()
    m = re.search(r"Size:\s*\[\s*([0-9]+)\s*,\s*([0-9]+)\s*\]", txt)
    if not m:
        raise RuntimeError("Missing 'Size: [w, h]' in calibration YAML")
    w, h = int(m.group(1)), int(m.group(2))

    # 2) Matrices con FileStorage (aquí sí es opencv-matrix)
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise RuntimeError(f"Cannot open calibration file: {path}")

    K_L = fs.getNode("K_LEFT").mat()
    D_L = fs.getNode("D_LEFT").mat()
    K_R = fs.getNode("K_RIGHT").mat()
    D_R = fs.getNode("D_RIGHT").mat()
    Rnode = fs.getNode("R").mat()
    Tnode = fs.getNode("T").mat()
    fs.release()

    if any(x is None for x in [K_L, D_L, K_R, D_R, Rnode, Tnode]):
        raise RuntimeError("Missing one of: K_LEFT/D_LEFT/K_RIGHT/D_RIGHT/R/T")

    D_L = np.array(D_L, dtype=np.float64).reshape(-1, 1)
    D_R = np.array(D_R, dtype=np.float64).reshape(-1, 1)
    K_L = np.array(K_L, dtype=np.float64)
    K_R = np.array(K_R, dtype=np.float64)
    T = np.array(Tnode, dtype=np.float64).reshape(3, 1)

    Rnode = np.array(Rnode, dtype=np.float64)
    if Rnode.shape == (3, 3):
        R_lr = Rnode
    else:
        rvec_lr = Rnode.reshape(3, 1)
        R_lr, _ = cv2.Rodrigues(rvec_lr)

    return w, h, K_L, D_L, K_R, D_R, R_lr, T

def extract_index(p):
    m = re.search(r"(\d+)(?:\.\w+)?$", os.path.splitext(os.path.basename(p))[0])
    return int(m.group(1)) if m else None

def load_pairs(folder, left_prefix="image_left_", right_prefix="image_right_"):
    left = glob.glob(os.path.join(folder, f"{left_prefix}*.png"))
    right = glob.glob(os.path.join(folder, f"{right_prefix}*.png"))
    if not left or not right:
        raise RuntimeError("No images found. Check folder/prefixes.")

    L = {extract_index(p): p for p in left if extract_index(p) is not None}
    R = {extract_index(p): p for p in right if extract_index(p) is not None}
    keys = sorted(set(L.keys()) & set(R.keys()))
    pairs = [(L[k], R[k], k) for k in keys]
    return pairs

def chessboard_object_points(pattern_w, pattern_h, square_size):
    # pattern_w x pattern_h = inner corners (e.g., 9x6)
    objp = np.zeros((pattern_w * pattern_h, 3), np.float64)
    grid = np.mgrid[0:pattern_w, 0:pattern_h].T.reshape(-1, 2)
    objp[:, :2] = grid * float(square_size)
    return objp

def detect_corners(gray, pattern_w, pattern_h):
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    ok, corners = cv2.findChessboardCorners(gray, (pattern_w, pattern_h), flags)
    if not ok:
        return None
    # refine
    term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-4)
    corners = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), term)
    return corners.reshape(-1, 2)

def reproj_errors(img_points, proj_points):
    # per-corner Euclidean pixel error
    d = img_points - proj_points
    e = np.sqrt((d[:,0]**2 + d[:,1]**2))
    return e

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--calib", required=True, help="OpenCV YAML (zed_underwater_calibration.yml)")
    ap.add_argument("--folder", required=True, help="Folder with image_left_*.png and image_right_*.png")
    ap.add_argument("--pattern_w", type=int, default=9)
    ap.add_argument("--pattern_h", type=int, default=6)
    ap.add_argument("--square_size", type=float, default=40.0, help="Use SAME units as T. If T is in mm, use 40.0 (mm).")
    ap.add_argument("--left_prefix", default="image_left_")
    ap.add_argument("--right_prefix", default="image_right_")
    ap.add_argument("--max_pairs", type=int, default=0, help="0 = all")
    args = ap.parse_args()

    w, h, K_L, D_L, K_R, D_R, R_lr, T_lr = read_ocv_yaml(args.calib)
    pairs = load_pairs(args.folder, args.left_prefix, args.right_prefix)
    if args.max_pairs > 0:
        pairs = pairs[:args.max_pairs]

    objp = chessboard_object_points(args.pattern_w, args.pattern_h, args.square_size)

    all_err_L = []
    all_err_R = []
    used = 0
    skipped = 0

    for lp, rp, idx in pairs:
        imL = cv2.imread(lp, cv2.IMREAD_COLOR)
        imR = cv2.imread(rp, cv2.IMREAD_COLOR)
        if imL is None or imR is None:
            skipped += 1
            continue

        grayL = cv2.cvtColor(imL, cv2.COLOR_BGR2GRAY)
        grayR = cv2.cvtColor(imR, cv2.COLOR_BGR2GRAY)

        ptsL = detect_corners(grayL, args.pattern_w, args.pattern_h)
        ptsR = detect_corners(grayR, args.pattern_w, args.pattern_h)
        if ptsL is None or ptsR is None:
            skipped += 1
            continue

        # Solve pose from LEFT only
        ok, rvec_L, tvec_L = cv2.solvePnP(objp, ptsL, K_L, D_L, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            skipped += 1
            continue

        # Project back to LEFT
        projL, _ = cv2.projectPoints(objp, rvec_L, tvec_L, K_L, D_L)
        projL = projL.reshape(-1, 2)

        # Derive RIGHT pose using stereo extrinsics: X_r = R_lr * X_l + T_lr
        R_L, _ = cv2.Rodrigues(rvec_L)
        R_R = R_lr @ R_L
        t_R = R_lr @ tvec_L + T_lr
        rvec_R, _ = cv2.Rodrigues(R_R)

        projR, _ = cv2.projectPoints(objp, rvec_R, t_R, K_R, D_R)
        projR = projR.reshape(-1, 2)

        eL = reproj_errors(ptsL, projL)
        eR = reproj_errors(ptsR, projR)

        all_err_L.append(eL)
        all_err_R.append(eR)
        used += 1

    if used == 0:
        raise RuntimeError("No valid pairs processed (chessboard not detected / solvePnP failed).")

    all_err_L = np.concatenate(all_err_L)
    all_err_R = np.concatenate(all_err_R)
    all_err = np.concatenate([all_err_L, all_err_R])

    def stats(e):
        mean = float(np.mean(e))
        rms = float(np.sqrt(np.mean(e**2)))
        med = float(np.median(e))
        return mean, rms, med

    mL, rL, medL = stats(all_err_L)
    mR, rR, medR = stats(all_err_R)
    m, r, med = stats(all_err)

    print(f"Calibration file Size: {w}x{h}")
    print(f"Pairs found: {len(pairs)} | used: {used} | skipped: {skipped}")
    print(f"LEFT  reproj error: mean={mL:.3f}px  rms={rL:.3f}px  median={medL:.3f}px")
    print(f"RIGHT reproj error: mean={mR:.3f}px  rms={rR:.3f}px  median={medR:.3f}px")
    print(f"ALL   reproj error: mean={m:.3f}px   rms={r:.3f}px   median={med:.3f}px")

if __name__ == "__main__":
    main()