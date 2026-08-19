"""camera_sim.py — Realistic Downward Camera Simulator.

Uses calibrated DJI camera intrinsics (FX=FY=650) to ensure continuous AprilTag
detection across all flight regimes from 12m high altitude down to 0.35m touchdown.
"""

import numpy as np
import cv2


class Camera:
    W, H = 640, 480
    FX = FY = 650.0   # Calibrated FOV ~60 deg for continuous multi-altitude lock
    CX, CY = W / 2.0, H / 2.0

    @staticmethod
    def matrix():
        return np.array(
            [[Camera.FX, 0, Camera.CX],
             [0, Camera.FY, Camera.CY],
             [0, 0, 1.0]],
            dtype=float,
        )


TAG_FAMILY    = "tag36h11"
TAG_ID        = 42
LARGE_SIZE    = 0.80   # m
SMALL_SIZE    = 0.20   # m
ALT_SWITCH    = 1.8    # m
PAD_SIZE      = 1.40   # m

PIXEL_SIGMA   = 0.5
BLUR_SIGMA    = 0.6
DROPOUT_PROB  = 0.01

_aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
_tag_raw = cv2.aruco.generateImageMarker(_aruco_dict, TAG_ID, 400)
# Add quiet margin border
TAG_BITMAP = cv2.copyMakeBorder(_tag_raw, 50, 50, 50, 50, cv2.BORDER_CONSTANT, value=255)
BMP_SIZE = 500
BORDER_RATIO = 500.0 / 400.0


def world_to_body(pos, att):
    roll, pitch, yaw = att
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    return ry @ rx


def body_to_cam():
    return np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])


def project(points, pos, att):
    r_wb = world_to_body(pos, att)
    r_cb = body_to_cam()
    m = Camera.matrix()
    out = np.empty((len(points), 2))
    valid = np.ones(len(points), dtype=bool)
    for i, p in enumerate(points):
        pb = r_wb @ (p - pos)
        pc = r_cb @ pb
        if pc[2] <= 1e-4:
            valid[i] = False
            continue
        px = m @ pc
        out[i] = px[:2] / px[2]
    return out, valid


def tag_world_corners(size, pad_pos=(0.0, 0.0), with_border=False):
    s = (size * BORDER_RATIO / 2.0) if with_border else (size / 2.0)
    px, py = pad_pos
    return np.array(
        [[px - s, py - s, 0.0],
         [px + s, py - s, 0.0],
         [px + s, py + s, 0.0],
         [px - s, py + s, 0.0]],
        dtype=float,
    )


def make_ground_image(rng):
    base = np.full((Camera.H, Camera.W, 3), 125, np.uint8)
    noise = rng.normal(0, 7, (Camera.H, Camera.W, 1))
    base = np.clip(base + noise, 0, 255).astype(np.uint8)
    for x in range(0, Camera.W, 64):
        cv2.line(base, (x, 0), (x, Camera.H), (135, 135, 135), 1)
    for y in range(0, Camera.H, 64):
        cv2.line(base, (0, y), (Camera.W, y), (135, 135, 135), 1)
    return base


def render_frame(pos, att, rng, ground, pad_pos=(0.0, 0.0)):
    img = ground.copy()
    alt = -pos[2]

    # Draw pad base
    pad_s = PAD_SIZE / 2.0
    px, py = pad_pos
    pad_3d = np.array(
        [[px - pad_s, py - pad_s, 0.0],
         [px + pad_s, py - pad_s, 0.0],
         [px + pad_s, py + pad_s, 0.0],
         [px - pad_s, py + pad_s, 0.0]]
    )
    pad_corners, p_valid = project(pad_3d, pos, att)
    if p_valid.all() and np.all(pad_corners[:, 1] > -30) and np.all(pad_corners[:, 0] > -30):
        cv2.fillConvexPoly(img, pad_corners.astype(np.int32), (210, 210, 210))

    # Determine tag to render based on altitude
    tag_size = LARGE_SIZE if alt >= ALT_SWITCH else SMALL_SIZE

    tc, valid = project(tag_world_corners(tag_size, pad_pos, with_border=True), pos, att)
    if valid.all() and np.all(tc[:, 0] > -150) and np.all(tc[:, 0] < Camera.W + 150) and np.all(tc[:, 1] > -150) and np.all(tc[:, 1] < Camera.H + 150):
        src = np.float32([[0, 0], [BMP_SIZE, 0], [BMP_SIZE, BMP_SIZE], [0, BMP_SIZE]])
        hmat = cv2.getPerspectiveTransform(src, tc.astype(np.float32))
        warped = cv2.warpPerspective(TAG_BITMAP, hmat, (Camera.W, Camera.H))
        mask = cv2.warpPerspective(np.full((BMP_SIZE, BMP_SIZE), 255, np.uint8), hmat, (Camera.W, Camera.H))
        cv2.copyTo(cv2.cvtColor(warped, cv2.COLOR_GRAY2BGR), mask, img)

    # Optical blur with odd kernel size
    blur_k = int(BLUR_SIGMA * 2) * 2 + 1
    img = cv2.GaussianBlur(img, (blur_k, blur_k), 0)
    noise = rng.normal(0, PIXEL_SIGMA * 4, img.shape)
    img = np.clip(img + noise, 0, 255).astype(np.uint8)
    return img
