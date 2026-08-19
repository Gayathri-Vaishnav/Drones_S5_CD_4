"""apriltag_detect.py — Robust Multi-Altitude AprilTag Pose Estimator."""

import numpy as np
import cv2
from pupil_apriltags import Detector

from .camera_sim import (
    Camera, TAG_ID, LARGE_SIZE, SMALL_SIZE, ALT_SWITCH,
    world_to_body, body_to_cam, tag_world_corners
)

_detector = Detector(
    families="tag36h11",
    quad_decimate=1.0,
    quad_sigma=0.0,
    nthreads=1
)

MEAS_ALPHA = 0.55


class ApriltTagMeasure:
    """Stateful AprilTag pose estimator with measurement filtering."""

    def __init__(self):
        self._filt = np.zeros(3)
        self._last_lock = False

    def detect(self, img_bgr, att, altitude_hint=10.0):
        gray = img_bgr if img_bgr.ndim == 2 else cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        results = _detector.detect(gray)

        best_match = None
        min_err = 999.0

        for res in results:
            if res.tag_id != TAG_ID:
                continue

            corners = res.corners.astype(np.float64)[[1, 0, 3, 2]]

            # Test candidate tag sizes
            sizes_to_test = [SMALL_SIZE, LARGE_SIZE] if altitude_hint < 3.0 else [LARGE_SIZE, SMALL_SIZE]

            for sz in sizes_to_test:
                obj = tag_world_corners(sz, (0.0, 0.0), with_border=False)
                ok, rvec, tvec = cv2.solvePnP(
                    obj, corners, Camera.matrix(), np.zeros((5, 1)),
                    flags=cv2.SOLVEPNP_ITERATIVE
                )
                if not ok:
                    continue

                t = tvec.flatten()
                off = world_to_body(np.zeros(3), att).T @ (body_to_cam().T @ t)
                alt_meas = off[2]

                if alt_meas <= 0.03:
                    continue

                err = abs(alt_meas - altitude_hint)
                if err < min_err:
                    min_err = err
                    best_match = (alt_meas, off[1], off[0], corners)

        if best_match is None or (min_err > 3.0 and altitude_hint < 6.0):
            self._last_lock = False
            return False, 0.0, 0.0, 0.0, None

        alt, east, north, corners = best_match
        raw = np.array([alt, east, north])

        if self._last_lock:
            self._filt = MEAS_ALPHA * raw + (1.0 - MEAS_ALPHA) * self._filt
        else:
            self._filt = raw.copy()

        self._last_lock = True
        return True, float(self._filt[0]), float(self._filt[1]), float(self._filt[2]), corners

    def reset(self):
        self._filt[:] = 0.0
        self._last_lock = False
