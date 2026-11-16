"""Offline rigid registration of corresponding physical points, in meters."""
import numpy as np


def points(values):
    a = np.asarray(values, dtype=float)
    if a.ndim != 2 or a.shape[1] != 3 or not np.isfinite(a).all():
        raise ValueError("All point coordinates must be finite XYZ in meters; fill missing measurements")
    return a


def fit_rigid(reference, base):
    reference, base = points(reference), points(base)
    if reference.shape != base.shape or len(reference) < 3:
        raise ValueError("Need at least three corresponding point pairs")
    x, y = reference - reference.mean(0), base - base.mean(0)
    for centered in (x, y):
        singular = np.linalg.svd(centered, compute_uv=False)
        if singular[1] < max(1e-6, singular[0] * 1e-3):
            raise ValueError("Points are collinear or poorly spread; use points across the board")
    u, _, vt = np.linalg.svd(x.T @ y)
    correction = np.eye(3)
    correction[2, 2] = np.linalg.det(vt.T @ u.T)
    rotation = vt.T @ correction @ u.T
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = base.mean(0) - rotation @ reference.mean(0)
    return transform


def residuals(transform, reference, base):
    reference, base = points(reference), points(base)
    if reference.shape != base.shape:
        raise ValueError("Point counts differ")
    predicted = reference @ transform[:3, :3].T + transform[:3, 3]
    return np.linalg.norm(predicted - base, axis=1)
