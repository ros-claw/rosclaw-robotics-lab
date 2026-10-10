"""Actual finite-return PointCloud2 ray traversal; no predicted/ground-truth credit."""

import numpy as np


def observed_height_bands(points, origin, region, resolution, width, height):
    """Conservative sample spacing in XY and Z, clipped before actual returns.

    Only finite measured rays are used; a missing return grants no free space.
    Each cell needs witnesses in all three declared obstacle-height bands.
    This finite-resolution inspection does not prove unmeasured thin objects absent.
    """
    points = np.asarray(points, dtype=float)
    origin = np.asarray(origin, dtype=float)
    delta = points - origin
    lower = np.array([*region["min"], 0.1])
    upper = np.array([*region["max"], 0.8])
    near = np.zeros(len(points))
    far = np.ones(len(points))
    valid = np.ones(len(points), dtype=bool)
    for axis in range(3):
        zero = np.abs(delta[:, axis]) < 1e-10
        valid &= ~zero | ((origin[axis] >= lower[axis]) & (origin[axis] <= upper[axis]))
        denom = np.where(zero, 1.0, delta[:, axis])
        a = (lower[axis] - origin[axis]) / denom
        b = (upper[axis] - origin[axis]) / denom
        near = np.maximum(near, np.where(zero, -np.inf, np.minimum(a, b)))
        far = np.minimum(far, np.where(zero, np.inf, np.maximum(a, b)))
    # Actual hit and uncertainty neighborhood are never clearing observations.
    lengths = np.linalg.norm(delta, axis=1)
    far = np.minimum(far, 1.0 - 0.06 / np.maximum(lengths, 1e-9))
    valid &= far > near
    indexes = np.flatnonzero(valid)
    masks = np.zeros((height, width), dtype=np.uint8)
    for offset in range(0, len(indexes), 1024):
        ix = indexes[offset : offset + 1024]
        span = far[ix] - near[ix]
        steps = max(
            2,
            int(
                np.ceil(
                    np.max(np.linalg.norm(delta[ix] * span[:, None], axis=1))
                    / min(resolution / 3, 0.04)
                )
            ),
        )
        t = near[ix, None] + span[:, None] * np.linspace(0, 1, steps + 1)[None, :]
        xyz = origin + delta[ix, None, :] * t[:, :, None]
        xyz = xyz.reshape(-1, 3)
        cell = np.floor((xyz[:, :2] - lower[:2]) / resolution).astype(int)
        inside = (
            (cell[:, 0] >= 0)
            & (cell[:, 0] < width)
            & (cell[:, 1] >= 0)
            & (cell[:, 1] < height)
            & (xyz[:, 2] >= 0.1)
            & (xyz[:, 2] <= 0.8)
        )
        band = np.where(xyz[:, 2] < 0.3, 1, np.where(xyz[:, 2] < 0.55, 2, 4)).astype(
            np.uint8
        )
        np.bitwise_or.at(masks, (cell[inside, 1], cell[inside, 0]), band[inside])
    return masks
