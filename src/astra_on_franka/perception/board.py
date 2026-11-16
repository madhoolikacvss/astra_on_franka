"""Planar four-corner normalization in reference-tag XY; not robot geometry."""
import numpy as np

ORDER = ("near_left", "near_right", "far_right", "far_left")
TARGETS = ((0., 0.), (1., 0.), (1., 1.), (0., 1.))


class BoardMapping:
    def __init__(self, corners):
        points = np.asarray([corners[k] for k in ORDER], dtype=float)
        if points.shape != (4, 3) or not np.isfinite(points).all():
            raise ValueError("Expected four finite XYZ corners")
        xy = points[:, :2]
        edges = np.roll(xy, -1, axis=0) - xy
        turns = [np.linalg.det(np.stack((edges[i], edges[(i+1)%4]))) for i in range(4)]
        if not (all(v > 1e-8 for v in turns) or all(v < -1e-8 for v in turns)):
            raise ValueError("Board corners must form a nondegenerate convex perimeter")
        A, b = [], []
        for (x, y), (u, v) in zip(xy, TARGETS):
            A.extend(([x,y,1,0,0,0,-u*x,-u*y], [0,0,0,x,y,1,-v*x,-v*y]))
            b.extend((u,v))
        self.matrix = np.append(np.linalg.solve(A, b), 1).reshape(3,3)
        denominators = np.column_stack((xy, np.ones(4))) @ self.matrix[2]
        if not (np.all(denominators > 1e-9) or np.all(denominators < -1e-9)):
            raise ValueError("Mapping has a singularity within the board")

    def normalize(self, xy):
        xy = np.asarray(xy, dtype=float)
        if xy.shape != (2,) or not np.isfinite(xy).all():
            raise ValueError("Expected finite XY")
        p = self.matrix @ np.append(xy, 1)
        if abs(p[2]) < 1e-9:
            raise ValueError("Position is at normalization singularity")
        return (p[:2] / p[2]).tolist()
