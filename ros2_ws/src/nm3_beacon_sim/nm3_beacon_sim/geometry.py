"""Horizontal range-circle helpers (top view at transducer depth)."""
import numpy as np


def horizontal_radius(rho: float, dz2: float) -> float:
    """R_h = sqrt(rho^2 - dz^2); NaN when the slant range is shorter than the depth offset."""
    r2 = rho * rho - dz2
    return float(np.sqrt(r2)) if r2 >= 0 else float("nan")


def circle_points(cx: float, cy: float, r: float, n: int = 72) -> np.ndarray:
    th = np.linspace(0.0, 2 * np.pi, n + 1)
    return np.c_[cx + r * np.cos(th), cy + r * np.sin(th)]


def circle_intersections(c1, r1, c2, r2, tol: float = 1e-9):
    """Intersection points of two circles in the plane: list of 0, 1 or 2 (x, y).

    When the noisy circles just miss each other (separate or nested), returns the single
    closest-approach point on the centre line, so there is still a candidate to show.
    """
    c1, c2 = np.asarray(c1, float), np.asarray(c2, float)
    d = float(np.linalg.norm(c2 - c1))
    if d < tol or not (np.isfinite(r1) and np.isfinite(r2)):
        return []
    u = (c2 - c1) / d
    if d > r1 + r2:                                   # separate: midpoint of the gap
        return [tuple(c1 + u * (r1 + (d - r1 - r2) / 2))]
    if d < abs(r1 - r2):                              # nested: midpoint between the rims
        s = 1.0 if r1 > r2 else -1.0
        p1, p2 = c1 + s * u * r1, c2 + s * u * r2
        return [tuple((p1 + p2) / 2)]
    a = (r1 * r1 - r2 * r2 + d * d) / (2 * d)
    h = np.sqrt(max(r1 * r1 - a * a, 0.0))
    m = c1 + a * u
    perp = np.array([-u[1], u[0]])
    if h < tol:
        return [tuple(m)]
    return [tuple(m + h * perp), tuple(m - h * perp)]
