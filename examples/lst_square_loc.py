"""
Method 1 — Linear least-squares trilateration of a static underwater beacon
from a GPS-equipped surface vessel (simulation only).

Model (ideal, Method 1 assumptions):
    rho_k = || p_k - x ||,  p_k = (px_k, py_k, z_s),  x = (x, y, z)
Lifting (gamma = x^2 + y^2 + dz^2,  dz = z - z_s):
    rho_k^2 - px_k^2 - py_k^2 = -2 px_k x - 2 py_k y + gamma
Solve A theta = b by least squares, theta = [x, y, gamma]; then
    z_hat = z_s - sqrt(gamma - x^2 - y^2)       (beacon below transducer)

Frame: local ENU, metres, z up (beacon z < 0).
Run:   python method1_trilateration_sim.py
"""
from dataclasses import dataclass
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


# ----------------------------------------------------------------------------- config
@dataclass
class SimConfig:
    beacon: tuple = (120.0, -80.0, -60.0)   # true beacon position (x, y, z) [m]
    chart: tuple = (100.0, -60.0)           # wrong charted horizontal position [m]
    z_s: float = -2.0                       # transducer depth (fixed) [m]
    track: str = "circle"                   # circle | arc | line | line_turn
    radius: float = 80.0                    # survey radius [m]
    n_pings: int = 60                       # number of pings
    sigma_rho: float = 0.3                  # range noise std [m]
    sigma_gps: float = 0.0                  # horizontal GPS noise std [m]
    outlier_rate: float = 0.0               # multipath probability per ping
    heave_std: float = 0.0                  # transducer depth jitter [m] (violates fixed z_s)
    seed: int = 7


# ----------------------------------------------------------------------------- survey track
def make_track(cfg: SimConfig) -> np.ndarray:
    """Horizontal transducer positions (N, 2) around the charted position."""
    c, R, N = np.asarray(cfg.chart), cfg.radius, cfg.n_pings
    f = np.linspace(0.0, 1.0, N)
    if cfg.track == "circle":
        a = 2 * np.pi * np.arange(N) / N
        return c + R * np.c_[np.cos(a), np.sin(a)]
    if cfg.track == "arc":                                  # 90-degree arc, one-sided geometry
        a = -np.pi / 4 + (np.pi / 2) * f
        return c + R * np.c_[np.cos(a), np.sin(a)]
    if cfg.track == "line":                                 # collinear -> rank deficient
        return c + np.c_[-R + 2 * R * f, np.full(N, 30.0)]
    if cfg.track == "line_turn":                            # straight leg + perpendicular leg
        h = N // 2
        leg1 = np.c_[-R + 2 * R * np.linspace(0, 1, h), np.full(h, 30.0)]
        leg2 = np.c_[np.full(N - h, R), 30.0 - 2 * R * np.linspace(0, 1, N - h)]
        return c + np.r_[leg1, leg2]
    raise ValueError(f"unknown track '{cfg.track}'")


# ----------------------------------------------------------------------------- measurements
def simulate_ranges(cfg: SimConfig):
    """Returns P_meas (N,2) GPS-derived positions, rho (N,), P_true (N,3)."""
    rng = np.random.default_rng(cfg.seed)
    xy = make_track(cfg)
    N = len(xy)
    z_true = cfg.z_s + cfg.heave_std * rng.standard_normal(N)          # actual transducer depth
    P_true = np.c_[xy, z_true]
    rho = np.linalg.norm(P_true - np.asarray(cfg.beacon), axis=1)
    rho += cfg.sigma_rho * rng.standard_normal(N)
    is_out = rng.random(N) < cfg.outlier_rate
    rho[is_out] += rng.uniform(2.0, 15.0, is_out.sum())               # multipath: always longer
    P_meas = xy + cfg.sigma_gps * rng.standard_normal((N, 2))          # what GPS tells us
    return P_meas, rho, P_true


# ----------------------------------------------------------------------------- Method 1
def method1_linear_ls(P_xy: np.ndarray, rho: np.ndarray, z_s: float, weighted: bool = False):
    """Linear LS trilateration with lifting. Returns dict with estimate and diagnostics."""
    A = np.c_[-2 * P_xy[:, 0], -2 * P_xy[:, 1], np.ones(len(P_xy))]
    b = rho**2 - P_xy[:, 0]**2 - P_xy[:, 1]**2

    if weighted:                                   # var(rho^2) ~ 4 rho^2 sigma^2 -> w = 1/rho^2
        w = 1.0 / np.maximum(rho, 1e-6)
        A, b = A * w[:, None], b * w

    # conditioning diagnostic on column-normalized A
    An = A / np.linalg.norm(A, axis=0)
    sv = np.linalg.svd(An, compute_uv=False)
    cond = sv[0] / sv[-1] if sv[-1] > 0 else np.inf
    rank = int(np.sum(sv > 1e-10 * sv[0]))

    if rank < 3:
        return dict(ok=False, rank=rank, cond=cond, msg="rank deficient (collinear track)")

    theta, *_ = np.linalg.lstsq(A, b, rcond=None)
    x, y, gamma = theta
    dz2 = gamma - x**2 - y**2
    clamped = dz2 < 0
    z = z_s - np.sqrt(max(dz2, 0.0))               # negative root: beacon below transducer
    return dict(ok=True, x=np.array([x, y, z]), gamma=gamma, dz2=dz2,
                clamped=clamped, rank=rank, cond=cond)


# ----------------------------------------------------------------------------- reporting
def report(cfg: SimConfig, res: dict, label: str = ""):
    xt = np.asarray(cfg.beacon)
    if not res["ok"]:
        print(f"{label:<28} FAILED: {res['msg']} (rank={res['rank']})")
        return
    e = res["x"] - xt
    print(f"{label:<28} est={np.round(res['x'], 2)}  "
          f"|e|H={np.hypot(*e[:2]):6.2f}  |e|z={abs(e[2]):6.2f}  |e|3D={np.linalg.norm(e):6.2f}  "
          f"cond={res['cond']:9.1f}" + ("  [depth clamped]" if res["clamped"] else ""))


def monte_carlo(cfg: SimConfig, runs: int = 300):
    """RMS errors over independent noise draws."""
    errs = []
    for s in range(runs):
        c = SimConfig(**{**cfg.__dict__, "seed": s})
        P, rho, _ = simulate_ranges(c)
        r = method1_linear_ls(P, rho, c.z_s)
        if r["ok"]:
            errs.append(r["x"] - np.asarray(c.beacon))
    E = np.array(errs)
    return np.sqrt(np.mean(E[:, 0]**2 + E[:, 1]**2)), np.sqrt(np.mean(E[:, 2]**2)), len(E)


def plot(cfg: SimConfig, P_xy, rho, res, fname):
    xt = np.asarray(cfg.beacon)
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5.5))
    # top view: horizontal range circles at estimated depth
    dz2 = res["dz2"] if res["ok"] else (xt[2] - cfg.z_s)**2
    th = np.linspace(0, 2 * np.pi, 200)
    for p, r in zip(P_xy, rho):
        Rh = np.sqrt(max(r**2 - max(dz2, 0), 0))
        ax1.plot(p[0] + Rh * np.cos(th), p[1] + Rh * np.sin(th), color="tab:green", lw=0.4, alpha=0.3)
    ax1.plot(P_xy[:, 0], P_xy[:, 1], ".-", color="tab:red", ms=4, lw=0.6, label="pings")
    ax1.plot(*xt[:2], "o", mfc="none", mec="k", ms=12, mew=1.5, label="true beacon")
    ax1.plot(*cfg.chart, "s", color="gray", label="chart position")
    if res["ok"]:
        ax1.plot(*res["x"][:2], "x", color="tab:orange", ms=12, mew=3, label="M1 estimate")
    ax1.set_aspect("equal"); ax1.set_xlabel("x east [m]"); ax1.set_ylabel("y north [m]")
    ax1.set_title(f"Top view — track: {cfg.track}"); ax1.legend(loc="upper right", fontsize=8)
    pad = cfg.radius * 1.4
    ax1.set_xlim(cfg.chart[0] - pad, cfg.chart[0] + pad); ax1.set_ylim(cfg.chart[1] - pad, cfg.chart[1] + pad)
    # residual check: true-model range residual of estimate
    if res["ok"]:
        pred = np.linalg.norm(np.c_[P_xy, np.full(len(P_xy), cfg.z_s)] - res["x"], axis=1)
        ax2.stem(np.arange(len(rho)), rho - pred, basefmt=" ")
        ax2.set_title("Range residuals  ρₖ − ‖pₖ − x̂‖")
    ax2.set_xlabel("ping index k"); ax2.set_ylabel("residual [m]"); ax2.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(fname, dpi=130); plt.close(fig)


# ----------------------------------------------------------------------------- main
if __name__ == "__main__":
    print("=== Single runs (true beacon = (120, -80, -60)) ===")
    for track in ["circle", "arc", "line", "line_turn"]:
        cfg = SimConfig(track=track)
        P, rho, _ = simulate_ranges(cfg)
        res = method1_linear_ls(P, rho, cfg.z_s)
        report(cfg, res, f"track={track}")
        plot(cfg, P, rho, res, f"m1_{track}.png")

    print("\n=== Error sources (circle track) ===")
    cases = {
        "clean (sigma=0.3)":       SimConfig(),
        "GPS noise 1.0 m":         SimConfig(sigma_gps=1.0),
        "outliers 10%":            SimConfig(outlier_rate=0.10),
        "heave 0.5 m (z_s varies)": SimConfig(heave_std=0.5),
        "shallow beacon z=-15":    SimConfig(beacon=(120.0, -80.0, -15.0)),
    }
    for name, cfg in cases.items():
        P, rho, _ = simulate_ranges(cfg)
        report(cfg, method1_linear_ls(P, rho, cfg.z_s), name)

    print("\n=== Unweighted vs weighted (circle, range noise 1 m) ===")
    cfg = SimConfig(sigma_rho=1.0)
    P, rho, _ = simulate_ranges(cfg)
    report(cfg, method1_linear_ls(P, rho, cfg.z_s), "unweighted")
    report(cfg, method1_linear_ls(P, rho, cfg.z_s, weighted=True), "weighted (1/rho)")

    print("\n=== Monte Carlo RMS, 300 runs ===")
    for track in ["circle", "arc", "line_turn"]:
        h, z, n = monte_carlo(SimConfig(track=track))
        print(f"track={track:<10} RMS horiz={h:5.2f} m   RMS depth={z:5.2f} m   ({n} valid runs)")
    print("\nPlots saved: m1_circle.png, m1_arc.png, m1_line.png, m1_line_turn.png")