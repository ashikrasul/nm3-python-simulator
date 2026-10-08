"""Live plot of iPhone accelerometer + gyroscope from phyphox remote access.
Usage:  PHONE_URL=http://<phone-ip>:8080 python live_plot.py
"""
import os, sys, threading, time
from collections import deque
import requests
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

URL = os.environ.get("PHONE_URL", "").rstrip("/")
if not URL:
    sys.exit("Set PHONE_URL, e.g. PHONE_URL=http://10.0.0.108")
WINDOW_S = float(os.environ.get("WINDOW_S", "10"))   # seconds shown
POLL_S = 0.05
MAXLEN = int(WINDOW_S * 120)                          # ~100 Hz + margin

CANDIDATES = {  # prefix -> list of (time buffer, components, axis label); first match wins
    "acc": [("acc_time", ["accX", "accY", "accZ"], "accel [m/s²]")],
    "gyr": [("gyr_time", ["gyrX", "gyrY", "gyrZ"], "gyro [rad/s]"),
            ("gyro_time", ["gyroX", "gyroY", "gyroZ"], "gyro [rad/s]")],
}

# Keep only sensors the open experiment actually provides
names = {b["name"] for b in requests.get(f"{URL}/config", timeout=3).json()["buffers"]}
SENSORS = {}
for k, opts in CANDIDATES.items():
    for tbuf, comps, label in opts:
        if tbuf in names and all(c in names for c in comps):
            SENSORS[k] = (tbuf, comps, label)
            break
print("Plotting:", {k: v[0] for k, v in SENSORS.items()}, flush=True)
if not SENSORS:
    sys.exit(f"No acc/gyr buffers found. Experiment buffers: {sorted(names)}")

data = {k: [deque(maxlen=MAXLEN) for _ in range(4)] for k in SENSORS}  # t, x, y, z
lock = threading.Lock()

def poller():
    last = {v[0]: None for v in SENSORS.values()}
    requests.get(f"{URL}/control?cmd=start", timeout=3)
    while True:
        parts = []
        for tbuf, comps, _ in SENSORS.values():
            thr = last[tbuf]
            if thr is None:
                parts += [f"{tbuf}=full"] + [f"{c}=full" for c in comps]
            else:
                parts += [f"{tbuf}={thr}"] + [f"{c}={thr}|{tbuf}" for c in comps]
        try:
            buf = requests.get(f"{URL}/get?" + "&".join(parts), timeout=3).json()["buffer"]
        except Exception as e:
            print("poll error:", e, flush=True); time.sleep(1); continue
        with lock:
            for k, (tbuf, comps, _) in SENSORS.items():
                cols = [buf[tbuf]["buffer"]] + [buf[c]["buffer"] for c in comps]
                n = min(len(c) for c in cols)
                for d, col in zip(data[k], cols):
                    d.extend(col[:n])
                if n:
                    last[tbuf] = cols[0][n - 1]
        time.sleep(POLL_S)

threading.Thread(target=poller, daemon=True).start()

fig, axes = plt.subplots(len(SENSORS), 1, sharex=True, figsize=(10, 3.2 * len(SENSORS)))
axes = axes if hasattr(axes, "__len__") else [axes]
lines = {}
for ax, (k, (_, _, label)) in zip(axes, SENSORS.items()):
    lines[k] = [ax.plot([], [], lw=1, label=l)[0] for l in ("x", "y", "z")]
    ax.set_ylabel(label); ax.grid(alpha=0.3); ax.legend(loc="upper left", ncol=3)
axes[-1].set_xlabel("phone time [s]")
fig.tight_layout()

def update(_):
    with lock:
        snap = {k: [list(d) for d in v] for k, v in data.items()}
    for ax, k in zip(axes, SENSORS):
        t, *xyz = snap[k]
        if not t:
            continue
        for ln, v in zip(lines[k], xyz):
            ln.set_data(t, v)
        ax.set_xlim(max(0.0, t[-1] - WINDOW_S), max(WINDOW_S, t[-1]))
        ax.relim(); ax.autoscale_view(scalex=False)
    return [ln for ls in lines.values() for ln in ls]

ani = FuncAnimation(fig, update, interval=50, cache_frame_data=False)
plt.show()