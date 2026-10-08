import requests, time
URL = "http://10.0.0.108"
requests.get(f"{URL}/control?cmd=start")
t_last = 0.0
while True:
    r = requests.get(f"{URL}/get?gyrX={t_last}|gyr_time&gyrY={t_last}|gyr_time"
                     f"&gyrZ={t_last}|gyr_time&gyr_time={t_last}").json()["buffer"]
    ts = r["gyr_time"]["buffer"]
    for i, t in enumerate(ts):
        print(t, r["gyrX"]["buffer"][i], r["gyrY"]["buffer"][i], r["gyrZ"]["buffer"][i])
    if ts: t_last = ts[-1]
    time.sleep(0.05)