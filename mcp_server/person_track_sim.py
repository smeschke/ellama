"""Closed-loop sim of person_track.PersonTracker: a fake spin plant (deadband, lag), a
camera that is ~0.25 s late at 16 fps, and a person who stands still or walks around.
A sanity check of the logic, NOT a prediction of the real robot. Run: python person_track_sim.py"""
import math
from collections import deque

import person_track as pt
from person_vision import bearing_deg

DT, FPS, CAM_LAG, HFOV = 0.01, 16, 0.25, 51.0


DEAD = 28


def plant_rate(t):          # deg/s, + = left (IMU sign); t + = turn right
    m = abs(t)
    r = 0.0 if m < DEAD else 0.5 * (m - DEAD)
    return -math.copysign(r, t)


def run(person_yaw, T=20.0, seed_yaw=0.0, name="", walk=None, **kw):
    trk = pt.PersonTracker(max_run_s=T + 1, **kw)
    yaw, rate, now = seed_yaw, 0.0, 100.0
    trk.start(now, yaw)
    cam = deque()           # (arrival_time, bearing list)
    cmd, nxt, log = None, now, []
    for i in range(int(T / DT)):
        now += DT
        py = person_yaw(now - 100) if callable(person_yaw) else person_yaw
        if now >= nxt:      # a frame is captured; it arrives CAM_LAG later
            nxt += 1 / FPS
            b = -(py - yaw)                       # + = person right of heading
            vis = abs(b) < HFOV / 2
            cam.append((now + CAM_LAG, [dict(bearing_deg=bearing_deg(0.5 + math.tan(math.radians(b)) * 0.5 / math.tan(math.radians(HFOV / 2)))) if vis else None]))
        while cam and cam[0][0] <= now:
            _, ppl = cam.popleft()
            trk.on_frame(dict(people=[p for p in ppl if p]), now)
        cmd = trk.step(now, yaw)
        t = cmd.left_raw / trk.sign if cmd.status == "running" else 0.0
        rate += (plant_rate(t) - rate) * min(1, DT / 0.15)      # ~0.15 s lag
        yaw += rate * DT
        log.append((now - 100, yaw, py))
        if cmd.status != "running":
            break
    err = abs(log[-1][2] - log[-1][1])
    print(f"{name:34s} end={trk.status:8s} final err {err:5.1f} deg, max |yaw-person| over last 5s "
          f"{max(abs(p - y) for _, y, p in log[-500:]):5.1f}")


if __name__ == "__main__":
    for d in (28, 56):
        DEAD = d
        print(f"-- spin deadband {d}")
        run(20, name="person 20 deg to the left")
        run(-22, name="person 22 deg to the right")
        run(5, name="small 5 deg")
        run(lambda t: 20 * math.sin(t / 3), T=30, name="person walks back and forth")
    print("-- faster spin / lost handling (deadband 56)")
    for u in (18, 30, 45):
        run(lambda t: 20 * math.sin(t / 3), T=30, name=f"walks back and forth, spin {u} dps", spin_u_max=u)
        run(35 if False else 20, name=f"stationary person 20 deg, spin {u} dps", spin_u_max=u)
    # person vanishes at t=6s: default gives up, lost_timeout_s=0 holds on
    gone = lambda t: 20 if t < 6 else 1e9
    run(gone, T=12, name="person disappears, lost_timeout 1s")
    run(gone, T=12, name="person disappears, lost_timeout 0", lost_timeout_s=0)
