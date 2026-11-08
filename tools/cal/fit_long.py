"""Fit long-pulse steps (dual waveform) from long-pulse ladder photos.

usage: fit_long.py long_stage1.jpg long_stage2.jpg WD WL N DOSES_FILE [drop]

DOSES_FILE supplies the calibrated SHORT steps (dark_steps/light_steps), used
to simulate the stage-2 pre-dose (WD / WL short pulses from white). Fits
dark_long_steps / light_long_steps (levels moved by one long pulse at levels
0, .5, .. 3) to the six long ladders by coordinate descent. The last `drop`
patches of each row are ignored (right-edge lighting), default 2.
"""
import os
import sys
sys.path.insert(0, __file__.rsplit("/", 1)[0])
import ladder_levels as LL

KNOTS = [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0]


def interp(vals, l):
    if l <= 0: return vals[0]
    if l >= 3: return vals[-1]
    i = int(l / 0.5)
    t = (l - KNOTS[i]) / 0.5
    return vals[i] + (vals[i + 1] - vals[i]) * t


def step(steps, l, direction):
    return max(0.0, l - interp(steps, l)) if direction < 0 else min(3.0, l + interp(steps, l))


def read_short(path):
    ds = ls = None
    for line in open(path):
        w = line.split()
        if w and w[0] == "dark_steps": ds = [float(v) for v in w[1:8]]
        if w and w[0] == "light_steps": ls = [float(v) for v in w[1:8]]
    if not ds or not ls:
        sys.exit(f"{path}: needs dark_steps and light_steps")
    return ds, ls


def main():
    s1, s2, wd, wl, n, doses = sys.argv[1], sys.argv[2], int(sys.argv[3]), int(sys.argv[4]), int(sys.argv[5]), sys.argv[6]
    drop = int(sys.argv[7]) if len(sys.argv) > 7 else 2
    sd, sl = read_short(doses)
    force = os.environ.get("KDC_CAL_RK")   # override the stage-1 black anchor
    m1, rk = LL.measure(s1, "stage1", n, rk=float(force) if force else None, quiet=True)
    print(f"black anchor K = D {rk:+.2f}*(L-D)")
    m2, _ = LL.measure(s2, "stage2", n, rk=rk, quiet=True)
    keep = n - drop
    pre_d = pre_l = 3.0
    for _ in range(wd): pre_d = step(sd, pre_d, -1)
    for _ in range(wl): pre_l = step(sd, pre_l, -1)
    print(f"stage-2 starts (short model): D {pre_d:.2f}, L {pre_l:.2f}")
    spec = [("W>", 3.0, -1, m1["W>"]), ("K>", 0.0, +1, m1["K>"]),
            ("D>W", pre_d, +1, m2["D>W"]), ("D>K", pre_d, -1, m2["D>K"]),
            ("L>W", pre_l, +1, m2["L>W"]), ("L>K", pre_l, -1, m2["L>K"])]

    def fit_err(dl, ll):
        e = cnt = 0
        for _, st, dr, meas in spec:
            l = st
            for v in meas[:keep]:
                l = step(dl if dr < 0 else ll, l, dr)
                e += (l - v) ** 2
                cnt += 1
        return e / cnt

    # Knots the ladders never visit are otherwise free: a small curvature
    # penalty keeps the step tables smooth there.
    SMOOTH = 0.002

    def loss(dl, ll):
        rough = sum((a[i - 1] - 2 * a[i] + a[i + 1]) ** 2 for a in (dl, ll) for i in range(1, 6))
        return fit_err(dl, ll) + SMOOTH * rough

    def descend(dl, ll):
        best = loss(dl, ll)
        stp = 0.1
        while stp > 0.002:
            improved = True
            while improved:
                improved = False
                for arr in (dl, ll):
                    for i in range(7):
                        if (arr is dl and i == 0) or (arr is ll and i == 6):
                            continue            # no step at the rail itself
                        for dv in (stp, -stp):
                            old = arr[i]
                            arr[i] = min(3.0, max(0.0, old + dv))
                            e = loss(dl, ll)
                            if e < best - 1e-8:
                                best = e
                                improved = True
                            else:
                                arr[i] = old
            stp /= 2
        return best, dl, ll

    # Coordinate descent has local minima (a plateau in one ladder can bend
    # the steps around it): start from several scaled guesses, keep the best.
    base_d = [0.0, 0.2, 0.55, 1.0, 1.5, 1.9, 2.2]
    base_l = [2.5, 2.3, 2.0, 1.5, 1.0, 0.5, 0.0]
    best = None
    for fd in (0.5, 0.75, 1.0):
        for fl in (0.5, 0.75, 1.0):
            r = descend([v * fd for v in base_d], [v * fl for v in base_l])
            if best is None or r[0] < best[0]:
                best = r
    best, dl, ll = best
    best = fit_err(dl, ll)
    print(f"fit rms {best ** 0.5:.3f} levels over {len(spec)} ladders (first {keep} patches)")
    for name, st, dr, meas in spec:
        l, sim = st, []
        for _ in meas[:keep]:
            l = step(dl if dr < 0 else ll, l, dr)
            sim.append(l)
        print(f"  {name:4s} meas " + " ".join(f"{v:.2f}" for v in meas[:6]) +
              "\n       sim  " + " ".join(f"{v:.2f}" for v in sim[:6]))
    print("dark_long_steps  " + " ".join(f"{v:.3f}" for v in dl))
    print("light_long_steps " + " ".join(f"{v:.3f}" for v in ll))


main()
