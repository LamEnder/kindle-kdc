"""Joint fit of per-level pulse steps to all kdc-cal ladders.

usage: fit_joint.py stage1.jpg stage2.jpg WD WL

Model (Markov in level, piecewise linear):
  dark pulse:  l' = max(0, l - rd(l))     rd: steps at knots 0, .5, .., 3
  light pulse: l' = min(3, l + ru(l))
Fitted by coordinate descent to six ladders: W>, D>K, L>K (dark) and K>, D>W,
L>W (light). Starts of the stage-2 ladders come from simulating WD / WL dark
pulses from white with the same model, so the fit is self-consistent.
"""
import sys
sys.path.insert(0, __file__.rsplit("/", 1)[0])
import ladder_measure as L
from PIL import Image

KNOTS = [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0]


def setup(path):
    img = Image.open(path)
    g = img.convert("L")
    tl, tr, bl, br = L.find_screen(img)
    return g, L.homography([(0, 0), (L.SW, 0), (0, L.SH), (L.SW, L.SH)], [tl, tr, bl, br])


def local(g, h, by, i):
    x, y = L.patch_pos(by, i)
    p = L.mean(g, h, x + 14, y + 14, x + L.PATCH - 14, y + L.PATCH - 14)
    sy = y + L.PATCH
    d = L.mean(g, h, x + 6, sy + 6, x + L.PATCH - 6, sy + L.STRIPE_H - 6)
    l = L.mean(g, h, x + 6, sy + L.STRIPE_H + 6, x + L.PATCH - 6, sy + 2 * L.STRIPE_H - 6)
    return p, d, l


def interp(vals, l):
    if l <= 0: return vals[0]
    if l >= 3: return vals[-1]
    i = int(l / 0.5)
    t = (l - KNOTS[i]) / 0.5
    return vals[i] + (vals[i + 1] - vals[i]) * t


def measure(s1, s2):
    g1, h1 = setup(s1)
    n1 = 40
    by_w, by_k = 8, 8 + L.block_h(n1)
    rk, rw = [], []
    for i in range(43, 47):
        for by, acc in ((by_w, rw), (by_k, rk)):
            bg, d, l = local(g1, h1, by, i)
            acc.append((bg - d) / (l - d))
    RK, RW = sum(rk) / len(rk), sum(rw) / len(rw)

    def level(p, d, l):
        s = l - d
        pts = [(d + RK * s, 0.0), (d, 1.0), (l, 2.0), (d + RW * s, 3.0)]
        if p <= pts[0][0]:
            v = (p - pts[0][0]) / (pts[1][0] - pts[0][0])
        else:
            v = None
            for (a, la), (b, lb) in zip(pts, pts[1:]):
                if p <= b:
                    v = la + (p - a) / (b - a) * (lb - la)
                    break
            if v is None:
                v = 3.0 + (p - pts[3][0]) / (pts[3][0] - pts[2][0])
        return min(3.0, max(0.0, v))

    seqs = {}
    seqs["W>"] = [level(*local(g1, h1, by_w, i)) for i in range(1, n1 + 1)]
    seqs["K>"] = [level(*local(g1, h1, by_k, i)) for i in range(1, n1 + 1)]
    g2, h2 = setup(s2)
    n2 = 32
    for k, t in enumerate(["D>W", "D>K", "L>W", "L>K"]):
        by = 8 + k * L.block_h(n2)
        seqs[t] = [level(*local(g2, h2, by, i)) for i in range(1, n2 + 1)]
    return seqs


def simulate(rd, ru, start, direction, n):
    l, out = start, []
    for _ in range(n):
        l = max(0.0, l - interp(rd, l)) if direction < 0 else min(3.0, l + interp(ru, l))
        out.append(l)
    return out


def loss(rd, ru, seqs, wd, wl):
    d_start = simulate(rd, ru, 3.0, -1, wd)[-1]
    l_start = simulate(rd, ru, 3.0, -1, wl)[-1]
    spec = [("W>", 3.0, -1), ("K>", 0.0, +1), ("D>W", d_start, +1), ("D>K", d_start, -1),
            ("L>W", l_start, +1), ("L>K", l_start, -1)]
    e = n = 0
    for name, st, dr in spec:
        meas = seqs[name]
        sim = simulate(rd, ru, st, dr, len(meas))
        e += sum((a - b) ** 2 for a, b in zip(sim, meas))
        n += len(meas)
    # Also anchor the stage-2 starts to what stage 1 measured at WD / WL.
    e += 4 * ((d_start - seqs["W>"][wd - 1]) ** 2 + (l_start - seqs["W>"][wl - 1]) ** 2)
    return e / n


def main():
    s1, s2, wd, wl = sys.argv[1], sys.argv[2], int(sys.argv[3]), int(sys.argv[4])
    seqs = measure(s1, s2)
    # Start from the parametric fits (dark: 3exp(-n/11.1) -> step ~ l*0.086).
    rd = [max(0.004, k * 0.086) for k in KNOTS]
    ru = [0.30, 0.33, 0.30, 0.25, 0.17, 0.10, 0.0]
    rd[0] = 0.0
    best = loss(rd, ru, seqs, wd, wl)
    print(f"start loss {best:.4f} (rms {best ** 0.5:.3f})")
    step = 0.04
    while step > 0.0015:
        improved = True
        while improved:
            improved = False
            for arr, lo_i, hi_i in ((rd, 1, 7), (ru, 0, 6)):
                for i in range(lo_i, hi_i):
                    for dv in (step, -step):
                        old = arr[i]
                        arr[i] = min(1.0, max(0.003, old + dv))
                        e = loss(rd, ru, seqs, wd, wl)
                        if e < best - 1e-7:
                            best = e
                            improved = True
                        else:
                            arr[i] = old
        step /= 2
    print(f"final loss {best:.4f} (rms {best ** 0.5:.3f} levels)")
    print("dark_steps ", " ".join(f"{v:.3f}" for v in rd))
    print("light_steps", " ".join(f"{v:.3f}" for v in ru))
    d_start = simulate(rd, ru, 3.0, -1, wd)[-1]
    l_start = simulate(rd, ru, 3.0, -1, wl)[-1]
    for name, st, dr in [("W>", 3.0, -1), ("K>", 0.0, +1), ("D>W", d_start, +1), ("D>K", d_start, -1),
                         ("L>W", l_start, +1), ("L>K", l_start, -1)]:
        meas = seqs[name]
        sim = simulate(rd, ru, st, dr, len(meas))
        rms = (sum((a - b) ** 2 for a, b in zip(sim, meas)) / len(meas)) ** 0.5
        print(f"  {name:4s} rms {rms:.2f}   sim/meas @1 {sim[0]:.2f}/{meas[0]:.2f}  @4 {sim[3]:.2f}/{meas[3]:.2f}"
              f"  @10 {sim[9]:.2f}/{meas[9]:.2f}  @24 {sim[23]:.2f}/{meas[23]:.2f}")
    # pulses to reach shades from the fitted model, for reference
    def pulses(start, target, dr):
        l, n = start, 0
        while n < 80 and ((dr < 0 and l > target + 0.05) or (dr > 0 and l < target - 0.05)):
            l = max(0.0, l - interp(rd, l)) if dr < 0 else min(3.0, l + interp(ru, l))
            n += 1
        return n
    print("pulses W->L %d W->D %d W->K(0.1) %d | K->D %d K->L %d K->W(2.9) %d" % (
        pulses(3, 2, -1), pulses(3, 1, -1), pulses(3, 0.05, -1), pulses(0, 1, 1), pulses(0, 2, 1), pulses(0, 2.95, 1)))


main()
