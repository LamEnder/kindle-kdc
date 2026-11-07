"""Fit kdc response curves from kdc-cal photos and validate the state model.

usage: fit_curves.py stage1.jpg stage2.jpg WD WL [out.doses]

Response curves (levels 0=K .. 3=W):
  dark(n)  = level after n darkening pulses from W   (stage 1, W> block)
  light(n) = level after n lightening pulses from K  (stage 1, K> block)
Both are made monotone (pool-adjacent-violators) and clamped to [0, 3].
State model: a pulse moves a pixel along the curve of its direction from its
current level ("Markov in level"). Validated by predicting stage 2.
"""
import sys
sys.path.insert(0, __file__.rsplit("/", 1)[0])
import ladder_measure as L
from PIL import Image


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


def pav(ys, increasing):
    """Isotonic regression (pool adjacent violators)."""
    v = ys if increasing else [-y for y in ys]
    blocks = [[x, 1] for x in v]
    i = 0
    while i < len(blocks) - 1:
        if blocks[i][0] > blocks[i + 1][0]:
            a, b = blocks[i], blocks[i + 1]
            blocks[i] = [(a[0] * a[1] + b[0] * b[1]) / (a[1] + b[1]), a[1] + b[1]]
            del blocks[i + 1]
            i = max(i - 1, 0)
        else:
            i += 1
    out = []
    for m, c in blocks:
        out += [m] * c
    return out if increasing else [-y for y in out]


def main():
    s1, s2, wd, wl = sys.argv[1], sys.argv[2], int(sys.argv[3]), int(sys.argv[4])
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
            return (p - pts[0][0]) / (pts[1][0] - pts[0][0])
        for (a, la), (b, lb) in zip(pts, pts[1:]):
            if p <= b:
                return la + (p - a) / (b - a) * (lb - la)
        return 3.0 + (p - pts[3][0]) / (pts[3][0] - pts[2][0])

    clamp = lambda v: min(3.0, max(0.0, v))
    raw_dark = [3.0] + [clamp(level(*local(g1, h1, by_w, i))) for i in range(1, n1 + 1)]
    raw_light = [0.0] + [clamp(level(*local(g1, h1, by_k, i))) for i in range(1, n1 + 1)]
    dark = pav(raw_dark, increasing=False)
    light = pav(raw_light, increasing=True)
    # Lightening saturates near white: noise around 2.7..3.2; anything past
    # the first sample >= 2.9 is white.
    for i, v in enumerate(light):
        if v >= 2.9:
            light = light[:i] + [3.0] * (len(light) - i)
            break

    # Smooth parametric fits (the monotone fit keeps photo artifacts such as
    # per-line lighting steps, which would stall the model on a plateau):
    #   dark(n)  = 3 * exp(-(n / td) ** pd)
    #   light(n) = 3 * (1 - exp(-(n / tl) ** pl))
    import math
    def fit(data, fn):
        best = None
        for t10 in range(10, 600):
            t = t10 / 10
            for p100 in range(50, 300, 2):
                pp = p100 / 100
                e = sum((fn(n, t, pp) - v) ** 2 for n, v in enumerate(data))
                if best is None or e < best[0]:
                    best = (e, t, pp)
        return best
    fd = lambda n, t, pp: 3 * math.exp(-((n / t) ** pp))
    fl = lambda n, t, pp: 3 * (1 - math.exp(-((n / t) ** pp)))
    ed, td, pd = fit(raw_dark, fd)
    el, tl, pl = fit(raw_light, fl)
    print(f"fit dark : tau {td:.1f} p {pd:.2f}  rms {(ed / len(raw_dark)) ** 0.5:.2f}")
    print(f"fit light: tau {tl:.1f} p {pl:.2f}  rms {(el / len(raw_light)) ** 0.5:.2f}")
    NC = 64
    dark = [fd(n, td, pd) for n in range(NC)]
    light = [fl(n, tl, pl) for n in range(NC)]

    def inv(curve, lvl, decreasing):
        """Fractional pulse count n with curve(n) == lvl."""
        for i in range(1, len(curve)):
            a, b = curve[i - 1], curve[i]
            if (decreasing and a >= lvl >= b) or (not decreasing and a <= lvl <= b):
                return i - 1 + ((lvl - a) / (b - a) if b != a else 0.0)
        return None

    def at(curve, n):
        if n >= len(curve) - 1:
            return curve[-1]
        i = int(n)
        return curve[i] + (curve[i + 1] - curve[i]) * (n - i)

    def step(lvl, direction):
        curve, dec = (dark, True) if direction < 0 else (light, False)
        n = inv(curve, lvl, dec)
        if n is None:        # beyond the curve's reach: no further change
            return lvl
        return at(curve, n + 1)

    print("raw dark :", " ".join(f"{v:.2f}" for v in raw_dark))
    print("raw light:", " ".join(f"{v:.2f}" for v in raw_light))

    print("dark :", " ".join(f"{v:.2f}" for v in dark[:41]))
    print("light:", " ".join(f"{v:.2f}" for v in light[:41]))
    # Validate on stage 2.
    g2, h2 = setup(s2)
    n2 = 32
    cases = [("D>W", wd, +1), ("D>K", wd, -1), ("L>W", wl, +1), ("L>K", wl, -1)]
    tot2 = cnt = 0
    for k, (title, pre, direction) in enumerate(cases):
        by = 8 + k * L.block_h(n2)
        meas = [level(*local(g2, h2, by, i)) for i in range(1, n2 + 1)]
        lvl = 3.0
        for _ in range(pre):
            lvl = step(lvl, -1)
        pred = []
        for _ in range(n2):
            lvl = step(lvl, direction)
            pred.append(lvl)
        err = [p - m for p, m in zip(pred, meas)]
        rms = (sum(e * e for e in err) / len(err)) ** 0.5
        tot2 += sum(e * e for e in err); cnt += len(err)
        print(f"{title}: start {at(dark, pre):.2f}  rms {rms:.2f}  "
              f"pred/meas @1 {pred[0]:.2f}/{meas[0]:.2f}  @4 {pred[3]:.2f}/{meas[3]:.2f}  "
              f"@10 {pred[9]:.2f}/{meas[9]:.2f}  @20 {pred[19]:.2f}/{meas[19]:.2f}")
    print(f"stage-2 prediction rms over all: {(tot2 / cnt) ** 0.5:.2f} levels")

    if len(sys.argv) > 5:
        with open(sys.argv[5], "w") as f:
            f.write("# kdc response curves: Kindle PaperWhite 4 (ED060KC4 panel, fw 5.18.1.1.1)\n"
                    "# Calibrated 2026-11-07 with kdc-cal at 18-19 C, 1-frame pulses (install 1).\n"
                    "# dark  = level after n darkening pulses from white (n = 0, 1, ...)\n"
                    "# light = level after n lightening pulses from black\n"
                    "# Levels: 0 = K, 1 = D (0x55), 2 = L (0xAA), 3 = W. Monotone fit of photo data.\n"
                    "# The 4x4 dose matrix is the fallback for dose-counting mode.\n"
                    "install 1\n")
            f.write("dark  " + " ".join(f"{v:.3f}" for v in dark) + "\n")
            f.write("light " + " ".join(f"{v:.3f}" for v in light) + "\n")
            f.write("#    K   D   L   W\nK    0   3   6  10\nD   20   0   3   9\nL   22   4   0   8\nW   30  11   5   0\n")
        print(f"wrote {sys.argv[5]}")


main()
