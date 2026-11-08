"""Joint fit of per-level pulse steps to all kdc-cal ladders.

usage: fit_joint.py stage1.jpg stage2.jpg WD WL [N1=40 [N2=32]] [--min-light X]

Model (Markov in level, piecewise linear):
  dark pulse:  l' = max(0, l - rd(l))     rd: steps at knots 0, .5, .., 3
  light pulse: l' = min(3, l + ru(l))
Fitted by coordinate descent to six ladders: W>, D>K, L>K (dark) and K>, D>W,
L>W (light). Starts of the stage-2 ladders come from simulating WD / WL dark
pulses from white with the same model, so the fit is self-consistent.
"""
import sys
sys.path.insert(0, __file__.rsplit("/", 1)[0])
import ladder_levels as LL

KNOTS = [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0]


def measure(s1, s2, n1, n2):
    """Levels of all six ladders (same measurement as ladder_levels.py)."""
    m1, rk = LL.measure(s1, "stage1", n1, quiet=True)
    m2, _ = LL.measure(s2, "stage2", n2, rk=rk, quiet=True)
    m1.update(m2)
    return m1


def interp(vals, l):
    if l <= 0: return vals[0]
    if l >= 3: return vals[-1]
    i = int(l / 0.5)
    t = (l - KNOTS[i]) / 0.5
    return vals[i] + (vals[i + 1] - vals[i]) * t


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
    args = sys.argv[1:]
    min_light = 0.003
    if "--min-light" in args:   # lower bound for light steps below 3 (white must stay reachable)
        k = args.index("--min-light")
        min_light = float(args[k + 1])
        del args[k:k + 2]
    s1, s2, wd, wl = args[0], args[1], int(args[2]), int(args[3])
    n1 = int(args[4]) if len(args) > 4 else 40
    n2 = int(args[5]) if len(args) > 5 else 32
    seqs = measure(s1, s2, n1, n2)
    # Start from the parametric fits (dark: 3exp(-n/11.1) -> step ~ l*0.086).
    rd = [max(0.004, k * 0.086) for k in KNOTS]
    ru = [0.30, 0.33, 0.30, 0.25, 0.17, 0.10, 0.0]
    rd[0] = 0.0
    ru = [max(v, min_light) for v in ru[:6]] + [0.0]
    best = loss(rd, ru, seqs, wd, wl)
    print(f"start loss {best:.4f} (rms {best ** 0.5:.3f})")
    step = 0.04
    while step > 0.0015:
        improved = True
        while improved:
            improved = False
            for arr, lo_i, hi_i, lo in ((rd, 1, 7, 0.003), (ru, 0, 6, min_light)):
                for i in range(lo_i, hi_i):
                    for dv in (step, -step):
                        old = arr[i]
                        arr[i] = min(3.0, max(lo, old + dv))
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
        at = "  ".join(f"@{k} {sim[k - 1]:.2f}/{meas[k - 1]:.2f}" for k in (1, 2, 4, 10, len(meas)))
        print(f"  {name:4s} rms {rms:.2f}   sim/meas {at}")
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
