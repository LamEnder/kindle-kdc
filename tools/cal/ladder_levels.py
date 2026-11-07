"""Patch levels (0=K .. 3=W) of a kdc-cal ladder photo, any N.

usage: ladder_levels.py PHOTO stage1|stage2 N [debug.png]

Anchors: the D (0x55) and L (0xAA) stripes under each patch, plus white and
black references taken from the blank title-row area of blocks with that
background (stage 1: W block = white, K block = black; stage 2 has only white
backgrounds, so black uses the stage-1 ratio passed via KDC_CAL_RK, default
-1.20). Prints one line per block: the level after 1..N pulses.
"""
import os
import sys
sys.path.insert(0, __file__.rsplit("/", 1)[0])
import ladder_measure as L
from PIL import Image, ImageDraw


def measure(photo, stage, n, debug=None, rk=None, quiet=False):
    """Returns {title: [level after 1..n pulses]}."""
    img = Image.open(photo)
    g = img.convert("L")
    tl, tr, bl, br = L.find_screen(img)
    h = L.homography([(0, 0), (L.SW, 0), (0, L.SH), (L.SW, L.SH)], [tl, tr, bl, br])
    nb = 2 if stage == "stage1" else 4
    titles = ["W>", "K>"] if stage == "stage1" else ["D>W", "D>K", "L>W", "L>K"]
    dbg = img.copy()
    dr = ImageDraw.Draw(dbg)

    def box(x0, y0, x1, y1, col=(255, 0, 0)):
        dr.polygon([L.apply(h, x0, y0), L.apply(h, x1, y0), L.apply(h, x1, y1), L.apply(h, x0, y1)], outline=col)
        return L.mean(g, h, x0, y0, x1, y1, 2)

    def stripes(by, i):
        x, y = L.patch_pos(by, i)
        sy = y + L.PATCH
        return (box(x + 6, sy + 6, x + L.PATCH - 6, sy + L.STRIPE_H - 6, (0, 160, 255)),
                box(x + 6, sy + L.STRIPE_H + 6, x + L.PATCH - 6, sy + 2 * L.STRIPE_H - 6, (0, 160, 255)))

    # anchors as ratios of (bg - D) / (L - D), from title rows
    ratios = {}
    for k in range(nb):
        by = 8 + k * L.block_h(n)
        bg = box(300, by + 6, 1000, by + 22, (0, 200, 0))
        ds, ls = zip(*(stripes(by, i) for i in range(5, min(n, 12) + 1)))
        d, l = sum(ds) / len(ds), sum(ls) / len(ls)
        ratios[k] = (bg - d) / (l - d)
    if stage == "stage1":
        RW, RK = ratios[0], ratios[1]
    else:
        RW = sum(ratios.values()) / len(ratios)
        RK = rk if rk is not None else float(os.environ.get("KDC_CAL_RK", "-1.20"))
    if not quiet:
        print(f"anchors: K = D {RK:+.2f}*(L-D), W = D {RW:+.2f}*(L-D)")
    out = {}

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

    for k in range(nb):
        by = 8 + k * L.block_h(n)
        lv = []
        for i in range(1, n + 1):
            x, y = L.patch_pos(by, i)
            p = box(x + 14, y + 14, x + L.PATCH - 14, y + L.PATCH - 14)
            d, l = stripes(by, i)
            lv.append(level(p, d, l))
        out[titles[k]] = lv
        if not quiet:
            print(f"{titles[k]:4s} " + " ".join(f"{v:.2f}" for v in lv))
    if debug:
        dbg.save(debug)
    return out, RK


if __name__ == "__main__":
    measure(sys.argv[1], sys.argv[2], int(sys.argv[3]), sys.argv[4] if len(sys.argv) > 4 else None)
