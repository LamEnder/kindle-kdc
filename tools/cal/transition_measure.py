"""Measure a kdc-demo transition-test photo.

usage: transition_measure.py photo.jpg [debug.png]

Per band and row: trail (swept area) vs the untouched band strips directly
above/below it (same band shade, drawn by GC16), and the rectangle at the
right end vs its GC16 reference square at the left edge. Also a streak index:
std-dev of per-column means along the trail.
"""
import sys
from PIL import Image, ImageDraw
sys.path.insert(0, __file__.rsplit("/", 1)[0])
import ladder_measure as L

W, H = 1072, 1448
BH, RW, RH, REFX, X0 = H // 4, 60, 70, 10, 76
X1 = W - RW - 12
NAMES = "KDLW"


def rows():
    out = []
    for b in range(4):
        j = 0
        for s in range(4):
            if s != b:
                out.append((b, s, b * BH + 30 + j * (RH + 40)))
                j += 1
    return out


def measure(path, debug=None, quiet=False):
    """Per row (band, rect): dict with trail, gap, streak, rect, ref."""
    img = Image.open(path)
    g = img.convert("L")
    tl, tr, bl, br = L.find_screen(img)
    h = L.homography([(0, 0), (W, 0), (0, H), (W, H)], [tl, tr, bl, br])
    dbg = img.copy()
    dr = ImageDraw.Draw(dbg)

    def box(x0, y0, x1, y1, color=(255, 0, 0)):
        dr.polygon([L.apply(h, x0, y0), L.apply(h, x1, y0), L.apply(h, x1, y1), L.apply(h, x0, y1)], outline=color)
        return L.mean(g, h, x0, y0, x1, y1, 3)

    out = []
    if not quiet:
        print("band row  | trail  gap   trail-gap | streak | rect   ref   rect-ref")
    for b, s, y in rows():
        ty0, ty1 = y + 12, y + RH - 12
        tx0, tx1 = X0 + RW + 30, X1 - 30
        trail = box(tx0, ty0, tx1, ty1)
        gap_hi = box(tx0, y - 30, tx1, y - 12, (0, 160, 255)) if y - 30 > b * BH else None
        gap_lo = box(tx0, y + RH + 10, tx1, y + RH + 28, (0, 160, 255))
        gaps = [v for v in (gap_hi, gap_lo) if v is not None]
        gap = sum(gaps) / len(gaps)
        cols = [L.mean(g, h, x, ty0, x + 8, ty1, 3) for x in range(tx0, tx1 - 8, 8)]
        m = sum(cols) / len(cols)
        streak = (sum((c - m) ** 2 for c in cols) / len(cols)) ** 0.5
        rect = box(X1 + 12, y + 12, X1 + RW - 12, y + RH - 12, (0, 200, 0))
        ref = box(REFX + 10, y + 12, REFX + 40, y + RH - 12, (0, 200, 0))
        out.append(dict(band=b, rect_shade=s, trail=trail, gap=gap, streak=streak, rect=rect, ref=ref))
        if not quiet:
            print(f" {NAMES[b]}    {NAMES[s]}   | {trail:5.1f} {gap:5.1f}  {trail - gap:+6.1f}   | {streak:5.1f}  | {rect:5.1f} {ref:5.1f}  {rect - ref:+6.1f}")
    if not quiet:
        print(f"mean |trail-gap| = {sum(abs(r['trail'] - r['gap']) for r in out) / len(out):.1f}")
    if debug:
        dbg.save(debug)
    return out


if __name__ == "__main__":
    measure(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else None)
