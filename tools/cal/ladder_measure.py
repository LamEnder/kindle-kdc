"""Measure kdc-cal ladder patches from a phone photo.

usage: ladder_measure.py photo.jpg stage1|stage2 N [debug.png]

Finds the screen edges (bright screen vs dark bezel), fits a homography from
screen pixels (1072x1448) to the photo, then samples the inner part of every
patch and of the D (0x55) / L (0xAA) stripe segments under it.
"""
import sys
from PIL import Image, ImageDraw

SW, SH = 1072, 1448
PER_LINE, MARGIN, PITCH, PATCH = 16, 16, 65, 52
TITLE_H, LABEL_H, STRIPE_H = 28, 18, 22
LINE_H = LABEL_H + PATCH + 2 * STRIPE_H + 14


def lines_for(n):
    return (n + PER_LINE - 1) // PER_LINE


def block_h(n):
    return TITLE_H + lines_for(n) * LINE_H + 10


def patch_pos(by, i):
    line, col = (i - 1) // PER_LINE, (i - 1) % PER_LINE
    return MARGIN + col * PITCH, by + TITLE_H + line * LINE_H + LABEL_H


# ---- geometry -----------------------------------------------------------
def solve(a, b):
    n = len(a)
    m = [row[:] + [b[i]] for i, row in enumerate(a)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(m[r][c]))
        m[c], m[p] = m[p], m[c]
        for r in range(n):
            if r != c:
                f = m[r][c] / m[c][c]
                for k in range(c, n + 1):
                    m[r][k] -= f * m[c][k]
    return [m[i][n] / m[i][i] for i in range(n)]


def homography(src, dst):
    a, b = [], []
    for (x, y), (u, v) in zip(src, dst):
        a.append([x, y, 1, 0, 0, 0, -u * x, -u * y]); b.append(u)
        a.append([0, 0, 0, x, y, 1, -v * x, -v * y]); b.append(v)
    h = solve(a, b)
    return h + [1.0]


def apply(h, x, y):
    d = h[6] * x + h[7] * y + h[8]
    return (h[0] * x + h[1] * y + h[2]) / d, (h[3] * x + h[4] * y + h[5]) / d


def fit_line(pts):  # x = a*y + b  (or y = a*x + b with swapped input)
    n = len(pts)
    sy = sum(p[0] for p in pts); sx = sum(p[1] for p in pts)
    syy = sum(p[0] * p[0] for p in pts); sxy = sum(p[0] * p[1] for p in pts)
    a = (n * sxy - sy * sx) / (n * syy - sy * sy)
    return a, (sx - a * sy) / n


def robust_line(pts, iters=3):
    for _ in range(iters):
        a, b = fit_line(pts)
        res = sorted(abs(p[1] - (a * p[0] + b)) for p in pts)
        cut = res[int(len(res) * 0.8)] + 1
        pts = [p for p in pts if abs(p[1] - (a * p[0] + b)) <= cut]
    return fit_line(pts)


def find_screen(img):
    g = img.convert("L")
    w, h = g.size
    px = g.load()
    # Adaptive threshold: the bezel is the darkest large area at the photo
    # border; the screen (even a black GC16 area under the frontlight) is
    # clearly brighter than it.
    border = sorted(px[x, y] for x in range(0, w, 7) for y in list(range(0, h // 40)) + list(range(h - h // 40, h)))
    bezel = border[len(border) // 2]
    thr = max(60, bezel + 40)

    lmin = max(40, int(min(w, h) * 0.03))

    def span(seq):
        """First start / last end of bright runs at least lmin long. Short
        bright things (objects beyond the bezel, glints) are ignored."""
        first = last = None
        run = 0
        for k, v in enumerate(seq):
            if v > thr:
                run += 1
                if run == lmin:
                    if first is None:
                        first = k - lmin + 1
                if run >= lmin:
                    last = k
            else:
                run = 0
        return first, last

    top, bot, left, right = [], [], [], []
    for x in range(int(w * 0.2), int(w * 0.8), 6):
        a, b = span([px[x, y] for y in range(h)])
        if a is not None:
            top.append((x, a)); bot.append((x, b))
    for y in range(int(h * 0.1), int(h * 0.9), 6):
        a, b = span([px[x, y] for x in range(w)])
        if a is not None:
            left.append((y, a)); right.append((y, b))

    # Lines crossing a dark block report an edge too far in: keep the
    # outermost points of each edge.
    def outer(pts, small):
        vals = sorted(p[1] for p in pts)
        ref = vals[len(vals) // 10] if small else vals[-len(vals) // 10 - 1]
        return [p for p in pts if abs(p[1] - ref) < min(w, h) * 0.03]
    top, bot = outer(top, True), outer(bot, False)
    left, right = outer(left, True), outer(right, False)
    lt, lb = robust_line(top), robust_line(bot)          # y = a*x + b
    ll, lr = robust_line(left), robust_line(right)       # x = a*y + b

    def cross(hl, vl):  # y = a1 x + b1 ; x = a2 y + b2
        a1, b1 = hl; a2, b2 = vl
        y = (a1 * b2 + b1) / (1 - a1 * a2)
        return a2 * y + b2, y
    return cross(lt, ll), cross(lt, lr), cross(lb, ll), cross(lb, lr)


def mean(g, h, x0, y0, x1, y1, step=2):
    s = n = 0
    for y in range(y0, y1, step):
        for x in range(x0, x1, step):
            u, v = apply(h, x, y)
            s += g.getpixel((int(u), int(v)))
            n += 1
    return s / n


def main():
    photo, stage, n = sys.argv[1], sys.argv[2], int(sys.argv[3])
    img = Image.open(photo)
    tl, tr, bl, br = find_screen(img)
    h = homography([(0, 0), (SW, 0), (0, SH), (SW, SH)], [tl, tr, bl, br])
    g = img.convert("L")
    nb = 2 if stage == "stage1" else 4
    titles = ["W>", "K>"] if stage == "stage1" else ["D>W", "D>K", "L>W", "L>K"]
    dbg = img.copy()
    dr = ImageDraw.Draw(dbg)
    print(f"screen corners in photo: {[(round(x), round(y)) for x, y in (tl, tr, bl, br)]}")
    results = {}
    for k in range(nb):
        by = 8 + k * block_h(n)
        rows = []
        for i in range(1, n + 1):
            x, y = patch_pos(by, i)
            inset = 14
            p = mean(g, h, x + inset, y + inset, x + PATCH - inset, y + PATCH - inset)
            sy = y + PATCH
            dref = mean(g, h, x + 6, sy + 6, x + PATCH - 6, sy + STRIPE_H - 6)
            lref = mean(g, h, x + 6, sy + STRIPE_H + 6, x + PATCH - 6, sy + 2 * STRIPE_H - 6)
            rows.append((i, p, dref, lref))
            for (a, b, c, d) in [(x + inset, y + inset, x + PATCH - inset, y + PATCH - inset)]:
                pts = [apply(h, a, b), apply(h, c, b), apply(h, c, d), apply(h, a, d)]
                dr.polygon(pts, outline=(255, 0, 0))
        results[titles[k]] = rows
        print(f"\n{titles[k]}  patch  value   D-ref  L-ref   (patch - D)  (patch - L)")
        for i, p, dref, lref in rows:
            print(f"      {i:3d}  {p:6.1f}  {dref:6.1f} {lref:6.1f}   {p - dref:+7.1f}     {p - lref:+7.1f}")
    if len(sys.argv) > 4:
        dbg.save(sys.argv[4])
    return results


if __name__ == "__main__":
    main()
