"""History (route) corrections from a kdc-demo transition-test photo.

usage: route_fit.py photo.jpg DOSES_FILE [gain=0.8]

Each transition-test row is a round trip: band shade O -> rect shade V -> O.
Its trail error (trail - band, photo brightness) is converted to levels with
the local brightness step between shades (from the GC16 band strips and
reference squares in the same photo) and added, times gain, to the current
"route O V" value of DOSES_FILE. A trail that comes back too dark gives a
negative route: kdc then believes such pixels are darker than it thought and
drives them lighter. Prints the new route lines; paste them into the file,
rerun the test, repeat until the trails stop improving.
"""
import sys
sys.path.insert(0, __file__.rsplit("/", 1)[0])
import transition_measure as T

NAMES = "KDLW"


def read_routes(path):
    r = {}
    for line in open(path):
        w = line.split()
        if len(w) == 4 and w[0] == "route":
            r[(NAMES.index(w[1]), NAMES.index(w[2]))] = float(w[3])
    return r


def main():
    photo, doses = sys.argv[1], sys.argv[2]
    gain = float(sys.argv[3]) if len(sys.argv) > 3 else 0.8
    rows = T.measure(photo, quiet=True)
    # brightness of each shade: its band strips and its reference squares
    acc = {s: [] for s in range(4)}
    for r in rows:
        acc[r["band"]].append(r["gap"])
        acc[r["rect_shade"]].append(r["ref"])
    B = [sum(v) / len(v) for v in (acc[s] for s in range(4))]
    print("# shade brightness K D L W: " + " ".join(f"{b:.1f}" for b in B))
    if not all(b1 > b0 + 5 for b0, b1 in zip(B, B[1:])):
        sys.exit("shades are not ordered in the photo; check the debug overlay of transition_measure.py")
    # brightness per level around each shade (one-sided at the rails)
    slope = [B[1] - B[0], (B[2] - B[0]) / 2, (B[3] - B[1]) / 2, B[3] - B[2]]
    old = read_routes(doses)
    tot = 0
    print(f"# trail errors (levels) and new routes, gain {gain}")
    for r in rows:
        o, v = r["band"], r["rect_shade"]
        e = (r["trail"] - r["gap"]) / slope[o]
        tot += abs(r["trail"] - r["gap"])
        new = old.get((o, v), 0.0) + gain * e
        new = max(-1.5, min(1.5, new))
        print(f"route {NAMES[o]} {NAMES[v]} {new:+.2f}    # {NAMES[o]}>{NAMES[v]}>{NAMES[o]}: "
              f"trail {e:+.2f} levels, was {old.get((o, v), 0.0):+.2f}")
    print(f"# mean |trail-gap| = {tot / len(rows):.1f}")


main()
