# Calibration from phone photos

Python 3 + Pillow (`python3 -m venv venv && venv/bin/pip install pillow`).
Run with `python3 -I`; the scripts import each other from this folder.

| Script | Input | Output |
| --- | --- | --- |
| `ladder_measure.py PHOTO stage1\|stage2 N [debug.png]` | photo of `kdc-cal` ladders | per-patch brightness vs the D/L stripes under it |
| `fit_curves.py S1.jpg S2.jpg WD WL [out]` | both ladder photos | parametric curves (`dark TAU P`, `light TAU P`) + stage-2 check |
| `fit_joint.py S1.jpg S2.jpg WD WL` | both ladder photos | `dark_steps` / `light_steps` fitted to all six ladders (preferred) |
| `transition_measure.py PHOTO [debug.png]` | photo of `kdc-demo -g -k transition` | trail vs band error per row, streak index, rect vs reference |
| `pgm2png.py in.pgm out.png [step]` | `KDC_CAL_SIM=x.pgm kdc-cal ...` | preview of a ladder rendered by the simulator |

The screen is located automatically: the threshold sits just above the bezel
brightness, and each scan line uses its first/last long bright run, so small
bright objects next to the Kindle are ignored. Always look at the debug
overlay before trusting numbers.

Workflow (UI paused with `kdc-session.sh pause`, pulse installed):

1. `kdc-cal stage1` → photo → `fit_curves.py`/`ladder_measure.py` give W→D (WD) and W→L (WL).
2. `kdc-cal stage2 WD WL` → photo.
3. `fit_joint.py stage1.jpg stage2.jpg WD WL` → put `dark_steps`/`light_steps` in a `.doses` file.
4. `kdc-demo -g -D FILE -k transition` → photo → `transition_measure.py` (mean |trail-gap|; lower is better).
5. `kdc-cal clear`, `kdc-pulse.sh restore`, `kdc-session.sh resume`.
