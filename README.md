# kdc: per-pixel pulse-drive display controller for the Kindle PaperWhite 4

kdc drives the PW4's e-ink panel the way [InkBoy](https://github.com/kuslota/InkBoy)
drives its SSD1681 panel. InkBoy's approach comes from PaperBoy. kdc doesn't run
a full stock waveform (A2/DU/GL16) over every pixel of a region. Instead it loads
a custom **pulse waveform** that is only a few EPDC frames long, and sends a
command plane every scan:

| fb value | pulse waveform does |
| --- | --- |
| `0x00` | one pulse toward black |
| `0xFF` | one pulse toward white |
| anything else (`0x88`) | HOLD: no drive |

A planner keeps per-pixel state: the target shade, the shade the current drive
started from, and the scans still owed. Only pixels that still need to move are
pulsed; everything else holds. A pixel that changes target mid-way is
retargeted without disturbing its neighbors.

It runs entirely in userspace on the **stock kernel**. It needs no kernel module
and never writes to flash.

> [!WARNING]
> **Experimental. Use at your own risk.** Custom waveforms and fast refresh can
> cause image retention, and their long-term effect on the panel is unknown. This
> is the same caveat InkBoy carries. kdc was built and tested on a host
> simulator. **It has not been run on a real PW4 yet.** Take the first run
> slowly (see [First run on the device](#first-run-on-the-device)). A reboot
> always restores the stock waveform.

---

## What the PW4 kernel allows (findings from the GPL source)

Everything below was read from `Kindle_src_5.18.1.1.1` → `linux-4.1.15`,
`drivers/video/fbdev/mxc/mxc_epdc_v2_fb.c` + `mxc_epdc_fb_lab126.c` (PW4 =
i.MX6SLL, board "moonshine").

| Finding | Where | Used for |
| --- | --- | --- |
| Writing `reload` to `/proc/eink/waveform/source` reloads the waveform from `/mnt/wfm/waveform_to_use/*.wrf` at runtime. | `proc_wfm_source_write` → `mxc_epdc_fb_load_new_fw` | Installing the pulse waveform without a custom kernel |
| `/proc/eink/waveform/data` dumps the loaded waveform byte for byte (header + data). | `proc_wfm_data_read` | Saving the stock waveform; checking the install |
| Reload rewrites `EPDC_WVADDR`. If the new file is larger, it frees the DMA buffer **with the last recorded size, not the allocated size**. A stock restore (shrink) followed by a larger pulse file (grow) **hard-froze a PW4** (2026-11-07, reboot reason `PWRON_LONGPRESS`). | `mxc_epdc_fb_load_new_fw` | The pulse mode is patched **in place** in the DU4 slot's own phase blocks: same file size, so no reallocation ever. `kdc-pulse.sh` refuses any reload that would change the size |
| Waveform layout: mode table → temperature table → `u64 nframes` + frames. 5-bit frames are 512 bytes; the entry index is `(ext<<8)\|(new<<4)\|old`, and the values are 0 none, 1 black, 2 white. | verified on `firmware/imx/epdc/epdc_ED060KC4.fw` (DU, A2, GC16) | Building the pulse mode |
| The file checksum is zlib CRC-32 over the file, with the checksum field zeroed. | `crc32()` in lab126.c | Writing a valid header |
| Waveform requests are remapped (`get_waveform_by_type`), and the resolved slot is copied back to userspace by `MXCFB_SEND_UPDATE`. | `mxc_epdc_fb_send_single_update` | `kdc-wfm probe` finds the slot DU4 really uses, so nothing is guessed |
| The night-mode slots (GCK16/GLKW16) switch the frontlight and the PMIC night pin. | `fl_switch`, `mode_sel` | Why kdc uses DU4, not a night slot |
| `MXCFB_WAIT_FOR_UPDATE_COMPLETE`/`_SUBMISSION` block **while holding `fb_info->lock`**, so a waiting thread stalls every other fb ioctl on the system. | `do_fb_ioctl` + driver | Only the display thread submits and waits |
| The default scheme is QUEUE_AND_MERGE: the fb is read when the update is *processed*, and pending updates may be merged into their bounding box. | `upd_scheme` init, merge code | The fb only changes after completion, and idle tiles always hold HOLD |
| Partial GC16 is silently turned into GL16. | `convert_gc16_to_gl16` | Cleans use FULL GC16 |
| At low battery with low temperature, Lab126 runs a pixel pre-processing pass ("SW mitigation") on non-REAGL updates. | `is_SW_mitigation_needed` | kdc refuses to start or stops in that range |
| `MXCFB_SET_PWRDOWN_DELAY` keeps the EPDC rails up between updates. | ioctl | Power stays on between scans (InkBoy's "powering up once") |

## How kdc maps InkBoy onto the EPDC

| InkBoy (SSD1681) | kdc (PW4 EPDC) |
| --- | --- |
| Volatile LUT via command 0x32 | Pulse mode written in place over the DU4 slot's own phase blocks of a reloaded copy of the stock waveform (same size; the other modes are byte-identical) |
| Two image planes → HOLD/BLACK/WHITE | One 8-bit plane, decoded by the waveform: new=0 → black, new=15 → white, else hold; FULL update mode so repeated commands keep pulsing |
| One-frame waveform (~14–20 ms) | `frames` drive frames + one 0 V frame per scan, **scaled per temperature range** from the stock DU timing (e.g. 10 frames at 0 °C, 3 at 24 °C, 2 at 43 °C on the ED060KC4 sample) |
| Black/white pulse budgets | `budget_black`, `budget_white` (dither mode) |
| 12-transition dose table, retargeting | `dose[4][4]`, `retarget_pct` (gray4 mode), same estimate-and-redose rule |
| Latest-frame triple buffer, old frames never queued | Same; damage from skipped frames is merged |
| Panel power kept on | `keep_power_ms` via `MXCFB_SET_PWRDOWN_DELAY`, restored on close |
| 2-core planner | 32×32 tile activity tracking; idle tiles cost nothing; NEON-vectorized command and commit kernels |
| — | Up to `max_bands` update rectangles per scan, so separate moving areas don't make the EPDC process the whole screen |

**Scan pipeline** (one display thread):

```
ingest newest frame → build plane N+1        (CPU, while the panel runs scan N)
wait scan N → copy changed tiles to fb → submit bands of N+1 (FULL, DU4) → commit
```

**Invariant:** every fb pixel outside the tiles being driven holds the HOLD value.
Driver alignment or queue merging can therefore never pulse a pixel the planner
didn't ask for. The host tests check this by exact pulse counting, including with
`max_bands = 1`, which forces merged regions over idle tiles.

**Clean** (`kdc_clean`, open, close): kdc first writes its estimate of the panel
state as values the pulse mode holds. That syncs the EPDC working buffer without
driving anything. Then it runs a FULL GC16 of the targets. After `kdc_close` the
working buffer matches the screen, so the Kindle UI's partial updates are correct
again.

## Build

```sh
make            # host build with ASan/UBSan + tests (simulated panel)
make test WFM=some.wrf   # also patch-and-verify a real waveform file
make dist       # cross build → build/kindle/kdc-kindle.tar.gz
make vecreport  # shows the NEON-vectorized loops
```

The cross build uses `../x-tools/arm-kindlehf-linux-gnueabihf` (koxtoolchain
`kindlehf`, firmware 5.16.3+). Override it with `KINDLE_TC=` or `CROSS=`.

## First run on the device

1. Copy `kdc-kindle.tar.gz` to the Kindle and unpack it (for example to
   `/mnt/us/kdc`). Stop KOReader or anything else that is refreshing the screen.
2. **Look before installing anything:**
   ```sh
   cd /mnt/us/kdc
   ./kdc-wfm probe                       # prints du4=N du=M
   cat /proc/eink/waveform/data > /tmp/stock.wrf
   ./kdc-wfm info /tmp/stock.wrf         # modes, temperatures, frame counts
   ```
   Check that `info` reports a 5-bit waveform with a valid checksum. A valid
   checksum also confirms the CRC code matches Lab126's. Check that the slot
   `probe` printed for du4 exists. If something looks off, stop here and
   send me the output.
3. Install the pulse waveform (as root):
   ```sh
   ./kdc-pulse.sh install 3              # 3 drive frames per scan at ~25 °C
   ./kdc-pulse.sh status
   ```
   The script saves the stock waveform to `/var/local/kdc/stock.wrf`, mounts a
   tmpfs over `/mnt/wfm/waveform_to_use`, reloads, and verifies the loaded bytes.
   If verification fails, it reloads the stock waveform.
4. Try it, starting with a short run:
   ```sh
   ./kdc-demo -v -s 10 bounce
   ```
   The demo prints scans/s, average scan time and the pulse frames in use.
5. Restore when you are done (a reboot does the same):
   ```sh
   ./kdc-pulse.sh restore
   ```

While the pulse waveform is installed, anything else on the device that requests
DU4 gets pulses instead. Run `restore` before going back to the normal UI.

## Using the library

```c
#include "kdc.h"

kdc_config_t c;
kdc_config_default(&c);           /* dither mode, automatic budgets */
kdc_t *k;
if (kdc_open(&k, &c) != KDC_OK) { puts(kdc_last_error(k)); kdc_close(k); return; }

for (;;) {
    uint8_t *f = kdc_frame_begin(k, 0);      /* width*height, 0=black 255=white */
    draw(f);
    kdc_rect_t dmg = { x, y, w, h };
    kdc_frame_publish(k, &dmg, 1);           /* never blocks */
}
kdc_clean(k);      /* optional: full GC16 to remove accumulated ghosting */
kdc_close(k);      /* waits, cleans, restores power settings */
```

For gmplay, replace `canvas_present` with: scale and rotate the luma into
`kdc_frame_begin(k, 0)`, then `kdc_frame_publish`. Don't dither yourself; kdc
quantizes with a fixed 8×8 Bayer pattern tied to screen positions, so a still
image stays still. Overdrive and update waits go away. Frames the panel can't
keep up with are dropped at the mailbox, and only the latest one is driven.

## Tuning

| Knob | Where | Effect |
| --- | --- | --- |
| drive frames per scan | `kdc-pulse.sh install N` (1–8) | Pulse length. Shorter gives more scans/s and finer gray steps, but more per-update overhead. |
| `budget_black/white` | dither mode | Scans per change; `0` = automatic for the installed pulse length. Higher means deeper black/white and fewer trails, but softer motion. |
| `dose[4][4]`, `retarget_pct` | gray4 mode | Pulses per transition at the installed pulse length; calibrate with `kdc-cal`. The default is InkBoy's table, tuned for a different panel. |
| `route O V BIAS` | `.doses` file, gray4 + state model | History correction for the round trip O → V → O, in levels; fit with `tools/cal/route_fit.py`. Keep it small next to one pulse step. |
| `max_bands` | 1–8 | More rectangles per scan means less area processed, but more ioctls. |
| `min_scan_interval_us` | | Caps the scan rate. |
| `keep_power_ms` | | EPDC power-down delay during the session. |
| `min_battery_pct` | | The mitigation guard. `-1` disables it; not recommended. |

Pulses are temperature-compensated in the waveform itself, so budgets and doses
don't need changing with temperature.

## Pulse length settings (measured on a PW4, 20 °C)

Pick one at install time. The dither budget follows the installed pulse length
automatically (`budget = 0`, the default), so the drive stays matched.

| Setting | Install | Auto budget | Time per scan | Scans/s | Full black↔white change |
| --- | --- | --- | --- | --- | --- |
| Strong | `kdc-pulse.sh install 3` | 6 | ~64 ms | ~13.6 | ~390 ms |
| Balanced | `kdc-pulse.sh install 2` | 9 | ~53 ms | ~16.5 | ~480 ms |
| Fast | `kdc-pulse.sh install 1` | 18 | ~33 ms | ~25 | ~590 ms |

Measured with `kdc-demo bounce` (30 fps producer); all ran with 0 errors.
Shorter pulses update more often and show more frames, but each full change
takes longer, because every scan carries ~9 ms of driver overhead plus one 0 V
tail frame. The pulse is temperature-scaled: at 20 °C, `install 3` gives 4
frames per scan. Measured runs and budgets:

| Install | Budget | ms/scan | Scans/s | Frames shown / dropped (11 s) |
| --- | --- | --- | --- | --- |
| 3 | 6 | 64.7 | 13.6 | 142 / 159 |
| 3 | 4 | 63.6 | 13.9 | 145 / 156 |
| 2 | 8 | 53.3 | 16.3 | 171 / 130 |
| 2 | 5 | 51.9 | 17.0 | 178 / 123 |
| 1 | 24 | 33.0 | 24.7 | 256 / 45 |
| 1 | 14 | 32.2 | 26.2 | 272 / 29 |

Visually checked: `install 3` with budget 6 gave clean motion, full black and a
still background. All six runs looked acceptable at a glance.

## Performance work (PW4, single Cortex-A9 core)

Planner CPU per scan, full-screen content where every pixel changes every
frame (`kdc-bench`, on the Kindle; pure CPU):

| Mode | Initial | Now |
| --- | --- | --- |
| Dither, dose counting (video) | 85.4 ms | **33.1 ms** |
| Gray, state model | 162.4 ms | **70.7 ms** |
| Dither, state model | 117.6 ms | **61.8 ms** |

What did it: whole-row NEON kernels for frame intake (quantize, compare,
retarget, branch-free) and commit, a vector "anything to do?" pre-pass that
skips settled segments, and the state model's decision tabulated per (target,
level) so the per-pixel work is one lookup.

**Pipeline.** A helper thread waits for scan completion (that ioctl blocks
while holding the fb core's lock). Meanwhile the display thread waits for a
new frame or the end of the scan, and plans the next scan as soon as a frame
arrives, while the panel is still running. One intake and one plan per scan;
a frame arriving mid-scan is planned at once, so it is never held back a
whole scan, which was the cause of earlier stutter. Producers that are
CPU-bound can call `kdc_frame_pending()` and skip preparing frames that would
only be replaced.

gmplay with kdc (`-K`, video preset `install 3`), Bad Apple, 30 s:

| | Panel updates | New frames/s |
| --- | --- | --- |
| FBInk A2 | 136 | 4.6 |
| kdc, first integration | 316 | 4.0 (stutter) |
| kdc, wait-then-take | 229 | 8.1 |
| kdc, + vectorized planner | 240 | 8.7 |
| kdc, + overlapped planning | 303 | **10.9** |

A full-screen scan now takes ~93 ms, of which ~45 ms is driver/PXP overhead
independent of the pulse length, so full-screen video is panel-bound. Smaller
moving areas scan much faster (~33–65 ms).

## Gray mode (state model)

With response curves calibrated, kdc doesn't count doses. Every pixel keeps an
estimated level (0 = K … 3 = W, 10-bit), and each scan it is pulsed only if
the pulse brings that estimate closer to the target. The estimate is advanced
along measured per-level pulse steps. Retargeting a pixel mid-way therefore needs no
special handling: it continues from wherever the model says it is. Black and
white targets are overdriven (a third of the stop tolerance).

Calibrated presets (PW4, 17–19 °C), made with `kdc-cal` ladders and phone
photos (see `tools/cal/README.md`):

| Preset | Install | Pulses | Best for |
| --- | --- | --- | --- |
| `presets/pw4_install1.doses` | `kdc-pulse.sh install 1` | 1-frame short pulses, history corrections | the default: ~20 scans/s, best trails |
| `presets/pw4_install1_long3.doses` | `kdc-pulse.sh install 1 3` | short + long (3 frames at 25 °C, 4 at 18 °C), history corrections | dual pulses at ~11 scans/s |
| `presets/pw4_install1_long6.doses` | `kdc-pulse.sh install 1 6` | short + long (6 frames at 25 °C, 9 at 18 °C) | deepest blacks, ~9 scans/s |
| `presets/pw4_install2.doses` | `kdc-pulse.sh install 2` | 2-frame pulses | ~14 scans/s, coarse grays (not recommended) |
| `presets/pw4_install3.doses` | `kdc-pulse.sh install 3` | 3-frame pulses | not usable for gray; use `pw4_install3_video.doses` (dither) |

```sh
kdc-pulse.sh install 1 6
kdc-demo -g -D pw4_install1_long6.doses transition     # InkBoy-style transition test
```

`kdc_open` refuses a preset whose `install` line doesn't match the loaded
waveform.

Transition test results (mean |trail − band shade| in photo luminance; the
D–L step is ~45; lower is better):

| Planner | Mean trail error | Worst row | Scans/s |
| --- | --- | --- | --- |
| Dose counting (calibrated doses) | 18.1 | −59.3 | ~22 |
| State model, parametric curves | 9.8 | −20.2 | ~22 |
| State model, joint step fit + endpoint overdrive (`install1`) | 6.6 | +14.5 | ~20 |
| Dual pulses (`install1_long6`) | **5.0** | −12.3 | ~9 |
| Dual, shorter long pulse (`install1_long3`) | 6.2 | +14.2 | ~11 |
| 2-frame pulses (`install2`) | 11.4 | −44.3 | ~14 |
| 3-frame pulses (`install3`) | 36.2 | −81.5 | ~10 |

These numbers are from different days, lighting and photos; compare within a
row group, not across. On 2026-11-08 the same `install1` test scored 8.8.

**Longer pulses are coarser.** One 2-frame pulse moves a pixel about a whole
level (W → 2.3 → 1.5; K → 0.8 → 1.8), a 3-frame pulse more than a level
(W → 1.6). The model cannot land between steps, and pixels pulsed deep into
black lighten more slowly than the ladders predict, so trails behind moving
objects get worse with every frame added.

**History compensation.** A pixel that leaves shade O for V and is sent back
to O comes back off by a repeatable amount: it has a history, which the
Markov-in-level model ignores. Like InkBoy's route corrections, kdc keeps per
pixel the shade it last settled at (its origin; a return home keeps home) and,
when the target goes back to that origin, adds `route O V` (levels) to the
estimated level, scaled by how far the pixel had got toward V. This covers
finished trips and reversals cut short with one table. The 12 transition-test
rows are exactly the 12 routes, so `tools/cal/route_fit.py` turns each test
photo into updated `route` lines; a few rounds converge to the run-to-run
noise.

| Transition test, 2026-11-08 | No routes | With routes |
| --- | --- | --- |
| `install1`, the 9 rows not returning to black | 6.4 | **1.9** (3 rounds) |
| `install1`, D→K→D (black crossing dark gray) | −13.0 | −6.5 … +3.8 |
| `install1_long3`, the 9 rows not returning to black | 5.6 | **2.4** (2 rounds) |

Returns to black stay uncorrected: short pulses stop at the black floor
whatever the model believes. Routes need pulses that are small compared with
the correction. With `install 2` they flipped the error (D→K→D −44 → +44) or
left pixels stuck: once the model believes a pixel is where every possible
pulse would overshoot, it holds, and with L→D→L +0.65 the trail stayed dark
gray. That preset has no routes.

**Dual pulses.** With 1-frame pulses, darkening near black is very weak (~0.02
level per pulse below level 0.5). `install N L` also patches the DU slot with a
dual waveform: codes 0/15 are long pulses of L frames, and codes 1/14 are the
same short pulses as the DU4 slot. One long pulse takes white to ~0.85 and
black to ~2.6, against ~2.8 and ~0.3 for a short one. The planner picks a long
pulse only where short pulses are weak (gain < 0.05 level, i.e. near the rails)
and the long one gains more than 3× as much. A scan with any long pulse is sent
entirely with the DU slot. Both pulse types stop at the physical black floor
(~0.25 level above GC16 black) instead of pulsing pixels that no longer change.

Why one waveform per scan: the driver's queue merge
(`epdc_submit_merge`) turns touching pending updates with *different*
waveforms into `WAVEFORM_MODE_AUTO`, and AUTO draws the command plane as an
image with a stock waveform. A dual run with mixed waveforms per scan showed
exactly that (a large wrongly driven rectangle). With the EPDC debug log on
(`echo 1 > .../20f4000.epdc/mxc_epdc_debug`; only 0 and 1 are safe, the other
values switch the panel rails), the fixed build sent only DU/DU4 updates.

## Gray video (gmplay `-g`)

Tried 2026-11-08 on French Cancan, against kdc black/white dither
(`pw4_install3_video.doses`, 7.6 frames/s) and FBInk gray (1.4 frames/s):

| Preset | Frames/s | Notes |
| --- | --- | --- |
| `pw4_install1.doses` | 3.9 | 1-frame steps: a big change takes 15–40 scans |
| `pw4_install1_long3_video.doses` | 6.4 | long pulses wherever they gain 1.5× a short one, `settle_tol 0.2`, `rail_tol 0.3`, no routes: ≤ 8 scans per change |

Grays look clearly better than FBInk gray, but black/white dither still wins
on the PW4: faster transitions and a better-looking picture. gmplay's
launcher keeps `pw4_install3_video.doses`.

Video-only `.doses` keys: `long_ratio X`, `long_short_max X` (3 = long pulses
anywhere), `settle_tol X`, `rail_tol X` (black/white stop tolerance; default
`settle_tol / 3`).

## Limitations and what is unverified

- Verified on one PW4 (fw 5.18.1.1.1, ED060KC4 panel): waveform layout, slot
  probe (du4=7, du=1), in-place reloads, dither and gray modes.
- Only 5-bit waveforms are supported. `kdc-wfm` refuses others.
- The gray state model is open-loop (Markov in level plus route corrections
  for round trips). It fits the ladders to 0.06–0.18 (short) / 0.04–0.08
  (long) levels rms. A clean resets accumulated error. Presets are calibrated
  at 17–19 °C only. Short pulses cannot reach GC16 black (floor ~0.25 level),
  so black on gray and trails in black areas stay slightly light. Frame drops
  leave faint column stripes where corrected routes cross.
- 8 bpp framebuffers only, which is what the PW4 uses. Night mode (inverted
  gray) is handled by swapping commands.
- If the battery/temperature guard trips mid-session, kdc runs a clean and stops
  driving. `kdc_frame_publish` then returns `KDC_ERR_STOPPED`.

## Other Kindles

The Paperwhite 5 (11th gen, MediaTek MT8110 `hwtcon` driver) has everything
the approach needs (runtime waveform reload, per-pixel old/new lookup, mxcfb
ioctls) but a different waveform format and update struct. See
[`docs/pw5-port.md`](docs/pw5-port.md) for the analysis and a port plan.

## Files

| Path | What |
| --- | --- |
| `include/kdc.h` | Public API |
| `src/kdc.c` | Mailbox, display thread, scan pipeline, cleans, safety checks |
| `src/kdc_plan.c` | Per-pixel planner, tiles, bands (the hot loops) |
| `src/kdc_dev.c` | mxcfb ioctls, plus the simulated panel |
| `src/kdc_wfm.c` | Waveform parse, pulse patch, CRC |
| `src/mxcfb_pw4.h` | PW4 ioctl ABI (Lab126 80-byte `mxcfb_update_data`) |
| `tools/kdc-wfm.c` | `info`, `probe`, `patch`, `verify` |
| `tools/kdc-demo.c` | `bounce`, `transition` (gray test) or raw gray video from stdin |
| `tools/kdc-cal.c` | Gray calibration ladders (`stage1`, `stage2`, `--long`, `clear`) |
| `tools/cal/` | Photo analysis and model fitting (Python + Pillow) |
| `presets/*.doses` | Calibrated presets (gray `install1`/`install2`/`install3`, dual `install1_long3`/`install1_long6`, video `install3_video`, gray video `install1_long3_video`) |
| `tools/kdc-bench.c` | Planner CPU benchmark (safe to run on the Kindle) |
| `scripts/kdc-session.sh` | Pause / resume the Kindle UI across commands |
| `scripts/kdc-pulse.sh` | Install (`install N [L]`) / restore the pulse waveform on the device |
| `tests/test_host.c` | Waveform and controller tests on the simulator |

## License

MIT, see [LICENSE](LICENSE). Exception: `src/mxcfb_pw4.h` holds interface
definitions copied from the Linux kernel's `mxcfb.h` (© Freescale) and stays
under GPL-2.0-or-later.

kdc writes a custom waveform into the panel controller at runtime. Use at your
own risk: custom waveforms and fast refresh can cause image retention, and
their long-term effect on the panel is unknown.
