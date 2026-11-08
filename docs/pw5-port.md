# Porting kdc to the Kindle Paperwhite 5 (11th gen)

Findings from Amazon's GPL release `Kindle_src_5.19.2_4670760040` (linux-4.9,
`arch/arm/configs/mt8110-bellatrix_defconfig`). Nothing here has been tried on
a PW5 yet.

**Verdict: the userspace approach carries over.** The PW5 has the same
building blocks that made kdc possible on the PW4: a runtime waveform reload
from userspace, a per-pixel old/new lookup per frame, mxcfb-style ioctls with
a waveform mode per update, and framebuffer values that reach the lookup
unchanged if they are multiples of 0x11. The waveform format and the ioctl
struct differ, so a port is a new backend, not a recompile.

## What's different

| | PW4 (i.MX6SLL, `mxc_epdc_v2_fb`) | PW5 (MT8110, `hwtcon` / `hwtcon_v2`) |
| --- | --- | --- |
| Driver | `drivers/video/fbdev/mxc` | `drivers/misc/mediatek/hwtcon_v2` (and `hwtcon`); both built as modules, so check `lsmod` on the device for the one in use |
| Reload | `echo reload > /proc/eink/waveform/source`; file from `/mnt/wfm/waveform_to_use/` (kdc mounts a tmpfs over it) | `echo reload PATH > /proc/hwtcon/cmd`: any path, no mount needed. The loaded path is the module parameter `waveform_path` (default `/data/init_bin/wf_lut.gz`) |
| Size rule | never change the size (grow-after-shrink froze the device) | the file is read into a fixed reserved buffer. A `.gz` is bounded by it; a raw `.bin` is copied with **no bounds check**. Keep the patched file exactly the decompressed size |
| Format | E Ink `.wrf`: 5-bit, 512-byte frames, CRC32 | MediaTek `wf_lut`: header (name at 0x00, versions at 0x50/0x53/0x56, temperature thresholds at 0x80, count at 0xA2), then per temperature (32) × mode (16) a 32-bit frame-data offset at `0xC0 + t*0x40 + 4*m` and a byte length at `0x8C0 + t*0x40 + 4*m` (frames = len / 0x100), byte order per `wf_lut_convert_order`. A frame is 256 bytes = 16 old × 16 new (4-bit) |
| PMIC | VCOM etc. not in the file | `fiti_setting_get_from_waveform` reads PMIC settings from the header: **patch frame data and length fields only** |
| `mxcfb_update_data` | 80 bytes | 96 bytes (Lab126 `swipe_data`, `hist_*`, `ts_pxp`, `ts_epdc`) |
| Pulse slot | DU4 (7) | DU4 has **no hardware slot** (it falls back to slot 0, INIT). Use a mode the UI rarely sends: A2 (6), GC16_PARTIAL (10) or P2SW (13); DU (1) for the long pulses |
| Pixel path | fb Y8 → EPDC | fb Y8 → MDP (rotation, color map, optional dither) → Y4 → WF_LUT. The default color map rounds to 16 levels (`round(v/17)`), so kdc's commands (0x00, 0x11 … 0x88 … 0xFF) arrive unchanged. Never set the dither flags |
| Merges | different waveforms merge into AUTO | same, and a **collision** is resubmitted as PARTIAL + AUTO: keep one scan in flight (as kdc already does) |

## Port plan

1. On a PW5: `lsmod | grep hwtcon`, `cat /sys/module/hwtcon*/parameters/waveform_path`,
   copy the `wf_lut.gz` off the device and decompress it.
2. Work out the frame entry encoding from a known mode (GC16/DU frames:
   which bits mean none/black/white for each old/new byte).
3. `kdc-wfm`: a `wf_lut` backend. In place, for one mode slot: write pulse
   frames (every old row the same; new 0/1 → black, 14/15 → white, else
   none), set its per-temperature length to the scaled pulse length, keep
   everything else byte-identical. Long pulses go into a second mode the same
   way.
4. `src/mxcfb_pw5.h` (96-byte struct) and a device switch in `kdc_dev.c`;
   `kdc-pulse.sh`: write the raw `.bin` to a tmpfs, `reload PATH`, and restore
   with `reload <original waveform_path>`.
5. Recalibrate gray presets (different panel and controller).

Unknowns to check on hardware: whether the WF_LUT engine skips pixels with
old == new in FULL updates (it shouldn't: `rg_partial_up_en` is 0 in all
configs), the per-scan overhead of the MDP + cmdq pipeline (it bounds the scan
rate), and whether `reload` while the UI is paused is as safe as on the PW4.
