/*
 * kdc - per-pixel pulse-drive display controller for the Kindle PaperWhite 4.
 *
 * The app draws 8-bit grayscale frames. A display thread turns each frame
 * into per-pixel targets and drives only the pixels that still have to move,
 * a few EPDC frames at a time, with a custom "pulse" waveform (installed with
 * scripts/kdc-pulse.sh). See README.md for the design and the safety notes.
 *
 * Thread safety: kdc_frame_begin/kdc_frame_publish are for one producer
 * thread. The other calls may come from any thread.
 */
#ifndef KDC_H
#define KDC_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct kdc kdc_t;

typedef struct {
	int x, y, w, h;
} kdc_rect_t;

typedef enum {
	KDC_MODE_DITHER = 0, /* 1-bit ordered dither, pulse budgets (InkBoy "dithered") */
	KDC_MODE_GRAY4 = 1,  /* 4 shades by partial doses (InkBoy "grayscale") */
} kdc_mode_t;

typedef struct {
	const char *fb_path;        /* "/dev/fb0" */
	kdc_mode_t mode;

	/* KDC_MODE_DITHER: scans a pixel is driven after it changes, 1..60.
	 * 0 (default) = automatic: 18 reference-temperature frames of drive,
	 * divided by the installed pulse length (install 3 -> 6, 2 -> 9, 1 -> 18). */
	uint8_t budget_black, budget_white;

	/* KDC_MODE_GRAY4: pulses per transition, [from][to], 0=black .. 3=white,
	 * for the installed pulse length (calibrate with kdc-cal). The default is
	 * InkBoy's table, a starting point only. */
	uint8_t dose[4][4];
	uint8_t retarget_pct;       /* dose scale when a pixel changes target mid-way */
	/* State model (preferred when calibrated): measured response curves, in
	 * levels 0 (K) .. 3 (W), n = pulses of the installed length:
	 *   dark(n)  = 3 exp(-(n/dark_tau)^dark_p)        from white
	 *   light(n) = 3 (1 - exp(-(n/light_tau)^light_p)) from black
	 * Every pixel tracks an estimated level; it is pulsed only while a pulse
	 * brings it closer to its target. Set by kdc_config_load_doses ("dark",
	 * "light" lines). 0 = off (dose counting). Applies to both modes. */
	double dark_tau, dark_p, light_tau, light_p;
	/* Alternative, preferred when set: per-level pulse steps (levels moved by
	 * one pulse) at levels 0, 0.5, ..., 3, linearly interpolated; fitted
	 * jointly to all calibration ladders ("dark_steps"/"light_steps" lines). */
	double dark_steps[7], light_steps[7];
	int have_steps;
	/* Dual pulses ("install N L": long pulses in the DU slot). Per-level steps
	 * of one LONG pulse ("dark_long_steps"/"light_long_steps" lines). When set,
	 * the planner may pick a long pulse where it beats long_ratio short ones
	 * (default 3; a long scan costs ~4 short scans). Needs the state model. */
	double dark_long_steps[7], light_long_steps[7];
	int have_long_steps;
	int long_pulse_frames;      /* L of "install N L" the steps were calibrated for */
	double long_ratio;          /* "long_ratio X" in a .doses file */
	double long_short_max;      /* levels; long only where a short pulse gains less (0.05);
	                             * "long_short_max X". 3 = anywhere a long pulse wins
	                             * (video: fewer scans per change) */
	double settle_tol;          /* levels; stop driving when this close (0.1);
	                             * "settle_tol X" */
	double rail_tol;            /* same for black and white targets; 0 = settle_tol / 3
	                             * (overdrive). "rail_tol X". Video presets loosen
	                             * both: far fewer scans per change. */
	/* History compensation (KDC_MODE_GRAY4 with the state model): a pixel
	 * that left shade o for shade v and is sent back to o comes back wrong by
	 * a repeatable amount (it has a history). route[o][v], in levels, is
	 * added to its estimated level at that moment, scaled by how far it had
	 * got toward v: negative = it comes back too dark, so drive it lighter.
	 * "route O V BIAS" lines (shades K D L W); tools/cal/route_fit.py fits
	 * them from transition-test photos. have_routes = any set. */
	double route[4][4];
	int have_routes;

	int dose_pulse_frames;      /* install setting dose[] was calibrated for
	                             * (kdc-pulse.sh install N); 0 = uncalibrated.
	                             * kdc_open refuses a mismatch. When it matches,
	                             * automatic dither budgets use dose[W][K] and
	                             * dose[K][W]. */
	int gray_dither;            /* ordered dither between the 4 shades */

	int max_bands;              /* update rectangles per scan, 1..8 */
	int min_scan_interval_us;   /* 0 = as fast as the panel allows */
	int keep_power_ms;          /* EPDC power-down delay during the session */
	int clean_on_open;          /* full GC16 refresh to establish a reference */
	int clean_on_close;
	int min_battery_pct;        /* refuse below this (Lab126 SW-mitigation guard) */
	int verbose;

	/* Host testing: no /dev/fb0, a simulated panel instead. */
	int sim;
	int sim_width, sim_height;
} kdc_config_t;

typedef struct {
	uint64_t scans;             /* pulse updates submitted (one per scan) */
	uint64_t updates;           /* ioctl updates (bands) submitted */
	uint64_t frames_in;         /* frames ingested by the display thread */
	uint64_t frames_dropped;    /* published but replaced before ingest */
	uint64_t cleans;
	uint64_t submit_errors;
	double avg_scan_ms;         /* submit -> complete, last 64 scans */
	double avg_plan_ms;         /* ingest + build, last 64 scans */
	int active_tiles;
	int pulse_slot;             /* waveform slot that DU4 requests resolve to */
	int pulse_frames;           /* drive frames per scan at current temperature */
	int pulse_frames_ref;       /* installed pulse length (frames at ~25 C) */
	int budget_black, budget_white; /* effective dither budgets (dose counting) */
	int model;                  /* 1 = state model with response curves */
	int dual;                   /* 1 = long pulses in use */
	int history;                /* 1 = route (history) corrections in use */
	uint64_t long_updates;      /* updates sent with the long slot */
	int temperature_c;
} kdc_stats_t;

enum {
	KDC_OK = 0,
	KDC_ERR_IO = -1,            /* framebuffer open/mmap/ioctl failed */
	KDC_ERR_FORMAT = -2,        /* framebuffer is not 8 bpp grayscale */
	KDC_ERR_NO_PULSE = -3,      /* pulse waveform not installed */
	KDC_ERR_BATTERY = -4,       /* battery/temperature would trigger Lab126 mitigation */
	KDC_ERR_ARG = -5,
	KDC_ERR_NOMEM = -6,
	KDC_ERR_STOPPED = -7,       /* controller hit a fatal error, see kdc_last_error */
};

void kdc_config_default(kdc_config_t *c);

/* Loads a dose file (see presets/): an "install N" line and a 4x4 matrix,
 * rows = from K D L W, columns = to K D L W, diagonal 0. Sets dose[] and
 * dose_pulse_frames. Returns 0, or -1 with a message in err. */
int kdc_config_load_doses(kdc_config_t *c, const char *path, char *err, size_t errlen);

/* Opens the framebuffer, checks that the pulse waveform is installed, does
 * the reference refresh and starts the display thread. Returns KDC_OK or a
 * KDC_ERR_*. On failure *out may still be set so kdc_last_error() can explain;
 * always pass it to kdc_close(). */
int kdc_open(kdc_t **out, const kdc_config_t *c);
void kdc_close(kdc_t *k);

int kdc_width(const kdc_t *k);
int kdc_height(const kdc_t *k);

/* Buffer to draw the next frame into (width*height bytes, stride = width,
 * 0 = black, 255 = white). With keep != 0 it starts as a copy of the last
 * published frame; otherwise its content is undefined. */
uint8_t *kdc_frame_begin(kdc_t *k, int keep);

/* Hands the frame to the display thread. damage lists the changed areas
 * (NULL/0 = whole frame). Never blocks: an unconsumed older frame is
 * replaced and its damage is merged. */
int kdc_frame_publish(kdc_t *k, const kdc_rect_t *damage, int ndamage);

/* 1 while a published frame has not been taken by the display thread yet.
 * Producers that are CPU-bound (e.g. video on the PW4's single core) can skip
 * preparing frames while this is set instead of preparing frames that would
 * only be replaced. */
int kdc_frame_pending(kdc_t *k);

/* Full GC16 refresh of the current targets (removes accumulated ghosting).
 * Asynchronous; the display thread runs it before the next scan. */
int kdc_clean(kdc_t *k);

/* Waits until every pixel has received its full drive. timeout_ms < 0 waits
 * forever. Returns 0, or 1 on timeout. */
int kdc_flush(kdc_t *k, int timeout_ms);

void kdc_get_stats(kdc_t *k, kdc_stats_t *s);
const char *kdc_last_error(kdc_t *k);

/* Simulator only: estimated reflectance per pixel, 0..255 (NULL on device). */
const uint8_t *kdc_sim_optical(kdc_t *k);
/* Simulator only: total pixel-pulses the simulated panel received. */
uint64_t kdc_sim_pulses(kdc_t *k);

#ifdef __cplusplus
}
#endif
#endif
