/*
 * Per-pixel drive planner (the InkBoy/PaperBoy model, sized for the PW4).
 *
 * Two pixel models:
 *  - dose counting (InkBoy): target shade, the shade the drive started from,
 *    and a count of remaining scans per transition;
 *  - state model (when response curves are calibrated): an estimated
 *    continuous level per pixel, advanced along measured response curves; a
 *    pixel is pulsed only while that brings it closer to its target. This
 *    handles retargeting mid-way without any estimate-and-redose step.
 * Each scan the planner emits a command plane: BLACK or WHITE for pixels that
 * still need drive, HOLD for the rest. The pulse waveform turns those values
 * into one short pulse per scan.
 *
 * Work is tracked in 32x32 tiles so idle parts of the screen cost nothing.
 */
#ifndef KDC_PLAN_H
#define KDC_PLAN_H

#include <stdint.h>
#include "kdc.h"

#define KDC_TILE 32
#define KDC_MAX_BANDS 8
#define KDC_LV_MAX 1023          /* model level scale: 0 = K .. 1023 = W */
#define KDC_LV_N (KDC_LV_MAX + 1)

typedef struct {
	int levels;                 /* 2 (dither) or 4 (gray) */
	uint8_t dose[4][4];         /* scans per transition [from][to] */
	uint8_t retarget_pct;
	int gray_dither;
	uint8_t cmd_black, cmd_white, cmd_hold;
	int model;                  /* use the state model below */
	uint16_t next_dark[KDC_LV_N], next_light[KDC_LV_N]; /* level after one pulse */
	uint16_t tol;               /* settled when within tol of the target level */
	/* Dual pulses (state model only): long pulses from the long slot. */
	int dual;
	uint16_t next_dark_long[KDC_LV_N], next_light_long[KDC_LV_N];
	uint8_t cmd_black_long, cmd_white_long;
	uint16_t long_ratio_x16;    /* long only if its gain > ratio * short gain (x16) */
	uint16_t long_short_max;    /* ...and only where a short pulse gains < this */
} kdc_plan_cfg_t;

typedef struct {
	int w, h;
	int tcols, trows, ntiles;
	kdc_plan_cfg_t cfg;

	uint8_t *prev;              /* last ingested frame */
	uint8_t *tgt, *lvl, *rem, *tot;
	uint16_t *lv;               /* state model: estimated level (model only) */
	uint8_t *dec[4];            /* state model: command for (target shade, level),
	                             * precomputed from the decision rule (4 x 1 KiB) */
	uint8_t *plane;             /* command plane, mirrors what the fb must hold */

	uint8_t *t_active;          /* tile has pixels with rem > 0 */
	uint8_t *t_dirty;           /* fb copy of tile may hold BLACK/WHITE commands */
	uint8_t *t_copy;            /* plane tile rewritten, copy to fb */
	uint8_t *t_long;            /* tile has a long pulse planned this scan */
	int n_active;

	uint8_t thr[8][8];          /* ordered-dither thresholds */
	uint8_t *thr_row[8];        /* thr expanded to full rows (vector kernels) */
	uint8_t *chg_row;           /* scratch: per-pixel changed mask of one row */
	uint8_t *t_chg;             /* scratch: per-tile changed flags of a tile row */
} kdc_plan_t;

int kdc_plan_init(kdc_plan_t *p, int w, int h, const kdc_plan_cfg_t *cfg);

/* Response-curve lookup tables for the state model. Curves, levels 0..3:
 *   dark(n)  = 3 * exp(-(n / dark_tau) ^ dark_p)          (from white)
 *   light(n) = 3 * (1 - exp(-(n / light_tau) ^ light_p))  (from black)
 * A pulse moves a pixel one step along its direction's curve from wherever
 * it currently is. */
void kdc_model_tables(double dark_tau, double dark_p, double light_tau, double light_p,
                      uint16_t *next_dark, uint16_t *next_light);

/* Same tables from per-level steps at levels 0, 0.5, ..., 3 (piecewise
 * linear): one dark pulse moves level l to l - dark_steps(l). */
void kdc_model_tables_steps(const double dark_steps[7], const double light_steps[7],
                            uint16_t *next_dark, uint16_t *next_light);
void kdc_plan_free(kdc_plan_t *p);

/* Panel now shows quantize(frame) exactly (after a clean). */
void kdc_plan_reset(kdc_plan_t *p, const uint8_t *frame);

/* Takes a new frame. tile_damage: per-tile flags, or NULL for all tiles.
 * Returns the number of tiles whose targets changed. */
int kdc_plan_ingest(kdc_plan_t *p, const uint8_t *frame, const uint8_t *tile_damage);

/* Plans the next scan into p->plane. Returns the number of update rects
 * (0 = nothing to drive). band_long[i] (may be NULL) is set when rect i
 * contains long pulses and must go out with the long slot. */
int kdc_plan_build(kdc_plan_t *p, kdc_rect_t *bands, int *band_long, int max_bands);

/* Copies rewritten plane tiles into the framebuffer. */
void kdc_plan_copy(kdc_plan_t *p, uint8_t *fb, int fb_stride);

/* The planned scan was delivered: spend one scan of every driven pixel. */
void kdc_plan_commit(kdc_plan_t *p);

/* Images for the clean sequence. kind 0: the targets (0/255 or 4 grays);
 * kind 1: estimated current state mapped to values the pulse waveform holds
 * (used to sync the EPDC working buffer without driving). */
void kdc_plan_render(const kdc_plan_t *p, int kind, uint8_t *out, int stride);

#endif
