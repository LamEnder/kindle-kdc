#include "kdc_plan.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

static const uint8_t bayer8[64] = {
	 0, 32,  8, 40,  2, 34, 10, 42, 48, 16, 56, 24, 50, 18, 58, 26,
	12, 44,  4, 36, 14, 46,  6, 38, 60, 28, 52, 20, 62, 30, 54, 22,
	 3, 35, 11, 43,  1, 33,  9, 41, 51, 19, 59, 27, 49, 17, 57, 25,
	15, 47,  7, 39, 13, 45,  5, 37, 63, 31, 55, 23, 61, 29, 53, 21,
};

static int model_decide_tables(kdc_plan_t *p);

int kdc_plan_init(kdc_plan_t *p, int w, int h, const kdc_plan_cfg_t *cfg)
{
	memset(p, 0, sizeof *p);
	if (w <= 0 || h <= 0 || (cfg->levels != 2 && cfg->levels != 4))
		return -1;
	p->w = w;
	p->h = h;
	p->cfg = *cfg;
	p->tcols = (w + KDC_TILE - 1) / KDC_TILE;
	p->trows = (h + KDC_TILE - 1) / KDC_TILE;
	p->ntiles = p->tcols * p->trows;
	size_t n = (size_t)w * h;
	p->prev = malloc(n);
	p->tgt = calloc(1, n);
	p->lvl = calloc(1, n);
	p->rem = calloc(1, n);
	p->tot = calloc(1, n);
	p->plane = malloc(n);
	p->t_active = calloc(1, (size_t)p->ntiles);
	p->t_dirty = calloc(1, (size_t)p->ntiles);
	p->t_copy = calloc(1, (size_t)p->ntiles);
	p->t_long = calloc(1, (size_t)p->ntiles);
	if (cfg->model)
		p->lv = calloc(n, sizeof *p->lv);
	if (cfg->model && cfg->history && cfg->levels == 4)
		p->org = calloc(1, n);
	else
		p->cfg.history = 0;
	if (!p->prev || !p->tgt || !p->lvl || !p->rem || !p->tot || !p->plane ||
	    !p->t_active || !p->t_dirty || !p->t_copy || !p->t_long || (cfg->model && !p->lv) ||
	    (p->cfg.history && !p->org)) {
		kdc_plan_free(p);
		return -1;
	}
	memset(p->plane, cfg->cmd_hold, n);
	for (int y = 0; y < 8; y++)
		for (int x = 0; x < 8; x++) {
			int b = bayer8[y * 8 + x];
			p->thr[y][x] = (uint8_t)(cfg->levels == 2 ? b * 4 + 2 : b * 255 / 64);
		}
	p->chg_row = malloc((size_t)w);
	p->t_chg = calloc(1, (size_t)p->tcols);
	for (int r = 0; r < 8; r++) {
		p->thr_row[r] = malloc((size_t)w);
		if (!p->thr_row[r]) break;
		for (int x = 0; x < w; x++)
			p->thr_row[r][x] = cfg->levels == 4 && !cfg->gray_dither ? 127 : p->thr[r][x & 7];
	}
	if (!p->chg_row || !p->t_chg || !p->thr_row[7] || (cfg->model && model_decide_tables(p) < 0)) {
		kdc_plan_free(p);
		return -1;
	}
	return 0;
}

void kdc_plan_free(kdc_plan_t *p)
{
	free(p->prev); free(p->tgt); free(p->lvl); free(p->rem); free(p->tot);
	free(p->plane); free(p->t_active); free(p->t_dirty); free(p->t_copy);
	free(p->lv); free(p->org); free(p->t_long);
	free(p->chg_row); free(p->t_chg);
	for (int t = 0; t < 4; t++) free(p->dec[t]);
	for (int r = 0; r < 8; r++) free(p->thr_row[r]);
	memset(p, 0, sizeof *p);
}

void kdc_model_tables(double dark_tau, double dark_p, double light_tau, double light_p,
                      uint16_t *next_dark, uint16_t *next_light)
{
	for (int l = 0; l < KDC_LV_N; l++) {
		double x = 3.0 * l / KDC_LV_MAX, v;
		/* dark: x = 3 exp(-(n/t)^p)  ->  n = t (ln(3/x))^(1/p) */
		if (l == 0) {
			v = 0;
		} else {
			double n = dark_tau * pow(log(3.0 / x), 1.0 / dark_p);
			v = 3.0 * exp(-pow((n + 1.0) / dark_tau, dark_p));
		}
		next_dark[l] = (uint16_t)lround(v / 3.0 * KDC_LV_MAX);
		/* light: x = 3 (1 - exp(-(n/t)^p))  ->  n = t (-ln(1 - x/3))^(1/p) */
		if (l == KDC_LV_MAX) {
			v = 3.0;
		} else {
			double n = light_tau * pow(-log(1.0 - x / 3.0), 1.0 / light_p);
			v = 3.0 * (1.0 - exp(-pow((n + 1.0) / light_tau, light_p)));
		}
		next_light[l] = (uint16_t)lround(v / 3.0 * KDC_LV_MAX);
		if (next_dark[l] > l) next_dark[l] = (uint16_t)l;     /* never backwards */
		if (next_light[l] < l) next_light[l] = (uint16_t)l;
	}
}

static double interp7(const double *v, double x)
{
	if (x <= 0) return v[0];
	if (x >= 3) return v[6];
	int i = (int)(x / 0.5);
	double t = (x - i * 0.5) / 0.5;
	return v[i] + (v[i + 1] - v[i]) * t;
}

void kdc_model_tables_steps(const double dark_steps[7], const double light_steps[7],
                            uint16_t *next_dark, uint16_t *next_light)
{
	for (int l = 0; l < KDC_LV_N; l++) {
		double x = 3.0 * l / KDC_LV_MAX;
		/* Plain rounding: where the measured step is tiny (the physical floor
		 * near black), the model stalls just like the panel does. Forcing
		 * progress here made the model creep below the floor and keep
		 * pulsing pixels that no longer change. */
		long d = lround((x - interp7(dark_steps, x)) / 3.0 * KDC_LV_MAX);
		long u = lround((x + interp7(light_steps, x)) / 3.0 * KDC_LV_MAX);
		next_dark[l] = (uint16_t)(d < 0 ? 0 : d > l ? l : d);
		next_light[l] = (uint16_t)(u > KDC_LV_MAX ? KDC_LV_MAX : u < l ? l : u);
	}
}

static inline uint16_t shade_lv(const kdc_plan_t *p, uint8_t s)
{
	return (uint16_t)(p->cfg.levels == 2 ? s * KDC_LV_MAX : s * KDC_LV_MAX / 3);
}

static inline uint8_t quantize(const kdc_plan_t *p, int x, int y, uint8_t v)
{
	uint8_t t = p->thr[y & 7][x & 7];
	if (p->cfg.levels == 2)
		return v > t;
	return (uint8_t)((v * 3 + (p->cfg.gray_dither ? t : 127)) / 255);
}

static void quantize_row(const kdc_plan_t *p, int x0, int y, int n, const uint8_t *src, uint8_t *dst)
{
	for (int i = 0; i < n; i++)
		dst[i] = quantize(p, x0 + i, y, src[i]);
}

void kdc_plan_reset(kdc_plan_t *p, const uint8_t *frame)
{
	size_t n = (size_t)p->w * p->h;
	if (frame != p->prev)
		memcpy(p->prev, frame, n);
	for (int y = 0; y < p->h; y++)
		quantize_row(p, 0, y, p->w, frame + (size_t)y * p->w, p->tgt + (size_t)y * p->w);
	memcpy(p->lvl, p->tgt, n);
	if (p->lv)
		for (size_t i = 0; i < n; i++) p->lv[i] = shade_lv(p, p->tgt[i]);
	if (p->org)
		memcpy(p->org, p->tgt, n);
	memset(p->rem, 0, n);
	memset(p->tot, 0, n);
	memset(p->t_active, 0, (size_t)p->ntiles);
	/* The clean left image values in the fb: every tile must be reset to HOLD
	 * before the next pulse scan. */
	memset(p->t_dirty, 1, (size_t)p->ntiles);
	memset(p->plane, p->cfg.cmd_hold, n);
	p->n_active = 0;
}

/* Pixel i gets new target q. */
static inline void retarget(kdc_plan_t *p, size_t i, uint8_t q)
{
	uint8_t t = p->tgt[i];
	if (q == t)
		return;
	uint8_t r = p->rem[i], from;
	if (p->cfg.levels == 2 || r == 0) {
		/* Dither mode restarts the full budget from the old target, like
		 * InkBoy; a settled gray pixel starts from where it is. */
		from = t;
	} else {
		/* Mid-way: estimate the shade reached so far. */
		int l = p->lvl[i], tt = p->tot[i];
		int num = l * tt + (t - l) * (tt - r);
		from = (uint8_t)((2 * num + tt) / (2 * tt));
	}
	int d = p->cfg.dose[from][q];
	if (p->cfg.levels == 4 && r)
		d = (d * p->cfg.retarget_pct + 50) / 100;
	p->tgt[i] = q;
	if (from == q) {
		p->lvl[i] = q;
		p->rem[i] = p->tot[i] = 0;
		return;
	}
	if (d < 1) d = 1;
	if (d > 255) d = 255;
	p->lvl[i] = from;
	p->rem[i] = p->tot[i] = (uint8_t)d;
}

/* Branch-free intake kernels (NEON-vectorized by GCC). chg[i] = 0xFF where
 * the pixel's target changed. */

/* Dither + dose counting: a changed pixel restarts its full budget from the
 * old target (InkBoy's rule); same as retarget() for levels == 2. */
static void ingest_span_dither(kdc_plan_t *p, size_t o, const uint8_t *restrict src,
                               const uint8_t *restrict thr, int n, uint8_t *restrict chg)
{
	uint8_t *restrict tgt = p->tgt + o, *restrict lvl = p->lvl + o;
	uint8_t *restrict rem = p->rem + o, *restrict tot = p->tot + o;
	const uint8_t d01 = p->cfg.dose[0][1], d10 = p->cfg.dose[1][0];
	for (int i = 0; i < n; i++) {
		uint8_t q = src[i] > thr[i];
		uint8_t t = tgt[i];
		uint8_t m = (uint8_t)-(q != t);
		uint8_t d = q ? d01 : d10;
		lvl[i] = (uint8_t)((t & m) | (lvl[i] & ~m));
		rem[i] = (uint8_t)((d & m) | (rem[i] & ~m));
		tot[i] = (uint8_t)((d & m) | (tot[i] & ~m));
		tgt[i] = q;
		chg[i] = m;
	}
}

/* State model: only the target changes; build decides the pulses. */
static void ingest_span_model(kdc_plan_t *p, size_t o, const uint8_t *restrict src,
                              const uint8_t *restrict thr, int n, uint8_t *restrict chg)
{
	uint8_t *restrict tgt = p->tgt + o;
	if (p->cfg.levels == 2) {
		for (int i = 0; i < n; i++) {
			uint8_t q = src[i] > thr[i];
			chg[i] = (uint8_t)-(q != tgt[i]);
			tgt[i] = q;
		}
	} else {
		for (int i = 0; i < n; i++) {
			unsigned v = src[i] * 3u + thr[i];         /* <= 1016 */
			uint8_t q = (uint8_t)((v + 1 + (v >> 8)) >> 8);   /* exact v / 255 */
			chg[i] = (uint8_t)-(q != tgt[i]);
			tgt[i] = q;
		}
	}
}

/* State model with history compensation (4 levels). A pixel whose target
 * goes back to the shade it last settled at (org) gets the route correction
 * for that round trip, scaled by how far it had got toward the shade it is
 * leaving: route[o][v] for a finished trip o -> v -> o, less for a reversal
 * cut short. A pixel that had settled at its old target starts a new trip
 * from there; a return home keeps home as the origin. Scalar: only changed
 * pixels do any work. */
static void ingest_span_history(kdc_plan_t *p, size_t o, const uint8_t *restrict src,
                                const uint8_t *restrict thr, int n, uint8_t *restrict chg)
{
	uint8_t *restrict tgt = p->tgt + o, *restrict org = p->org + o;
	uint16_t *restrict lv = p->lv + o;
	const uint8_t ch = p->cfg.cmd_hold;
	for (int i = 0; i < n; i++) {
		unsigned v = src[i] * 3u + thr[i];
		uint8_t q = (uint8_t)((v + 1 + (v >> 8)) >> 8);   /* exact v / 255 */
		uint8_t t = tgt[i];
		chg[i] = (uint8_t)-(q != t);
		if (q == t)
			continue;
		tgt[i] = q;
		int l = lv[i];
		uint8_t og = org[i];
		int settled = p->dec[t][l] == ch;
		if (og == q && og != t) {
			int r = p->cfg.route[og][t];
			if (r) {
				int lo = shade_lv(p, og), lt = shade_lv(p, t);
				/* progress toward t, 0..1 (x1024) */
				int f = (l - lo) * 1024 / (lt - lo);
				f = f < 0 ? 0 : f > 1024 ? 1024 : f;
				l += r * f / 1024;
				lv[i] = (uint16_t)(l < 0 ? 0 : l > KDC_LV_MAX ? KDC_LV_MAX : l);
			}
		}
		/* A pixel that settled at t and moves on starts its next trip there.
		 * A return home keeps home: a pixel resting on a background that
		 * something crosses again and again stays a pixel of that background
		 * (otherwise it alternates between two routes, column by column). */
		if (settled && og != q)
			org[i] = t;
	}
}

int kdc_plan_ingest(kdc_plan_t *p, const uint8_t *frame, const uint8_t *tile_damage)
{
	int changed_tiles = 0;
	const int model = p->cfg.model, fast = model || p->cfg.levels == 2;
	uint8_t q[KDC_TILE];
	for (int ty = 0; ty < p->trows; ty++) {
		int y0 = ty * KDC_TILE, y1 = y0 + KDC_TILE < p->h ? y0 + KDC_TILE : p->h;
		const uint8_t *dmg = tile_damage ? tile_damage + ty * p->tcols : NULL;
		int tx = 0;
		while (tx < p->tcols) {
			/* run of damaged tiles: whole rows of it go through one kernel call */
			if (dmg && !dmg[tx]) { tx++; continue; }
			int end = tx + 1;
			while (end < p->tcols && (!dmg || dmg[end])) end++;
			int x0 = tx * KDC_TILE, x1 = end * KDC_TILE < p->w ? end * KDC_TILE : p->w, n = x1 - x0;
			memset(p->t_chg + tx, 0, (size_t)(end - tx));
			for (int y = y0; y < y1; y++) {
				size_t o = (size_t)y * p->w + x0;
				if (!memcmp(frame + o, p->prev + o, (size_t)n))
					continue;
				memcpy(p->prev + o, frame + o, (size_t)n);
				if (fast) {
					if (p->org)
						ingest_span_history(p, o, frame + o, p->thr_row[y & 7] + x0, n, p->chg_row);
					else if (model)
						ingest_span_model(p, o, frame + o, p->thr_row[y & 7] + x0, n, p->chg_row);
					else
						ingest_span_dither(p, o, frame + o, p->thr_row[y & 7] + x0, n, p->chg_row);
					for (int t = tx; t < end; t++) {
						const uint8_t *c = p->chg_row + (t - tx) * KDC_TILE;
						int len = t * KDC_TILE + KDC_TILE < x1 ? KDC_TILE : x1 - t * KDC_TILE;
						uint8_t a = 0;
						for (int i = 0; i < len; i++) a |= c[i];
						p->t_chg[t] |= a;
					}
				} else {
					/* gray + dose counting: scalar retarget (mid-way estimate) */
					for (int x = x0; x < x1; x += KDC_TILE) {
						int len = x + KDC_TILE < x1 ? KDC_TILE : x1 - x;
						size_t oo = (size_t)y * p->w + x;
						quantize_row(p, x, y, len, frame + oo, q);
						for (int i = 0; i < len; i++)
							if (q[i] != p->tgt[oo + i]) {
								retarget(p, oo + i, q[i]);
								p->t_chg[x / KDC_TILE] = 1;
							}
					}
				}
			}
			for (int t = tx; t < end; t++) {
				if (!p->t_chg[t])
					continue;
				int ti = ty * p->tcols + t;
				changed_tiles++;
				uint8_t acc = 1;
				if (!fast) {
					/* gray dose counting: a retarget may also settle a pixel */
					int tx0 = t * KDC_TILE, len = tx0 + KDC_TILE < p->w ? KDC_TILE : p->w - tx0;
					acc = 0;
					for (int y = y0; y < y1; y++) {
						const uint8_t *r = p->rem + (size_t)y * p->w + tx0;
						for (int i = 0; i < len; i++)
							acc |= r[i];
					}
				}
				/* model: build decides per scan; dither: a changed pixel always
				 * has a budget >= 1 */
				if (acc && !p->t_active[ti]) { p->t_active[ti] = 1; p->n_active++; }
				if (!acc && p->t_active[ti]) { p->t_active[ti] = 0; p->n_active--; }
			}
			tx = end;
		}
	}
	return changed_tiles;
}

/* Vectorizable command kernel. */
static void plan_span(const kdc_plan_t *p, size_t o, int n, uint8_t *restrict out)
{
	const uint8_t *restrict rem = p->rem + o, *restrict tgt = p->tgt + o, *restrict lvl = p->lvl + o;
	const uint8_t cb = p->cfg.cmd_black, cw = p->cfg.cmd_white, ch = p->cfg.cmd_hold;
	for (int i = 0; i < n; i++) {
		uint8_t c = tgt[i] < lvl[i] ? cb : cw;
		out[i] = rem[i] ? c : ch;
	}
}

#define MIN_GAIN 3   /* model units (1023 = 3 levels) */

/* State-model decision for one pixel: pulse only while a pulse brings its
 * predicted level closer to the target. With dual pulses, a long pulse is
 * chosen only where short pulses are weak (gain < long_short_max, i.e. near
 * the rails) and the long gain beats the short one by long_ratio: a long
 * scan slows every pixel of its update rect (~135 vs ~35 ms on a PW4), so
 * using long pulses for ordinary gray steps collapsed the scan rate.
 * It depends only on (target level, current level), so kdc_plan_init
 * tabulates it once (model_decide_tables) and the per-pixel kernel is a
 * single lookup. */
static uint8_t model_decide(const kdc_plan_cfg_t *cf, int T, int l)
{
	const uint16_t *nd = cf->next_dark, *nl = cf->next_light;
	const uint16_t *ndl = cf->next_dark_long, *nll = cf->next_light_long;
	const int tol = cf->tol, dual = cf->dual, ratio = cf->long_ratio_x16, gsmax = cf->long_short_max;
	int e = l - T;
	/* Black and white targets are overdriven (a third of the tolerance):
	 * extra pulses at the rails cost nothing visually, a gray-ish black does. */
	int tl = (T == 0 || T == KDC_LV_MAX) ? tol / 3 : tol;
	/* A pulse must gain at least MIN_GAIN units (~0.01 level): at the physical
	 * floor near black the predicted gain vanishes and the pixel stops instead
	 * of being pulsed for nothing. */
	if (e > tl) {
		int gs = e - abs(nd[l] - T);              /* error reduction, short */
		if (dual) {
			int gl = e - abs(ndl[l] - T);
			if (gs < gsmax && gl >= MIN_GAIN && gl * 16 > ratio * gs) return cf->cmd_black_long;
		}
		if (gs >= MIN_GAIN) return cf->cmd_black;
	} else if (-e > tl) {
		int gs = -e - abs(nl[l] - T);
		if (dual) {
			int gl = -e - abs(nll[l] - T);
			if (gs < gsmax && gl >= MIN_GAIN && gl * 16 > ratio * gs) return cf->cmd_white_long;
		}
		if (gs >= MIN_GAIN) return cf->cmd_white;
	}
	return cf->cmd_hold;
}

static int model_decide_tables(kdc_plan_t *p)
{
	for (int s = 0; s < p->cfg.levels; s++) {
		p->dec[s] = malloc(KDC_LV_N);
		if (!p->dec[s]) return -1;
		int T = shade_lv(p, (uint8_t)s);
		for (int l = 0; l < KDC_LV_N; l++)
			p->dec[s][l] = model_decide(&p->cfg, T, l);
	}
	return 0;
}

/* State-model command kernel. Returns bit 0: any pulse, bit 1: any long. */
static int model_span(const kdc_plan_t *p, size_t o, int n, uint8_t *restrict out)
{
	const uint8_t *restrict tgt = p->tgt + o;
	const uint16_t *restrict lv = p->lv + o;
	const uint8_t ch = p->cfg.cmd_hold;
	/* Vector pre-pass: does any pixel of the segment need a pulse at all?
	 * (tol/3 = the smallest tolerance the decision uses) */
	{
		const int mul = p->cfg.levels == 2 ? KDC_LV_MAX : KDC_LV_MAX / 3, tmin = p->cfg.tol / 3;
		uint8_t need = 0;
		for (int i = 0; i < n; i++) {
			int e = (int)lv[i] - tgt[i] * mul;
			need |= (uint8_t)((e > tmin) | (-e > tmin));
		}
		if (!need) {
			memset(out, ch, (size_t)n);
			return 0;
		}
	}
	const uint8_t *d0 = p->dec[0], *d1 = p->dec[1], *d2 = p->dec[2], *d3 = p->dec[3];
	if (p->cfg.levels == 2) {
		for (int i = 0; i < n; i++)
			out[i] = (tgt[i] ? d1 : d0)[lv[i]];
	} else {
		for (int i = 0; i < n; i++) {
			uint8_t t = tgt[i];
			out[i] = (t == 0 ? d0 : t == 1 ? d1 : t == 2 ? d2 : d3)[lv[i]];
		}
	}
	/* vectorized reductions */
	const uint8_t cbl = p->cfg.cmd_black_long, cwl = p->cfg.cmd_white_long;
	uint8_t any = 0, lg = 0;
	for (int i = 0; i < n; i++) any |= (uint8_t)(out[i] != ch);
	if (p->cfg.dual)
		for (int i = 0; i < n; i++) lg |= (uint8_t)((out[i] == cbl) | (out[i] == cwl));
	return (any != 0) | ((lg != 0) << 1);
}

typedef struct { int r0, r1, c0, c1; } tband_t;


static int make_bands(const kdc_plan_t *p, tband_t *b, int max_bands)
{
	int n = 0, open = 0;
	int gap = 0;
	for (int ty = 0; ty < p->trows; ty++) {
		int c0 = -1, c1 = -1;
		const uint8_t *row = p->t_active + ty * p->tcols;
		for (int tx = 0; tx < p->tcols; tx++)
			if (row[tx]) { if (c0 < 0) c0 = tx; c1 = tx; }
		if (c0 < 0) {
			if (open && ++gap > 1) open = 0;
			continue;
		}
		if (open) {
			tband_t *cur = &b[n - 1];
			cur->r1 = ty;
			if (c0 < cur->c0) cur->c0 = c0;
			if (c1 > cur->c1) cur->c1 = c1;
		} else {
			if (n == KDC_MAX_BANDS * 4) {
				/* Too many: extend the last band instead. */
				tband_t *cur = &b[n - 1];
				cur->r1 = ty;
				if (c0 < cur->c0) cur->c0 = c0;
				if (c1 > cur->c1) cur->c1 = c1;
			} else {
				b[n++] = (tband_t){ ty, ty, c0, c1 };
			}
			open = 1;
		}
		gap = 0;
	}
	/* Merge neighbors with the least extra area until within max_bands. */
	while (n > max_bands) {
		int best = 0;
		long best_cost = -1;
		for (int i = 0; i + 1 < n; i++) {
			int c0 = b[i].c0 < b[i + 1].c0 ? b[i].c0 : b[i + 1].c0;
			int c1 = b[i].c1 > b[i + 1].c1 ? b[i].c1 : b[i + 1].c1;
			long u = (long)(b[i + 1].r1 - b[i].r0 + 1) * (c1 - c0 + 1);
			long a = (long)(b[i].r1 - b[i].r0 + 1) * (b[i].c1 - b[i].c0 + 1) +
			         (long)(b[i + 1].r1 - b[i + 1].r0 + 1) * (b[i + 1].c1 - b[i + 1].c0 + 1);
			if (best_cost < 0 || u - a < best_cost) { best_cost = u - a; best = i; }
		}
		tband_t *x = &b[best], *y = &b[best + 1];
		x->r1 = y->r1;
		if (y->c0 < x->c0) x->c0 = y->c0;
		if (y->c1 > x->c1) x->c1 = y->c1;
		memmove(y, y + 1, (size_t)(n - best - 2) * sizeof *y);
		n--;
	}
	return n;
}

int kdc_plan_build(kdc_plan_t *p, kdc_rect_t *bands, int *band_long, int max_bands)
{
	if (max_bands < 1) max_bands = 1;
	if (max_bands > KDC_MAX_BANDS) max_bands = KDC_MAX_BANDS;

	if (p->cfg.model) {
		/* Per tile: plan, and retire tiles that need no more drive. */
		for (int ti = 0; ti < p->ntiles; ti++) {
			if (!p->t_active[ti] && !p->t_dirty[ti])
				continue;
			int ty = ti / p->tcols, tx = ti % p->tcols;
			int y0 = ty * KDC_TILE, y1 = y0 + KDC_TILE < p->h ? y0 + KDC_TILE : p->h;
			int x0 = tx * KDC_TILE, n = x0 + KDC_TILE < p->w ? KDC_TILE : p->w - x0;
			int any = 0;
			for (int y = y0; y < y1; y++) {
				size_t o = (size_t)y * p->w + x0;
				if (p->t_active[ti])
					any |= model_span(p, o, n, p->plane + o);
				else
					memset(p->plane + o, p->cfg.cmd_hold, (size_t)n);
			}
			p->t_long[ti] = (any & 2) != 0;
			if (p->t_active[ti] && !(any & 1)) {
				p->t_active[ti] = 0;
				p->n_active--;
			}
			p->t_copy[ti] = 1;
		}
		goto bands;
	}

	/* Plane: active tiles get commands, dirty idle tiles go back to HOLD.
	 * Walk runs of consecutive tiles of the same kind for long rows. */
	for (int ty = 0; ty < p->trows; ty++) {
		int y0 = ty * KDC_TILE, y1 = y0 + KDC_TILE < p->h ? y0 + KDC_TILE : p->h;
		int tx = 0;
		while (tx < p->tcols) {
			int ti = ty * p->tcols + tx;
			int kind = p->t_active[ti] ? 2 : p->t_dirty[ti] ? 1 : 0;
			int end = tx + 1;
			while (end < p->tcols) {
				int tj = ty * p->tcols + end;
				int k2 = p->t_active[tj] ? 2 : p->t_dirty[tj] ? 1 : 0;
				if (k2 != kind) break;
				end++;
			}
			if (kind) {
				int x0 = tx * KDC_TILE;
				int x1 = end * KDC_TILE < p->w ? end * KDC_TILE : p->w;
				for (int y = y0; y < y1; y++) {
					size_t o = (size_t)y * p->w + x0;
					if (kind == 2)
						plan_span(p, o, x1 - x0, p->plane + o);
					else
						memset(p->plane + o, p->cfg.cmd_hold, (size_t)(x1 - x0));
				}
				memset(p->t_copy + ty * p->tcols + tx, 1, (size_t)(end - tx));
			}
			tx = end;
		}
	}

bands:
	if (!p->n_active)
		return 0;
	tband_t tb[KDC_MAX_BANDS * 4];
	int n = make_bands(p, tb, max_bands);
	for (int i = 0; i < n; i++) {
		int x0 = tb[i].c0 * KDC_TILE, y0 = tb[i].r0 * KDC_TILE;
		int x1 = (tb[i].c1 + 1) * KDC_TILE, y1 = (tb[i].r1 + 1) * KDC_TILE;
		if (x1 > p->w) x1 = p->w;
		if (y1 > p->h) y1 = p->h;
		bands[i] = (kdc_rect_t){ x0, y0, x1 - x0, y1 - y0 };
		if (band_long) {
			int lg = 0;
			if (p->cfg.dual)
				for (int ty = tb[i].r0; ty <= tb[i].r1 && !lg; ty++)
					for (int tx = tb[i].c0; tx <= tb[i].c1 && !lg; tx++)
						lg = p->t_long[ty * p->tcols + tx] && p->t_active[ty * p->tcols + tx];
			band_long[i] = lg;
		}
	}
	return n;
}

void kdc_plan_copy(kdc_plan_t *p, uint8_t *fb, int fb_stride)
{
	for (int ty = 0; ty < p->trows; ty++) {
		int y0 = ty * KDC_TILE, y1 = y0 + KDC_TILE < p->h ? y0 + KDC_TILE : p->h;
		uint8_t *cp = p->t_copy + ty * p->tcols;
		int tx = 0;
		while (tx < p->tcols) {
			if (!cp[tx]) { tx++; continue; }
			int end = tx;
			while (end < p->tcols && cp[end]) {
				int ti = ty * p->tcols + end;
				p->t_dirty[ti] = p->t_active[ti];
				cp[end] = 0;
				end++;
			}
			int x0 = tx * KDC_TILE, x1 = end * KDC_TILE < p->w ? end * KDC_TILE : p->w;
			for (int y = y0; y < y1; y++)
				memcpy(fb + (size_t)y * fb_stride + x0, p->plane + (size_t)y * p->w + x0, (size_t)(x1 - x0));
			tx = end;
		}
	}
}

/* Vectorizable commit kernel: spend one scan of every driven pixel and OR
 * the remaining counts into acc[] (per column, reduced per tile later). */
static void commit_span(kdc_plan_t *p, size_t o, int n, uint8_t *restrict acc)
{
	uint8_t *restrict rem = p->rem + o, *restrict lvl = p->lvl + o;
	const uint8_t *restrict tgt = p->tgt + o;
	for (int i = 0; i < n; i++) {
		uint8_t r = rem[i];
		uint8_t nr = (uint8_t)(r - (r != 0));
		uint8_t keep = (uint8_t)-(nr != 0);     /* branch-free so it vectorizes */
		rem[i] = nr;
		lvl[i] = (uint8_t)((lvl[i] & keep) | (tgt[i] & ~keep));
		acc[i] |= nr;
	}
}

/* Model commit for one row segment: table steps only where a pulse was sent;
 * all-HOLD segments (the common case once pixels settle) are skipped by a
 * vectorized test first. */
static void model_commit_span(kdc_plan_t *p, size_t o, int n)
{
	const uint8_t *restrict c = p->plane + o;
	const uint8_t ch = p->cfg.cmd_hold;
	uint8_t any = 0;
	for (int i = 0; i < n; i++) any |= (uint8_t)(c[i] != ch);
	if (!any)
		return;
	const uint16_t *nd = p->cfg.next_dark, *nl = p->cfg.next_light;
	const uint16_t *ndl = p->cfg.next_dark_long, *nll = p->cfg.next_light_long;
	const uint8_t cb = p->cfg.cmd_black, cw = p->cfg.cmd_white;
	const uint8_t cbl = p->cfg.cmd_black_long, cwl = p->cfg.cmd_white_long;
	const int dual = p->cfg.dual;
	uint16_t *lv = p->lv + o;
	for (int i = 0; i < n; i++) {
		uint8_t ci = c[i];
		if (ci == ch) continue;
		if (ci == cb) lv[i] = nd[lv[i]];
		else if (ci == cw) lv[i] = nl[lv[i]];
		else if (dual && ci == cbl) lv[i] = ndl[lv[i]];
		else if (dual && ci == cwl) lv[i] = nll[lv[i]];
	}
}

void kdc_plan_commit(kdc_plan_t *p)
{
	/* Runs of active tiles per tile row: long rows through the kernels. */
	for (int ty = 0; ty < p->trows; ty++) {
		int y0 = ty * KDC_TILE, y1 = y0 + KDC_TILE < p->h ? y0 + KDC_TILE : p->h;
		const uint8_t *act = p->t_active + ty * p->tcols;
		int tx = 0;
		while (tx < p->tcols) {
			if (!act[tx]) { tx++; continue; }
			int end = tx + 1;
			while (end < p->tcols && act[end]) end++;
			int x0 = tx * KDC_TILE, x1 = end * KDC_TILE < p->w ? end * KDC_TILE : p->w, n = x1 - x0;
			if (p->cfg.model) {
				/* Tile activity is re-decided by the next build. */
				for (int y = y0; y < y1; y++)
					model_commit_span(p, (size_t)y * p->w + x0, n);
			} else {
				uint8_t *acc = p->chg_row + x0;
				memset(acc, 0, (size_t)n);
				for (int y = y0; y < y1; y++)
					commit_span(p, (size_t)y * p->w + x0, n, acc);
				for (int t = tx; t < end; t++) {
					const uint8_t *a = acc + (t - tx) * KDC_TILE;
					int len = t * KDC_TILE + KDC_TILE < x1 ? KDC_TILE : x1 - t * KDC_TILE;
					uint8_t any = 0;
					for (int i = 0; i < len; i++) any |= a[i];
					if (!any) {
						p->t_active[ty * p->tcols + t] = 0;
						p->n_active--;
					}
				}
			}
			tx = end;
		}
	}
}

void kdc_plan_render(const kdc_plan_t *p, int kind, uint8_t *out, int stride)
{
	static const uint8_t show2[2] = { 0x00, 0xFF }, show4[4] = { 0x00, 0x55, 0xAA, 0xFF };
	/* Sync values sit in 2..13 (4-bit), the range both pulse slots hold
	 * (0, 1, 14, 15 are pulse codes). */
	static const uint8_t sync2[2] = { 0x22, 0xDD }, sync4[4] = { 0x22, 0x55, 0xAA, 0xDD };
	const uint8_t *map = p->cfg.levels == 2 ? (kind ? sync2 : show2) : (kind ? sync4 : show4);
	for (int y = 0; y < p->h; y++) {
		size_t o = (size_t)y * p->w;
		uint8_t *dst = out + (size_t)y * stride;
		if (kind && p->lv) {
			/* model estimate, mapped into the held range 2..13 (4-bit) */
			for (int x = 0; x < p->w; x++)
				dst[x] = (uint8_t)(17 * (2 + (p->lv[o + x] * 11 + KDC_LV_MAX / 2) / KDC_LV_MAX));
			continue;
		}
		for (int x = 0; x < p->w; x++) {
			uint8_t s = p->tgt[o + x];
			if (kind && p->rem[o + x]) {
				int l = p->lvl[o + x], tt = p->tot[o + x], r = p->rem[o + x];
				int num = l * tt + (s - l) * (tt - r);
				s = (uint8_t)((2 * num + tt) / (2 * tt));
			}
			dst[x] = map[s];
		}
	}
}
