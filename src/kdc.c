/*
 * kdc controller: frame mailbox, display thread, scan pipeline, cleans.
 *
 * Pipeline per scan (one display thread, so the blocking EPDC waits, which
 * hold the global fb_info lock in this kernel, never stall a submit):
 *
 *   ingest newest frame -> build plane N+1      (CPU, while scan N runs)
 *   wait scan N complete -> copy plane -> submit bands of N+1 -> commit
 *
 * The fb outside planned tiles always holds HOLD values, so whatever region
 * the driver ends up processing (alignment, queue merging) never drives a
 * pixel the planner did not ask for.
 */
#define _GNU_SOURCE
#include "kdc.h"

#include <errno.h>
#include <math.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "kdc_dev.h"
#include "kdc_plan.h"
#include "kdc_wfm.h"
#include "mxcfb_pw4.h"

#define WFM_PROC "/proc/eink/waveform/data"
#define CMD_HOLD 0x88
#define RING 64

struct kdc {
	kdc_config_t cfg;
	kdc_dev_t dev;
	kdc_plan_t plan;
	int w, h;

	/* pulse waveform */
	int slot, du_slot;
	kdc_pulse_info_t pulse;
	kdc_wfm_info_t wfm;         /* only trt/temps used after open */
	uint8_t trt[KDC_PULSE_MAX_TEMPS];

	/* frame mailbox: back (producer), mail, front (display thread) */
	uint8_t *buf[3];
	uint8_t *dmg[3];            /* per-tile damage flags of each buffer */
	int back, mail, front, latest, mail_fresh;
	uint8_t *thread_dmg;

	pthread_mutex_t mu;
	pthread_cond_t wake, idle_cv;
	pthread_t thread;
	int thread_started, stop, clean_req, idle, failed;
	/* scan-completion waiter (blocks in the driver so the display thread can
	 * plan the next scan while the panel runs) */
	pthread_t waiter;
	pthread_cond_t wcv;
	uint32_t wait_markers[KDC_MAX_BANDS];
	int wait_n, scan_done, waiter_stop, waiter_started;
	char err[256];

	uint8_t *scratch;           /* w*h, clean images */
	kdc_stats_t st;             /* written by the display thread only */
	kdc_stats_t pub;            /* snapshot for kdc_get_stats, under mu */
	uint64_t frames_dropped;    /* under mu */
	double scan_ms[RING], plan_ms[RING];
	int scan_i, plan_i, scan_n, plan_n;
	struct timespec last_submit, last_env_check;
};

static double now_ms(void)
{
	struct timespec t;
	clock_gettime(CLOCK_MONOTONIC, &t);
	return t.tv_sec * 1e3 + t.tv_nsec / 1e6;
}

static void set_err(kdc_t *k, const char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);
	vsnprintf(k->err, sizeof k->err, fmt, ap);
	va_end(ap);
	if (k->cfg.verbose)
		fprintf(stderr, "kdc: %s\n", k->err);
}

void kdc_config_default(kdc_config_t *c)
{
	/* InkBoy's default grayscale doses; a starting point, tune per panel. */
	static const uint8_t dose[4][4] = {
		{ 0, 2, 4, 7 },
		{ 4, 0, 1, 6 },
		{ 4, 1, 0, 5 },
		{ 7, 3, 2, 0 },
	};
	memset(c, 0, sizeof *c);
	c->fb_path = "/dev/fb0";
	c->mode = KDC_MODE_DITHER;
	c->budget_black = 0;        /* automatic, from the installed pulse length */
	c->budget_white = 0;
	memcpy(c->dose, dose, sizeof dose);
	c->retarget_pct = 100;
	c->settle_tol = 0.1;
	c->long_ratio = 3.0;
	c->long_short_max = 0.05;
	c->gray_dither = 0;
	c->max_bands = 4;
	c->min_scan_interval_us = 0;
	c->keep_power_ms = 1000;
	c->clean_on_open = 1;
	c->clean_on_close = 1;
	c->min_battery_pct = 25;
}

int kdc_config_load_doses(kdc_config_t *c, const char *path, char *err, size_t errlen)
{
	FILE *f = fopen(path, "r");
	if (!f) {
		snprintf(err, errlen, "%s: %s", path, strerror(errno));
		return -1;
	}
	char line[256];
	int install = 0, install_long = 0, row = 0, lineno = 0;
	uint8_t d[4][4];
	while (fgets(line, sizeof line, f)) {
		lineno++;
		char *p = line;
		while (*p == ' ' || *p == '\t') p++;
		if (*p == '#' || *p == '\n' || *p == '\0') continue;
		if (!strncmp(p, "install", 7)) {
			if (sscanf(p + 7, "%d %d", &install, &install_long) < 1) install = 0;
			continue;
		}
		int is_long = !strncmp(p, "dark_long_steps", 15) || !strncmp(p, "light_long_steps", 16);
		if (is_long || !strncmp(p, "dark_steps", 10) || !strncmp(p, "light_steps", 11)) {
			int dark = p[0] == 'd';
			double *dst = is_long ? (dark ? c->dark_long_steps : c->light_long_steps)
			                      : (dark ? c->dark_steps : c->light_steps), v[7];
			int skip = is_long ? (dark ? 15 : 16) : (dark ? 10 : 11);
			if (sscanf(p + skip, "%lf %lf %lf %lf %lf %lf %lf",
			           &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6]) != 7) {
				snprintf(err, errlen, "%s:%d: expected 7 step values (levels 0, .5, .. 3)", path, lineno);
				fclose(f);
				return -1;
			}
			for (int i = 0; i < 7; i++) {
				if (v[i] < 0 || v[i] > 3) {
					snprintf(err, errlen, "%s:%d: steps must be 0..3 levels", path, lineno);
					fclose(f);
					return -1;
				}
				dst[i] = v[i];
			}
			if (is_long) c->have_long_steps |= dark ? 1 : 2;
			else c->have_steps |= dark ? 1 : 2;
			continue;
		}
		if (!strncmp(p, "route", 5) && (p[5] == ' ' || p[5] == '\t')) {
			char a, b;
			double v;
			const char *sh = "KDLW", *pa, *pb;
			if (sscanf(p + 5, " %c %c %lf", &a, &b, &v) != 3 || !(pa = strchr(sh, a)) ||
			    !(pb = strchr(sh, b)) || !a || !b || a == b || v < -1.5 || v > 1.5) {
				snprintf(err, errlen, "%s:%d: expected 'route O V BIAS' (shades K D L W, O != V, "
				         "BIAS -1.5..1.5 levels)", path, lineno);
				fclose(f);
				return -1;
			}
			c->route[pa - sh][pb - sh] = v;
			c->have_routes = 1;
			continue;
		}
		if (!strncmp(p, "dark", 4) || !strncmp(p, "light", 5)) {
			int dark = p[0] == 'd';
			double tau, pw;
			if (sscanf(p + (dark ? 4 : 5), "%lf %lf", &tau, &pw) != 2 ||
			    tau < 0.5 || tau > 500 || pw < 0.3 || pw > 4) {
				snprintf(err, errlen, "%s:%d: expected '%s TAU P' (TAU 0.5..500, P 0.3..4)",
				         path, lineno, dark ? "dark" : "light");
				fclose(f);
				return -1;
			}
			if (dark) { c->dark_tau = tau; c->dark_p = pw; }
			else { c->light_tau = tau; c->light_p = pw; }
			continue;
		}
		if (strchr("KDLW", *p) && (p[1] == ' ' || p[1] == '\t')) p++;   /* optional row label */
		int v[4];
		if (row >= 4 || sscanf(p, "%d %d %d %d", &v[0], &v[1], &v[2], &v[3]) != 4) {
			snprintf(err, errlen, "%s:%d: expected 4 numbers", path, lineno);
			fclose(f);
			return -1;
		}
		for (int i = 0; i < 4; i++) {
			if ((i == row && v[i] != 0) || (i != row && (v[i] < 1 || v[i] > 60))) {
				snprintf(err, errlen, "%s:%d: doses must be 1..60 off the diagonal, 0 on it", path, lineno);
				fclose(f);
				return -1;
			}
			d[row][i] = (uint8_t)v[i];
		}
		row++;
	}
	fclose(f);
	if (row != 4 || install < 1 || install > 8) {
		snprintf(err, errlen, "%s: need an 'install N' line (1..8) and 4 rows", path);
		return -1;
	}
	if (c->have_long_steps && (c->have_long_steps != 3 || c->have_steps != 3 || install_long <= install)) {
		snprintf(err, errlen, "%s: long steps need both long lines, the short step lines and 'install N L' (L > N)", path);
		return -1;
	}
	c->long_pulse_frames = c->have_long_steps ? install_long : 0;
	if (c->have_steps && c->have_steps != 3) {
		snprintf(err, errlen, "%s: 'dark_steps' and 'light_steps' go together", path);
		return -1;
	}
	if ((c->dark_tau > 0) != (c->light_tau > 0)) {
		snprintf(err, errlen, "%s: 'dark' and 'light' curves go together", path);
		return -1;
	}
	memcpy(c->dose, d, sizeof d);
	c->dose_pulse_frames = install;
	return 0;
}

int kdc_width(const kdc_t *k) { return k->w; }
int kdc_height(const kdc_t *k) { return k->h; }
const char *kdc_last_error(kdc_t *k) { return k->err; }
const uint8_t *kdc_sim_optical(kdc_t *k) { return k->dev.sim ? k->dev.opt8 : NULL; }
uint64_t kdc_sim_pulses(kdc_t *k) { return k->dev.sim_pulses; }

/* Lab126 enables a pixel pre-processing pass ("SW mitigation") at low battery
 * and low temperature, which would alter the command plane. Same thresholds
 * as is_SW_mitigation_needed(), with a margin, using the panel temperature. */
static int env_unsafe(kdc_t *k, char *why, size_t n)
{
	if (k->dev.sim || k->cfg.min_battery_pct < 0)
		return 0;
	int batt = kdc_battery_pct(), temp = 25;
	if (kdc_dev_temperature(&k->dev, &temp) == 0)
		k->st.temperature_c = temp;
	if (batt < 0)
		return 0;
	if (batt < k->cfg.min_battery_pct) {
		snprintf(why, n, "battery %d%% is below %d%%", batt, k->cfg.min_battery_pct);
		return 1;
	}
	if (temp <= 18 && batt <= 55) {
		snprintf(why, n, "battery %d%% at %d C is in the driver's mitigation range", batt, temp);
		return 1;
	}
	return 0;
}

static void update_pulse_frames(kdc_t *k)
{
	if (k->dev.sim) { k->st.pulse_frames = 3; return; }
	int idx = 0;
	for (int t = 0; t < k->wfm.temps; t++)
		if (k->st.temperature_c >= k->trt[t]) idx = t;
	k->st.pulse_frames = k->pulse.frames_per_temp[idx];
}

/* ---- clean sequence -------------------------------------------------- */

static void write_full(kdc_t *k, const uint8_t *img)
{
	for (int y = 0; y < k->h; y++) {
		uint8_t *dst = k->dev.fb + (size_t)y * k->dev.stride;
		const uint8_t *src = img + (size_t)y * k->w;
		if (k->dev.inverted)
			for (int x = 0; x < k->w; x++) dst[x] = (uint8_t)~src[x];
		else
			memcpy(dst, src, (size_t)k->w);
	}
}

static int submit_wait(kdc_t *k, int waveform, int full)
{
	uint32_t m;
	kdc_rect_t all = { 0, 0, k->w, k->h };
	if (kdc_dev_update(&k->dev, all, waveform, full, &m, NULL) < 0)
		return -1;
	return kdc_dev_wait(&k->dev, m);
}

/* Sync the EPDC working buffer to the estimated panel state without driving
 * (all values are HOLD under the pulse waveform), then GC16 to the targets. */
static int do_clean(kdc_t *k, int sync)
{
	if (sync) {
		kdc_plan_render(&k->plan, 1, k->scratch, k->w);
		write_full(k, k->scratch);
		if (submit_wait(k, WAVEFORM_MODE_DU4, 1) < 0)
			return -1;
	}
	kdc_plan_render(&k->plan, 0, k->scratch, k->w);
	write_full(k, k->scratch);
	if (submit_wait(k, WAVEFORM_MODE_GC16, 1) < 0)
		return -1;
	kdc_plan_reset(&k->plan, k->plan.prev);
	k->st.cleans++;
	return 0;
}

/* ---- display thread -------------------------------------------------- */

static void set_failed(kdc_t *k)
{
	pthread_mutex_lock(&k->mu);
	k->failed = 1;
	pthread_mutex_unlock(&k->mu);
}

static void ring_add(double *ring, int *i, int *n, double v)
{
	ring[*i] = v;
	*i = (*i + 1) % RING;
	if (*n < RING) (*n)++;
}

static double ring_avg(const double *ring, int n)
{
	double s = 0;
	for (int i = 0; i < n; i++) s += ring[i];
	return n ? s / n : 0;
}

/* Takes the mailbox frame into front. Caller holds k->mu. */
static int take_frame(kdc_t *k)
{
	if (!k->mail_fresh)
		return 0;
	int t = k->front; k->front = k->mail; k->mail = t;
	k->mail_fresh = 0;
	memcpy(k->thread_dmg, k->dmg[k->front], (size_t)k->plan.ntiles);
	memset(k->dmg[k->front], 0, (size_t)k->plan.ntiles);
	return 1;
}

static void publish_stats(kdc_t *k)   /* caller holds mu */
{
	k->pub = k->st;
	k->pub.avg_scan_ms = ring_avg(k->scan_ms, k->scan_n);
	k->pub.avg_plan_ms = ring_avg(k->plan_ms, k->plan_n);
	k->pub.active_tiles = k->plan.n_active;
}

/* Waits for submitted scans to complete. MXCFB_WAIT_FOR_UPDATE_COMPLETE
 * blocks holding the fb core's fb_info lock, so it runs here; the display
 * thread does no fb ioctls while a scan is in flight. */
static void *waiter_thread(void *arg)
{
	kdc_t *k = arg;
	uint32_t m[KDC_MAX_BANDS];
	pthread_mutex_lock(&k->mu);
	for (;;) {
		while (!k->wait_n && !k->waiter_stop)
			pthread_cond_wait(&k->wcv, &k->mu);
		if (!k->wait_n)
			break;                      /* stop, nothing in flight */
		int n = k->wait_n;
		memcpy(m, k->wait_markers, sizeof m[0] * (size_t)n);
		pthread_mutex_unlock(&k->mu);
		for (int i = 0; i < n; i++)
			kdc_dev_wait(&k->dev, m[i]);
		pthread_mutex_lock(&k->mu);
		k->wait_n = 0;
		k->scan_done = 1;
		pthread_cond_broadcast(&k->wake);
	}
	pthread_mutex_unlock(&k->mu);
	return NULL;
}

/*
 * Display loop. While scan N runs on the panel, the thread waits for either
 * the scan to finish or a new frame; a new frame is ingested and scan N+1
 * planned right away, overlapping the panel. When scan N completes, the
 * planned scan is copied and submitted at once. One ingest and one build per
 * scan; a frame arriving mid-scan is planned immediately (no stutter).
 */
static void *display_thread(void *arg)
{
	kdc_t *k = arg;
	int inflight = 0, planned = 0, n = 0, fails = 0;
	double submit_t = 0;
	kdc_rect_t bands[KDC_MAX_BANDS];
	int band_long[KDC_MAX_BANDS] = { 0 };
	uint32_t markers[KDC_MAX_BANDS];

	for (;;) {
		pthread_mutex_lock(&k->mu);
		for (;;) {
			if (inflight && k->scan_done) {
				inflight = 0;
				ring_add(k->scan_ms, &k->scan_i, &k->scan_n, now_ms() - submit_t);
			}
			if (k->stop)
				break;
			if (inflight) {
				if (k->mail_fresh && !planned)
					break;              /* plan the next scan now */
			} else if (k->clean_req || k->mail_fresh || planned ||
			           (k->plan.n_active && !k->failed)) {
				break;
			} else {
				k->idle = 1;
				publish_stats(k);
				pthread_cond_broadcast(&k->idle_cv);
			}
			pthread_cond_wait(&k->wake, &k->mu);
		}
		k->idle = 0;
		publish_stats(k);
		if (k->stop) {
			/* Finish the scan in flight before the caller touches the fb. */
			while (inflight && !k->scan_done)
				pthread_cond_wait(&k->wake, &k->mu);
			pthread_mutex_unlock(&k->mu);
			break;
		}
		int have_frame = planned ? 0 : take_frame(k);
		int clean = 0;
		if (!inflight) {
			clean = k->clean_req;
			k->clean_req = 0;
		}
		pthread_mutex_unlock(&k->mu);

		double t0 = now_ms();
		if (have_frame) {
			kdc_plan_ingest(&k->plan, k->buf[k->front], k->thread_dmg);
			k->st.frames_in++;
		}

		if (!inflight) {
			/* Periodic environment check: ioctls, so only between scans. */
			if (t0 - (k->last_env_check.tv_sec * 1e3 + k->last_env_check.tv_nsec / 1e6) > 30000) {
				char why[128];
				clock_gettime(CLOCK_MONOTONIC, &k->last_env_check);
				if (env_unsafe(k, why, sizeof why) && !k->failed) {
					set_err(k, "stopped: %s", why);
					do_clean(k, 1);
					set_failed(k);
				}
				update_pulse_frames(k);
			}
			if (k->failed) {
				/* Keep consuming frames so producers never block; drive nothing. */
				planned = 0;
				continue;
			}
			if (clean) {
				planned = 0;
				if (do_clean(k, 1) < 0) {
					set_err(k, "clean failed: %s", strerror(errno));
					set_failed(k);
				}
				continue;
			}
		}

		if (!planned) {
			n = kdc_plan_build(&k->plan, bands, band_long, k->cfg.max_bands);
			planned = 1;
			ring_add(k->plan_ms, &k->plan_i, &k->plan_n, now_ms() - t0);
		}
		if (inflight)
			continue;                   /* submit when the panel is done */

		kdc_plan_copy(&k->plan, k->dev.fb, k->dev.stride);
		planned = 0;
		if (!n)
			continue;

		if (k->cfg.min_scan_interval_us > 0) {
			struct timespec now;
			clock_gettime(CLOCK_MONOTONIC, &now);
			long long el = (now.tv_sec - k->last_submit.tv_sec) * 1000000LL +
			               (now.tv_nsec - k->last_submit.tv_nsec) / 1000;
			if (el < k->cfg.min_scan_interval_us) {
				long long us = k->cfg.min_scan_interval_us - el;
				struct timespec ts = { (time_t)(us / 1000000), (long)(us % 1000000) * 1000 };
				nanosleep(&ts, NULL);
			}
		}
		clock_gettime(CLOCK_MONOTONIC, &k->last_submit);
		submit_t = now_ms();
		int ok = 1, sent = 0;
		/* One waveform per scan. The driver merges touching pending updates
		 * and turns differing waveforms into WAVEFORM_MODE_AUTO, which would
		 * draw the command plane as an image with a stock waveform. The dual
		 * slot also does short pulses, so a scan with any long pulse goes out
		 * entirely with it. */
		int scan_long = 0;
		for (int i = 0; i < n; i++) scan_long |= band_long[i];
		for (int i = 0; i < n; i++) {
			int wfm = scan_long ? WAVEFORM_MODE_DU : WAVEFORM_MODE_DU4;
			if (kdc_dev_update(&k->dev, bands[i], wfm, 1, &markers[sent], NULL) < 0) {
				ok = 0;
				break;
			}
			sent++;
			k->st.updates++;
			if (scan_long) k->st.long_updates++;
		}
		if (ok) {
			kdc_plan_commit(&k->plan);
			k->st.scans++;
			fails = 0;
			pthread_mutex_lock(&k->mu);
			memcpy(k->wait_markers, markers, sizeof markers[0] * (size_t)sent);
			k->wait_n = sent;
			k->scan_done = 0;
			inflight = 1;
			pthread_cond_signal(&k->wcv);
			pthread_mutex_unlock(&k->mu);
		} else {
			/* Not committed: the pixels are planned again next scan. */
			k->st.submit_errors++;
			int e = errno;
			for (int i = 0; i < sent; i++) kdc_dev_wait(&k->dev, markers[i]);
			if (++fails >= 50) {
				set_err(k, "stopped: update ioctl keeps failing: %s%s", strerror(e),
				        e == EPERM ? " (EPDC paused or blanked?)" : "");
				set_failed(k);
			} else {
				struct timespec ts = { 0, 20 * 1000000L };
				nanosleep(&ts, NULL);
			}
		}
	}
	return NULL;
}

/* ---- public API ------------------------------------------------------ */

static int probe_pulse(kdc_t *k)
{
	/* A 1x1 partial update of an unchanged pixel drives nothing under any
	 * waveform; the driver writes back the slot DU4 resolves to. */
	uint32_t m;
	int resolved = -1;
	if (kdc_dev_update(&k->dev, (kdc_rect_t){ 0, 0, 1, 1 }, WAVEFORM_MODE_DU4, 0, &m, &resolved) < 0) {
		set_err(k, "probe update failed: %s", strerror(errno));
		return KDC_ERR_IO;
	}
	kdc_dev_wait(&k->dev, m);
	k->slot = resolved;
	k->st.pulse_slot = resolved;
	k->du_slot = -1;
	if (kdc_dev_update(&k->dev, (kdc_rect_t){ 0, 0, 1, 1 }, WAVEFORM_MODE_DU, 0, &m, &k->du_slot) == 0)
		kdc_dev_wait(&k->dev, m);

	uint8_t *f;
	size_t len;
	if (kdc_read_file(WFM_PROC, &f, &len) < 0) {
		set_err(k, "cannot read %s: %s", WFM_PROC, strerror(errno));
		return KDC_ERR_IO;
	}
	char err[128];
	int r = KDC_OK;
	if (kdc_wfm_parse(f, len, &k->wfm, err, sizeof err) < 0) {
		set_err(k, "loaded waveform: %s", err);
		r = KDC_ERR_NO_PULSE;
	} else if (kdc_wfm_find_pulse(f, len, resolved, &k->pulse) != 1) {
		set_err(k, "pulse waveform not installed in slot %d (DU4); run scripts/kdc-pulse.sh install", resolved);
		r = KDC_ERR_NO_PULSE;
	} else {
		memcpy(k->trt, k->wfm.trt, (size_t)k->wfm.temps);
		k->wfm.trt = k->trt;
	}
	free(f);
	return r;
}

int kdc_open(kdc_t **out, const kdc_config_t *cfg)
{
	*out = NULL;
	kdc_t *k = calloc(1, sizeof *k);
	if (!k) return KDC_ERR_NOMEM;
	k->cfg = *cfg;
	if (k->cfg.budget_black > 60 || k->cfg.budget_white > 60 ||
	    (k->cfg.mode != KDC_MODE_DITHER && k->cfg.mode != KDC_MODE_GRAY4)) {
		free(k);
		return KDC_ERR_ARG;
	}
	for (int a = 0; a < 4; a++)
		for (int b = 0; b < 4; b++)
			if (a != b && (k->cfg.dose[a][b] < 1 || k->cfg.dose[a][b] > 60)) {
				free(k);
				return KDC_ERR_ARG;
			}
	pthread_mutex_init(&k->mu, NULL);
	pthread_cond_init(&k->wake, NULL);
	pthread_cond_init(&k->idle_cv, NULL);
	pthread_cond_init(&k->wcv, NULL);
	*out = k;   /* so kdc_last_error works on failure; freed by kdc_close */

	char err[200];
	if (kdc_dev_open(&k->dev, cfg, err, sizeof err) < 0) {
		set_err(k, "%s", err);
		return strstr(err, "bpp") ? KDC_ERR_FORMAT : KDC_ERR_IO;
	}
	k->w = k->dev.w;
	k->h = k->dev.h;

	char why[128];
	if (env_unsafe(k, why, sizeof why)) {
		set_err(k, "refusing to start: %s (set min_battery_pct = -1 to override)", why);
		return KDC_ERR_BATTERY;
	}
	if (!k->dev.sim) {
		int r = probe_pulse(k);
		if (r) return r;
	} else {
		k->slot = k->st.pulse_slot = WAVEFORM_MODE_DU4;
		kdc_dev_temperature(&k->dev, &k->st.temperature_c);
	}
	update_pulse_frames(k);
	clock_gettime(CLOCK_MONOTONIC, &k->last_env_check);

	/* Match drive to the installed pulse length (kdc-pulse.sh install N). */
	int ref = k->dev.sim ? 3 : k->pulse.frames;
	if (ref < 1) ref = 3;
	k->st.pulse_frames_ref = ref;
	if (k->cfg.dose_pulse_frames && k->cfg.dose_pulse_frames != ref) {
		set_err(k, "dose table is calibrated for 'install %d' but 'install %d' is loaded",
		        k->cfg.dose_pulse_frames, ref);
		return KDC_ERR_ARG;
	}
	if (k->cfg.dose_pulse_frames) {
		/* Calibrated: the panel is much slower toward black than toward
		 * white under short pulses, so use the measured full swings. */
		if (!k->cfg.budget_black) k->cfg.budget_black = k->cfg.dose[3][0];
		if (!k->cfg.budget_white) k->cfg.budget_white = k->cfg.dose[0][3];
	} else {
		int auto_budget = (18 + ref / 2) / ref;
		if (!k->cfg.budget_black) k->cfg.budget_black = (uint8_t)auto_budget;
		if (!k->cfg.budget_white) k->cfg.budget_white = (uint8_t)auto_budget;
	}
	k->st.budget_black = k->cfg.budget_black;
	k->st.budget_white = k->cfg.budget_white;

	kdc_plan_cfg_t pc;
	memset(&pc, 0, sizeof pc);
	if (k->cfg.mode == KDC_MODE_DITHER) {
		pc.levels = 2;
		pc.dose[1][0] = k->cfg.budget_black;   /* white -> black */
		pc.dose[0][1] = k->cfg.budget_white;
	} else {
		pc.levels = 4;
		/* Doses are pulses of the installed length: calibrate them with
		 * kdc-cal for the install setting in use. */
		memcpy(pc.dose, k->cfg.dose, sizeof pc.dose);
	}
	if (k->cfg.have_steps == 3 || (k->cfg.dark_tau > 0 && k->cfg.light_tau > 0)) {
		pc.model = 1;
		if (k->cfg.have_steps == 3)
			kdc_model_tables_steps(k->cfg.dark_steps, k->cfg.light_steps, pc.next_dark, pc.next_light);
		else
			kdc_model_tables(k->cfg.dark_tau, k->cfg.dark_p, k->cfg.light_tau, k->cfg.light_p,
			                 pc.next_dark, pc.next_light);
		double tol = k->cfg.settle_tol > 0 ? k->cfg.settle_tol : 0.1;
		pc.tol = (uint16_t)(tol / 3.0 * KDC_LV_MAX + 0.5);
		k->st.model = 1;
		if (k->cfg.have_routes && pc.levels == 4) {
			pc.history = 1;
			for (int o = 0; o < 4; o++)
				for (int v = 0; v < 4; v++)
					pc.route[o][v] = (int16_t)lround(k->cfg.route[o][v] / 3.0 * KDC_LV_MAX);
			k->st.history = 1;
		}
	}
	pc.retarget_pct = k->cfg.retarget_pct ? k->cfg.retarget_pct : 100;
	pc.gray_dither = k->cfg.gray_dither;
	pc.cmd_black = k->dev.inverted ? 0xFF : 0x00;
	pc.cmd_white = k->dev.inverted ? 0x00 : 0xFF;
	pc.cmd_hold = CMD_HOLD;
	if (k->cfg.have_long_steps == 3) {
		if (!pc.model) {
			set_err(k, "long pulse steps need the state model (dark_steps/light_steps)");
			return KDC_ERR_ARG;
		}
		if (!k->dev.sim) {
			if (k->pulse.long_slot <= 0 || k->du_slot != k->pulse.long_slot) {
				set_err(k, "calibration uses long pulses but the dual waveform is not installed "
				        "(kdc-pulse.sh install %d %d)", ref, k->cfg.long_pulse_frames);
				return KDC_ERR_NO_PULSE;
			}
			if (k->cfg.long_pulse_frames && k->cfg.long_pulse_frames != k->pulse.long_frames) {
				set_err(k, "long steps are calibrated for long pulses of %d frames but %d are installed",
				        k->cfg.long_pulse_frames, k->pulse.long_frames);
				return KDC_ERR_ARG;
			}
		}
		pc.dual = 1;
		kdc_model_tables_steps(k->cfg.dark_long_steps, k->cfg.light_long_steps,
		                       pc.next_dark_long, pc.next_light_long);
		/* Codes: short 1 / 14, long 0 / 15 (inverted in night mode). */
		pc.cmd_black = k->dev.inverted ? 0xEE : 0x11;
		pc.cmd_white = k->dev.inverted ? 0x11 : 0xEE;
		pc.cmd_black_long = k->dev.inverted ? 0xFF : 0x00;
		pc.cmd_white_long = k->dev.inverted ? 0x00 : 0xFF;
		double r = k->cfg.long_ratio > 0 ? k->cfg.long_ratio : 3.0;
		pc.long_ratio_x16 = (uint16_t)(r * 16 + 0.5);
		double gm = k->cfg.long_short_max > 0 ? k->cfg.long_short_max : 0.05;
		pc.long_short_max = (uint16_t)(gm / 3.0 * KDC_LV_MAX + 0.5);
		k->st.dual = 1;
	}
	if (kdc_plan_init(&k->plan, k->w, k->h, &pc) < 0) {
		set_err(k, "out of memory");
		return KDC_ERR_NOMEM;
	}
	if (pc.model && k->dev.sim) {
		/* The simulated panel follows the same curves: tests then check the
		 * control logic exactly. */
		k->dev.sim_next_dark = k->plan.cfg.next_dark;
		k->dev.sim_next_light = k->plan.cfg.next_light;
		if (pc.dual) {
			k->dev.sim_next_dark_long = k->plan.cfg.next_dark_long;
			k->dev.sim_next_light_long = k->plan.cfg.next_light_long;
		}
	}
	size_t n = (size_t)k->w * k->h;
	for (int i = 0; i < 3; i++) {
		k->buf[i] = malloc(n);
		k->dmg[i] = calloc(1, (size_t)k->plan.ntiles);
		if (!k->buf[i] || !k->dmg[i]) { set_err(k, "out of memory"); return KDC_ERR_NOMEM; }
		memset(k->buf[i], 0xFF, n);
	}
	k->thread_dmg = calloc(1, (size_t)k->plan.ntiles);
	k->scratch = malloc(n);
	if (!k->thread_dmg || !k->scratch) { set_err(k, "out of memory"); return KDC_ERR_NOMEM; }
	k->back = 0; k->mail = 1; k->front = 2; k->latest = 2;

	kdc_dev_set_pwrdown(&k->dev, k->cfg.keep_power_ms);

	/* Reference: white screen, model = white. */
	kdc_plan_reset(&k->plan, k->buf[2]);
	if (k->cfg.clean_on_open && do_clean(k, 0) < 0) {
		set_err(k, "initial GC16 refresh failed: %s", strerror(errno));
		return KDC_ERR_IO;
	}
	k->pub = k->st;   /* valid stats before the thread's first snapshot */
	if (pthread_create(&k->waiter, NULL, waiter_thread, k)) {
		set_err(k, "pthread_create failed");
		return KDC_ERR_NOMEM;
	}
	k->waiter_started = 1;
	if (pthread_create(&k->thread, NULL, display_thread, k)) {
		set_err(k, "pthread_create failed");
		return KDC_ERR_NOMEM;
	}
	k->thread_started = 1;
	if (k->cfg.verbose)
		fprintf(stderr, "kdc: %dx%d stride %d%s, pulse slot %d, %d frames/scan at %d C\n",
		        k->w, k->h, k->dev.stride, k->dev.inverted ? " (inverted)" : "",
		        k->slot, k->st.pulse_frames, k->st.temperature_c);
	return KDC_OK;
}

void kdc_close(kdc_t *k)
{
	if (!k) return;
	if (k->thread_started) {
		pthread_mutex_lock(&k->mu);
		k->stop = 1;
		pthread_cond_signal(&k->wake);
		pthread_mutex_unlock(&k->mu);
		pthread_join(k->thread, NULL);   /* returns after its last scan completed */
		/* Leave the working buffer and the panel consistent for whatever
		 * draws next (the Kindle UI computes partial updates from it). */
		if (k->cfg.clean_on_close && !k->failed)
			do_clean(k, 1);
	}
	if (k->waiter_started) {
		pthread_mutex_lock(&k->mu);
		k->waiter_stop = 1;
		pthread_cond_signal(&k->wcv);
		pthread_mutex_unlock(&k->mu);
		pthread_join(k->waiter, NULL);
	}
	if (k->dev.map || k->dev.sim)
		kdc_dev_restore_pwrdown(&k->dev);
	kdc_plan_free(&k->plan);
	kdc_dev_close(&k->dev);
	for (int i = 0; i < 3; i++) { free(k->buf[i]); free(k->dmg[i]); }
	free(k->thread_dmg);
	free(k->scratch);
	pthread_cond_destroy(&k->wake);
	pthread_cond_destroy(&k->idle_cv);
	pthread_cond_destroy(&k->wcv);
	pthread_mutex_destroy(&k->mu);
	free(k);
}

uint8_t *kdc_frame_begin(kdc_t *k, int keep)
{
	if (keep && k->latest != k->back)
		memcpy(k->buf[k->back], k->buf[k->latest], (size_t)k->w * k->h);
	return k->buf[k->back];
}

int kdc_frame_publish(kdc_t *k, const kdc_rect_t *damage, int ndamage)
{
	int tc = k->plan.tcols, tr = k->plan.trows;
	uint8_t *d = k->dmg[k->back];
	if (!damage || ndamage <= 0) {
		memset(d, 1, (size_t)k->plan.ntiles);
	} else {
		for (int i = 0; i < ndamage; i++) {
			kdc_rect_t r = damage[i];
			if (r.x < 0) { r.w += r.x; r.x = 0; }
			if (r.y < 0) { r.h += r.y; r.y = 0; }
			if (r.x + r.w > k->w) r.w = k->w - r.x;
			if (r.y + r.h > k->h) r.h = k->h - r.y;
			if (r.w <= 0 || r.h <= 0) continue;
			int c0 = r.x / KDC_TILE, c1 = (r.x + r.w - 1) / KDC_TILE;
			int r0 = r.y / KDC_TILE, r1 = (r.y + r.h - 1) / KDC_TILE;
			for (int ty = r0; ty <= r1 && ty < tr; ty++)
				memset(d + ty * tc + c0, 1, (size_t)(c1 - c0 + 1));
		}
	}
	pthread_mutex_lock(&k->mu);
	if (k->mail_fresh) {
		k->frames_dropped++;
		const uint8_t *old = k->dmg[k->mail];
		for (int i = 0; i < k->plan.ntiles; i++) d[i] |= old[i];
	}
	int t = k->back; k->back = k->mail; k->mail = t;
	memset(k->dmg[k->back], 0, (size_t)k->plan.ntiles);
	k->mail_fresh = 1;
	k->latest = k->mail;
	int failed = k->failed;
	pthread_cond_signal(&k->wake);
	pthread_mutex_unlock(&k->mu);
	return failed ? KDC_ERR_STOPPED : KDC_OK;
}

int kdc_frame_pending(kdc_t *k)
{
	pthread_mutex_lock(&k->mu);
	int r = k->mail_fresh;
	pthread_mutex_unlock(&k->mu);
	return r;
}

int kdc_clean(kdc_t *k)
{
	pthread_mutex_lock(&k->mu);
	k->clean_req = 1;
	int failed = k->failed;
	pthread_cond_signal(&k->wake);
	pthread_mutex_unlock(&k->mu);
	return failed ? KDC_ERR_STOPPED : KDC_OK;
}

int kdc_flush(kdc_t *k, int timeout_ms)
{
	struct timespec dl;
	clock_gettime(CLOCK_REALTIME, &dl);
	if (timeout_ms >= 0) {
		dl.tv_sec += timeout_ms / 1000;
		dl.tv_nsec += (long)(timeout_ms % 1000) * 1000000L;
		if (dl.tv_nsec >= 1000000000L) { dl.tv_sec++; dl.tv_nsec -= 1000000000L; }
	}
	int r = 0;
	pthread_mutex_lock(&k->mu);
	/* idle is only set while the thread sleeps with nothing left to drive. */
	while (!(k->idle && !k->mail_fresh && !k->clean_req)) {
		if (timeout_ms < 0) {
			pthread_cond_wait(&k->idle_cv, &k->mu);
		} else if (pthread_cond_timedwait(&k->idle_cv, &k->mu, &dl) == ETIMEDOUT) {
			r = 1;
			break;
		}
	}
	pthread_mutex_unlock(&k->mu);
	return r;
}

void kdc_get_stats(kdc_t *k, kdc_stats_t *s)
{
	pthread_mutex_lock(&k->mu);
	*s = k->thread_started ? k->pub : k->st;
	s->frames_dropped = k->frames_dropped;
	pthread_mutex_unlock(&k->mu);
}
