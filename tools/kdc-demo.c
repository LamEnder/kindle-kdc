/*
 * kdc-demo: exercises the controller on the Kindle (or the simulator).
 *
 *   kdc-demo [options] bounce          moving shapes over a gradient
 *   kdc-demo [options] raw WxH         8-bit gray frames from stdin, e.g.
 *     ffmpeg -i in.mp4 -vf scale=-2:480,format=gray -f rawvideo - | kdc-demo raw 640x480
 *   kdc-demo [options] transition      gray transition test (InkBoy-style): bands of
 *     K/D/L/W, each with the 3 other shades sweeping left to right; GC16-drawn
 *     reference squares at the left edge. Use with -g.
 *
 * options: -g gray4 mode   -b N budget (dither)   -s SECONDS   -B N max bands
 *          -D FILE dose file (presets/NAME.doses)   -k keep the last frame (no closing
 *          refresh; run "kdc-cal clear" afterwards)
 *          -f FPS cap on produced frames   -S simulator   -v verbose
 *          -m N draw N black marker squares top-left (to identify a run)
 *          -W N budget_white (default = -b)
 */
#define _GNU_SOURCE
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "kdc.h"

static volatile sig_atomic_t quit;
static void on_sig(int s) { (void)s; quit = 1; }

static double now_s(void)
{
	struct timespec t;
	clock_gettime(CLOCK_MONOTONIC, &t);
	return t.tv_sec + t.tv_nsec / 1e9;
}

static void sleep_until(double t)
{
	double d = t - now_s();
	if (d <= 0) return;
	struct timespec ts = { (time_t)d, (long)((d - (time_t)d) * 1e9) };
	nanosleep(&ts, NULL);
}

static void disc(uint8_t *f, int w, int h, int cx, int cy, int r, uint8_t v)
{
	for (int y = cy - r; y <= cy + r; y++) {
		if (y < 0 || y >= h) continue;
		int dx = (int)sqrt((double)(r * r - (y - cy) * (y - cy)));
		int x0 = cx - dx < 0 ? 0 : cx - dx, x1 = cx + dx >= w ? w - 1 : cx + dx;
		if (x1 >= x0) memset(f + (size_t)y * w + x0, v, (size_t)(x1 - x0 + 1));
	}
}

static void background(uint8_t *f, int w, int h, int x0, int y0, int x1, int y1)
{
	for (int y = y0; y < y1; y++)
		for (int x = x0; x < x1; x++)
			f[(size_t)y * w + x] = (uint8_t)(255 - (x * 160 / w) - (y * 60 / h));
}

static int marker;

static void draw_marker(uint8_t *f, int w)
{
	for (int i = 0; i < marker; i++)
		for (int y = 12; y < 52; y++)
			memset(f + (size_t)y * w + 12 + i * 56, 0, 40);
}

static kdc_rect_t box(int cx, int cy, int r, int w, int h)
{
	kdc_rect_t b = { cx - r - 1, cy - r - 1, 2 * r + 3, 2 * r + 3 };
	if (b.x < 0) { b.w += b.x; b.x = 0; }
	if (b.y < 0) { b.h += b.y; b.y = 0; }
	if (b.x + b.w > w) b.w = w - b.x;
	if (b.y + b.h > h) b.h = h - b.y;
	return b;
}

static void run_bounce(kdc_t *k, double secs, double fps)
{
	int w = kdc_width(k), h = kdc_height(k);
	uint8_t *f = kdc_frame_begin(k, 0);
	background(f, w, h, 0, 0, w, h);
	draw_marker(f, w);
	kdc_frame_publish(k, NULL, 0);
	kdc_flush(k, -1);
	kdc_clean(k);

	double x = w / 3.0, y = h / 3.0, vx = w / 2.2, vy = h / 2.9;   /* px per second */
	int r = w / 10;
	int px = (int)x, py = (int)y;
	double t0 = now_s(), last = t0, next = t0;
	while (!quit && now_s() - t0 < secs) {
		double t = now_s(), dt = t - last;
		last = t;
		x += vx * dt; y += vy * dt;
		if (x < r) { x = r; vx = -vx; }
		if (x > w - r - 1) { x = w - r - 1; vx = -vx; }
		if (y < r) { y = r; vy = -vy; }
		if (y > h - r - 1) { y = h - r - 1; vy = -vy; }
		int cx = (int)x, cy = (int)y;
		f = kdc_frame_begin(k, 1);
		kdc_rect_t d[2] = { box(px, py, r, w, h), box(cx, cy, r, w, h) };
		background(f, w, h, d[0].x, d[0].y, d[0].x + d[0].w, d[0].y + d[0].h);
		disc(f, w, h, cx, cy, r, 0);
		disc(f, w, h, cx, cy, r / 2, 255);
		if (d[0].y < 60) draw_marker(f, w);
		if (kdc_frame_publish(k, d, 2) == KDC_ERR_STOPPED) break;
		px = cx; py = cy;
		next += 1.0 / fps;
		sleep_until(next);
		if (now_s() > next + 0.5) next = now_s();
	}
}

static const uint8_t SH4[4] = { 0x00, 0x55, 0xAA, 0xFF };

static void run_transition(kdc_t *k, double secs, double fps)
{
	int w = kdc_width(k), h = kdc_height(k);
	int bh = h / 4, rw = 60, rh = 70, refx = 10, x0 = 76, x1 = w - rw - 12;
	const double sweep = 5.0, hold = 1.0;
	struct { int band, shade, y; } r[12];
	int n = 0;
	for (int b = 0; b < 4; b++)
		for (int s = 0, j = 0; s < 4; s++)
			if (s != b) r[n++] = (typeof(r[0])){ b, s, b * bh + 30 + j++ * (rh + 40) };

	uint8_t *f = kdc_frame_begin(k, 0);
	for (int b = 0; b < 4; b++)
		memset(f + (size_t)b * bh * w, SH4[b], (size_t)(b == 3 ? h - 3 * bh : bh) * w);
	for (int i = 0; i < n; i++)
		for (int y = r[i].y; y < r[i].y + rh; y++) {
			memset(f + (size_t)y * w + refx, SH4[r[i].shade], 50);   /* static reference */
			memset(f + (size_t)y * w + x0, SH4[r[i].shade], (size_t)rw);
		}
	kdc_frame_publish(k, NULL, 0);
	kdc_flush(k, -1);
	kdc_clean(k);            /* references and start positions drawn exactly by GC16 */
	kdc_flush(k, -1);

	int sweeps = (int)(secs / (sweep + hold) + 0.5);
	if (sweeps < 1) sweeps = 1;
	int px = x0;
	double t0 = now_s(), next = t0;
	while (!quit) {
		double t = now_s() - t0;
		int done = (int)(t / (sweep + hold));
		if (done >= sweeps) break;
		double ph = t - done * (sweep + hold);
		int x = ph >= sweep ? x1 : x0 + (int)((x1 - x0) * ph / sweep);
		if (x != px) {
			f = kdc_frame_begin(k, 1);
			kdc_rect_t d[2] = { { px < x ? px : x, 0, abs(x - px) + rw, h }, { 0, 0, 0, 0 } };
			for (int i = 0; i < n; i++) {
				for (int y = r[i].y; y < r[i].y + rh; y++) {
					memset(f + (size_t)y * w + px, SH4[r[i].band], (size_t)rw);
					memset(f + (size_t)y * w + x, SH4[r[i].shade], (size_t)rw);
				}
			}
			/* a sweep that restarted jumps back: damage both ends */
			if (x < px) d[1] = (kdc_rect_t){ px, 0, rw, h };
			if (kdc_frame_publish(k, d, x < px ? 2 : 1) == KDC_ERR_STOPPED) break;
			px = x;
		}
		next += 1.0 / fps;
		sleep_until(next);
		if (now_s() > next + 0.5) next = now_s();
	}
}

static void run_raw(kdc_t *k, int sw, int sh, double fps)
{
	int w = kdc_width(k), h = kdc_height(k);
	size_t n = (size_t)sw * sh;
	uint8_t *in = malloc(n);
	int ox = (w - sw) / 2, oy = (h - sh) / 2;
	uint8_t *f = kdc_frame_begin(k, 0);
	memset(f, 255, (size_t)w * h);
	double next = now_s();
	while (!quit && fread(in, 1, n, stdin) == n) {
		f = kdc_frame_begin(k, 1);
		for (int y = 0; y < sh; y++) {
			int fy = oy + y;
			if (fy < 0 || fy >= h) continue;
			for (int x = 0; x < sw; x++) {
				int fx = ox + x;
				if (fx >= 0 && fx < w) f[(size_t)fy * w + fx] = in[(size_t)y * sw + x];
			}
		}
		kdc_rect_t d = { ox < 0 ? 0 : ox, oy < 0 ? 0 : oy, sw < w ? sw : w, sh < h ? sh : h };
		if (kdc_frame_publish(k, &d, 1) == KDC_ERR_STOPPED) break;
		if (fps > 0) { next += 1.0 / fps; sleep_until(next); }
	}
	free(in);
}

int main(int argc, char **argv)
{
	kdc_config_t c;
	kdc_config_default(&c);
	double secs = 20, fps = 30;
	int opt, keep = 0;
	char err[200];
	while ((opt = getopt(argc, argv, "gb:W:s:B:f:Svm:D:k")) != -1) {
		switch (opt) {
		case 'g': c.mode = KDC_MODE_GRAY4; break;
		case 'D':
			if (kdc_config_load_doses(&c, optarg, err, sizeof err) < 0) {
				fprintf(stderr, "kdc-demo: %s\n", err);
				return 1;
			}
			break;
		case 'k': keep = 1; c.clean_on_close = 0; break;
		case 'b': c.budget_black = c.budget_white = (uint8_t)atoi(optarg); break;
		case 'W': c.budget_white = (uint8_t)atoi(optarg); break;
		case 'm': marker = atoi(optarg); break;
		case 's': secs = atof(optarg); break;
		case 'B': c.max_bands = atoi(optarg); break;
		case 'f': fps = atof(optarg); break;
		case 'S': c.sim = 1; break;
		case 'v': c.verbose = 1; break;
		default: goto usage;
		}
	}
	if (optind >= argc) goto usage;
	/* Photos/video: dither between shades; the transition test uses exact shades. */
	c.gray_dither = strcmp(argv[optind], "transition") != 0;
	signal(SIGINT, on_sig);
	signal(SIGTERM, on_sig);

	kdc_t *k = NULL;
	int r = kdc_open(&k, &c);
	if (r) {
		fprintf(stderr, "kdc-demo: kdc_open failed (%d): %s\n", r, k ? kdc_last_error(k) : "");
		kdc_close(k);
		return 1;
	}
	double t0 = now_s();
	if (!strcmp(argv[optind], "bounce")) {
		run_bounce(k, secs, fps);
	} else if (!strcmp(argv[optind], "transition")) {
		run_transition(k, secs, fps);
	} else if (!strcmp(argv[optind], "raw") && optind + 1 < argc) {
		int sw, sh;
		if (sscanf(argv[optind + 1], "%dx%d", &sw, &sh) != 2 || sw <= 0 || sh <= 0) goto usage;
		run_raw(k, sw, sh, fps);
	} else {
		kdc_close(k);
		goto usage;
	}
	kdc_flush(k, 5000);
	double el = now_s() - t0;
	kdc_stats_t st;
	kdc_get_stats(k, &st);
	fprintf(stderr,
	        "kdc-demo: %.1f s, %llu scans (%.1f/s), %llu updates, frames in %llu, dropped %llu\n"
	        "          scan %.1f ms, plan %.2f ms, pulse slot %d, %d frames/scan at %d C (install %d), %s %d/%d, errors %llu\n",
	        el, (unsigned long long)st.scans, st.scans / el, (unsigned long long)st.updates,
	        (unsigned long long)st.frames_in, (unsigned long long)st.frames_dropped,
	        st.avg_scan_ms, st.avg_plan_ms, st.pulse_slot, st.pulse_frames, st.temperature_c,
	        st.pulse_frames_ref, !st.model ? "budget" : st.history ? "state model + history (budgets unused)"
	        : "state model (budgets unused)",
	        st.budget_black, st.budget_white,
	        (unsigned long long)st.submit_errors);
	if (*kdc_last_error(k)) fprintf(stderr, "kdc-demo: %s\n", kdc_last_error(k));
	kdc_close(k);
	if (keep) fprintf(stderr, "kdc-demo: last frame kept on screen; run 'kdc-cal clear' when done\n");
	return 0;
usage:
	fprintf(stderr, "usage: kdc-demo [-g] [-D FILE] [-k] [-b N] [-W N] [-s SEC] [-B N] [-f FPS] [-m N] [-S] [-v]\n"
	                "                bounce | transition | raw WxH\n");
	return 2;
}
