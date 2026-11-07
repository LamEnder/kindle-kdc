/*
 * kdc-bench: CPU cost of the planner on a full PW4-sized screen.
 * Pure CPU (no framebuffer), safe to run on the Kindle while the UI runs.
 *
 *   kdc-bench [ITER]
 *
 * Content: a full-screen moving pattern (video-like: every tile changes every
 * frame). Reports ms per call for ingest, build, copy and commit, in dither
 * mode (dose counting) and gray mode (state model).
 */
#define _GNU_SOURCE
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "kdc_plan.h"

static double now_ms(void)
{
	struct timespec t;
	clock_gettime(CLOCK_MONOTONIC, &t);
	return t.tv_sec * 1e3 + t.tv_nsec / 1e6;
}

static void make_frame(uint8_t *f, int w, int h, int phase)
{
	/* soft diagonal bands + a moving disc: lots of mid-grays and edges */
	for (int y = 0; y < h; y++) {
		uint8_t *row = f + (size_t)y * w;
		for (int x = 0; x < w; x++) {
			int v = ((x + 2 * y + phase * 9) & 255);
			int dx = x - (w / 2 + (int)(300 * sin(phase * 0.1))), dy = y - h / 2;
			if (dx * dx + dy * dy < 200 * 200) v = 255 - v;
			row[x] = (uint8_t)v;
		}
	}
}

static void bench(const char *name, kdc_plan_cfg_t *pc, int w, int h, int iters)
{
	kdc_plan_t p;
	if (kdc_plan_init(&p, w, h, pc) < 0) { fprintf(stderr, "init failed\n"); return; }
	size_t n = (size_t)w * h;
	int nf = 8;
	uint8_t *frames = malloc(n * nf), *fb = malloc(n);
	for (int i = 0; i < nf; i++) make_frame(frames + n * i, w, h, i);
	kdc_plan_reset(&p, frames);
	kdc_rect_t bands[KDC_MAX_BANDS];
	int bl[KDC_MAX_BANDS];
	double ti = 0, tb = 0, tc = 0, tm = 0;
	int changed = 0;
	for (int it = 0; it < iters; it++) {
		double t0 = now_ms();
		changed += kdc_plan_ingest(&p, frames + n * ((it + 1) % nf), NULL);
		double t1 = now_ms();
		kdc_plan_build(&p, bands, bl, 4);
		double t2 = now_ms();
		kdc_plan_copy(&p, fb, w);
		double t3 = now_ms();
		kdc_plan_commit(&p);
		double t4 = now_ms();
		ti += t1 - t0; tb += t2 - t1; tc += t3 - t2; tm += t4 - t3;
	}
	printf("%-28s ingest %6.2f  build %6.2f  copy %5.2f  commit %5.2f  = %6.2f ms/scan  (%d tiles changed/frame)\n",
	       name, ti / iters, tb / iters, tc / iters, tm / iters, (ti + tb + tc + tm) / iters, changed / iters);
	free(frames);
	free(fb);
	kdc_plan_free(&p);
}

int main(int argc, char **argv)
{
	int iters = argc > 1 ? atoi(argv[1]) : 40;
	const int w = 1072, h = 1448;
	static kdc_plan_cfg_t pc;   /* large (tables): not on the stack */

	memset(&pc, 0, sizeof pc);
	pc.levels = 2;
	pc.dose[1][0] = 6; pc.dose[0][1] = 6;
	pc.cmd_black = 0x00; pc.cmd_white = 0xFF; pc.cmd_hold = 0x88;
	bench("dither, dose counting", &pc, w, h, iters);

	memset(&pc, 0, sizeof pc);
	pc.levels = 4;
	pc.gray_dither = 1;
	pc.retarget_pct = 100;
	pc.cmd_black = 0x00; pc.cmd_white = 0xFF; pc.cmd_hold = 0x88;
	pc.model = 1;
	static const double ds[7] = { 0.000, 0.018, 0.096, 0.161, 0.295, 0.245, 0.163 };
	static const double ls[7] = { 0.228, 0.350, 0.377, 0.350, 0.298, 0.050, 0.000 };
	kdc_model_tables_steps(ds, ls, pc.next_dark, pc.next_light);
	pc.tol = 34;
	bench("gray4, state model", &pc, w, h, iters);

	pc.levels = 2;
	bench("dither, state model", &pc, w, h, iters);
	return 0;
}
