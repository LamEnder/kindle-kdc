/*
 * Host tests: waveform patching and the controller against the simulated
 * panel. Run: make test [WFM=path/to/real.wrf]
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "kdc.h"
#include "kdc_wfm.h"

static int failures;
#define CHECK(c) do { if (!(c)) { fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); failures++; } } while (0)

static void wr64(uint8_t *p, uint64_t v) { memcpy(p, &v, 8); }
static void wr32(uint8_t *p, uint32_t v) { memcpy(p, &v, 4); }

/* Minimal 5-bit waveform: 4 modes, 3 temp ranges. Mode 1 is DU-like with a
 * full swing of (6, 4, 2) frames; mode 2 has private 12-frame blocks (the
 * patchable slot); modes 0 and 3 share one phase block (not patchable). */
static uint8_t *synth_wfm(size_t *len)
{
	const int modes = 4, temps = 3;
	const int du_frames[3] = { 6, 4, 2 };
	size_t data_off = 48 + temps + 1;
	size_t cap = data_off + 4096 + 64 * 512;
	uint8_t *f = calloc(1, cap);
	f[36] = 0x04;              /* luts: 5-bit */
	f[37] = modes - 1;
	f[38] = temps - 1;
	f[48] = 0; f[49] = 15; f[50] = 30;   /* TRT */
	uint8_t *d = f + data_off;
	size_t shared = 512;
	wr64(d + shared, 3);
	size_t pos = shared + 8 + 3 * 512;
	size_t du_ph[3], own[3];
	for (int t = 0; t < temps; t++) {
		pos = (pos + 63) & ~(size_t)63;
		du_ph[t] = pos;
		wr64(d + pos, (uint64_t)du_frames[t] + 1);
		for (int fr = 0; fr < du_frames[t]; fr++) {
			uint8_t *frame = d + pos + 8 + (size_t)fr * 512;
			for (int old = 1; old < 16; old++) {     /* any -> black */
				int i = (0 << 4) | old;
				frame[i >> 1] |= (uint8_t)(1 << ((i & 1) * 4));
			}
		}
		pos += 8 + (size_t)(du_frames[t] + 1) * 512;
	}
	for (int t = 0; t < temps; t++) {
		pos = (pos + 63) & ~(size_t)63;
		own[t] = pos;
		wr64(d + pos, 12);
		memset(d + pos + 8, 0x55, 12 * 512);         /* recognizable old data */
		pos += 8 + 12 * 512;
	}
	for (int m = 0; m < modes; m++) {
		pos = (pos + 63) & ~(size_t)63;
		for (int t = 0; t < temps; t++)
			wr64(d + pos + 8 * t, m == 1 ? du_ph[t] : m == 2 ? own[t] : shared);
		wr64(d + 8 * m, pos);
		pos += 8 * temps;
	}
	*len = data_off + pos;
	wr32(f + 4, (uint32_t)*len);
	wr32(f, 0);
	wr32(f, kdc_crc32(f, *len));
	return f;
}

static void check_pulse_file(const uint8_t *f, size_t len, int slot, int ref_slot, const char *name)
{
	kdc_pulse_params_t p = { .slot = slot, .frames = 3, .ref_slot = ref_slot, .ref_temp_c = 25, .max_frames = 15 };
	uint8_t *o;
	size_t olen;
	char err[200];
	int r = kdc_wfm_make_pulse(f, len, &p, &o, &olen, err, sizeof err);
	if (r) fprintf(stderr, "%s: %s\n", name, err);
	CHECK(r == 0);
	if (r) return;
	kdc_wfm_info_t a, b;
	CHECK(kdc_wfm_parse(f, len, &a, NULL, 0) == 0);
	CHECK(kdc_wfm_parse(o, olen, &b, NULL, 0) == 0);
	CHECK(olen == len);                       /* never changes the driver's buffer size */
	CHECK(b.checksum_ok == a.checksum_ok);
	CHECK(!memcmp(o + 4, f + 4, KDC_WFM_HDR_SIZE - 4));   /* header untouched */
	CHECK(olen <= 4u * 1024 * 1024);
	kdc_pulse_info_t pi;
	CHECK(kdc_wfm_find_pulse(o, olen, slot, &pi) == 1);
	CHECK(kdc_wfm_find_pulse(f, len, slot, NULL) == 0);
	/* Every other mode is byte-identical in what it drives. */
	for (int m = 0; m < a.modes; m++) {
		if (m == slot) continue;
		for (int t = 0; t < a.temps; t++) {
			uint64_t na, nb;
			size_t oa, ob;
			kdc_wfm_phase(f, len, &a, m, t, &na, &oa);
			kdc_wfm_phase(o, olen, &b, m, t, &nb, &ob);
			CHECK(na == nb && !memcmp(f + oa, o + ob, na * 512));
		}
	}
	/* The slot: frames_per_temp drive frames + 1 zero tail; black for new=0,
	 * white for new=15, nothing else, independent of old. */
	for (int t = 0; t < b.temps; t++) {
		uint64_t n;
		size_t off;
		kdc_wfm_phase(o, olen, &b, slot, t, &n, &off);
		CHECK(n == (uint64_t)pi.frames_per_temp[t] + 1);
		CHECK(pi.frames_per_temp[t] >= 1 && pi.frames_per_temp[t] <= 15);
		for (int old = 0; old < 16; old++) {
			CHECK(kdc_wfm_drive_frames(o, olen, &b, slot, t, old, 0) == pi.frames_per_temp[t]);
			CHECK(kdc_wfm_drive_frames(o, olen, &b, slot, t, old, 15) == pi.frames_per_temp[t]);
			/* codes 1 and 14 are short pulses too; 2..13 hold */
			CHECK(kdc_wfm_drive_frames(o, olen, &b, slot, t, old, 1) == pi.frames_per_temp[t]);
			CHECK(kdc_wfm_drive_frames(o, olen, &b, slot, t, old, 14) == pi.frames_per_temp[t]);
			CHECK(kdc_wfm_drive_frames(o, olen, &b, slot, t, old, 8) == 0);
			CHECK(kdc_wfm_drive_frames(o, olen, &b, slot, t, old, 2) == 0);
			CHECK(kdc_wfm_drive_frames(o, olen, &b, slot, t, old, 13) == 0);
		}
		const uint8_t *fr0 = o + off;
		int idx_b = (1 << 8) | (0 << 4) | 9, idx_w = (3 << 8) | (15 << 4) | 2;
		CHECK(((fr0[idx_b >> 1] >> ((idx_b & 1) * 4)) & 0xF) == 1);
		CHECK(((fr0[idx_w >> 1] >> ((idx_w & 1) * 4)) & 0xF) == 2);
	}
	/* Patching twice is refused. */
	uint8_t *o2;
	size_t olen2;
	CHECK(kdc_wfm_make_pulse(o, olen, &p, &o2, &olen2, err, sizeof err) != 0);
	printf("  %s: slot %d frames/temp:", name, slot);
	for (int t = 0; t < b.temps; t++) printf(" %d", pi.frames_per_temp[t]);
	printf("\n");
	free(o);
}

static int nib(const uint8_t *frame, int old4, int new4)
{
	int i = (new4 << 4) | old4;
	return (frame[i >> 1] >> ((i & 1) * 4)) & 0xF;
}

/* Dual waveform on a real file: short pulses in slot 7, long in slot 1. */
static void check_dual_file(const uint8_t *f, size_t len)
{
	kdc_pulse_params_t p = { .slot = 7, .frames = 1, .ref_slot = 1, .ref_temp_c = 25,
	                         .max_frames = 15, .long_slot = 1, .long_frames = 6 };
	uint8_t *o;
	size_t olen;
	char err[200];
	int r = kdc_wfm_make_pulse(f, len, &p, &o, &olen, err, sizeof err);
	if (r) fprintf(stderr, "dual: %s\n", err);
	CHECK(r == 0);
	if (r) return;
	CHECK(olen == len);
	kdc_wfm_info_t a, b;
	kdc_wfm_parse(f, len, &a, NULL, 0);
	kdc_wfm_parse(o, olen, &b, NULL, 0);
	kdc_pulse_info_t pi;
	CHECK(kdc_wfm_find_pulse(o, olen, 7, &pi) == 1);
	CHECK(pi.long_slot == 1 && pi.long_frames == 6);
	/* other modes untouched */
	for (int m = 0; m < a.modes; m++) {
		if (m == 7 || m == 1) continue;
		for (int t = 0; t < a.temps; t++) {
			uint64_t na, nb;
			size_t oa, ob;
			kdc_wfm_phase(f, len, &a, m, t, &na, &oa);
			kdc_wfm_phase(o, olen, &b, m, t, &nb, &ob);
			CHECK(na == nb && !memcmp(f + oa, o + ob, na * 512));
		}
	}
	for (int t = 0; t < b.temps; t++) {
		int fs = pi.frames_per_temp[t], fl = pi.long_frames_per_temp[t];
		CHECK(fl > fs);
		uint64_t n;
		size_t off;
		/* short slot: codes 0,1 black and 14,15 white for fs frames */
		kdc_wfm_phase(o, olen, &b, 7, t, &n, &off);
		CHECK(n == (uint64_t)fs + 1);
		for (int fr = 0; fr < (int)n; fr++) {
			const uint8_t *F = o + off + (size_t)fr * 512;
			int on = fr < fs;
			CHECK(nib(F, 5, 0) == (on ? 1 : 0) && nib(F, 5, 1) == (on ? 1 : 0));
			CHECK(nib(F, 5, 14) == (on ? 2 : 0) && nib(F, 5, 15) == (on ? 2 : 0));
			CHECK(nib(F, 5, 2) == 0 && nib(F, 5, 8) == 0 && nib(F, 5, 13) == 0);
		}
		/* long slot: 0/15 long, 1/14 short, rest hold */
		kdc_wfm_phase(o, olen, &b, 1, t, &n, &off);
		CHECK(n == (uint64_t)fl + 1);
		for (int fr = 0; fr < (int)n; fr++) {
			const uint8_t *F = o + off + (size_t)fr * 512;
			int lon = fr < fl, son = fr < fs;
			for (int old = 0; old < 16; old += 5) {
				CHECK(nib(F, old, 0) == (lon ? 1 : 0) && nib(F, old, 15) == (lon ? 2 : 0));
				CHECK(nib(F, old, 1) == (son ? 1 : 0) && nib(F, old, 14) == (son ? 2 : 0));
				CHECK(nib(F, old, 7) == 0 && nib(F, old, 2) == 0 && nib(F, old, 13) == 0);
			}
		}
	}
	printf("  dual: short frames/temp:");
	for (int t = 0; t < b.temps; t++) printf(" %d", pi.frames_per_temp[t]);
	printf("\n        long frames/temp: ");
	for (int t = 0; t < b.temps; t++) printf(" %d", pi.long_frames_per_temp[t]);
	printf("\n");
	free(o);
}

static void test_wfm(const char *real)
{
	printf("waveform patching\n");
	size_t len;
	uint8_t *f = synth_wfm(&len);
	kdc_wfm_info_t wi;
	char err[200];
	CHECK(kdc_wfm_parse(f, len, &wi, err, sizeof err) == 0);
	CHECK(wi.checksum_ok);
	CHECK(kdc_wfm_drive_frames(f, len, &wi, 1, 1, 15, 0) == 4);
	check_pulse_file(f, len, 2, 1, "synthetic");
	{   /* slot 3 shares its phase block with mode 0: must be refused */
		uint8_t *o3;
		size_t l3;
		kdc_pulse_params_t p3 = { .slot = 3, .frames = 3, .ref_slot = 1, .ref_temp_c = 25 };
		CHECK(kdc_wfm_make_pulse(f, len, &p3, &o3, &l3, err, sizeof err) != 0);
		printf("  shared slot refused: %s\n", err);
	}
	/* scaling: ref range (25 C -> index 1) has 4 DU frames: 3*6/4=4.5->5, 3, 3*2/4=1.5->2 */
	uint8_t *o;
	size_t olen;
	kdc_pulse_params_t p = { .slot = 2, .frames = 3, .ref_slot = 1, .ref_temp_c = 25 };
	kdc_wfm_make_pulse(f, len, &p, &o, &olen, err, sizeof err);
	kdc_pulse_info_t pi;
	kdc_wfm_find_pulse(o, olen, 2, &pi);
	CHECK(pi.frames_per_temp[0] == 5 && pi.frames_per_temp[1] == 3 && pi.frames_per_temp[2] == 2);
	free(o);
	free(f);

	if (real) {
		uint8_t *rf;
		size_t rlen;
		if (kdc_read_file(real, &rf, &rlen) == 0) {
			check_pulse_file(rf, rlen, 7, 1, real);
			check_dual_file(rf, rlen);
			free(rf);
		} else {
			fprintf(stderr, "cannot read %s\n", real);
			failures++;
		}
	}
}

/* ---- controller on the simulated panel ------------------------------- */

static kdc_t *open_sim(int w, int h, kdc_mode_t mode, int max_bands)
{
	kdc_config_t c;
	kdc_config_default(&c);
	c.sim = 1;
	c.sim_width = w;
	c.sim_height = h;
	c.mode = mode;
	c.max_bands = max_bands;
	if (mode == KDC_MODE_GRAY4) {
		/* Simulated physics: 1/6 per pulse, shades 1/3 apart. */
		for (int a = 0; a < 4; a++)
			for (int b = 0; b < 4; b++)
				c.dose[a][b] = (uint8_t)(2 * abs(a - b));
	}
	kdc_t *k = NULL;
	int r = kdc_open(&k, &c);
	if (r) fprintf(stderr, "kdc_open: %d %s\n", r, k ? kdc_last_error(k) : "");
	return r ? NULL : k;
}

static void fill(uint8_t *f, int w, kdc_rect_t r, uint8_t v)
{
	for (int y = r.y; y < r.y + r.h; y++)
		memset(f + (size_t)y * w + r.x, v, (size_t)r.w);
}

static int count_diff(const uint8_t *opt, const uint8_t *want, size_t n, int tol)
{
	int bad = 0;
	for (size_t i = 0; i < n; i++)
		if (abs(opt[i] - want[i]) > tol) bad++;
	return bad;
}

static void test_dither_exact(int max_bands)
{
	printf("dither: exact pulse accounting (max_bands=%d)\n", max_bands);
	const int W = 300, H = 400;
	kdc_t *k = open_sim(W, H, KDC_MODE_DITHER, max_bands);
	CHECK(k);
	if (!k) return;
	uint64_t p0 = kdc_sim_pulses(k);
	kdc_rect_t a = { 10, 10, 40, 30 }, b = { 230, 350, 50, 40 };

	uint8_t *f = kdc_frame_begin(k, 0);
	memset(f, 255, (size_t)W * H);
	fill(f, W, a, 0);
	kdc_frame_publish(k, NULL, 0);
	CHECK(kdc_flush(k, 5000) == 0);
	/* each of a's pixels: 6 pulses (budget_black) */
	CHECK(kdc_sim_pulses(k) - p0 == (uint64_t)a.w * a.h * 6);

	/* Move a, add b far away: union of damage, both bands. Only changed
	 * pixels may be pulsed, even when bands get merged over idle tiles. */
	uint64_t p1 = kdc_sim_pulses(k);
	f = kdc_frame_begin(k, 1);
	fill(f, W, a, 255);
	kdc_rect_t a2 = { 20, 15, 40, 30 };
	fill(f, W, a2, 0);
	fill(f, W, b, 0);
	kdc_rect_t dmg[3] = { a, a2, b };
	kdc_frame_publish(k, dmg, 3);
	CHECK(kdc_flush(k, 5000) == 0);
	/* changed pixels: a xor a2 (each 6 pulses) + b */
	int ax = 0;
	for (int y = 0; y < H; y++)
		for (int x = 0; x < W; x++) {
			int in1 = x >= a.x && x < a.x + a.w && y >= a.y && y < a.y + a.h;
			int in2 = x >= a2.x && x < a2.x + a2.w && y >= a2.y && y < a2.y + a2.h;
			ax += in1 != in2;
		}
	CHECK(kdc_sim_pulses(k) - p1 == (uint64_t)(ax + b.w * b.h) * 6);

	uint8_t *want = malloc((size_t)W * H);
	memcpy(want, f, (size_t)W * H);
	CHECK(count_diff(kdc_sim_optical(k), want, (size_t)W * H, 1) == 0);

	/* Idle: no more pulses. */
	uint64_t p2 = kdc_sim_pulses(k);
	kdc_frame_begin(k, 1);
	kdc_frame_publish(k, NULL, 0);   /* same content */
	kdc_flush(k, 5000);
	CHECK(kdc_sim_pulses(k) == p2);

	kdc_stats_t st;
	kdc_get_stats(k, &st);
	printf("  scans %llu, updates %llu, frames %llu, dropped %llu\n",
	       (unsigned long long)st.scans, (unsigned long long)st.updates,
	       (unsigned long long)st.frames_in, (unsigned long long)st.frames_dropped);
	CHECK(st.scans >= 12);   /* two changes, 6 scans each */
	free(want);
	kdc_close(k);
}

static unsigned rng = 12345;
static unsigned rnd(void) { rng = rng * 1103515245u + 12345u; return rng >> 8; }

static void test_dither_stress(void)
{
	printf("dither: random frames, retargets, frame drops\n");
	const int W = 257, H = 190;   /* not tile multiples */
	kdc_t *k = open_sim(W, H, KDC_MODE_DITHER, 3);
	CHECK(k);
	if (!k) return;
	for (int it = 0; it < 300; it++) {
		uint8_t *f = kdc_frame_begin(k, 1);
		int n = 1 + rnd() % 4;
		kdc_rect_t d[4];
		for (int j = 0; j < n; j++) {
			kdc_rect_t r = { (int)(rnd() % W), (int)(rnd() % H), 1 + (int)(rnd() % 80), 1 + (int)(rnd() % 80) };
			if (r.x + r.w > W) r.w = W - r.x;
			if (r.y + r.h > H) r.h = H - r.y;
			for (int y = r.y; y < r.y + r.h; y++)
				for (int x = r.x; x < r.x + r.w; x++)
					f[(size_t)y * W + x] = (uint8_t)rnd();   /* dithered noise */
			d[j] = r;
		}
		kdc_frame_publish(k, d, n);
		if (it % 37 == 0) kdc_flush(k, 5000);
	}
	CHECK(kdc_flush(k, 10000) == 0);
	/* Panel must equal the dithered last frame. Re-quantize with the same
	 * thresholds by asking a clean for comparison: optical after clean is
	 * the rendered targets, which must already match. */
	uint8_t *before = malloc((size_t)W * H);
	memcpy(before, kdc_sim_optical(k), (size_t)W * H);
	kdc_clean(k);
	kdc_flush(k, 5000);
	int bad = count_diff(kdc_sim_optical(k), before, (size_t)W * H, 1);
	if (bad) fprintf(stderr, "  %d pixels differ from targets\n", bad);
	CHECK(bad == 0);
	kdc_stats_t st;
	kdc_get_stats(k, &st);
	CHECK(st.cleans == 2);
	printf("  scans %llu, frames %llu, dropped %llu\n", (unsigned long long)st.scans,
	       (unsigned long long)st.frames_in, (unsigned long long)st.frames_dropped);
	free(before);
	kdc_close(k);
}

static void test_gray(void)
{
	printf("gray4: doses and retargeting\n");
	const int W = 128, H = 96;
	kdc_t *k = open_sim(W, H, KDC_MODE_GRAY4, 4);
	CHECK(k);
	if (!k) return;
	static const uint8_t shade[4] = { 0, 85, 170, 255 };
	uint8_t *want = malloc((size_t)W * H);
	for (int it = 0; it < 60; it++) {
		uint8_t *f = kdc_frame_begin(k, 1);
		for (int j = 0; j < 3; j++) {
			kdc_rect_t r = { (int)(rnd() % (W - 20)), (int)(rnd() % (H - 20)), 20, 20 };
			fill(f, W, r, shade[rnd() % 4]);
		}
		kdc_frame_publish(k, NULL, 0);
		if (it % 5 == 4) {
			/* settle: every pixel must sit on its target shade */
			CHECK(kdc_flush(k, 5000) == 0);
			memcpy(want, f, (size_t)W * H);
		}
	}
	CHECK(kdc_flush(k, 5000) == 0);
	/* Without retargets the linear sim lands exactly; with mid-way retargets
	 * the estimate can be off by one pulse (1/6 = 42.5). */
	const uint8_t *o = kdc_sim_optical(k);
	int hist[8] = {0};
	for (size_t i = 0; i < (size_t)W * H; i++) {
		int e = (abs(o[i] - want[i]) + 21) / 42;   /* in pulses */
		hist[e > 7 ? 7 : e]++;
	}
	printf("  error after retargets, pulses 0..7+:");
	for (int i = 0; i < 8; i++) printf(" %d", hist[i]);
	printf("\n");
	/* Open-loop rounding can compound across retargets; a clean resets it. */
	CHECK(hist[0] + hist[1] >= (W * H) * 95 / 100);
	kdc_clean(k);
	kdc_flush(k, 5000);
	CHECK(count_diff(kdc_sim_optical(k), want, (size_t)W * H, 1) == 0);
	free(want);
	kdc_close(k);
}

/* One retarget half-way: the estimate is off by at most one pulse. */
static void test_gray_retarget(void)
{
	printf("gray4: single mid-way retarget\n");
	const int W = 64, H = 64;
	kdc_config_t c;
	kdc_config_default(&c);
	c.sim = 1; c.sim_width = W; c.sim_height = H;
	c.mode = KDC_MODE_GRAY4;
	for (int a = 0; a < 4; a++)
		for (int b = 0; b < 4; b++)
			c.dose[a][b] = (uint8_t)(2 * abs(a - b));
	c.min_scan_interval_us = 20000;   /* make "half-way" deterministic */
	kdc_t *k = NULL;
	CHECK(kdc_open(&k, &c) == 0);
	if (!k) return;
	uint8_t *f = kdc_frame_begin(k, 0);
	memset(f, 0, (size_t)W * H);            /* W -> K: 6 pulses */
	kdc_frame_publish(k, NULL, 0);
	struct timespec ts = { 0, 50 * 1000000L };
	nanosleep(&ts, NULL);                   /* ~2-3 scans in */
	f = kdc_frame_begin(k, 1);
	memset(f, 170, (size_t)W * H);          /* retarget to L */
	kdc_frame_publish(k, NULL, 0);
	CHECK(kdc_flush(k, 5000) == 0);
	const uint8_t *o = kdc_sim_optical(k);
	int worst = 0;
	for (int i = 0; i < W * H; i++) {
		int e = abs(o[i] - 170);
		if (e > worst) worst = e;
	}
	printf("  worst error %d (one pulse = 42.5)\n", worst);
	CHECK(worst <= 43);
	kdc_close(k);
}

static void test_args(void)
{
	printf("argument checks\n");
	kdc_config_t c;
	kdc_config_default(&c);
	c.sim = 1;
	c.budget_black = 61;
	kdc_t *k = NULL;
	CHECK(kdc_open(&k, &c) == KDC_ERR_ARG);
	kdc_close(k);

	/* budget 0 = automatic: 18 ref frames / 3-frame sim pulse = 6 */
	kdc_config_default(&c);
	c.sim = 1; c.sim_width = 64; c.sim_height = 64;
	k = NULL;
	CHECK(kdc_open(&k, &c) == KDC_OK);
	kdc_stats_t st;
	kdc_get_stats(k, &st);
	CHECK(st.budget_black == 6 && st.budget_white == 6 && st.pulse_frames_ref == 3);
	kdc_close(k);
}

static void test_dose_file(void)
{
	printf("dose files\n");
	char path[] = "/tmp/kdc_test_dosesXXXXXX";
	int fd = mkstemp(path);
	CHECK(fd >= 0);
	FILE *f = fdopen(fd, "w");
	fputs("# test\ninstall 3\n#    K  D  L  W\nK    0  3  6 10\nD   20  0  3  9\nL   22  4  0  8\nW   30 11  5  0\n", f);
	fclose(f);
	kdc_config_t c;
	kdc_config_default(&c);
	char err[200];
	CHECK(kdc_config_load_doses(&c, path, err, sizeof err) == 0);
	CHECK(c.dose_pulse_frames == 3 && c.dose[3][0] == 30 && c.dose[0][3] == 10 && c.dose[2][1] == 4);

	/* sim pulse length is 3: calibrated budgets come from W->K and K->W */
	c.sim = 1; c.sim_width = 64; c.sim_height = 64;
	kdc_t *k = NULL;
	CHECK(kdc_open(&k, &c) == KDC_OK);
	kdc_stats_t st;
	if (k) {
		kdc_get_stats(k, &st);
		CHECK(st.budget_black == 30 && st.budget_white == 10);
	}
	kdc_close(k);

	/* calibrated for another pulse length: refused */
	c.dose_pulse_frames = 1;
	k = NULL;
	CHECK(kdc_open(&k, &c) == KDC_ERR_ARG);
	if (k) printf("  mismatch refused: %s\n", kdc_last_error(k));
	kdc_close(k);

	/* malformed: nonzero diagonal */
	f = fopen(path, "w");
	fputs("install 1\n1 3 6 10\n20 0 3 9\n22 4 0 8\n30 11 5 0\n", f);
	fclose(f);
	CHECK(kdc_config_load_doses(&c, path, err, sizeof err) < 0);
	printf("  bad file rejected: %s\n", err);
	/* the shipped preset parses */
	kdc_config_t c2;
	kdc_config_default(&c2);
	CHECK(kdc_config_load_doses(&c2, "presets/pw4_install1.doses", err, sizeof err) == 0);
	CHECK(c2.dose_pulse_frames == 1 && c2.dose[3][0] == 30);
	CHECK(c2.dark_tau > 11.0 && c2.dark_tau < 11.2 && c2.light_p > 1.2 && c2.light_p < 1.3);
	CHECK(c2.have_steps == 3 && c2.dark_steps[4] > 0.29 && c2.light_steps[6] == 0.0);
	CHECK(c2.have_routes && c2.route[1][0] < -0.1 && c2.route[2][0] > 0.4);
	kdc_config_t c3;
	kdc_config_default(&c3);
	CHECK(kdc_config_load_doses(&c3, "presets/pw4_install1_long3_video.doses", err, sizeof err) == 0);
	CHECK(c3.have_long_steps == 3 && c3.long_ratio == 1.5 && c3.long_short_max == 3 &&
	      c3.settle_tol == 0.2 && c3.rail_tol == 0.3 && !c3.have_routes);
	unlink(path);
}

/* State model on a simulated panel that follows the same curves. */
static int model_steps, model_dual;
static double model_route_lkl;          /* route L > K > L (history test) */
static kdc_t *open_model(int w, int h, kdc_mode_t mode)
{
	kdc_config_t c;
	kdc_config_default(&c);
	c.sim = 1; c.sim_width = w; c.sim_height = h;
	c.mode = mode;
	c.dark_tau = 11.1; c.dark_p = 1.02;     /* the PW4 install-1 curves */
	c.light_tau = 6.3; c.light_p = 1.24;
	if (model_dual) {
		/* Long pulses: ~3x the short step at mid grays, far stronger near
		 * black (where the measured short step collapses). */
		static const double dl[7] = { 0.050, 0.200, 0.450, 0.600, 0.800, 0.700, 0.500 };
		static const double ll[7] = { 0.700, 0.900, 0.900, 0.800, 0.600, 0.150, 0.000 };
		memcpy(c.dark_long_steps, dl, sizeof dl);
		memcpy(c.light_long_steps, ll, sizeof ll);
		c.have_long_steps = 3;
	}
	if (model_steps) {                      /* the jointly fitted step tables */
		static const double ds[7] = { 0.000, 0.018, 0.096, 0.161, 0.295, 0.245, 0.163 };
		static const double ls[7] = { 0.228, 0.350, 0.377, 0.350, 0.298, 0.050, 0.000 };
		memcpy(c.dark_steps, ds, sizeof ds);
		memcpy(c.light_steps, ls, sizeof ls);
		c.have_steps = 3;
	}
	if (model_route_lkl != 0) {
		c.route[2][0] = model_route_lkl;
		c.have_routes = 1;
	}
	kdc_t *k = NULL;
	if (kdc_open(&k, &c) != KDC_OK) {
		fprintf(stderr, "open_model: %s\n", k ? kdc_last_error(k) : "?");
		kdc_close(k);
		return NULL;
	}
	return k;
}

/* Worst |optical - target| over the frame, in 8-bit units. Black targets
 * are measured against the black floor instead: with the step tables the
 * model (like the panel) stops where a pulse gains < MIN_GAIN, ~0.2 level
 * above GC16 black. Anything within FLOOR of black counts as reached. */
#define FLOOR 20
static int worst_err(kdc_t *k, const uint8_t *want, int n)
{
	const uint8_t *o = kdc_sim_optical(k);
	int worst = 0;
	for (int i = 0; i < n; i++) {
		int e = abs(o[i] - want[i]);
		if (want[i] == 0 && o[i] <= FLOOR) e = 0;
		if (e > worst) worst = e;
	}
	return worst;
}

static void test_model_gray(void)
{
	printf("model gray4: random retargets with frame drops\n");
	const int W = 160, H = 120;
	kdc_t *k = open_model(W, H, KDC_MODE_GRAY4);
	CHECK(k);
	if (!k) return;
	kdc_stats_t st;
	kdc_get_stats(k, &st);
	CHECK(st.model == 1);
	static const uint8_t shade[4] = { 0, 85, 170, 255 };
	uint8_t *want = malloc((size_t)W * H);
	for (int it = 0; it < 120; it++) {
		uint8_t *f = kdc_frame_begin(k, 1);
		for (int j = 0; j < 4; j++) {
			kdc_rect_t r = { (int)(rnd() % (W - 24)), (int)(rnd() % (H - 24)), 24, 24 };
			fill(f, W, r, shade[rnd() % 4]);
		}
		memcpy(want, f, (size_t)W * H);
		kdc_frame_publish(k, NULL, 0);
		if (it % 9 == 8) { struct timespec ts = { 0, 2000000 }; nanosleep(&ts, NULL); }
	}
	CHECK(kdc_flush(k, 20000) == 0);
	/* Bound: a pixel stops when the next pulse would overshoot more than it
	 * corrects, so the floor is half of one pulse step. With 1-frame pulses
	 * a step is at most ~0.35 level (~30/255) at mid grays. */
	int worst = worst_err(k, want, W * H);
	printf("  worst error %d / 255 (bound: half a pulse step, ~16)\n", worst);
	CHECK(worst <= 16);
	/* idle afterwards: no more pulses */
	uint64_t p0 = kdc_sim_pulses(k);
	kdc_frame_begin(k, 1);
	kdc_frame_publish(k, NULL, 0);
	kdc_flush(k, 5000);
	CHECK(kdc_sim_pulses(k) == p0);
	free(want);
	kdc_close(k);
}

/* The on-device failure case: rectangles sweeping across bands, each pixel
 * retargeted long before it settles. Trails must return to the band shade. */
static void test_model_sweep(void)
{
	printf("model gray4: transition-test sweep (trails)\n");
	const int W = 400, H = 160, bh = 40, rw = 30;
	kdc_t *k = open_model(W, H, KDC_MODE_GRAY4);
	CHECK(k);
	if (!k) return;
	static const uint8_t shade[4] = { 0, 85, 170, 255 };
	uint8_t *f = kdc_frame_begin(k, 0);
	for (int b = 0; b < 4; b++) memset(f + (size_t)b * bh * W, shade[b], (size_t)bh * W);
	kdc_frame_publish(k, NULL, 0);
	kdc_flush(k, 5000);
	kdc_clean(k);
	kdc_flush(k, 5000);
	int px = 0;
	for (int x = 4; x <= W - rw; x += 4) {          /* 4 px per frame, faster than settling */
		f = kdc_frame_begin(k, 1);
		for (int b = 0; b < 4; b++)
			for (int y = b * bh + 8; y < b * bh + 32; y++) {
				memset(f + (size_t)y * W + px, shade[b], (size_t)rw);
				memset(f + (size_t)y * W + x, shade[(b + 2) % 4], (size_t)rw);
			}
		kdc_rect_t d = { px, 0, x - px + rw, H };
		kdc_frame_publish(k, &d, 1);
		px = x;
		struct timespec ts = { 0, 300000 };
		nanosleep(&ts, NULL);
	}
	CHECK(kdc_flush(k, 20000) == 0);
	uint8_t *want = malloc((size_t)W * H);
	memcpy(want, kdc_frame_begin(k, 1), (size_t)W * H);
	int worst = worst_err(k, want, W * H);
	printf("  worst trail/rect error %d / 255 (bound: half a pulse step, ~16)\n", worst);
	CHECK(worst <= 16);
	free(want);
	kdc_close(k);
}

static void test_model_dither(void)
{
	printf("model dither: asymmetric curves, retargets\n");
	const int W = 128, H = 96;
	kdc_t *k = open_model(W, H, KDC_MODE_DITHER);
	CHECK(k);
	if (!k) return;
	uint8_t *want = malloc((size_t)W * H);
	for (int it = 0; it < 60; it++) {
		uint8_t *f = kdc_frame_begin(k, 1);
		kdc_rect_t r = { (int)(rnd() % (W - 30)), (int)(rnd() % (H - 30)), 30, 30 };
		fill(f, W, r, (rnd() & 1) ? 0 : 255);
		memcpy(want, f, (size_t)W * H);
		kdc_frame_publish(k, NULL, 0);
	}
	CHECK(kdc_flush(k, 20000) == 0);
	int worst = worst_err(k, want, W * H);
	printf("  worst error %d / 255\n", worst);
	CHECK(worst <= 10);
	free(want);
	kdc_close(k);
}

/* Scans to settle a full-screen white -> black change. */
static uint64_t scans_to_black(int dual, uint64_t *long_updates)
{
	model_steps = 1;
	model_dual = dual;
	const int W = 96, H = 64;
	kdc_t *k = open_model(W, H, KDC_MODE_GRAY4);
	model_dual = 0;
	if (!k) return 0;
	uint8_t *f = kdc_frame_begin(k, 0);
	memset(f, 0, (size_t)W * H);
	kdc_frame_publish(k, NULL, 0);
	CHECK(kdc_flush(k, 20000) == 0);
	kdc_stats_t st;
	kdc_get_stats(k, &st);
	uint8_t want[96 * 64];
	memset(want, 0, sizeof want);
	int worst = worst_err(k, want, W * H);
	printf("  %s: %llu scans, %llu long updates, worst %d/255\n", dual ? "dual " : "short",
	       (unsigned long long)st.scans, (unsigned long long)st.long_updates, worst);
	CHECK(worst <= 16);
	if (long_updates) *long_updates = st.long_updates;
	kdc_close(k);
	return st.scans;
}

static void test_dual(void)
{
	printf("dual pulses: white -> black, short only vs dual\n");
	uint64_t lu_s, lu_d;
	uint64_t s_short = scans_to_black(0, &lu_s), s_dual = scans_to_black(1, &lu_d);
	CHECK(lu_s == 0 && lu_d > 0);
	CHECK(s_dual * 2 < s_short);          /* long pulses must pay off near black */

	printf("dual pulses: random gray retargets and sweep\n");
	model_steps = 1;
	model_dual = 1;
	test_model_gray();
	test_model_sweep();
	model_dual = 0;
}

/* Mean optical value of a full-frame shade sequence (flush after each). */
static double shade_trip(kdc_t *k, int w, int h, const uint8_t *seq, int n)
{
	for (int i = 0; i < n; i++) {
		uint8_t *f = kdc_frame_begin(k, 0);
		memset(f, seq[i], (size_t)w * h);
		kdc_frame_publish(k, NULL, 0);
		CHECK(kdc_flush(k, 20000) == 0);
	}
	const uint8_t *o = kdc_sim_optical(k);
	double sum = 0;
	for (int i = 0; i < w * h; i++) sum += o[i];
	return sum / (w * h);
}

/* History: the simulated panel has none, so a route correction must show up
 * as exactly that offset after the round trip, and nowhere else. */
static void test_history(void)
{
	printf("history: route corrections\n");
	char path[] = "/tmp/kdc_test_routesXXXXXX";
	int fd = mkstemp(path);
	CHECK(fd >= 0);
	FILE *f = fdopen(fd, "w");
	fputs("install 1\ndark_steps 0 .018 .096 .161 .295 .245 .163\n"
	      "light_steps .228 .35 .377 .35 .298 .05 0\nroute L K -0.30\nroute D W 0.2\n"
	      "0 3 6 10\n20 0 3 9\n22 4 0 8\n30 11 5 0\n", f);
	fclose(f);
	kdc_config_t c;
	kdc_config_default(&c);
	char err[200];
	CHECK(kdc_config_load_doses(&c, path, err, sizeof err) == 0);
	CHECK(c.have_routes && c.route[2][0] == -0.30 && c.route[1][3] == 0.2 && c.route[0][2] == 0);
	f = fopen(path, "w");
	fputs("install 1\nroute L L 0.1\n0 3 6 10\n20 0 3 9\n22 4 0 8\n30 11 5 0\n", f);
	fclose(f);
	CHECK(kdc_config_load_doses(&c, path, err, sizeof err) < 0);
	printf("  bad route rejected: %s\n", err);
	unlink(path);

	const int W = 64, H = 64;
	static const uint8_t lkl[3] = { 170, 0, 170 }, ldl[3] = { 170, 85, 170 };
	model_steps = 1;
	kdc_t *k = open_model(W, H, KDC_MODE_GRAY4);
	CHECK(k);
	double plain = k ? shade_trip(k, W, H, lkl, 3) : 0;
	kdc_close(k);
	model_route_lkl = -0.30;                  /* comes back too dark: drive lighter */
	k = open_model(W, H, KDC_MODE_GRAY4);
	CHECK(k);
	kdc_stats_t st;
	if (k) { kdc_get_stats(k, &st); CHECK(st.history == 1); }
	double comp = k ? shade_trip(k, W, H, lkl, 3) : 0;
	kdc_close(k);
	k = open_model(W, H, KDC_MODE_GRAY4);   /* fresh panel: L > D > L has no route */
	double other = k ? shade_trip(k, W, H, ldl, 3) : 0;
	kdc_close(k);
	model_route_lkl = 0;
	model_steps = 0;
	printf("  L>K>L back at %.1f (plain %.1f, want 170 + ~25), L>D>L %.1f\n", comp, plain, other);
	CHECK(fabs(plain - 170) < 12);
	CHECK(comp - plain > 12 && comp - plain < 40);
	CHECK(fabs(other - 170) < 12);
}

int main(int argc, char **argv)
{
	test_wfm(argc > 1 ? argv[1] : NULL);
	test_args();
	test_dither_exact(4);
	test_dither_exact(1);
	test_dither_stress();
	test_gray();
	test_gray_retarget();
	test_dose_file();
	test_model_gray();
	test_model_sweep();
	test_model_dither();
	printf("-- again with jointly fitted step tables --\n");
	model_steps = 1;
	test_model_gray();
	test_model_sweep();
	test_model_dither();
	test_dual();
	test_history();
	if (failures) {
		printf("%d check(s) FAILED\n", failures);
		return 1;
	}
	printf("all tests passed\n");
	return 0;
}
