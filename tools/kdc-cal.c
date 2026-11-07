/*
 * kdc-cal: gray dose calibration ladders for kdc's KDC_MODE_GRAY4.
 *
 *   kdc-cal stage1 [N=40]       from white and from black
 *   kdc-cal stage2 WD WL [N]    from D and L (reached with WD / WL pulses from white)
 *   kdc-cal clear               full white GC16 (leaves the EPDC consistent)
 *   --long (after the stage)    ladders of LONG pulses (dual waveform, DU slot);
 *                               stage-2 pre-doses stay short pulses
 *
 * A stock GC16 refresh draws the layout: blocks with a background, numbered
 * patch positions, and reference stripes of D (0x55) and L (0xAA) right under
 * each patch line. Then the pulse waveform drives patch n with exactly n
 * pulses (after an optional pre-dose). The patch matching a stripe gives the
 * dose for that transition. The result stays on screen for reading or a photo;
 * run "clear" afterwards.
 *
 * Needs the pulse waveform installed (kdc-pulse.sh install N) and the Kindle
 * UI paused. Doses are in pulses of the installed length.
 */
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "kdc.h"
#include "kdc_dev.h"
#include "kdc_wfm.h"
#include "mxcfb_pw4.h"

#define PER_LINE 16
#define MARGIN 16
#define PITCH 65
#define PATCH 52
#define CMD_HOLD 0x88

static const uint8_t SHADE[4] = { 0x00, 0x55, 0xAA, 0xFF };   /* K D L W */

/* ---- 3x5 font -------------------------------------------------------- */
static const char *glyph(char c)
{
	switch (c) {
	case '0': return "111101101101111";
	case '1': return "010110010010111";
	case '2': return "111001111100111";
	case '3': return "111001111001111";
	case '4': return "101101111001001";
	case '5': return "111100111001111";
	case '6': return "111100111101111";
	case '7': return "111001001001001";
	case '8': return "111101111101111";
	case '9': return "111101111001111";
	case 'K': return "101101110101101";
	case 'D': return "110101101101110";
	case 'L': return "100100100100111";
	case 'W': return "101101101111101";
	case '>': return "100010001010100";
	case '+': return "000010111010000";
	default:  return "000000000000000";
	}
}

static void text(uint8_t *img, int w, int x, int y, int scale, const char *s, uint8_t v)
{
	for (; *s; s++, x += 4 * scale) {
		const char *g = glyph(*s);
		for (int r = 0; r < 5; r++)
			for (int c = 0; c < 3; c++)
				if (g[r * 3 + c] == '1')
					for (int yy = 0; yy < scale; yy++)
						memset(img + (size_t)(y + r * scale + yy) * w + x + c * scale, v, (size_t)scale);
	}
}

static void rect(uint8_t *img, int w, int x, int y, int rw, int rh, uint8_t v)
{
	for (int yy = y; yy < y + rh; yy++)
		memset(img + (size_t)yy * w + x, v, (size_t)rw);
}

/* ---- layout ---------------------------------------------------------- */
typedef struct {
	int bg;                     /* shade index of background and patches */
	int pre_dir, pre_n;         /* pre-dose: -1 black / +1 white, pulses */
	int dir;                    /* ladder direction */
	char title[16];
	int y;                      /* top of block */
} block_t;

#define TITLE_H 28
#define LABEL_H 18
#define STRIPE_H 22
#define LINE_H (LABEL_H + PATCH + 2 * STRIPE_H + 14)

static int lines_for(int n) { return (n + PER_LINE - 1) / PER_LINE; }
static int block_h(int n) { return TITLE_H + lines_for(n) * LINE_H + 10; }

static void patch_pos(const block_t *b, int i, int *x, int *y)   /* i: 1-based */
{
	int line = (i - 1) / PER_LINE, col = (i - 1) % PER_LINE;
	*x = MARGIN + col * PITCH;
	*y = b->y + TITLE_H + line * LINE_H + LABEL_H;
}

static void draw_layout(uint8_t *img, int w, const block_t *bl, int nb, int n)
{
	for (int k = 0; k < nb; k++) {
		const block_t *b = &bl[k];
		uint8_t bg = SHADE[b->bg], ink = b->bg >= 2 ? 0x00 : 0xFF;
		rect(img, w, 0, b->y, w, block_h(n), bg);
		text(img, w, MARGIN, b->y + 4, 4, b->title, ink);
		for (int line = 0; line < lines_for(n); line++) {
			int sy = b->y + TITLE_H + line * LINE_H + LABEL_H + PATCH;
			rect(img, w, MARGIN, sy, PER_LINE * PITCH - (PITCH - PATCH), STRIPE_H, SHADE[1]);
			rect(img, w, MARGIN, sy + STRIPE_H, PER_LINE * PITCH - (PITCH - PATCH), STRIPE_H, SHADE[2]);
		}
		for (int i = 1; i <= n; i++) {
			int x, y;
			patch_pos(b, i, &x, &y);
			char num[4];
			snprintf(num, sizeof num, "%d", i);
			int tw = (int)strlen(num) * 12 - 3;
			text(img, w, x + (PATCH - tw) / 2, y - LABEL_H + 1, 3, num, ink);
			/* 2 px outline just outside the patch so white-on-white or
			 * black-on-black patches stay visible; the patch itself starts
			 * as the background. */
			rect(img, w, x - 2, y - 2, PATCH + 4, 2, ink);
			rect(img, w, x - 2, y + PATCH, PATCH + 4, 2, ink);
			rect(img, w, x - 2, y, 2, PATCH, ink);
			rect(img, w, x + PATCH, y, 2, PATCH, ink);
		}
	}
}

/* ---- device ---------------------------------------------------------- */
static int submit_wait(kdc_dev_t *d, kdc_rect_t r, int wfm, int full)
{
	uint32_t m;
	if (kdc_dev_update(d, r, wfm, full, &m, NULL) < 0) {
		fprintf(stderr, "kdc-cal: update failed: %s\n", strerror(errno));
		return -1;
	}
	if (kdc_dev_wait(d, m) < 0) {
		fprintf(stderr, "kdc-cal: wait failed: %s\n", strerror(errno));
		return -1;
	}
	return 0;
}

static void blit(kdc_dev_t *d, const uint8_t *img)
{
	for (int y = 0; y < d->h; y++) {
		uint8_t *dst = d->fb + (size_t)y * d->stride;
		const uint8_t *src = img + (size_t)y * d->w;
		if (d->inverted)
			for (int x = 0; x < d->w; x++) dst[x] = (uint8_t)~src[x];
		else
			memcpy(dst, src, (size_t)d->w);
	}
}

static int pulse_installed(kdc_dev_t *d, kdc_pulse_info_t *pi, int *temp_c, int need_long)
{
	uint32_t m;
	int slot = -1, du = -1;
	if (kdc_dev_update(d, (kdc_rect_t){ 0, 0, 1, 1 }, WAVEFORM_MODE_DU4, 0, &m, &slot) < 0)
		return -1;
	kdc_dev_wait(d, m);
	if (kdc_dev_update(d, (kdc_rect_t){ 0, 0, 1, 1 }, WAVEFORM_MODE_DU, 0, &m, &du) < 0)
		return -1;
	kdc_dev_wait(d, m);
	uint8_t *f;
	size_t len;
	if (kdc_read_file("/proc/eink/waveform/data", &f, &len) < 0)
		return -1;
	int r = kdc_wfm_find_pulse(f, len, slot, pi);
	if (r == 1 && need_long && (pi->long_slot <= 0 || pi->long_slot != du)) {
		fprintf(stderr, "kdc-cal: --long needs the dual waveform (kdc-pulse.sh install N L)\n");
		r = 0;
	}
	kdc_wfm_info_t wi;
	if (r == 1 && kdc_dev_temperature(d, temp_c) == 0 && kdc_wfm_parse(f, len, &wi, NULL, 0) == 0) {
		int t = kdc_wfm_temp_index(&wi, *temp_c);
		printf("pulse slot %d, %d frames/pulse at %d C (installed: %d)", slot,
		       pi->frames_per_temp[t], *temp_c, pi->frames);
		if (pi->long_slot > 0)
			printf("; long slot %d, %d frames/long pulse (installed: %d)", pi->long_slot,
			       pi->long_frames_per_temp[t], pi->long_frames);
		printf("\n");
	}
	free(f);
	return r == 1 ? 0 : -1;
}

static int run_ladder(kdc_dev_t *d, block_t *bl, int nb, int n, int use_long)
{
	int w = d->w, h = d->h;
	int total = 0;
	for (int k = 0; k < nb; k++) {
		bl[k].y = (k == 0 ? 8 : bl[k - 1].y + block_h(n));
		if (bl[k].pre_n + n > total) total = bl[k].pre_n + n;
	}
	if (bl[nb - 1].y + block_h(n) > h) {
		fprintf(stderr, "kdc-cal: layout does not fit (n=%d)\n", n);
		return -1;
	}
	uint8_t *img = malloc((size_t)w * h);
	if (!img) return -1;
	memset(img, 0xFF, (size_t)w * h);
	draw_layout(img, w, bl, nb, n);

	kdc_rect_t all = { 0, 0, w, h };
	/* Clean reference: white, then the layout, both FULL GC16. */
	uint8_t *white = malloc((size_t)w * h);
	memset(white, 0xFF, (size_t)w * h);
	blit(d, white);
	free(white);
	if (submit_wait(d, all, WAVEFORM_MODE_GC16, 1)) { free(img); return -1; }
	blit(d, img);
	if (submit_wait(d, all, WAVEFORM_MODE_GC16, 1)) { free(img); return -1; }

	/* Pulse scans. The fb holds HOLD everywhere except the commanded patches;
	 * the panel keeps showing the layout. Codes (kdc_wfm.h): short 1 / 14
	 * (short in both slots), long 0 / 15 (long under the DU slot). */
	uint8_t cb = d->inverted ? 0xEE : 0x11, cw = d->inverted ? 0x11 : 0xEE;
	uint8_t cbl = d->inverted ? 0xFF : 0x00, cwl = d->inverted ? 0x00 : 0xFF;
	int top = bl[0].y, bottom = bl[nb - 1].y + block_h(n);
	kdc_rect_t region = { 0, top, w, bottom - top };
	for (int s = 1; s <= total; s++) {
		int any_long = 0;
		for (int y = top; y < bottom; y++)
			memset(d->fb + (size_t)y * d->stride, CMD_HOLD, (size_t)w);
		for (int k = 0; k < nb; k++) {
			const block_t *b = &bl[k];
			int t = s - b->pre_n;      /* ladder step, <= 0 during pre-dose */
			for (int i = 1; i <= n; i++) {
				int dir = 0, lng = 0;
				if (t <= 0) dir = b->pre_dir;
				else if (i >= t) { dir = b->dir; lng = use_long; }
				if (!dir) continue;
				any_long |= lng;
				uint8_t code = dir < 0 ? (lng ? cbl : cb) : (lng ? cwl : cw);
				int x, y;
				patch_pos(b, i, &x, &y);
				for (int yy = y; yy < y + PATCH; yy++)
					memset(d->fb + (size_t)yy * d->stride + x, code, PATCH);
			}
		}
		if (submit_wait(d, region, any_long ? WAVEFORM_MODE_DU : WAVEFORM_MODE_DU4, 1)) { free(img); return -1; }
	}
	printf("%d pulse scans done; the ladders stay on screen. Run 'kdc-cal clear' when finished.\n", total);
	free(img);
	return 0;
}

static int usage(void)
{
	fputs("usage: kdc-cal stage1 [N] [--long] | stage2 WD WL [N] [--long] | clear\n", stderr);
	return 2;
}

int main(int argc, char **argv)
{
	if (argc < 2) return usage();
	kdc_config_t c;
	kdc_config_default(&c);
	/* Host preview: KDC_CAL_SIM=out.pgm renders on the simulated panel. */
	const char *sim_out = getenv("KDC_CAL_SIM");
	if (sim_out) c.sim = 1;
	kdc_dev_t d;
	char err[200];
	if (kdc_dev_open(&d, &c, err, sizeof err) < 0) {
		fprintf(stderr, "kdc-cal: %s\n", err);
		return 1;
	}
	int rc = 0;
	if (!strcmp(argv[1], "clear")) {
		for (int y = 0; y < d.h; y++) memset(d.fb + (size_t)y * d.stride, d.inverted ? 0x00 : 0xFF, (size_t)d.w);
		rc = submit_wait(&d, (kdc_rect_t){ 0, 0, d.w, d.h }, WAVEFORM_MODE_GC16, 1) ? 1 : 0;
		kdc_dev_close(&d);
		return rc;
	}
	kdc_pulse_info_t pi;
	int temp = 0;
	int use_long = 0;
	for (int i = 2; i < argc; i++)
		if (!strcmp(argv[i], "--long")) {
			use_long = 1;
			for (int j = i; j + 1 < argc; j++) argv[j] = argv[j + 1];
			argc--;
			break;
		}
	if (!sim_out && pulse_installed(&d, &pi, &temp, use_long) < 0) {
		fprintf(stderr, "kdc-cal: pulse waveform not installed (kdc-pulse.sh install N)\n");
		kdc_dev_close(&d);
		return 1;
	}
	kdc_dev_set_pwrdown(&d, 1000);
	block_t bl[4];
	memset(bl, 0, sizeof bl);
	int nb, n;
	if (!strcmp(argv[1], "stage1")) {
		n = argc > 2 ? atoi(argv[2]) : 40;
		bl[0] = (block_t){ .bg = 3, .dir = -1 };
		strcpy(bl[0].title, "W>");
		bl[1] = (block_t){ .bg = 0, .dir = +1 };
		strcpy(bl[1].title, "K>");
		nb = 2;
	} else if (!strcmp(argv[1], "stage2") && argc >= 4) {
		int wd = atoi(argv[2]), wl = atoi(argv[3]);
		n = argc > 4 ? atoi(argv[4]) : 32;
		if (wd < 1 || wd > 60 || wl < 1 || wl > 60) { kdc_dev_close(&d); return usage(); }
		bl[0] = (block_t){ .bg = 3, .pre_dir = -1, .pre_n = wd, .dir = +1 };
		strcpy(bl[0].title, "D>W");
		bl[1] = (block_t){ .bg = 3, .pre_dir = -1, .pre_n = wd, .dir = -1 };
		strcpy(bl[1].title, "D>K");
		bl[2] = (block_t){ .bg = 3, .pre_dir = -1, .pre_n = wl, .dir = +1 };
		strcpy(bl[2].title, "L>W");
		bl[3] = (block_t){ .bg = 3, .pre_dir = -1, .pre_n = wl, .dir = -1 };
		strcpy(bl[3].title, "L>K");
		nb = 4;
	} else {
		kdc_dev_close(&d);
		return usage();
	}
	if (n < 1 || n > 48) { kdc_dev_close(&d); return usage(); }
	if (use_long)
		for (int k = 0; k < nb; k++) strcat(bl[k].title, "+");   /* long-pulse ladder */
	rc = run_ladder(&d, bl, nb, n, use_long) ? 1 : 0;
	if (sim_out) {
		FILE *f = fopen(sim_out, "wb");
		if (f) {
			fprintf(f, "P5\n%d %d\n255\n", d.w, d.h);
			fwrite(d.opt8, 1, (size_t)d.w * d.h, f);
			fclose(f);
		}
	}
	kdc_dev_restore_pwrdown(&d);
	kdc_dev_close(&d);
	return rc;
}
