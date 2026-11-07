/*
 * kdc-wfm: inspect and patch PW4 waveform files for kdc.
 *
 *   kdc-wfm info FILE
 *   kdc-wfm probe [FBDEV]                     (on the Kindle)
 *   kdc-wfm patch IN OUT --slot N [--ref-slot N] [--frames K] [--max-frames M]
 *                        [--long-slot N --long-frames L]   (dual pulse waveform)
 *   kdc-wfm verify FILE --slot N
 */
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "kdc.h"
#include "kdc_dev.h"
#include "kdc_wfm.h"
#include "mxcfb_pw4.h"

static int usage(void)
{
	fputs("usage:\n"
	      "  kdc-wfm info FILE\n"
	      "  kdc-wfm probe [FBDEV]\n"
	      "  kdc-wfm patch IN OUT --slot N [--ref-slot N] [--frames K] [--max-frames M]\n"
	      "                       [--long-slot N --long-frames L]\n"
	      "  kdc-wfm verify FILE --slot N\n", stderr);
	return 2;
}

static int load(const char *path, uint8_t **f, size_t *len, kdc_wfm_info_t *wi)
{
	char err[160];
	if (kdc_read_file(path, f, len) < 0) {
		fprintf(stderr, "kdc-wfm: %s: %s\n", path, strerror(errno));
		return -1;
	}
	if (kdc_wfm_parse(*f, *len, wi, err, sizeof err) < 0) {
		fprintf(stderr, "kdc-wfm: %s: %s\n", path, err);
		free(*f);
		return -1;
	}
	return 0;
}

static int cmd_info(const char *path)
{
	uint8_t *f;
	size_t len;
	kdc_wfm_info_t wi;
	if (load(path, &f, &len, &wi)) return 1;
	printf("%s: %zu bytes, %d-bit, %d modes, %d temperature ranges\n",
	       path, len, wi.bit_depth, wi.modes, wi.temps);
	printf("mode_version 0x%02x  wf_version %u  wf_type 0x%02x  frame_rate 0x%02x  adv %u\n",
	       wi.mode_version, wi.wf_version, wi.wf_type, wi.frame_rate, wi.adv);
	printf("checksum 0x%08x (%s), file_length %u\n", wi.checksum,
	       wi.checksum_ok ? "valid" : "does not match", wi.file_length);
	printf("temperature ranges (C):");
	for (int t = 0; t < wi.temps; t++) printf(" %d", wi.trt[t]);
	printf("\n\n");
	int rt = kdc_wfm_temp_index(&wi, 25);
	printf("mode  frames@25C  W->K  K->W  frames per range\n");
	for (int m = 0; m < wi.modes; m++) {
		uint64_t n;
		size_t off;
		kdc_wfm_phase(f, len, &wi, m, rt, &n, &off);
		printf("%4d  %10llu  %4d  %4d  ", m, (unsigned long long)n,
		       kdc_wfm_drive_frames(f, len, &wi, m, rt, 15, 0),
		       kdc_wfm_drive_frames(f, len, &wi, m, rt, 0, 15));
		for (int t = 0; t < wi.temps; t++) {
			kdc_wfm_phase(f, len, &wi, m, t, &n, &off);
			printf("%llu ", (unsigned long long)n);
		}
		kdc_pulse_info_t pi;
		if (kdc_wfm_find_pulse(f, len, m, &pi) == 1)
			printf(" <- kdc pulse (%d frames/scan at ref%s)", pi.frames,
			       pi.long_slot > 0 ? ", dual" : "");
		printf("\n");
	}
	free(f);
	return 0;
}

static int cmd_probe(const char *fbdev)
{
	kdc_config_t c;
	kdc_config_default(&c);
	if (fbdev) c.fb_path = fbdev;
	kdc_dev_t d;
	char err[200];
	if (kdc_dev_open(&d, &c, err, sizeof err) < 0) {
		fprintf(stderr, "kdc-wfm: %s\n", err);
		return 1;
	}
	/* 1x1 partial update of an unchanged pixel: drives nothing. */
	static const int types[] = { WAVEFORM_MODE_DU4, WAVEFORM_MODE_DU };
	static const char *names[] = { "du4", "du" };
	for (int i = 0; i < 2; i++) {
		uint32_t m;
		int slot = -1;
		if (kdc_dev_update(&d, (kdc_rect_t){ 0, 0, 1, 1 }, types[i], 0, &m, &slot) < 0) {
			fprintf(stderr, "kdc-wfm: probe update failed: %s\n", strerror(errno));
			kdc_dev_close(&d);
			return 1;
		}
		kdc_dev_wait(&d, m);
		printf("%s=%d\n", names[i], slot);
	}
	kdc_dev_close(&d);
	return 0;
}

static int get_int(int argc, char **argv, const char *name, int def)
{
	for (int i = 0; i + 1 < argc; i++)
		if (!strcmp(argv[i], name))
			return atoi(argv[i + 1]);
	return def;
}

static int cmd_patch(const char *in, const char *out, int argc, char **argv)
{
	kdc_pulse_params_t p = {
		.slot = get_int(argc, argv, "--slot", -1),
		.frames = get_int(argc, argv, "--frames", 3),
		.ref_slot = get_int(argc, argv, "--ref-slot", 1),
		.ref_temp_c = 25,
		.max_frames = get_int(argc, argv, "--max-frames", 15),
		.long_slot = get_int(argc, argv, "--long-slot", 0),
		.long_frames = get_int(argc, argv, "--long-frames", 0),
	};
	if (p.slot < 0) {
		fprintf(stderr, "kdc-wfm: --slot is required (see kdc-wfm probe)\n");
		return 2;
	}
	uint8_t *f, *o;
	size_t len, olen;
	kdc_wfm_info_t wi;
	char err[200];
	if (load(in, &f, &len, &wi)) return 1;
	if (!wi.checksum_ok)
		fprintf(stderr, "kdc-wfm: warning: input checksum does not match (continuing)\n");
	if (kdc_wfm_make_pulse(f, len, &p, &o, &olen, err, sizeof err) < 0) {
		fprintf(stderr, "kdc-wfm: %s\n", err);
		free(f);
		return 1;
	}
	FILE *fo = fopen(out, "wb");
	if (!fo || fwrite(o, 1, olen, fo) != olen || fclose(fo)) {
		fprintf(stderr, "kdc-wfm: write %s: %s\n", out, strerror(errno));
		return 1;
	}
	kdc_pulse_info_t pi;
	kdc_wfm_find_pulse(o, olen, p.slot, &pi);
	printf("wrote %s (%zu bytes): slot %d = pulse, frames per temperature range:", out, olen, p.slot);
	for (int t = 0; t < wi.temps; t++) printf(" %d", pi.frames_per_temp[t]);
	printf("\n");
	if (pi.long_slot > 0) {
		printf("  slot %d = dual (long pulses), long frames per range:", pi.long_slot);
		for (int t = 0; t < wi.temps; t++) printf(" %d", pi.long_frames_per_temp[t]);
		printf("\n");
	}
	free(f);
	free(o);
	return 0;
}

static int cmd_verify(const char *path, int slot)
{
	uint8_t *f;
	size_t len;
	kdc_wfm_info_t wi;
	if (load(path, &f, &len, &wi)) return 1;
	kdc_pulse_info_t pi;
	int r = kdc_wfm_find_pulse(f, len, slot, &pi);
	free(f);
	if (r == 1 && pi.long_slot > 0)
		printf("pulse dual (long slot %d)\n", pi.long_slot);
	else
		printf("%s\n", r == 1 ? "pulse" : "stock");
	return r == 1 ? 0 : 3;
}

int main(int argc, char **argv)
{
	if (argc < 2) return usage();
	if (!strcmp(argv[1], "info") && argc == 3) return cmd_info(argv[2]);
	if (!strcmp(argv[1], "probe")) return cmd_probe(argc > 2 ? argv[2] : NULL);
	if (!strcmp(argv[1], "patch") && argc >= 4) return cmd_patch(argv[2], argv[3], argc, argv);
	if (!strcmp(argv[1], "verify") && argc >= 3) return cmd_verify(argv[2], get_int(argc, argv, "--slot", -1));
	return usage();
}
