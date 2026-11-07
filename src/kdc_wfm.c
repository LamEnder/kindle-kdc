#include "kdc_wfm.h"

#include <errno.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define PULSE_BLOCK 192   /* signature size, stored after the slot's block-0 frames */
#define PULSE_VERSION 3   /* 2: in place, same file size; 3: + optional long slot */

static uint64_t rd64(const uint8_t *p) { uint64_t v; memcpy(&v, p, 8); return v; }
static uint32_t rd32(const uint8_t *p) { uint32_t v; memcpy(&v, p, 4); return v; }
static void wr64(uint8_t *p, uint64_t v) { memcpy(p, &v, 8); }
static void wr32(uint8_t *p, uint32_t v) { memcpy(p, &v, 4); }

static int fail(char *err, size_t errlen, const char *fmt, ...)
{
	if (err && errlen) {
		va_list ap;
		va_start(ap, fmt);
		vsnprintf(err, errlen, fmt, ap);
		va_end(ap);
	}
	return -1;
}

/* zlib CRC-32, the same as Lab126's crc32() in mxc_epdc_fb_lab126.c. */
static uint32_t crc_update(uint32_t c, const uint8_t *buf, size_t len)
{
	static uint32_t table[256];
	static int ready;
	if (!ready) {
		for (uint32_t n = 0; n < 256; n++) {
			uint32_t x = n;
			for (int k = 0; k < 8; k++)
				x = (x & 1) ? 0xEDB88320u ^ (x >> 1) : x >> 1;
			table[n] = x;
		}
		ready = 1;
	}
	for (size_t i = 0; i < len; i++)
		c = table[(c ^ buf[i]) & 0xFF] ^ (c >> 8);
	return c;
}

uint32_t kdc_crc32(const uint8_t *buf, size_t len)
{
	return crc_update(0xFFFFFFFFu, buf, len) ^ 0xFFFFFFFFu;
}

/* CRC as Lab126 computes it: whole file, embedded checksum field zeroed. */
static uint32_t file_crc(const uint8_t *f, size_t len)
{
	static const uint8_t zero[4];
	uint32_t c = crc_update(0xFFFFFFFFu, zero, len < 4 ? len : 4);
	if (len > 4)
		c = crc_update(c, f + 4, len - 4);
	return c ^ 0xFFFFFFFFu;
}

int kdc_wfm_parse(const uint8_t *f, size_t len, kdc_wfm_info_t *wi,
                  char *err, size_t errlen)
{
	memset(wi, 0, sizeof *wi);
	if (len < KDC_WFM_HDR_SIZE + 2)
		return fail(err, errlen, "file too short (%zu bytes)", len);
	wi->checksum = rd32(f + 0);
	wi->file_length = rd32(f + 4);
	wi->mode_version = f[16];
	wi->wf_version = f[17];
	wi->wf_type = f[19];
	wi->frame_rate = f[23];
	wi->luts = f[36];
	wi->mc = f[37];
	wi->trc = f[38];
	wi->adv = f[39];
	wi->modes = wi->mc + 1;
	wi->temps = wi->trc + 1;
	wi->bit_depth = ((wi->luts & 0xC) == 0x4) ? 5 : 4;
	wi->frame_bytes = wi->bit_depth == 5 ? 512 : 0;
	wi->trt = f + KDC_WFM_HDR_SIZE;
	wi->data_off = KDC_WFM_HDR_SIZE + (size_t)wi->temps + 1;
	if (wi->data_off + 8u * wi->modes > len)
		return fail(err, errlen, "truncated mode table");
	wi->data_len = len - wi->data_off;
	if (wi->temps > KDC_PULSE_MAX_TEMPS)
		return fail(err, errlen, "too many temperature ranges (%d)", wi->temps);
	if (wi->adv > 3)
		return fail(err, errlen, "bad advanced_wfm_flags %u", wi->adv);

	const uint8_t *d = f + wi->data_off;
	for (int m = 0; m < wi->modes; m++) {
		uint64_t mo = rd64(d + 8 * m);
		if (mo + 8u * wi->temps > wi->data_len)
			return fail(err, errlen, "mode %d table out of range", m);
		for (int t = 0; t < wi->temps; t++) {
			uint64_t to = rd64(d + mo + 8 * t);
			if (to + 8 > wi->data_len)
				return fail(err, errlen, "mode %d temp %d out of range", m, t);
			uint64_t n = rd64(d + to);
			if (wi->frame_bytes && (n > 4096 || to + 8 + n * wi->frame_bytes > wi->data_len))
				return fail(err, errlen, "mode %d temp %d: bad frame count %llu",
				            m, t, (unsigned long long)n);
		}
	}
	wi->checksum_ok = wi->file_length == len && file_crc(f, len) == wi->checksum;
	return 0;
}

int kdc_wfm_phase(const uint8_t *f, size_t len, const kdc_wfm_info_t *wi,
                  int mode, int temp, uint64_t *nframes, size_t *frames_off)
{
	(void)len;
	if (mode < 0 || mode >= wi->modes || temp < 0 || temp >= wi->temps)
		return -1;
	const uint8_t *d = f + wi->data_off;
	uint64_t to = rd64(d + rd64(d + 8 * mode) + 8 * temp);
	*nframes = rd64(d + to);
	*frames_off = wi->data_off + to + 8;
	return 0;
}

int kdc_wfm_drive_frames(const uint8_t *f, size_t len, const kdc_wfm_info_t *wi,
                         int mode, int temp, int old4, int new4)
{
	uint64_t n;
	size_t off;
	if (!wi->frame_bytes || kdc_wfm_phase(f, len, wi, mode, temp, &n, &off))
		return -1;
	int idx = (new4 << 4) | old4, count = 0;
	for (uint64_t fr = 0; fr < n; fr++) {
		uint8_t b = f[off + fr * wi->frame_bytes + (idx >> 1)];
		if ((b >> ((idx & 1) * 4)) & 0xF)
			count++;
	}
	return count;
}

int kdc_wfm_temp_index(const kdc_wfm_info_t *wi, int temp_c)
{
	int idx = 0;
	for (int t = 0; t < wi->temps; t++)
		if (temp_c >= wi->trt[t])
			idx = t;
	return idx;
}

/* One frame of the pulse LUT. short_on / long_on: whether short / long
 * pulses still drive in this frame. In the short slot both code pairs are
 * short pulses (long_codes_long = 0). */
static void fill_frame(uint8_t *fr, int short_on, int long_on, int long_codes_long)
{
	memset(fr, 0, 512);
	for (int i = 0; i < 1024; i++) {
		int nw = (i >> 4) & 0xF, v = 0;
		int is_long = long_codes_long && (nw == 0 || nw == 15);
		int on = is_long ? long_on : short_on;
		if (on) {
			if (nw == 0 || nw == 1) v = 1;
			else if (nw == 14 || nw == 15) v = 2;
		}
		fr[i >> 1] |= (uint8_t)(v << ((i & 1) * 4));
	}
}

/*
 * The pulse mode is written IN PLACE over the slot's own phase blocks, so the
 * file keeps exactly the stock size. That matters: the Lab126 reload path
 * (mxc_epdc_fb_load_new_fw) frees its DMA buffer with the last *recorded*
 * size, not the allocated one, so any grow after a shrink frees with the
 * wrong size. A larger pulse file loaded after a stock restore hard-froze a
 * PW4 (2026-11-07). Equal sizes never reallocate.
 *
 * Requirements: the slot's phase blocks are used by no other mode (and not
 * twice by the slot), and each block holds the pulse frames + the 0 V tail
 * (block 0 also holds the signature). The signature sits in block 0 right
 * after the frames the EPDC reads.
 */
static int slot_blocks(const uint8_t *f, const kdc_wfm_info_t *wi, int slot,
                       uint64_t *blk, uint64_t *cap, char *err, size_t errlen)
{
	const uint8_t *d = f + wi->data_off;
	uint64_t mo = rd64(d + 8 * slot);
	for (int t = 0; t < wi->temps; t++) {
		blk[t] = rd64(d + mo + 8 * t);
		cap[t] = rd64(d + blk[t]);
		for (int u = 0; u < t; u++)
			if (blk[u] == blk[t])
				return fail(err, errlen, "slot %d reuses a phase block across temperatures; cannot patch in place", slot);
	}
	for (int m = 0; m < wi->modes; m++) {
		if (m == slot) continue;
		uint64_t om = rd64(d + 8 * m);
		for (int t = 0; t < wi->temps; t++) {
			uint64_t ob = rd64(d + om + 8 * t);
			for (int u = 0; u < wi->temps; u++)
				if (ob == blk[u])
					return fail(err, errlen, "slot %d shares phase data with mode %d; cannot patch in place", slot, m);
		}
	}
	return 0;
}

static int scale_frames(const uint8_t *in, size_t len, const kdc_wfm_info_t *wi, int ref_slot,
                        int ref_t, int frames, int maxf, uint8_t *out)
{
	int du_ref = kdc_wfm_drive_frames(in, len, wi, ref_slot, ref_t, 15, 0);
	for (int t = 0; t < wi->temps; t++) {
		int du = kdc_wfm_drive_frames(in, len, wi, ref_slot, t, 15, 0);
		int fr = frames;
		if (du_ref > 0 && du > 0)
			fr = (frames * du + du_ref / 2) / du_ref;
		if (fr < 1) fr = 1;
		if (fr > maxf) fr = maxf;
		out[t] = (uint8_t)fr;
	}
	return 0;
}

int kdc_wfm_make_pulse(const uint8_t *in, size_t len, const kdc_pulse_params_t *p,
                       uint8_t **out, size_t *outlen, char *err, size_t errlen)
{
	kdc_wfm_info_t wi;
	if (kdc_wfm_parse(in, len, &wi, err, errlen))
		return -1;
	if (wi.bit_depth != 5)
		return fail(err, errlen, "only 5-bit waveforms are supported (luts=0x%x)", wi.luts);
	if (p->slot < 1 || p->slot >= wi.modes)
		return fail(err, errlen, "slot %d out of range (waveform has %d modes)", p->slot, wi.modes);
	int dual = p->long_slot > 0;     /* 0 (INIT, never a valid choice) = none */
	if (dual && (p->long_slot < 1 || p->long_slot >= wi.modes || p->long_slot == p->slot))
		return fail(err, errlen, "bad long slot %d", p->long_slot);
	/* The reference (stock DU) is read from the unmodified input, so it may
	 * be the long slot itself. */
	if (p->ref_slot < 0 || p->ref_slot >= wi.modes || p->ref_slot == p->slot)
		return fail(err, errlen, "bad reference slot %d", p->ref_slot);
	if (p->frames < 1 || p->frames > 8)
		return fail(err, errlen, "frames must be 1..8");
	if (dual && (p->long_frames <= p->frames || p->long_frames > 30))
		return fail(err, errlen, "long frames must be > short frames and <= 30");
	int maxf = p->max_frames > 0 ? p->max_frames : 15;
	if (maxf > 60) maxf = 60;
	if (kdc_wfm_find_pulse(in, len, p->slot, NULL) == 1)
		return fail(err, errlen, "input already has a kdc pulse waveform in slot %d; "
		            "patch the stock waveform instead", p->slot);

	uint64_t blk[KDC_PULSE_MAX_TEMPS], cap[KDC_PULSE_MAX_TEMPS];
	uint64_t lblk[KDC_PULSE_MAX_TEMPS], lcap[KDC_PULSE_MAX_TEMPS];
	if (slot_blocks(in, &wi, p->slot, blk, cap, err, errlen))
		return -1;
	if (dual && slot_blocks(in, &wi, p->long_slot, lblk, lcap, err, errlen))
		return -1;

	int ref_t = kdc_wfm_temp_index(&wi, p->ref_temp_c ? p->ref_temp_c : 25);
	uint8_t fpt[KDC_PULSE_MAX_TEMPS], fpl[KDC_PULSE_MAX_TEMPS];
	scale_frames(in, len, &wi, p->ref_slot, ref_t, p->frames, maxf, fpt);
	if (dual)
		scale_frames(in, len, &wi, p->ref_slot, ref_t, p->long_frames, maxf > 30 ? maxf : 30, fpl);
	for (int t = 0; t < wi.temps; t++) {
		/* room: drive + 1 tail frame (+ 1 frame for the signature in block 0) */
		int room = (int)cap[t] - 1 - (t == 0);
		if (room < 1)
			return fail(err, errlen, "slot %d temp %d block too small (%llu frames)",
			            p->slot, t, (unsigned long long)cap[t]);
		if (fpt[t] > room) fpt[t] = (uint8_t)room;
		if (dual) {
			int lroom = (int)lcap[t] - 1;
			if (fpl[t] <= fpt[t]) fpl[t] = (uint8_t)(fpt[t] + 1);
			if (fpl[t] > lroom)
				return fail(err, errlen, "long slot %d temp %d block too small (%llu frames)",
				            p->long_slot, t, (unsigned long long)lcap[t]);
		}
	}

	uint8_t *o = malloc(len);
	if (!o)
		return fail(err, errlen, "out of memory");
	memcpy(o, in, len);
	uint8_t *d = o + wi.data_off;
	uint8_t fr_short[512];
	fill_frame(fr_short, 1, 1, 0);
	for (int t = 0; t < wi.temps; t++) {
		uint8_t *b = d + blk[t];
		memset(b + 8, 0, (size_t)cap[t] * 512);             /* clear the old frames */
		wr64(b, (uint64_t)fpt[t] + 1);                      /* drive frames + 0 V tail */
		for (int fr = 0; fr < fpt[t]; fr++)
			memcpy(b + 8 + (size_t)fr * 512, fr_short, 512);
		if (dual) {
			/* Long slot: long codes drive for fpl frames, short codes for the
			 * first fpt frames (same short pulse as the short slot). */
			uint8_t *lb = d + lblk[t];
			memset(lb + 8, 0, (size_t)lcap[t] * 512);
			wr64(lb, (uint64_t)fpl[t] + 1);
			for (int fr = 0; fr < fpl[t]; fr++)
				fill_frame(lb + 8 + (size_t)fr * 512, fr < fpt[t], 1, 1);
		}
	}
	uint8_t *sig = d + blk[0] + 8 + (size_t)(fpt[0] + 1) * 512;
	memset(sig, 0, PULSE_BLOCK);
	memcpy(sig, KDC_PULSE_MAGIC, 8);
	wr32(sig + 8, PULSE_VERSION);
	wr32(sig + 12, (uint32_t)p->slot);
	wr32(sig + 16, (uint32_t)p->frames);
	wr32(sig + 20, (uint32_t)ref_t);
	wr32(sig + 24, (uint32_t)wi.temps);
	wr32(sig + 28, dual ? (uint32_t)p->long_slot : 0xFFFFFFFFu);
	wr32(sig + 32, dual ? (uint32_t)p->long_frames : 0);
	memcpy(sig + 64, fpt, (size_t)wi.temps);
	if (dual)
		memcpy(sig + 128, fpl, (size_t)wi.temps);

	/* Header untouched; only refresh a checksum that was valid before. */
	if (wi.checksum_ok)
		wr32(o + 0, file_crc(o, len));

	kdc_wfm_info_t check;
	kdc_pulse_info_t pi;
	if (kdc_wfm_parse(o, len, &check, err, errlen) || kdc_wfm_find_pulse(o, len, p->slot, &pi) != 1 ||
	    pi.long_slot != (dual ? p->long_slot : -1)) {
		free(o);
		return fail(err, errlen, "internal error: patched waveform does not verify");
	}
	*out = o;
	*outlen = len;
	return 0;
}

int kdc_wfm_find_pulse(const uint8_t *f, size_t len, int slot, kdc_pulse_info_t *pi)
{
	kdc_wfm_info_t wi;
	if (kdc_wfm_parse(f, len, &wi, NULL, 0))
		return -1;
	if (slot < 0 || slot >= wi.modes)
		return 0;
	const uint8_t *d = f + wi.data_off;
	uint64_t b0 = rd64(d + rd64(d + 8 * slot));
	uint64_t n = rd64(d + b0);
	uint64_t so = b0 + 8 + n * 512;
	if (so + 128 > wi.data_len)
		return 0;
	const uint8_t *sig = d + so;
	uint32_t ver = rd32(sig + 8);
	if (memcmp(sig, KDC_PULSE_MAGIC, 8) || (ver != 2 && ver != 3) ||
	    (int)rd32(sig + 12) != slot || (int)rd32(sig + 24) != wi.temps)
		return 0;
	if (ver == 3 && so + PULSE_BLOCK > wi.data_len)
		return 0;
	if (pi) {
		memset(pi, 0, sizeof *pi);
		pi->slot = slot;
		pi->frames = (int)rd32(sig + 16);
		pi->ref_temp_index = (int)rd32(sig + 20);
		memcpy(pi->frames_per_temp, sig + 64, (size_t)wi.temps);
		pi->long_slot = -1;
		if (ver == 3 && rd32(sig + 28) != 0xFFFFFFFFu) {
			pi->long_slot = (int)rd32(sig + 28);
			pi->long_frames = (int)rd32(sig + 32);
			memcpy(pi->long_frames_per_temp, sig + 128, (size_t)wi.temps);
		}
	}
	return 1;
}

int kdc_read_file(const char *path, uint8_t **buf, size_t *len)
{
	int fd = open(path, O_RDONLY | O_CLOEXEC);
	if (fd < 0)
		return -1;
	size_t cap = 1 << 20, n = 0;
	uint8_t *b = malloc(cap);
	if (!b) { close(fd); return -1; }
	for (;;) {
		if (n == cap) {
			uint8_t *nb = realloc(b, cap * 2);
			if (!nb) { free(b); close(fd); return -1; }
			b = nb;
			cap *= 2;
		}
		ssize_t r = read(fd, b + n, cap - n);
		if (r < 0) {
			if (errno == EINTR) continue;
			free(b); close(fd); return -1;
		}
		if (r == 0) break;
		n += (size_t)r;
	}
	close(fd);
	*buf = b;
	*len = n;
	return 0;
}
