/*
 * Waveform file handling for the PW4 EPDC (Lab126 .wrf format).
 *
 * File layout (verified against firmware/imx/epdc/epdc_ED060KC4.fw from the
 * GPL tree and the Lab126 loader in mxc_epdc_fb_lab126.c):
 *
 *   [48-byte header][TRT: trc+1 bytes][1 byte][waveform data ...]
 *
 * The waveform data is what the EPDC reads through EPDC_WVADDR. All offsets
 * inside it are relative to its own start:
 *
 *   u64 mode_table[mc+1]          -> offset of that mode's temperature table
 *   u64 adv[...]                  (voltage/algorithm control, optional)
 *   temp_table: u64 [trc+1]       -> offset of a phase block
 *   phase block: u64 nframes, then nframes * frame_bytes
 *
 * A 5-bit waveform frame is 512 bytes = 1024 4-bit entries. Entry index is
 * (ext << 8) | (new << 4) | old, where new/old are the 4-bit gray levels
 * (0 = black, 15 = white) and ext is two further bits. Entry values: 0 = no
 * drive, 1 = drive toward black, 2 = drive toward white.
 *
 * kdc pulse command codes (4-bit "new" value of the fb pixel):
 *   short slot (DU4):  0, 1 = short black   14, 15 = short white   else hold
 *   long slot  (DU):   0 = LONG black  1 = short black  14 = short white
 *                      15 = LONG white  else hold
 * A plane of these codes means the same thing under either slot, except that
 * long codes only drive long under the long slot.
 */
#ifndef KDC_WFM_H
#define KDC_WFM_H

#include <stddef.h>
#include <stdint.h>

#define KDC_WFM_HDR_SIZE 48
#define KDC_PULSE_MAGIC "KDCPULS1"
#define KDC_PULSE_MAX_TEMPS 64

typedef struct {
	uint32_t checksum, file_length;
	uint8_t mode_version, wf_version, wf_type, frame_rate;
	uint8_t luts, mc, trc, adv;
	int modes, temps;          /* mc+1, trc+1 */
	int bit_depth;             /* 4 or 5 */
	size_t data_off;           /* file offset of waveform data */
	size_t data_len;
	size_t frame_bytes;        /* bytes per frame (512 for 5-bit) */
	const uint8_t *trt;        /* temps entries, lower bounds in deg C */
	int checksum_ok;           /* embedded CRC matches (1), not (0) */
} kdc_wfm_info_t;

typedef struct {
	int slot;                  /* mode index that was replaced */
	int frames;                /* drive frames per scan at reference temp */
	int ref_temp_index;
	uint8_t frames_per_temp[KDC_PULSE_MAX_TEMPS]; /* drive frames, no tail */
	int long_slot;             /* dual waveform slot, -1 if none */
	int long_frames;           /* long pulse frames at reference temp */
	uint8_t long_frames_per_temp[KDC_PULSE_MAX_TEMPS];
} kdc_pulse_info_t;

typedef struct {
	int slot;                  /* mode index to replace */
	int frames;                /* drive frames per scan at reference temp, 1..8 */
	int ref_slot;              /* stock DU slot, scales frames by temperature */
	int ref_temp_c;            /* reference temperature, default 25 */
	int max_frames;            /* cap per temperature range, default 15 */
	int long_slot;             /* also patch this slot with the dual waveform; 0 = no */
	int long_frames;           /* long pulse frames at reference temp (> frames) */
} kdc_pulse_params_t;

uint32_t kdc_crc32(const uint8_t *buf, size_t len);

/* Returns 0 on success, -1 with a message in err. */
int kdc_wfm_parse(const uint8_t *f, size_t len, kdc_wfm_info_t *out,
                  char *err, size_t errlen);

/* Phase block of (mode, temp): frame count and file offset of frame 0. */
int kdc_wfm_phase(const uint8_t *f, size_t len, const kdc_wfm_info_t *wi,
                  int mode, int temp, uint64_t *nframes, size_t *frames_off);

/* Number of frames that drive (nonzero) for the old->new transition. */
int kdc_wfm_drive_frames(const uint8_t *f, size_t len, const kdc_wfm_info_t *wi,
                         int mode, int temp, int old4, int new4);

/* Temperature range index containing temp_c. */
int kdc_wfm_temp_index(const kdc_wfm_info_t *wi, int temp_c);

/* Builds a copy of in[] whose mode `slot` is the per-pixel pulse waveform,
 * written in place over that slot's own phase blocks. The result has exactly
 * the input's size (see kdc_wfm.c for why). *out is malloc'd. */
int kdc_wfm_make_pulse(const uint8_t *in, size_t len, const kdc_pulse_params_t *p,
                       uint8_t **out, size_t *outlen, char *err, size_t errlen);

/* 1 if mode `slot` holds a kdc pulse waveform (fills *pi), 0 if not, -1 bad file. */
int kdc_wfm_find_pulse(const uint8_t *f, size_t len, int slot, kdc_pulse_info_t *pi);

/* Reads a whole file (e.g. /proc/eink/waveform/data). malloc'd. */
int kdc_read_file(const char *path, uint8_t **buf, size_t *len);

#endif
