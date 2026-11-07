/*
 * mxcfb ioctl ABI of the Kindle PaperWhite 4 EPDC driver.
 *
 * Copied from include/uapi/linux/mxcfb.h of Amazon's GPL release
 * (Kindle_src_5.18.1.1.1, linux-4.1.15, Lab126 variant of NXP's
 * mxc_epdc_v2_fb.c). Only the parts kdc uses are kept. The struct layout
 * matters: the ioctl number encodes sizeof(struct mxcfb_update_data), which
 * is 80 bytes here because Lab126 appended the two hist_* fields.
 */
#ifndef KDC_MXCFB_PW4_H
#define KDC_MXCFB_PW4_H

#include <stdint.h>
#include <sys/ioctl.h>

struct mxcfb_rect {
	uint32_t top;
	uint32_t left;
	uint32_t width;
	uint32_t height;
};

struct mxcfb_alt_buffer_data {
	uint32_t phys_addr;
	uint32_t width;
	uint32_t height;
	struct mxcfb_rect alt_update_region;
};

struct mxcfb_update_data {
	struct mxcfb_rect update_region;
	uint32_t waveform_mode;
	uint32_t update_mode;
	uint32_t update_marker;
	int temp;
	unsigned int flags;
	int dither_mode;
	int quant_bit;
	struct mxcfb_alt_buffer_data alt_buffer_data;
	uint32_t hist_bw_waveform_mode;   /* Lab126 */
	uint32_t hist_gray_waveform_mode; /* Lab126 */
};

struct mxcfb_update_marker_data {
	uint32_t update_marker;
	uint32_t collision_test;
};

_Static_assert(sizeof(struct mxcfb_update_data) == 80, "mxcfb_update_data ABI");

#define UPDATE_MODE_PARTIAL 0x0
#define UPDATE_MODE_FULL    0x1

#define WAVEFORM_MODE_INIT 0x0
#define WAVEFORM_MODE_DU   0x1
#define WAVEFORM_MODE_GC16 0x2
#define WAVEFORM_MODE_GL16 0x3
#define WAVEFORM_MODE_A2   0x6
#define WAVEFORM_MODE_DU4  0x7

#define TEMP_USE_AMBIENT 0x1000

#define GRAYSCALE_8BIT          0x1
#define GRAYSCALE_8BIT_INVERTED 0x2

#define MXCFB_SEND_UPDATE              _IOW('F', 0x2E, struct mxcfb_update_data)
#define MXCFB_WAIT_FOR_UPDATE_COMPLETE _IOWR('F', 0x2F, struct mxcfb_update_marker_data)
#define MXCFB_SET_PWRDOWN_DELAY        _IOW('F', 0x30, int32_t)
#define MXCFB_GET_PWRDOWN_DELAY        _IOR('F', 0x31, int32_t)
#define MXCFB_GET_TEMPERATURE          _IOR('F', 0x38, int32_t)

#endif
