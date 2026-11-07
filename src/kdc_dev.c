#include "kdc_dev.h"

#include <errno.h>
#include <fcntl.h>
#include <glob.h>
#include <linux/fb.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "mxcfb_pw4.h"

static int fail(char *err, size_t errlen, const char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);
	vsnprintf(err, errlen, fmt, ap);
	va_end(ap);
	return -1;
}

static int xioctl(int fd, unsigned long req, void *arg)
{
	int r;
	do r = ioctl(fd, req, arg); while (r < 0 && errno == EINTR);
	return r;
}

int kdc_dev_open(kdc_dev_t *d, const kdc_config_t *c, char *err, size_t errlen)
{
	memset(d, 0, sizeof *d);
	d->fd = -1;
	if (c->sim) {
		d->sim = 1;
		d->w = c->sim_width > 0 ? c->sim_width : 1072;
		d->h = c->sim_height > 0 ? c->sim_height : 1448;
		d->stride = d->w + 16;          /* like the real fb: stride > width */
		d->map_len = (size_t)d->stride * d->h;
		d->map = malloc(d->map_len);
		d->opt = malloc(sizeof(float) * (size_t)d->w * d->h);
		d->opt8 = malloc((size_t)d->w * d->h);
		if (!d->map || !d->opt || !d->opt8)
			return fail(err, errlen, "out of memory");
		memset(d->map, 0xFF, d->map_len);
		for (size_t i = 0; i < (size_t)d->w * d->h; i++) d->opt[i] = 1.0f;
		memset(d->opt8, 0xFF, (size_t)d->w * d->h);
		d->fb = d->map;
		d->pulse_step = 1.0f / 6;
		return 0;
	}

	const char *path = c->fb_path ? c->fb_path : "/dev/fb0";
	d->fd = open(path, O_RDWR | O_CLOEXEC);
	if (d->fd < 0)
		return fail(err, errlen, "open %s: %s", path, strerror(errno));
	struct fb_var_screeninfo var;
	struct fb_fix_screeninfo fix;
	if (xioctl(d->fd, FBIOGET_VSCREENINFO, &var) || xioctl(d->fd, FBIOGET_FSCREENINFO, &fix))
		return fail(err, errlen, "FBIOGET_*SCREENINFO: %s", strerror(errno));
	if (var.bits_per_pixel != 8)
		return fail(err, errlen, "framebuffer is %u bpp; kdc needs 8 bpp grayscale", var.bits_per_pixel);
	d->w = (int)var.xres;
	d->h = (int)var.yres;
	d->stride = (int)fix.line_length;
	d->inverted = var.grayscale == GRAYSCALE_8BIT_INVERTED;
	d->map_len = fix.smem_len ? fix.smem_len : (size_t)fix.line_length * var.yres_virtual;
	d->map = mmap(NULL, d->map_len, PROT_READ | PROT_WRITE, MAP_SHARED, d->fd, 0);
	if (d->map == MAP_FAILED) {
		d->map = NULL;
		return fail(err, errlen, "mmap: %s", strerror(errno));
	}
	d->fb = d->map + (size_t)var.yoffset * fix.line_length + var.xoffset;
	return 0;
}

void kdc_dev_close(kdc_dev_t *d)
{
	if (d->sim) {
		free(d->map);
		free(d->opt);
		free(d->opt8);
	} else {
		if (d->map) munmap(d->map, d->map_len);
		if (d->fd >= 0) close(d->fd);
	}
	memset(d, 0, sizeof *d);
	d->fd = -1;
}

static void sim_apply(kdc_dev_t *d, kdc_rect_t r, int waveform, int full)
{
	for (int y = r.y; y < r.y + r.h; y++) {
		const uint8_t *src = d->fb + (size_t)y * d->stride;
		for (int x = r.x; x < r.x + r.w; x++) {
			size_t i = (size_t)y * d->w + x;
			int v4 = src[x] >> 4;
			float o = d->opt[i];
			if (waveform == WAVEFORM_MODE_DU4 || waveform == WAVEFORM_MODE_DU) {
				/* Pulse codes (see kdc_wfm.h): short slot DU4: 0,1 black, 14,15
				 * white; dual slot DU: 0 / 15 long, 1 / 14 short. */
				if (!full)
					continue;           /* kdc only uses FULL pulse updates */
				int dir = (v4 == 0 || v4 == 1) ? -1 : (v4 == 14 || v4 == 15) ? 1 : 0;
				int lng = waveform == WAVEFORM_MODE_DU && (v4 == 0 || v4 == 15);
				if (dir) {
					const uint16_t *tab = dir < 0 ? (lng ? d->sim_next_dark_long : d->sim_next_dark)
					                              : (lng ? d->sim_next_light_long : d->sim_next_light);
					if (tab) {
						int l = (int)(o * 1023 + 0.5);
						o = tab[l] / 1023.0f;
					} else {
						o += dir * d->pulse_step * (lng ? 3 : 1);
					}
					d->sim_pulses++;
				}
			} else {
				o = src[x] / 255.0f;
			}
			o = o < 0 ? 0 : o > 1 ? 1 : o;
			d->opt[i] = o;
			d->opt8[i] = (uint8_t)(o * 255 + 0.5f);
		}
	}
}

int kdc_dev_update(kdc_dev_t *d, kdc_rect_t r, int waveform, int full,
                   uint32_t *marker, int *resolved)
{
	if (r.w <= 0 || r.h <= 0 || r.x < 0 || r.y < 0 || r.x + r.w > d->w || r.y + r.h > d->h) {
		errno = EINVAL;
		return -1;
	}
	if (++d->marker == 0)
		d->marker = 1;
	if (marker) *marker = d->marker;
	if (d->sim) {
		sim_apply(d, r, waveform, full);
		if (resolved) *resolved = waveform;
		return 0;
	}
	struct mxcfb_update_data u;
	memset(&u, 0, sizeof u);
	u.update_region.top = (uint32_t)r.y;
	u.update_region.left = (uint32_t)r.x;
	u.update_region.width = (uint32_t)r.w;
	u.update_region.height = (uint32_t)r.h;
	u.waveform_mode = (uint32_t)waveform;
	u.update_mode = full ? UPDATE_MODE_FULL : UPDATE_MODE_PARTIAL;
	u.update_marker = d->marker;
	u.temp = TEMP_USE_AMBIENT;
	u.hist_bw_waveform_mode = WAVEFORM_MODE_DU;
	u.hist_gray_waveform_mode = WAVEFORM_MODE_GC16;
	if (xioctl(d->fd, MXCFB_SEND_UPDATE, &u) < 0)
		return -1;
	/* The Lab126 driver writes the resolved slot back (queue scheme). */
	if (resolved) *resolved = (int)u.waveform_mode;
	return 0;
}

int kdc_dev_wait(kdc_dev_t *d, uint32_t marker)
{
	if (d->sim)
		return 0;
	struct mxcfb_update_marker_data m = { marker, 0 };
	/* Note: the fb core holds fb_info->lock for the whole wait, so only the
	 * display thread may call this. */
	return xioctl(d->fd, MXCFB_WAIT_FOR_UPDATE_COMPLETE, &m) < 0 ? -1 : 0;
}

int kdc_dev_temperature(kdc_dev_t *d, int *celsius)
{
	if (d->sim) { *celsius = 25; return 0; }
	int32_t t = 0;
	if (xioctl(d->fd, MXCFB_GET_TEMPERATURE, &t) < 0)
		return -1;
	*celsius = t;
	return 0;
}

int kdc_dev_set_pwrdown(kdc_dev_t *d, int ms)
{
	if (d->sim) return 0;
	if (!d->have_saved_pwrdown) {
		int32_t cur = 0;
		if (xioctl(d->fd, MXCFB_GET_PWRDOWN_DELAY, &cur) == 0) {
			d->saved_pwrdown = cur;
			d->have_saved_pwrdown = 1;
		}
	}
	int32_t v = ms;
	return xioctl(d->fd, MXCFB_SET_PWRDOWN_DELAY, &v);
}

int kdc_dev_restore_pwrdown(kdc_dev_t *d)
{
	if (d->sim || !d->have_saved_pwrdown) return 0;
	int32_t v = d->saved_pwrdown;
	d->have_saved_pwrdown = 0;
	return xioctl(d->fd, MXCFB_SET_PWRDOWN_DELAY, &v);
}

int kdc_battery_pct(void)
{
	glob_t g;
	int pct = -1;
	if (glob("/sys/class/power_supply/*/capacity", 0, NULL, &g) == 0) {
		for (size_t i = 0; i < g.gl_pathc && pct < 0; i++) {
			FILE *f = fopen(g.gl_pathv[i], "r");
			if (!f) continue;
			int v;
			if (fscanf(f, "%d", &v) == 1 && v >= 0 && v <= 100) pct = v;
			fclose(f);
		}
		globfree(&g);
	}
	return pct;
}
