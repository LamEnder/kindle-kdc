/* Framebuffer/EPDC access: the real PW4 driver, or a simulated panel. */
#ifndef KDC_DEV_H
#define KDC_DEV_H

#include <stddef.h>
#include <stdint.h>
#include "kdc.h"

typedef struct {
	int sim;
	int fd;
	uint8_t *map;
	size_t map_len;
	uint8_t *fb;                /* first visible pixel */
	int w, h, stride;
	int inverted;               /* night mode: PXP inverts gray */
	uint32_t marker;
	int saved_pwrdown, have_saved_pwrdown;

	/* simulator */
	float *opt;                 /* reflectance 0..1 */
	uint8_t *opt8;
	float pulse_step;           /* reflectance change per pulse scan (linear sim) */
	const uint16_t *sim_next_dark, *sim_next_light; /* curve sim (0..1023 levels) */
	const uint16_t *sim_next_dark_long, *sim_next_light_long;
	uint64_t sim_pulses;        /* pixel-pulses applied */
} kdc_dev_t;

int kdc_dev_open(kdc_dev_t *d, const kdc_config_t *c, char *err, size_t errlen);
void kdc_dev_close(kdc_dev_t *d);

/* waveform: WAVEFORM_MODE_*; full: UPDATE_MODE_FULL. *resolved receives the
 * waveform slot the driver mapped the request to. */
int kdc_dev_update(kdc_dev_t *d, kdc_rect_t r, int waveform, int full,
                   uint32_t *marker, int *resolved);
int kdc_dev_wait(kdc_dev_t *d, uint32_t marker);
int kdc_dev_temperature(kdc_dev_t *d, int *celsius);
int kdc_dev_set_pwrdown(kdc_dev_t *d, int ms);
int kdc_dev_restore_pwrdown(kdc_dev_t *d);

/* Battery percentage from /sys/class/power_supply, -1 if unknown. */
int kdc_battery_pct(void);

#endif
