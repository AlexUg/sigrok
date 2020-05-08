/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2018 Alexandr Ugnenko <ugnenko@mail.ru>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef LIBSIGROK_HARDWARE_KINGST_LA1010_PROTOCOL_H
#define LIBSIGROK_HARDWARE_KINGST_LA1010_PROTOCOL_H

#include <stdint.h>
#include <glib.h>
#include <string.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"

#define LOG_PREFIX "kingst-la1010"

#define USB_INTERFACE		0
#define USB_CONFIGURATION	1
#define NUM_TRIGGER_STAGES	4

#define USB_UPLOAD_DATA_EP      0x02
#define USB_SAMPLING_DATA_EP    0x86

#define MAX_RENUM_DELAY_MS	3000
#define NUM_SIMUL_TRANSFERS	8
#define MAX_EMPTY_TRANSFERS	(NUM_SIMUL_TRANSFERS * 2)

#define NUM_CHANNELS		16

#define KINGST_LA1010_REQUIRED_VERSION_MAJOR	1

#define MAX_3BIT_SAMPLE_RATE	SR_MHZ(100)
#define MAX_16BIT_SAMPLE_RATE	SR_MHZ(16)

/* 6 delay states of up to 256 clock ticks */
#define MAX_SAMPLE_DELAY	(6 * 256)

#define DEV_CAPS_16BIT_POS	0

#define DEV_CAPS_16BIT		(1 << DEV_CAPS_16BIT_POS)

/* Protocol commands */
#define CMD_10                          0x10
#define CMD_CONTROL                     0x20
#define CMD_SAMPLING_START              0x30
#define CMD_SAMPLING_CONFIG             0x38
#define CMD_STATUS                      0xA2
#define CMD_SPARTAN_UPLOAD              0x50
#define CMD_60                          0x60

#define CMD_CONTROL_START               0x0000
#define CMD_STATUS_FX_STATUS            0x0008
#define CMD_CONTROL_END                 0x0001
#define CMD_CONTROL_PWM                 0x0002
#define CMD_STATUS_USB_STATUS           0x0020
#define CMD_CONTROL_SAMPLE_RATE         0x002E
#define CMD_CONTROL_CHAN_SELECT         0x0030
#define CMD_CONTROL_LOG_LEVEL           0x0048  // electric signal level (Volts)
#define CMD_CONTROL_PWM_1               0x0070
#define CMD_CONTROL_PWM_2               0x0078

#define SAMPLING_BASE_FREQUENCY         800000000
#define PWM_BASE_FREQUENCY              200000000

enum voltage_range {
	VOLTAGE_RANGE_TTL,
	VOLTAGE_RANGE_5_V,
	VOLTAGE_RANGE_3_3_V,
	VOLTAGE_RANGE_3_V,
	VOLTAGE_RANGE_2_5_V,
	VOLTAGE_RANGE_1_8_V,
	VOLTAGE_RANGE_1_5_V,
};

struct pwm_data {
	uint64_t freq;
	uint64_t duty;
	uint8_t enabled;
};

struct kingst_la1010_profile {
	uint16_t vid;
	uint16_t pid;

	const char *vendor;
	const char *model;
	const char *model_version;

	const char *fx_firmware;
	const char *spartan_firmware;

	uint32_t dev_caps;

	const char *usb_manufacturer;
	const char *usb_product;
};

struct dev_context {
	const struct kingst_la1010_profile *profile;
	/*
	 * Since we can't keep track of an fx2lafw device after upgrading
	 * the firmware (it renumerates into a different device address
	 * after the upgrade) this is like a global lock. No device will open
	 * until a proper delay after the last device was upgraded.
	 */
	int64_t fw_updated;

	const uint64_t *samplerates;
	int num_samplerates;

	uint64_t cur_samplerate;
	uint64_t limit_samples;

	uint64_t capture_ratio;

	struct pwm_data pwm[2];

	enum voltage_range selected_voltage_level;

	double user_defined_level;

	gboolean trigger_fired;
	gboolean acq_aborted;
	struct soft_trigger_logic *stl;

	int submitted_transfers;
	int empty_transfer_count;

	unsigned int num_transfers;
	struct libusb_transfer **transfers;
	struct sr_context *ctx;

	uint16_t cur_channels;
	int num_channels;
	int cur_channel;
	uint16_t channel_masks[16];
	uint16_t channel_data[16];
	uint64_t sent_samples;
	uint8_t *convbuffer;
	size_t convbuffer_size;
};

union fx_status {
	uint8_t bytes[8];
	uint32_t code[2];
};

union spartan_status {
	uint8_t bytes[4];
	uint32_t code;
};

SR_PRIV struct dev_context* kingst_la1010_dev_new(void);
SR_PRIV int kingst_la1010_has_fx_firmware(struct libusb_device_handle *hdl);
int kingst_la1010_upload_cypress_firmware(struct sr_context *ctx,
											struct libusb_device_handle *hdl,
											const struct kingst_la1010_profile *prof);
SR_PRIV int kingst_la1010_upload_spartan_firmware(const struct sr_dev_inst *sdi);
SR_PRIV int kingst_la1010_init_spartan(struct libusb_device_handle *handle);
SR_PRIV int kingst_la1010_dev_open(const struct sr_dev_inst *sdi);
SR_PRIV int kingst_la1010_abort_acquisition_request(libusb_device_handle *handle);
SR_PRIV int kingst_la1010_acquisition_start(const struct sr_dev_inst *sdi);
SR_PRIV int kingst_la1010_acquisition_stop(const struct sr_dev_inst *sdi);
SR_PRIV int kingst_la1010_set_logic_level(struct libusb_device_handle *hdl, double level);
SR_PRIV int kingst_la1010_receive_data(int fd, int revents, void *cb_data);
SR_PRIV int kingst_la1010_configure_channels(const struct sr_dev_inst *sdi);
SR_PRIV int kingst_la1010_configure_pwm(struct libusb_device_handle *hdl,
										uint64_t pwm1_freq,
										uint64_t pwm1_duty,
										uint64_t pwm2_freq,
										uint64_t pwm2_duty);

#endif
