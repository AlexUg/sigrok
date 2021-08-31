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

#include <config.h>
#include "protocol.h"


const uint16_t vendor_id = 0x77A1;
const char vendor_name[] = "Kingst";

const uint16_t supported_pids[] = { 0x01A1, 0x01A2, 0x01A3, 0x01A4, 0x03A1 };

static const uint32_t scanopts[] = { SR_CONF_CONN, };

static const uint32_t drvopts[] = { SR_CONF_LOGIC_ANALYZER, };

static const uint32_t devopts[] = {
		SR_CONF_CONTINUOUS,
		SR_CONF_LIMIT_SAMPLES | SR_CONF_GET | SR_CONF_SET, SR_CONF_CONN | SR_CONF_GET,
		SR_CONF_SAMPLERATE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
		SR_CONF_VOLTAGE_THRESHOLD | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
		SR_CONF_TRIGGER_MATCH | SR_CONF_LIST,
		SR_CONF_CAPTURE_RATIO | SR_CONF_GET | SR_CONF_SET, };

static const int32_t trigger_matches[] = {
		SR_TRIGGER_ZERO,
		SR_TRIGGER_ONE,
		SR_TRIGGER_RISING,
		SR_TRIGGER_FALLING,
		SR_TRIGGER_EDGE, };

static const uint64_t samplerates[] = {
		SR_KHZ(20),
		SR_KHZ(50),
		SR_KHZ(100),
		SR_KHZ(200),
		SR_KHZ(250),
		SR_KHZ(500),
		SR_MHZ(1),
		SR_MHZ(2),
		SR_MHZ(4),
		SR_MHZ(5),
		SR_MHZ(8),
		SR_MHZ(10),
		SR_KHZ(12500),
		SR_MHZ(16),
		SR_MHZ(25),
		SR_MHZ(32),
		SR_MHZ(40),
		SR_MHZ(50),
		SR_MHZ(80),
		SR_MHZ(100),
};

/*
 * Kingst LA1010 software provides next voltage leves:
 * TTL          -> 1.58 V
 * CMOS 5       -> 2.5 V
 * CMOS 3.3     -> 1.65
 * CMOS 3       -> 1.5 V
 * CMOS 2.5     -> 1.25 V
 * CMOS 1.8     -> 0.9 V
 * CMOS 1.5     -> 0.75 V
 *
 * and 'User Defined' -- between -4 V and 4 V
 * 'User Defined' not implemented
 */
static const double thresholds[][2] = {
		{ 1.58, 1.58 },     // TTL
		{ 2.5, 2.5 },       // CMOS 5
		{ 1.65, 1.65 },     // CMOS 3.3
		{ 1.5, 1.5 },       // CMOS 3
		{ 1.25, 1.25 },     // CMOS 2.5
		{ 0.9, 0.9 },       // CMOS 1.8
		{ 0.75, 0.75 },     // CMOS 1.5
		};


struct libusb_device_handle * reconnect(struct drv_context *drvc, char * connection_id, struct dev_context *devc);

struct libusb_device_handle * reconnect(struct drv_context *drvc, char * connection_id, struct dev_context *devc) {
	struct libusb_device **devlist;
	struct libusb_device_descriptor des;
	char connection_id_buff[64];
	struct libusb_device_handle *res = NULL;
	int device_count, i, ret = 0;
	int64_t timediff_us, timediff_ms;

	/* Takes >= 300ms for the FX2 to be gone from the USB bus. */
	g_usleep(300 * 1000);
	timediff_ms = 0;
	while (timediff_ms < MAX_RENUM_DELAY_MS) {

		device_count = libusb_get_device_list(drvc->sr_ctx->libusb_ctx, &devlist);
		if (device_count < 0) {
			sr_err("Failed to get device list: %s.", libusb_error_name(device_count));
			break;
		}

		for (i = 0; i < device_count; i++) {
			libusb_get_device_descriptor(devlist[i], &des);

			if (des.idVendor != devc->profile.vid
					|| des.idProduct != devc->profile.pid)
				continue;

			usb_get_port_path(devlist[i], connection_id_buff, sizeof(connection_id_buff));
			if (strcmp(connection_id_buff, connection_id))
				/* This is not the one. */
				continue;

			if ((ret = libusb_open(devlist[i], &res)) != 0) {
				sr_err("Failed to open device: %s.", libusb_error_name(ret));
				ret = SR_ERR;
				break;
			}

			if (libusb_has_capability(LIBUSB_CAP_SUPPORTS_DETACH_KERNEL_DRIVER)) {
				if (libusb_kernel_driver_active(res, USB_INTERFACE) == 1) {
					if ((ret = libusb_detach_kernel_driver(res, USB_INTERFACE)) < 0) {
						sr_err("Failed to detach kernel driver: %s.",
								libusb_error_name(ret));
						ret = SR_ERR;
						break;
					}
				}
			}

			ret = SR_OK;

			break;
		}

		libusb_free_device_list(devlist, 1);

		g_usleep(100 * 1000);

		timediff_us = g_get_monotonic_time() - devc->fw_updated;
		timediff_ms = timediff_us / 1000;
		sr_spew("Waited %" PRIi64 "ms.", timediff_ms);
	}

	if (ret) {
		if (res) {
			libusb_close(res);
			res = NULL;
		}
	}
	return res;
}

static GSList* scan(struct sr_dev_driver *di, GSList *options) {

	struct drv_context *drvc;
	struct dev_context *devc;
	struct sr_dev_inst *sdi;
	struct sr_usb_dev_inst *usb;
	struct sr_channel *ch;
	struct sr_channel_group *cg;
	struct sr_config *src;
	struct kingst_laxxxx_desc * device_desc;
	GSList *l, *devices, *conn_devices;
	struct libusb_device_descriptor des;
	libusb_device **devlist;
	struct libusb_device_handle *hdl;
	int ret, i, device_count;
	const char *conn;
	char serial_num[64], connection_id[64];
	char channel_name[32];
	unsigned int j;

	drvc = di->context;
	hdl = NULL;
	conn = NULL;
	device_desc = NULL;
	devc = NULL;

	for (l = options; l; l = l->next) {
		src = l->data;
		switch (src->key) {
		case SR_CONF_CONN:
			conn = g_variant_get_string(src->data, NULL);
			break;
		}
	}
	if (conn)
		conn_devices = sr_usb_find(drvc->sr_ctx->libusb_ctx, conn);
	else
		conn_devices = NULL;

	/* Find all Kingst LA1010 compatible devices and upload firmware to them. */
	devices = NULL;
	device_count = libusb_get_device_list(drvc->sr_ctx->libusb_ctx, &devlist);
	if (device_count > 0) {
		for (i = 0; i < device_count; i++) {

			if (devc && !sdi) {
				g_free(devc);
			}
			devc = NULL;

			if (hdl) {
				libusb_close(hdl);
				hdl = NULL;
			}

			if (conn) {
				usb = NULL;
				for (l = conn_devices; l; l = l->next) {
					usb = l->data;
					if ((usb->bus == libusb_get_bus_number(devlist[i]))
							&& (usb->address == libusb_get_device_address(devlist[i])))
						break;
				}
				if (!l)
					/* This device matched none of the ones that
					 * matched the conn specification. */
					continue;
			}

			libusb_get_device_descriptor(devlist[i], &des);

			if (des.idVendor == vendor_id) {
				for (j = 0; j < sizeof(supported_pids) / sizeof(uint16_t); j++) {
					if (des.idProduct == supported_pids[j]) {
						devc = kingst_laxxxx_dev_new(vendor_id, vendor_name);
						devc->profile.pid = des.idProduct;
						sr_dbg("Found candidate with vid:pid: %04X:%04X.", des.idVendor, des.idProduct);
						break;
					}
				}
			}

			if (devc) {

				sr_dbg("Candidate verification...");

				if ((ret = libusb_open(devlist[i], &hdl)) < 0) {
					sr_warn("Failed to open potential device with " "VID:PID %04x:%04x: %s.",
								des.idVendor,
								des.idProduct,
								libusb_error_name(ret));
					continue;
				}

				if (des.iSerialNumber == 0) {
					serial_num[0] = '\0';
				} else if ((ret = libusb_get_string_descriptor_ascii(hdl,
																		des.iSerialNumber,
																		(unsigned char*) serial_num,
																		sizeof(serial_num))) < 0) {
					sr_warn("Failed to get serial number string descriptor: %s.",
								libusb_error_name(ret));
					serial_num[0] = '\0';
				}

				usb_get_port_path(devlist[i], connection_id, sizeof(connection_id));

				if (kingst_laxxxx_has_fx_firmware(hdl, &device_desc)) {
					// No firmware in Cypress

					sr_dbg("Candidate without FX2 firmware");

					if (kingst_laxxxx_upload_cypress_firmware(drvc->sr_ctx,
																hdl,
																&(devc->profile)) == SR_OK) {
						/* Store when this device's FW was updated. */
						devc->fw_updated = g_get_monotonic_time();
						sr_dbg("FX2 firmware was uploaded to Kingst LA1010 device. Reconnecting...");

						// Reconnect device
						libusb_close(hdl);
						hdl = reconnect(drvc, connection_id, devc);

						if (kingst_laxxxx_has_fx_firmware(hdl, &device_desc)) {
							// No firmware in Cypress
							sr_err("Reconnecting after firmware upload failed for device %d.%d (logical).",
										libusb_get_bus_number(devlist[i]),
										libusb_get_device_address(devlist[i]));
						}
					} else {
						sr_err("Firmware upload failed for device %d.%d (logical).",
									libusb_get_bus_number(devlist[i]),
									libusb_get_device_address(devlist[i]));
					}
				}

				if (device_desc) {

					sr_dbg("Found supported device '%s' id '%d' variant '%d'.",
									device_desc->model,
									device_desc->device_id,
									device_desc->device_variant);

					sdi = g_malloc0(sizeof(struct sr_dev_inst));

					sdi->inst_type = SR_INST_USB;
					sdi->conn = sr_usb_dev_inst_new(libusb_get_bus_number(devlist[i]),
													libusb_get_device_address(devlist[i]),
													NULL);

					sdi->status = SR_ST_INITIALIZING;
					sdi->vendor = g_strdup(devc->profile.vendor);
					sdi->serial_num = g_strdup(serial_num);
					sdi->connection_id = g_strdup(connection_id);
					devc->profile.description = device_desc;

					sdi->model = g_strdup(device_desc->model);
					sdi->version = g_strdup("");

					/* Logic channels, all in one channel group. */
					cg = g_malloc0(sizeof(struct sr_channel_group));
					cg->name = g_strdup("Logic");
					for (j = 0; j < devc->profile.description->num_logic_channels; j++) {
						sprintf(channel_name, "D%d", j);
						ch = sr_channel_new(sdi, j, SR_CHANNEL_LOGIC, TRUE, channel_name);
						cg->channels = g_slist_append(cg->channels, ch);
					}
					sdi->channel_groups = g_slist_append(NULL, cg);

					devc->samplerates = samplerates;
					devc->num_samplerates = ARRAY_SIZE(samplerates);

					devc->pwm[0].freq = 1000;
					devc->pwm[0].duty = 50;
					devc->pwm[0].enabled = 0;
					devc->pwm[1].freq = 1000;
					devc->pwm[1].duty = 50;
					devc->pwm[1].enabled = 0;

					sdi->priv = devc;
					devices = g_slist_append(devices, sdi);
				}
			} else {
				libusb_close(hdl);
				hdl = NULL;
			}
		}

		if (devc && !sdi) {
			g_free(devc);
			devc = NULL;
		}

		if (hdl) {
			libusb_close(hdl);
			hdl = NULL;
		}
	} else {
		sr_err("Failed to get device list: %s.", libusb_error_name(device_count));
	}

	libusb_free_device_list(devlist, 1);
	g_slist_free_full(conn_devices, (GDestroyNotify) sr_usb_dev_inst_free);

	return std_scan_complete(di, devices);
}

static int dev_open(struct sr_dev_inst *sdi) {
	struct sr_usb_dev_inst *usb;
	struct dev_context *devc;
	int ret;

	devc = sdi->priv;
	usb = sdi->conn;

	/*
	 * If the firmware was recently uploaded, wait up to MAX_RENUM_DELAY_MS
	 * milliseconds for the FX2 to renumerate.
	 */
	ret = kingst_laxxxx_dev_open(sdi);
	if (ret != SR_OK) {
		sr_err("Unable to open device.");
		return SR_ERR;
	}

	ret = libusb_claim_interface(usb->devhdl, USB_INTERFACE);
	if (ret != 0) {
		switch (ret) {
		case LIBUSB_ERROR_BUSY:
			sr_err(
					"Unable to claim USB interface. Another " "program or driver has already claimed it.");
			break;
		case LIBUSB_ERROR_NO_DEVICE:
			sr_err("Device has been disconnected.");
			break;
		default:
			sr_err("Unable to claim interface: %s.", libusb_error_name(ret));
			break;
		}

		return SR_ERR;
	}

	sr_dbg("Upload Spartan firmware...");
	ret = kingst_laxxxx_upload_spartan_firmware(sdi);
	if (ret) {
		sr_err("Upload Spartan firmware failed. Return status: 0x%x", ret);
		return SR_ERR;
	}

	ret = kingst_laxxxx_init_spartan(usb->devhdl);
	if (ret) {
		sr_err("Initialization of Spartan failed. Error: %s",
				libusb_error_name(ret));
		return SR_ERR;
	}

	if (devc->cur_samplerate == 0) {
		/* Samplerate hasn't been set; default to the slowest one. */
		devc->cur_samplerate = devc->samplerates[0];
	}

	sr_dbg("Kingst LA1010 initialization done.");

	return SR_OK;
}

static int dev_close(struct sr_dev_inst *sdi) {
	struct sr_usb_dev_inst *usb;

	usb = sdi->conn;

	if (!usb->devhdl)
		return SR_ERR_BUG;

	sr_info("Closing device on %d.%d (logical) / %s (physical) interface %d.",
			usb->bus, usb->address, sdi->connection_id, USB_INTERFACE);
	libusb_release_interface(usb->devhdl, USB_INTERFACE);
	libusb_close(usb->devhdl);
	usb->devhdl = NULL;

	return SR_OK;
}

static int config_get(uint32_t key, GVariant **data,
		const struct sr_dev_inst *sdi, const struct sr_channel_group *cg) {
	unsigned int i;
	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;

	(void) cg;

	if (!sdi)
		return SR_ERR_ARG;

	devc = sdi->priv;

	switch (key) {
	case SR_CONF_CONN:
		if (!sdi->conn)
			return SR_ERR_ARG;
		usb = sdi->conn;
		if (usb->address == 255)
			/* Device still needs to re-enumerate after firmware
			 * upload, so we don't know its (future) address. */
			return SR_ERR;
		*data = g_variant_new_printf("%d.%d", usb->bus, usb->address);
		break;
	case SR_CONF_LIMIT_SAMPLES:
		*data = g_variant_new_uint64(devc->limit_samples);
		break;
	case SR_CONF_SAMPLERATE:
		*data = g_variant_new_uint64(devc->cur_samplerate);
		break;
	case SR_CONF_CAPTURE_RATIO:
		*data = g_variant_new_uint64(devc->capture_ratio);
		break;
	case SR_CONF_VOLTAGE_THRESHOLD:
		if (!sdi)
			return SR_ERR;
		devc = sdi->priv;
		for (i = 0; i < ARRAY_SIZE(thresholds); i++) {
			if (devc->selected_voltage_level != i)
				continue;
			*data = std_gvar_tuple_double(thresholds[i][0], thresholds[i][1]);
			return SR_OK;
		}
		return SR_ERR;
	default:
		return SR_ERR_NA;
	}

	return SR_OK;
}

static int config_set(uint32_t key, GVariant *data,
		const struct sr_dev_inst *sdi, const struct sr_channel_group *cg) {
	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;
	int idx;

	(void) cg;

	if (!sdi)
		return SR_ERR_ARG;

	devc = sdi->priv;

	switch (key) {
	case SR_CONF_SAMPLERATE:
		if ((idx = std_u64_idx(data, devc->samplerates, devc->num_samplerates))
				< 0)
			return SR_ERR_ARG;
		devc->cur_samplerate = devc->samplerates[idx];
		break;
	case SR_CONF_LIMIT_SAMPLES:
		devc->limit_samples = g_variant_get_uint64(data);
		break;
	case SR_CONF_CAPTURE_RATIO:
		devc->capture_ratio = g_variant_get_uint64(data);
		break;
	case SR_CONF_VOLTAGE_THRESHOLD:
		if ((idx = std_double_tuple_idx(data, ARRAY_AND_SIZE(thresholds))) < 0)
			return SR_ERR_ARG;
		devc->selected_voltage_level = idx;
		usb = sdi->conn;
		kingst_laxxxx_set_logic_level(usb->devhdl, thresholds[idx][0]);
		break;
	default:
		return SR_ERR_NA;
	}

	return SR_OK;
}

static int config_list(uint32_t key, GVariant **data,
		const struct sr_dev_inst *sdi, const struct sr_channel_group *cg) {
	struct dev_context *devc;

	devc = (sdi) ? sdi->priv : NULL;

	if (!cg) {
		switch (key) {
		case SR_CONF_SCAN_OPTIONS:
		case SR_CONF_DEVICE_OPTIONS:
			return STD_CONFIG_LIST(key, data, sdi, cg, scanopts, drvopts,
					devopts);
		case SR_CONF_SAMPLERATE:
			if (!devc)
				return SR_ERR_NA;
			*data = std_gvar_samplerates(devc->samplerates,
					devc->num_samplerates);
			break;
		case SR_CONF_VOLTAGE_THRESHOLD:
			*data = std_gvar_thresholds(ARRAY_AND_SIZE(thresholds));
			break;
		case SR_CONF_TRIGGER_MATCH:
			*data = std_gvar_array_i32(ARRAY_AND_SIZE(trigger_matches));
			break;
		default:
			return SR_ERR_NA;
		}
	} else {
		return SR_ERR_NA;
	}

	return SR_OK;
}

static int dev_acquisition_start(const struct sr_dev_inst *sdi) {
	sr_dbg("dev_acquisition_start(): start sampling");
	return kingst_laxxxx_acquisition_start(sdi);
}

static int dev_acquisition_stop(struct sr_dev_inst *sdi) {
	sr_dbg("dev_acquisition_start(): stop sampling");
	return kingst_laxxxx_acquisition_stop(sdi);
}

SR_PRIV struct sr_dev_driver kingst_la1010_driver_info = { .name =
		"kingst-la1010", .longname = "Kingst LA1010", .api_version = 1, .init =
		std_init, .cleanup = std_cleanup, .scan = scan,
		.dev_list = std_dev_list, .dev_clear = std_dev_clear, .config_get =
				config_get, .config_set = config_set,
		.config_list = config_list, .dev_open = dev_open,
		.dev_close = dev_close, .dev_acquisition_start = dev_acquisition_start,
		.dev_acquisition_stop = dev_acquisition_stop, .context = NULL, };

SR_REGISTER_DEV_DRIVER(kingst_la1010_driver_info);
