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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.	If not, see <http://www.gnu.org/licenses/>.
 */

#include <config.h>
#include <stdint.h>
#include <stdlib.h>
#include <libusb.h>

#include "protocol.h"

#define USB_TIMEOUT 1000


static int read_hex_digit(unsigned char *firmware, size_t fw_size, size_t *offset);
static int read_hex_byte(unsigned char *firmware, size_t fw_size, size_t *offset);
static int read_hex_line(unsigned char *firmware,
							size_t fw_size,
							size_t *offset,
							uint16_t *addr,
							unsigned char *buffer);
static int upload_cypress_firmware(struct sr_context *ctx,
									struct libusb_device_handle *hdl,
									const char *fw_file_name);

static void finish_acquisition(const struct sr_dev_inst *sdi);
static void free_transfer(struct libusb_transfer *transfer);
static void resubmit_transfer(struct libusb_transfer *transfer);
static size_t convert_sample_data(struct dev_context *devc,
									uint8_t *dest,
									size_t destcnt,
									const uint8_t *src,
									size_t srccnt);
static int command_start_acquisition(const struct sr_dev_inst *sdi);
static int start_transfers(const struct sr_dev_inst *sdi);

static unsigned int to_bytes_per_ms(unsigned int samplerate,
									uint8_t num_channels);
static size_t get_buffer_size(struct dev_context *devc);
static unsigned int get_number_of_transfers(struct dev_context *devc);
static unsigned int get_timeout(struct dev_context *devc);

static void LIBUSB_CALL
abort_acquisition_request_cb(struct libusb_transfer *transfer);
//void LIBUSB_CALL
//upload_transfer_complete_cb(struct libusb_transfer *xfr);
static void LIBUSB_CALL
receive_transfer(struct libusb_transfer *transfer);

static int control_in(libusb_device_handle *handle,
						uint8_t request,
						uint16_t value,
						uint8_t *data,
						uint16_t size);

static int control_out(libusb_device_handle *handle,
						uint8_t request,
						uint16_t value,
						uint8_t *data,
						uint16_t size);

static int upload_bindata_sync(libusb_device_handle *handle,
								uint8_t *data,
								int size,
								int trans_size);


int read_hex_digit(unsigned char *firmware, size_t fw_size, size_t *offset) {
	uint8_t data;

	if (*offset >= fw_size) {
		sr_err("read_hex_digit(): Unexpected end of data (offset %lu)", *offset);
		return SR_ERR;
	}
	data = firmware[*offset];
	*offset += 1;
	if ((data >= '0')
			&& (data <= '9')) {
		data -= '0';
	} else if ((data >= 'A')
			&& (data <= 'F')) {
		data -= 0x37;
	} else if ((data >= 'a')
			&& (data <= 'f')) {
		data -= 0x57;
	} else {
		sr_err("read_hex_digit(): Wrong hex digit: %c (offset %lu)", data, *offset);
		return SR_ERR;
	}
	return data;
}

int read_hex_byte(unsigned char *firmware, size_t fw_size, size_t *offset) {
	uint8_t d;
	int data;

	data = read_hex_digit(firmware, fw_size, offset);
	if (data >= 0) {
		d = data << 4;
		data = read_hex_digit(firmware, fw_size, offset);
		if (data >= 0) {
			data += d;
		}
	}
	return data;
}

int read_hex_line(unsigned char *firmware,
					size_t fw_size,
					size_t *offset,
					uint16_t *addr,
					unsigned char *buffer) {
	int data;
	uint8_t size, i, checksum;

	if (*offset >= fw_size) {
		sr_err("read_hex_line(): Unexpected end of data (offset %lu)", *offset);
		return SR_ERR;
	}
	if (firmware[*offset] != ':') {
		sr_err("read_hex_line(): Wrong hex line prefix, expected ':' (offset %lu)", *offset);
		return SR_ERR;
	}
	*offset += 1;

	// size -- 1 byte
	data = read_hex_byte(firmware, fw_size, offset);
	if (data < 0) {
		sr_err("read_hex_line(): Wrong size (offset %lu)", *offset);
		return SR_ERR;
	}
	checksum = data;
	size = data;
	if (size > 0x10) {
		sr_err("read_hex_line(): Size greater than 0x10, size: %02X (offset %lu)", size, *offset);
		return SR_ERR;
	}

	// address -- 2 byte
	data = read_hex_byte(firmware, fw_size, offset);
	if (data < 0) {
		sr_err("read_hex_line(): Wrong address first byte (offset %lu)", *offset);
		return SR_ERR;
	}
	checksum += data;
	*addr = data << 8;
	data = read_hex_byte(firmware, fw_size, offset);
	if (data < 0) {
		sr_err("read_hex_line(): Wrong address second byte (offset %lu)", *offset);
		return SR_ERR;
	}
	checksum += data;
	*addr += data;

	// type -- 1 byte
	data = read_hex_byte(firmware, fw_size, offset);
	if (data < 0) {
		sr_err("read_hex_line(): Wrong type (offset %lu)", *offset);
		return SR_ERR;
	}
	checksum += data;
	if (data == 0x01) {
		return 0;	// End of data
	}

	// data -- 'size' bytes
	for (i = 0; i < size; i++) {
		data = read_hex_byte(firmware, fw_size, offset);
		if (data < 0) {
			sr_err("read_hex_line(): Wrong data byte (offset %lu)", *offset);
			return SR_ERR;
		}
		checksum += data;
		buffer[i] = data;
	}

	// checksum -- 1 byte
	data = read_hex_byte(firmware, fw_size, offset);
	if (data < 0) {
		sr_err("read_hex_line(): Wrong checksum byte (offset %lu)", *offset);
		return SR_ERR;
	}

	if ((uint8_t) (checksum + data)) {
		sr_err("read_hex_line(): Wrong checksum, given %02X, expected %02X (offset %lu)",
					(uint8_t) (0 - checksum),
					(uint8_t) data,
					*offset);
		return SR_ERR;
	}

	// end of line
	while (firmware[*offset] != ':') {
		*offset += 1;
		if (*offset >= fw_size) {
			sr_err("read_hex_line(): Unexpected end of data (offset %lu)", *offset);
			return SR_ERR;
		}
	}

	return size;
}

/*
 * Check if the fx firmware was uploaded to cypress.
 * If FX firmware was loaded the cypress returns 0xFB04 (may be firmware version?).
 */
int kingst_la1010_has_fx_firmware(struct libusb_device_handle *hdl) {
	int err;
	struct libusb_device * dev;
 	struct libusb_config_descriptor * config;

	dev = libusb_get_device(hdl);
	err = libusb_get_active_config_descriptor(dev, &config);
	if (err) {
		sr_err(
				"kingst_la1010_has_fx_firmware(): get active usb config descriptor failed. libusb err: %s",
				libusb_error_name(err));
		return err;
	}
	if (config->bNumInterfaces == 1) {
		if (config->interface->altsetting->bNumEndpoints == 2) {
			return SR_OK;
		} else {
			sr_err(
					"kingst_la1010_has_fx_firmware(): iface->altsetting->bNumEndpoints: %d",
					config->interface->altsetting->bNumEndpoints);
		}
	} else {
		sr_err(
				"kingst_la1010_has_fx_firmware(): config->bNumInterfaces: %d",
				config->bNumInterfaces);
	}

	return SR_ERR;
}

int upload_cypress_firmware(struct sr_context *ctx,
							struct libusb_device_handle *hdl,
							const char *fw_file_name) {
	size_t s, offset, fw_size;
	uint16_t addr;
	int res;
	unsigned char *firmware, buffer[16];

	s = strlen(fw_file_name);
	if (s > 0) {
		if ((res = libusb_set_configuration(hdl, USB_CONFIGURATION)) < 0) {
			sr_err("upload_cypress_firmware(): Unable to set configuration: %s",
					libusb_error_name(res));
			return SR_ERR;
		}
		if ((ezusb_reset(hdl, 1)) < 0) {
			sr_err("upload_cypress_firmware(): Reset Cypress for upload FW failed");
			return SR_ERR;
		}
		if (memcmp(fw_file_name + (s - 3), "hex", 3)) {
			// binary
			if (ezusb_install_firmware(ctx, hdl, fw_file_name) < 0) {
				sr_err("upload_cypress_firmware(): Upload binary FW failed");
				return SR_ERR;
			}
		} else {
			// Intel-HEX
			firmware = sr_resource_load(ctx,
										SR_RESOURCE_FIRMWARE,
										fw_file_name,
										&fw_size,
										1 << 16);

			if (!firmware) {
				sr_err("upload_cypress_firmware(): Read Intel-HEX file failed");
				return SR_ERR;
			}

			offset = 0;
			res = read_hex_line(firmware, fw_size, &offset, &addr, buffer);
			while (res > 0) {
				res = control_out(hdl, 0xA0, addr, buffer, res);
				if (res < 0) {
					break;
				}
				res = read_hex_line(firmware, fw_size, &offset, &addr, buffer);
			}

			g_free(firmware);

			if (res < 0) {
				sr_err("upload_cypress_firmware(): Upload Intel-HEX FW failed");
				return SR_ERR;
			}
		}
		if ((ezusb_reset(hdl, 0)) < 0) {
			sr_err("upload_cypress_firmware(): Reset Cypress for upload FW failed");
			return SR_ERR;
		}
	} else {
		sr_err("upload_cypress_firmware(): FW file name has null size");
		return SR_ERR;
	}
	return SR_OK;
}


/*
 * Upload spartan bitstream.
 */
int kingst_la1010_upload_cypress_firmware(struct sr_context *ctx,
											struct libusb_device_handle *hdl,
											const struct kingst_la1010_profile *prof) {
	char fw_file_name[16];
	int res;

	if (prof->fx_firmware) {
		res = upload_cypress_firmware(ctx, hdl, prof->fx_firmware);
	} else {
		snprintf(fw_file_name, 16, "fw%04X.hex", prof->pid);
		res = upload_cypress_firmware(ctx, hdl, fw_file_name);
		if (res < 0) {
			snprintf(fw_file_name, 16, "fw%04X.fw", prof->pid);
			res = upload_cypress_firmware(ctx, hdl, fw_file_name);
		}
	}
	return res;
}

/*
 * Upload spartan bitstream.
 */
int kingst_la1010_upload_spartan_firmware(const struct sr_dev_inst *sdi) {
	struct drv_context *drvc;
	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;
	struct libusb_device *usbdev;
	union fx_status fx_status;
	union spartan_status spartan_status;
	uint8_t *bindata, verify_data[32];
	uint32_t binsize;
	int err, i;

	usb = sdi->conn;
	drvc = sdi->driver->context;
	devc = sdi->priv;

	err = control_in(usb->devhdl,
						CMD_STATUS,
						CMD_STATUS_USB_STATUS,
						fx_status.bytes,
						4);
	if (err) {
		sr_err(
				"kingst_la1010_upload_spartan_firmware(): check Cypress status failed. libusb err: %s",
				libusb_error_name(err));
		return err;
	}

	if (fx_status.code[0] != 0xFFFFFFFF) {
		sr_err(
				"kingst_la1010_upload_spartan_firmware(): wrong Cypress status: %x",
				fx_status.code[0]);
		return SR_ERR;
	}

	err = control_in(usb->devhdl,
						CMD_STATUS,
						CMD_STATUS_FX_STATUS,
						fx_status.bytes,
						sizeof(fx_status.bytes));
	if (err) {
		sr_err(
				"kingst_la1010_upload_spartan_firmware(): check Cypress FW status failed. libusb err: %s",
				libusb_error_name(err));
		return err;
	}

	if ((fx_status.bytes[0] ^ fx_status.bytes[1]) != 0xFF) {
		sr_err(
				"kingst_la1010_upload_spartan_firmware(): wrong Cypress FW status: %x, %x",
				fx_status.bytes[0],
				fx_status.bytes[1]);
		return SR_ERR;
	}

	bindata = sr_resource_load(drvc->sr_ctx,
								SR_RESOURCE_FIRMWARE,
								devc->profile->spartan_firmware,
								(size_t*) &binsize,
								0x020000);
	if (!bindata) {
		return SR_ERR_MALLOC;
	}

	err = control_out(usb->devhdl,
						CMD_SPARTAN_UPLOAD,
						0,
						(uint8_t*) &binsize,
						sizeof(binsize));
	if (err) {
		sr_err(
				"kingst_la1010_upload_spartan_firmware(): upload Spartan firmware failed. libusb err: %s",
				libusb_error_name(err));
		return err;
	}

	usbdev = libusb_get_device(usb->devhdl);
	if (usbdev) {
		i = libusb_get_max_packet_size(usbdev, USB_UPLOAD_DATA_EP);

		sr_dbg("Upload Spartan firmware using packet size %d", i);

		err = upload_bindata_sync(usb->devhdl, bindata, binsize, i);
		g_free(bindata);
		if (err) {
			sr_err(
					"kingst_la1010_upload_spartan_firmware(): upload Spartan firmware failed. libusb err: %s",
					libusb_error_name(err));
			return err;
		}

		g_usleep(100 * 1000);

		err = control_in(usb->devhdl,
							CMD_SPARTAN_UPLOAD,
							0,
							spartan_status.bytes,
							1);
		if (err) {
			sr_err(
					"kingst_la1010_upload_spartan_firmware(): check Spartan status failed. libusb err: %s",
					libusb_error_name(err));
			return err;
		}

		if (spartan_status.bytes[0]) {
			sr_err("Check Spartan returns wrong status: %d", spartan_status.bytes[0]);
			return SR_ERR;
		}

		g_usleep(30 * 1000);

		err = control_out(usb->devhdl,
							CMD_10,
							1,
							NULL,
							0);
		if (err) {
			sr_err(
					"kingst_la1010_upload_spartan_firmware(): start Spartan failed. libusb err: %s",
					libusb_error_name(err));
			return err;
		}

		g_usleep(40 * 1000);

		err = control_in(usb->devhdl,
							CMD_CONTROL,
							0,
							spartan_status.bytes,
							2);
		if (err) {
			sr_err(
					"kingst_la1010_upload_spartan_firmware(): check Spartan status failed. libusb err: %s",
					libusb_error_name(err));
			return err;
		}

		/// GetVerifyData
		verify_data[0] = 0xA3;
		verify_data[1] = 0x09;
		verify_data[2] = 0xC9;
		err = control_out(usb->devhdl,
							CMD_60,
							0,
							verify_data,
							11);
		if (err) {
			sr_err(
					"kingst_la1010_upload_spartan_firmware(): GetVerifyData out failed. libusb err: %s",
					libusb_error_name(err));
			return err;
		}
		err = control_in(usb->devhdl,
							CMD_60,
							0,
							verify_data,
							18);
		if (err) {
			sr_err(
					"kingst_la1010_upload_spartan_firmware(): GetVerifyData in failed. libusb err: %s",
					libusb_error_name(err));
			return err;
		}
		/// GetVerifyData end

		/// GetDeviceID
		verify_data[0] = 0xA3;
		verify_data[1] = 0x01;
		verify_data[2] = 0xCA;
		err = control_out(usb->devhdl,
							CMD_60,
							0,
							verify_data,
							3);
		if (err) {
			sr_err(
					"kingst_la1010_upload_spartan_firmware(): GetVerifyData out failed. libusb err: %s",
					libusb_error_name(err));
			return err;
		}
		err = control_in(usb->devhdl,
							CMD_60,
							0,
							verify_data,
							12);
		if (err) {
			sr_err(
					"kingst_la1010_upload_spartan_firmware(): GetVerifyData in failed. libusb err: %s",
					libusb_error_name(err));
			return err;
		}
		/// GetDeviceID end

	} else {
		sr_err(
				"Upload Spartan failed. Can't get usb device struct by device handle");
		return SR_ERR;
	}

	return err;
}

/*
 * Init the device for usage.
 */
int kingst_la1010_init_spartan(struct libusb_device_handle *handle) {
	int err;

	/*
	 * Configure voltage threshold.
	 */
	err = kingst_la1010_set_logic_level(handle, 1.58);
	if (err)
		return err;

	/*
	 * Configure PWM channels -- two channels.
	 */
	err = kingst_la1010_configure_pwm(handle, 0, 50, 0, 50);
	if (err)
		return err;

	return SR_OK;
}

int kingst_la1010_receive_data(int fd, int revents, void *cb_data) {
	struct timeval tv;
	struct drv_context *drvc;

	(void) fd;
	(void) revents;

	drvc = (struct drv_context*) cb_data;
	tv.tv_sec = tv.tv_usec = 0;
	libusb_handle_events_timeout(drvc->sr_ctx->libusb_ctx, &tv);

	return TRUE;
}

int kingst_la1010_acquisition_start(const struct sr_dev_inst *sdi) {
	struct sr_dev_driver *di;
	struct drv_context *drvc;
	struct dev_context *devc;
	int timeout, ret;

	di = sdi->driver;
	drvc = di->context;
	devc = sdi->priv;

	devc->ctx = drvc->sr_ctx;
	devc->sent_samples = 0;
	devc->cur_channel = 0;
	devc->empty_transfer_count = 0;
	devc->acq_aborted = FALSE;
	memset(devc->channel_data, 0, 16 * 2);

	if (kingst_la1010_configure_channels(sdi) != SR_OK) {
		sr_err("Failed to configure channels.");
		return SR_ERR;
	}

	timeout = get_timeout(devc);
	usb_source_add(sdi->session, devc->ctx, timeout, kingst_la1010_receive_data,
			drvc);

	devc->convbuffer_size = (get_buffer_size(devc) / devc->num_channels) * 16
			+ 32;

	devc->convbuffer = g_try_malloc(devc->convbuffer_size);

	if (devc->convbuffer) {
		if ((ret = command_start_acquisition(sdi)) != SR_OK) {
			kingst_la1010_acquisition_stop(sdi);
			if (devc->convbuffer)
				g_free(devc->convbuffer);
			devc->convbuffer = NULL;
			return ret;
		}

		start_transfers(sdi);
	} else {
		sr_err("Failed to allocate memory for data buffer.");
		return SR_ERR_MALLOC;
	}

	return SR_OK;
}

static int command_start_acquisition(const struct sr_dev_inst *sdi) {
	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;
	uint64_t samplerate;
	uint16_t division;
	int err;
	uint8_t data[4];

	devc = sdi->priv;
	usb = sdi->conn;
	samplerate = devc->cur_samplerate;
	usb = sdi->conn;

	data[0] = 0;
	err = control_out(usb->devhdl, CMD_CONTROL, CMD_CONTROL_START, data, 1);
	if (err) {
		sr_err("Start configure channels failed.");
		return err;
	}

	err = control_out(usb->devhdl, CMD_SAMPLING_CONFIG, CMD_SAMPLING_CONFIG,
			NULL, 0);
	if (err) {
		sr_err("Enter to sampling rate configuration mode failed.");
		return err;
	}

	division = SAMPLING_BASE_FREQUENCY / samplerate;

	err = control_out(usb->devhdl,
	CMD_CONTROL,
	CMD_CONTROL_SAMPLE_RATE, (uint8_t*) &division, sizeof(division));
	if (err) {
		sr_err("Set sample rate failed.");
		return err;
	}

	data[0] = devc->cur_channels & 0xFF;
	data[1] = devc->cur_channels >> 8;
	data[2] = 0;
	data[3] = 0;
	err = control_out(usb->devhdl, CMD_CONTROL, CMD_CONTROL_CHAN_SELECT, data,
			4);
	if (err) {
		sr_err("Set channel mask failed.");
		return err;
	}

	data[0] = 1;
	err = control_out(usb->devhdl, CMD_CONTROL, CMD_CONTROL_START, data, 1);
	if (err) {
		sr_err("Commit the channels and sample rate configuration failed.");
		return err;
	}

	err = control_out(usb->devhdl, CMD_SAMPLING_START, CMD_SAMPLING_START, NULL,
			0);
	if (err) {
		sr_err("Star sampling failed.");
		return err;
	}

	return SR_OK;
}

int kingst_la1010_acquisition_stop(const struct sr_dev_inst *sdi) {
	int i, ret;
	struct sr_usb_dev_inst *usb;
	struct dev_context *devc;

	sr_dbg("kingst_la1010_acquisition_stop(): stop requested");
	usb = sdi->conn;
	devc = sdi->priv;

	devc->acq_aborted = TRUE;

	/*
	 * There are need send request to stop sampling.
	 */
	ret = kingst_la1010_abort_acquisition_request(usb->devhdl);
	if (ret)
		sr_err(
				"kingst_la1010_acquisition_stop(): Stop sampling error %d. libusb err: %s",
				ret, libusb_error_name(ret));

	sr_dbg("kingst_la1010_acquisition_stop(): cancel %d transfers", devc->num_transfers);
	for (i = devc->num_transfers - 1; i >= 0; i--) {
		if (devc->transfers[i]) {
			ret = libusb_cancel_transfer(devc->transfers[i]);
			if (ret != LIBUSB_ERROR_NOT_FOUND) {
				sr_err(
						"kingst_la1010_acquisition_stop(): cancel %d transfer error %d. libusb err: %s",
						i, ret, libusb_error_name(ret));
			}
		}
	}

	return ret;
}


/*
 * Configure threshold levels.
 * Available from -4.0 to 4.0.
 */
int kingst_la1010_set_logic_level(struct libusb_device_handle *hdl,
		double level) {
	uint32_t data;
	int err;

	if (level > -0.4) {
		if (level < 3) {
			/*
			 * For voltage levels between -0.399 and 2.999 data is 0x00F1YYYY
			 * where YYYY -- is .....
			 */
			data = ((level + 0.4) * 302);
			data += 0x00F10000;
		} else {
			/*
			 * For voltage levels between 2.999 and 4 data is 0x0000YYYY
			 * where YYYY -- is .....
			 */
			if (level > 4) {
				level = 4;
			}
			data = ((level - 1.2) * 302);
		}
	} else {
		/*
		 * For voltage levels between -4 and -0.4 data is 0x02D4YYYY
		 * where YYYY -- is .....
		 */
		if (level < -3.6) {
			level = -3.6;
		}
		data = ((level + 3.6) * 302);
		data += 0x02D40000;
	}

	err = control_out(hdl,
						CMD_CONTROL,
						CMD_CONTROL_LOG_LEVEL,
						(uint8_t*) &data,
						sizeof(data));
	if (err)
		return err;

	return SR_OK;
}

/*
 * Configure PWM channels -- two channels.
 * For each channel: frequency between 1 kHz (1000) and 200 MHz (200000000)$
 *										duty between 1 and 99.
 * Frequency == 0 -- power off PWM channel.
 */
int kingst_la1010_configure_pwm(struct libusb_device_handle *hdl,
								uint64_t pwm1_freq,
								uint64_t pwm1_duty,
								uint64_t pwm2_freq,
								uint64_t pwm2_duty) {
	uint32_t data[2];
	int err;

	if (pwm1_duty > 100) {
		sr_err("Wrong PWM1 duty ratio, given %ld, but only 0 .. 100 allowed",
				pwm1_duty);
		pwm1_duty = 50;
	}
	if (pwm2_duty > 100) {
		sr_err("Wrong PWM2 duty ratio, given %ld, but only 0 .. 100 allowed",
				pwm2_duty);
		pwm2_duty = 50;
	}

	if (pwm1_freq) {
		/*
		 * PWM data is division 800000000 to frequency
		 */
		pwm1_freq = PWM_BASE_FREQUENCY / pwm1_freq;
		/*
		 * Duty data is between '0' and 'PWM data'
		 */
		pwm1_duty = pwm1_freq * pwm1_duty / 100;
	}

	if (pwm2_freq) {
		pwm2_freq = PWM_BASE_FREQUENCY / pwm2_freq;
		pwm2_duty = pwm2_freq * pwm2_duty / 100;
	}

	data[0] = 0x00;
	err = control_out(hdl, CMD_CONTROL, CMD_CONTROL_PWM, (uint8_t*) &data, 1);
	if (err)
		return err;

	data[0] = pwm1_freq;
	data[1] = pwm1_duty;
	err = control_out(hdl, CMD_CONTROL, CMD_CONTROL_PWM_1, (uint8_t*) &data, sizeof(data));
	if (err)
		return err;

	data[0] = pwm2_freq;
	data[1] = pwm2_duty;
	err = control_out(hdl, CMD_CONTROL, CMD_CONTROL_PWM_2, (uint8_t*) &data, sizeof(data));
	if (err)
		return err;

	data[0] = 0x01;
	err = control_out(hdl, CMD_CONTROL, CMD_CONTROL_PWM, (uint8_t*) &data, 1);
	if (err)
		return err;

	return SR_OK;
}

int kingst_la1010_dev_open(const struct sr_dev_inst *sdi) {
	libusb_device **devlist;
	struct sr_dev_driver *di;
	struct sr_usb_dev_inst *usb;
	struct libusb_device_descriptor des;
	struct dev_context *devc;
	struct drv_context *drvc;
	int ret = SR_ERR, i, device_count;
	char connection_id[64];

	di = sdi->driver;
	drvc = di->context;
	devc = sdi->priv;
	usb = sdi->conn;

	device_count = libusb_get_device_list(drvc->sr_ctx->libusb_ctx, &devlist);
	if (device_count < 0) {
		sr_err("Failed to get device list: %s.",
				libusb_error_name(device_count));
		return SR_ERR;
	}

	for (i = 0; i < device_count; i++) {
		libusb_get_device_descriptor(devlist[i], &des);

		if (des.idVendor != devc->profile->vid
				|| des.idProduct != devc->profile->pid)
			continue;

		if ((sdi->status == SR_ST_INITIALIZING)
				|| (sdi->status == SR_ST_INACTIVE)) {
			/*
			 * Check device by its physical USB bus/port address.
			 */
			usb_get_port_path(devlist[i], connection_id, sizeof(connection_id));
			if (strcmp(sdi->connection_id, connection_id))
				/* This is not the one. */
				continue;
		}

		if (!(ret = libusb_open(devlist[i], &usb->devhdl))) {
			if (usb->address == 0xff)
				/*
				 * First time we touch this device after FW
				 * upload, so we don't know the address yet.
				 */
				usb->address = libusb_get_device_address(devlist[i]);
		} else {
			sr_err("Failed to open device: %s.", libusb_error_name(ret));
			ret = SR_ERR;
			break;
		}

		if (libusb_has_capability(LIBUSB_CAP_SUPPORTS_DETACH_KERNEL_DRIVER)) {
			if (libusb_kernel_driver_active(usb->devhdl, USB_INTERFACE) == 1) {
				if ((ret = libusb_detach_kernel_driver(usb->devhdl,
				USB_INTERFACE)) < 0) {
					sr_err("Failed to detach kernel driver: %s.",
							libusb_error_name(ret));
					ret = SR_ERR;
					break;
				}
			}
		}

		sr_info(
				"Opened device on %d.%d (logical) / %s (physical), " "interface %d",
				usb->bus, usb->address, connection_id, USB_INTERFACE);

		ret = SR_OK;

		break;
	}

	libusb_free_device_list(devlist, 1);

	return ret;
}

static void LIBUSB_CALL
abort_acquisition_request_cb(struct libusb_transfer *transfer) {
	libusb_free_transfer(transfer);
}

int kingst_la1010_abort_acquisition_request(libusb_device_handle *handle) {
	struct libusb_transfer *transfer;
	unsigned char *buffer;
	int ret;

	transfer = libusb_alloc_transfer(0);
	if (!transfer)
		return LIBUSB_ERROR_NO_MEM;
	transfer->buffer = 0;

	buffer = (unsigned char*) g_try_malloc(LIBUSB_CONTROL_SETUP_SIZE + 1);
	if (!buffer) {
		libusb_free_transfer(transfer);
		return LIBUSB_ERROR_NO_MEM;
	}

	libusb_fill_control_setup(buffer,
								LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR,
								CMD_CONTROL,
								CMD_CONTROL_START,
								0,
								1);
	buffer[LIBUSB_CONTROL_SETUP_SIZE] = 0;

	libusb_fill_control_transfer(transfer,
									handle,
									buffer,
									abort_acquisition_request_cb,
									NULL,
									1000);
	transfer->flags = LIBUSB_TRANSFER_FREE_BUFFER;

	ret = libusb_submit_transfer(transfer);
	if (ret < 0) {
		libusb_free_transfer(transfer);
		if (transfer->buffer) {
			g_free(transfer->buffer);
			transfer->buffer = NULL;
		}
		return ret;
	}

	return ret;
}

static void finish_acquisition(const struct sr_dev_inst *sdi) {
	struct dev_context *devc;

	devc = sdi->priv;

	std_session_send_df_end(sdi);

	usb_source_remove(sdi->session, devc->ctx);

	devc->num_transfers = 0;
	g_free(devc->transfers);

	if (devc->convbuffer) {
		g_free(devc->convbuffer);
		devc->convbuffer = NULL;
	}

	if (devc->stl) {
		soft_trigger_logic_free(devc->stl);
		devc->stl = NULL;
	}
}

static void free_transfer(struct libusb_transfer *transfer) {
	struct sr_dev_inst *sdi;
	struct dev_context *devc;
	unsigned int i;

	sdi = transfer->user_data;
	devc = sdi->priv;

	libusb_free_transfer(transfer);
	if (transfer->buffer) {
		g_free(transfer->buffer);
		transfer->buffer = NULL;
	}

	for (i = 0; i < devc->num_transfers; i++) {
		if (devc->transfers[i] == transfer) {
			devc->transfers[i] = NULL;
			break;
		}
	}

	devc->submitted_transfers--;

	if (devc->submitted_transfers == 0)
		finish_acquisition(sdi);
}

static void resubmit_transfer(struct libusb_transfer *transfer) {
	int ret;

	if ((ret = libusb_submit_transfer(transfer)) == LIBUSB_SUCCESS)
		return;

	sr_err("%s: %s", __func__, libusb_error_name(ret));
	free_transfer(transfer);

}

/*
 * Sampling data same as Saleae Logic16.
 */
static size_t convert_sample_data(struct dev_context *devc,
									uint8_t *dest,
									size_t destcnt,
									const uint8_t *src,
									size_t srccnt) {
	uint16_t *channel_data;
	int i, cur_channel;
	size_t ret = 0;
	uint16_t sample, channel_mask;

	srccnt /= 2;

	channel_data = devc->channel_data;
	cur_channel = devc->cur_channel;

	while (srccnt--) {
		sample = src[0] | (src[1] << 8);
		src += 2;

		channel_mask = devc->channel_masks[cur_channel];

		for (i = 0; i < 16; ++i, sample >>= 1)
			if (sample & 1)
				channel_data[i] |= channel_mask;

		if (++cur_channel == devc->num_channels) {
			cur_channel = 0;
			if (destcnt < 16 * 2) {
				sr_err("Conversion buffer too small! dstcnt %ld, srccnt %ld",
						destcnt, srccnt);
				break;
			}
			memcpy(dest, channel_data, 16 * 2);
			memset(channel_data, 0, 16 * 2);
			dest += 16 * 2;
			ret += 16;
			destcnt -= 16 * 2;
		}
	}

	devc->cur_channel = cur_channel;

	return ret;
}

static void LIBUSB_CALL
receive_transfer(struct libusb_transfer *transfer) {
	struct sr_dev_inst *sdi;
	struct dev_context *devc;
	gboolean packet_has_error = FALSE;
	size_t num_samples, new_samples;
	int trigger_offset, pre_trigger_samples;
	struct sr_datafeed_logic logic;
	struct sr_datafeed_packet packet;

	sdi = transfer->user_data;
	devc = sdi->priv;

	/*
	 * If acquisition has already ended, just free any queued up
	 * transfer that come in.
	 */
	if (devc->acq_aborted) {

		free_transfer(transfer);
		return;
	}

	switch (transfer->status) {
	case LIBUSB_TRANSFER_NO_DEVICE:
		sr_err("receive_transfer(): no device");
		kingst_la1010_acquisition_stop(sdi);
		free_transfer(transfer);
		return;
	case LIBUSB_TRANSFER_COMPLETED:
	case LIBUSB_TRANSFER_TIMED_OUT: /* We may have received some data though. */
		break;
	default:
		packet_has_error = TRUE;
		break;
	}

	if (transfer->actual_length == 0 || packet_has_error) {
		devc->empty_transfer_count++;
		if (devc->empty_transfer_count > MAX_EMPTY_TRANSFERS) {
			sr_err("receive_transfer(): MAX_EMPTY_TRANSFERS exceeded");
			kingst_la1010_acquisition_stop(sdi);
			free_transfer(transfer);
		} else {
			sr_err(
					"receive_transfer(): resubmit transfer due error: actual_length %d, has_error %d",
					transfer->actual_length, packet_has_error);
			resubmit_transfer(transfer);
		}
		return;
	} else {
		devc->empty_transfer_count = 0;
	}

	logic.data = devc->convbuffer;
	logic.unitsize = 2;

	packet.type = SR_DF_LOGIC;
	packet.payload = &logic;

	new_samples = convert_sample_data(devc,
										devc->convbuffer,
										devc->convbuffer_size,
										(uint8_t*) transfer->buffer,
										transfer->actual_length);
	if (new_samples == 0) {
		if (transfer->actual_length) {
			sr_err("receive_transfer(): convert data failed");
			kingst_la1010_acquisition_stop(sdi);
			free_transfer(transfer);
			return;
		} else {
			resubmit_transfer(transfer);
			return;
		}
	}

	if (devc->trigger_fired) {
		if (devc->limit_samples
				&& (new_samples > devc->limit_samples - devc->sent_samples))
			new_samples = devc->limit_samples - devc->sent_samples;
		logic.length = new_samples * 2;
		sr_session_send(sdi, &packet);
		devc->sent_samples += new_samples;
	} else {
		trigger_offset = soft_trigger_logic_check(devc->stl,
													devc->convbuffer,
													new_samples * 2,
													&pre_trigger_samples);
		if (trigger_offset > -1) {
			devc->sent_samples += pre_trigger_samples;
			packet.type = SR_DF_LOGIC;
			packet.payload = &logic;
			num_samples = new_samples - trigger_offset;
			if (devc->limit_samples
					&& (num_samples > devc->limit_samples - devc->sent_samples))
				num_samples = devc->limit_samples - devc->sent_samples;
			logic.length = num_samples * 2;
			logic.data = devc->convbuffer + trigger_offset * 2;
			sr_session_send(sdi, &packet);
			devc->sent_samples += num_samples;

			devc->trigger_fired = TRUE;
		}
	}

	if (devc->limit_samples && (devc->sent_samples >= devc->limit_samples)) {
		sr_dbg("receive_transfer(): samples limit reached %ld",
				devc->sent_samples);
		kingst_la1010_acquisition_stop(sdi);
		free_transfer(transfer);
	} else
		resubmit_transfer(transfer);
}

int kingst_la1010_configure_channels(const struct sr_dev_inst *sdi) {
	struct dev_context *devc;
	const GSList *l;
	struct sr_channel *ch;
	uint16_t channel_bit;

	devc = sdi->priv;
	devc->num_channels = 0;
	devc->cur_channels = 0;
	for (l = sdi->channels; l; l = l->next) {
		ch = (struct sr_channel*) l->data;
		if (ch->enabled == FALSE)
			continue;

		channel_bit = 1 << (ch->index);
		devc->cur_channels |= channel_bit;
		devc->channel_masks[devc->num_channels++] = channel_bit;
	}

	return SR_OK;
}

static unsigned int to_bytes_per_ms(unsigned int samplerate, uint8_t num_channels) {
	unsigned long result = samplerate * num_channels / 8;
	return result / 1000;
}

static size_t get_buffer_size(struct dev_context *devc) {
	size_t s;

	/*
	 * This transfer size used by KingstVIS
	 */
	s = devc->num_channels * devc->cur_samplerate / 128;
	return (s + 511) & ~511;
}

static unsigned int get_number_of_transfers(struct dev_context *devc) {
	unsigned int n;

	/* Total buffer size should be able to hold about 500ms of data. */
	n = (500 * to_bytes_per_ms(devc->cur_samplerate, devc->num_channels))
			/ get_buffer_size(devc);

//	if (n > NUM_SIMUL_TRANSFERS)
//		return NUM_SIMUL_TRANSFERS;

	return n;
}

static unsigned int get_timeout(struct dev_context *devc) {
	size_t total_size;
	unsigned int timeout;

	total_size = get_buffer_size(devc) * get_number_of_transfers(devc);
	timeout = total_size
			/ to_bytes_per_ms(devc->cur_samplerate, devc->num_channels);
	return timeout + timeout / 4; /* Leave a headroom of 25% percent. */
}

static int start_transfers(const struct sr_dev_inst *sdi) {
	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;
	struct sr_trigger *trigger;
	struct libusb_transfer *transfer;
	unsigned int i, num_transfers;
	int timeout, ret;
	int pre_trigger_samples;
	unsigned char *buf;
	size_t size;

	sr_dbg("start_transfers():");

	devc = sdi->priv;
	usb = sdi->conn;

	devc->sent_samples = 0;
	devc->acq_aborted = FALSE;
	devc->empty_transfer_count = 0;

	if ((trigger = sr_session_trigger_get(sdi->session))) {
		pre_trigger_samples = 0;
		if (devc->limit_samples > 0)
			pre_trigger_samples = (devc->capture_ratio * devc->limit_samples) / 100;
		devc->stl = soft_trigger_logic_new(sdi, trigger, pre_trigger_samples);
		if (!devc->stl)
			return SR_ERR_MALLOC;
		devc->trigger_fired = FALSE;

		sr_dbg("Trigger was enabled");
	} else
		devc->trigger_fired = TRUE;

	sr_dbg("Samplerate: %ld", devc->cur_samplerate);
	sr_dbg("Number of channels: %d", devc->num_channels);

	num_transfers = get_number_of_transfers(devc);
	sr_dbg("Number transfers was calculated: %d (0x%X)", num_transfers,
			num_transfers);

	size = get_buffer_size(devc);
	sr_dbg("Buffer size for each transfer was calculated: %ld (0x%lX)", size,
			size);

	devc->submitted_transfers = 0;

	devc->transfers = g_try_malloc0(sizeof(*devc->transfers) * num_transfers);
	if (!devc->transfers) {
		sr_err("USB transfers malloc failed.");
		return SR_ERR_MALLOC;
	}

	timeout = get_timeout(devc);
	sr_dbg("Timeout for each transfer was calculated: %d (0x%X)", timeout,
			timeout);

	for (i = 0; i < num_transfers; i++) {
		if (!(buf = g_try_malloc(size))) {
			sr_err("USB transfer buffer malloc failed.");
			return SR_ERR_MALLOC;
		}
		transfer = libusb_alloc_transfer(0);
		libusb_fill_bulk_transfer(transfer, usb->devhdl,
		USB_SAMPLING_DATA_EP, buf, size, receive_transfer, (void*) sdi,
				timeout);

		if ((ret = libusb_submit_transfer(transfer)) != 0) {
			libusb_free_transfer(transfer);
			g_free(buf);
			if (i == 0) {
				sr_err("Failed to submit transfer: %s.",
						libusb_error_name(ret));
				kingst_la1010_acquisition_stop(sdi);
				return SR_ERR;
			} else {
				break;
			}
		}
		devc->transfers[i] = transfer;
		devc->submitted_transfers++;
	}

	devc->num_transfers = i;
	sr_info("%d transfers was submited: data size 0x%lx, timeout %d",
			devc->num_transfers, size, timeout);

	std_session_send_df_header(sdi);

	return SR_OK;
}

static int control_in(libusb_device_handle *handle,
						uint8_t request,
						uint16_t value,
						uint8_t *data,
						uint16_t size) {
	int actual_length = 0;
	uint8_t empty_data[1] = { 0 };

	actual_length = libusb_control_transfer(handle,
												LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR,
												request,
												value,
												0,
												data == NULL ? empty_data : data,
												size,
												USB_TIMEOUT);
	if (actual_length < 0) {
		sr_err("Failed to send 'control in' request to device: %s.",
				libusb_error_name(actual_length));
		return actual_length;
	} else if (actual_length != size) {
		sr_err(
				"Wrong response size for 'control in' request: expected %d given %d.",
				size, actual_length);
		return SR_ERR;
	}

	return SR_OK;
}

int control_out(libusb_device_handle *handle,
				uint8_t request,
				uint16_t value,
				uint8_t *data,
				uint16_t size) {
	int actual_length = 0;
	uint8_t empty_data[1] = { 0 };

	actual_length = libusb_control_transfer(handle,
			LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR, request, value, 0,
			data == NULL ? empty_data : data, size,
			USB_TIMEOUT);
	if (actual_length < 0) {
		sr_err("Failed to send 'control out' request to device: %s.",
				libusb_error_name(actual_length));
		return actual_length;
	} else if (actual_length != size) {
		sr_err(
				"Wrong response size for 'control out' request: expected %d given %d.",
				size, actual_length);
		return SR_ERR;
	}

	return SR_OK;
}

int upload_bindata_sync(libusb_device_handle *handle,
						uint8_t *bindata,
						int size,
						int trans_size) {
	int data_len, err, actual_len;

	while (size > 0) {
		data_len = size;
		if (data_len > trans_size) {
			data_len = trans_size;
		}
		err = libusb_bulk_transfer(handle,
									USB_UPLOAD_DATA_EP,
									bindata,
									data_len,
									&actual_len,
									100);
		if (err) {
			sr_err("Failed to upload Spartan firmware: %s.",
						libusb_error_name(err));
			return err;
		} else if (actual_len != data_len) {
			sr_err(
					"Failed to upload Spartan firmware: sent %d but actual sent %d.",
					data_len, actual_len);
			return SR_ERR_DATA;
		}
		size -= actual_len;
		bindata += actual_len;
	}
	err = libusb_bulk_transfer(handle,
								USB_UPLOAD_DATA_EP,
								(unsigned char*) &data_len, // fake buffer for 0-sized transfer (passing NULL may be not working in some systems)
								0,
								&actual_len,
								100);
	if (err) {
		sr_err("Failed to upload Spartan firmware: %s.",
					libusb_error_name(err));
		return err;
	}
	return size;
}

struct dev_context* kingst_la1010_dev_new(void) {
	struct dev_context *devc;

	devc = g_malloc0(sizeof(struct dev_context));
	devc->profile = NULL;
	devc->fw_updated = 0;
	devc->cur_samplerate = 0;
	devc->limit_samples = 0;
	devc->capture_ratio = 0;
	devc->num_channels = 0;
	devc->convbuffer = NULL;
	devc->stl = NULL;

	return devc;
}

