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

static unsigned int to_bytes_per_ms(unsigned int samplerate, uint8_t num_channels);
static size_t get_buffer_size(struct dev_context *devc);
static unsigned int get_number_of_transfers(struct dev_context *devc);
static unsigned int get_timeout(struct dev_context *devc);

static void LIBUSB_CALL
abort_acquisition_request_cb(struct libusb_transfer *transfer);
void LIBUSB_CALL
upload_transfer_complete_cb (struct libusb_transfer * xfr);
static void LIBUSB_CALL
receive_transfer(struct libusb_transfer * transfer);


static int control_in (libusb_device_handle * handle,
											 uint8_t request,
											 uint16_t value,
											 uint8_t *data,
											 uint16_t size);
static int control_out (libusb_device_handle * handle,
												uint8_t request,
												uint16_t value,
												uint8_t *data,
												uint16_t size);
//static int upload_bindata (libusb_device_handle * handle,
//							uint8_t *data,
//							int size);
static int upload_bindata_sync (libusb_device_handle * handle,
																uint8_t *data,
																int size,
																int trans_size);


int kingst_la1010_receive_data(int fd, int revents, void *cb_data)
{
	struct timeval tv;
	struct drv_context *drvc;

	(void)fd;
	(void) revents;

	drvc = (struct drv_context *) cb_data;
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
	usb_source_add(sdi->session, devc->ctx, timeout, kingst_la1010_receive_data, drvc);

	devc->convbuffer_size = (get_buffer_size(devc) / devc->num_channels) * 16;

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

	err = control_out(usb->devhdl, CMD_SAMPLING_CONFIG, CMD_SAMPLING_CONFIG, NULL, 0);
	if (err) {
		sr_err("Enter to sampling rate configuration mode failed.");
		return err;
	}

	division = SAMPLING_BASE_FREQUENCY / samplerate;

	err = control_out(usb->devhdl,
										CMD_CONTROL,
										CMD_CONTROL_SAMPLE_RATE,
										(uint8_t *) &division,
										sizeof(division));
	if (err) {
		sr_err("Set sample rate failed.");
		return err;
	}

	data[0] = devc->cur_channels & 0xFF;
	data[1] = devc->cur_channels >> 8;
	data[2] = 0;
	data[3] = 0;
	err = control_out(usb->devhdl, CMD_CONTROL, CMD_CONTROL_CHAN_SELECT, data, 4);
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

	err = control_out(usb->devhdl, CMD_SAMPLING_START, CMD_SAMPLING_START, NULL, 0);
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

	usb = sdi->conn;
	/*
	 * There are need send request to stop sampling.
	 */
	ret = kingst_la1010_abort_acquisition_request(usb->devhdl);
	if (ret)
		sr_err("kingst_la1010_acquisition_stop(): Stop sampling error %d. libusb err: %s",
					 ret, libusb_error_name(ret));

	devc = sdi->priv;
	devc->acq_aborted = TRUE;

	for (i = devc->num_transfers - 1; i >= 0; i--) {
		if (devc->transfers[i])
			libusb_cancel_transfer(devc->transfers[i]);
	}

	return ret;
}

/*
 * Check if the fx firmware was uploaded to cypress.
 * If FX firmware was loaded the cypress returns 0xFB04 (may be firmware version?).
 */
int kingst_la1010_has_fx_firmware(struct libusb_device_handle *hdl) {
	union fx_status status;
	int err;

	status.code = -1;
	err = control_in(hdl,
	CMD_STATUS,
	CMD_STATUS_FX_STATUS, status.bytes, sizeof(status.bytes));
	if (err) {
		return err;
	}

	if (status.code != 0xFB04) {
		return status.code;
	}
	return SR_OK;
}

/*
 * Check if the bitstream was uploaded to spartan.
 * After upload bitstream the device returns 0x00000000.
 * But by next initialization (device wasn't is powered off) the device returns different response.
 */
int kingst_la1010_has_spartan_firmware(struct libusb_device_handle *hdl) {
	union spartan_status status;
	int err;

	status.code = -1;
	err = control_in(hdl,
										CMD_CONTROL,
										CMD_CONTROL_START,
										status.bytes,
										sizeof(status.bytes));
	if (err)
		return err;

	return status.code;
}

/*
 * Upload spartan bitstream.
 */
int kingst_la1010_upload_spartan_firmware(const struct sr_dev_inst *sdi) {
	struct drv_context *drvc;
	struct dev_context *devc;
	struct sr_usb_dev_inst *usb;
	struct libusb_device *usbdev;
	union spartan_status status;
	uint8_t *bindata;
	uint32_t binsize;
	int err, i;

	drvc = sdi->driver->context;
	devc = sdi->priv;

	bindata = sr_resource_load(drvc->sr_ctx,
														 SR_RESOURCE_FIRMWARE,
														 devc->profile->spartan_firmware,
														 (size_t *) &binsize,
														 0x020000);
	if (!bindata) {
		return SR_ERR_MALLOC;
	}

	usb = sdi->conn;
	for (i = 0; i < 3; i++) {
		err = kingst_la1010_has_fx_firmware(usb->devhdl);
		if (err)
			return err;
	}

	err = control_out(usb->devhdl,
										CMD_SPARTAN_UPLOAD,
										0,
										(uint8_t *) &binsize,
										sizeof(binsize));
	if (err)
		return err;

	usbdev = libusb_get_device(usb->devhdl);
	if (usbdev) {
		i = libusb_get_max_packet_size(usbdev, 0x01);

		sr_dbg("Upload Spartan firmware using packet size %d", i);

		err = upload_bindata_sync(usb->devhdl, bindata, binsize, i);
		if (err)
			return err;

		err = control_in(usb->devhdl,
										 CMD_CONTROL,
										 CMD_CONTROL_START,
										 status.bytes,
										 sizeof(status.bytes));
		if (err)
			return err;

		if (status.code) {
			sr_err("Check Spartan returns wrong status: %d", status.code);
			return SR_ERR;
		}
	} else {
		sr_err("Upload Spartan failed. Can't get usb device struct by device handle");
		return SR_ERR;
	}

	return err;
}

/*
 * Init the device for usage.
 */
int kingst_la1010_init_spartan(struct libusb_device_handle * handle) {
	int err;
	uint8_t dev_data[20] = { 0xA3, 0x09, 0xC9, 0x11, 0x52, 0x78, 0xAB, 0x7C, 0x06, 0x4E,
														0x76, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

	/*
	 * I don't understand next three (CMD_10, CMD_60) control request.
	 */
	err = control_out(handle, CMD_10, 0xFF, NULL, 0);
	if (err)
		return err;

	err = control_out(handle, CMD_60, 0, dev_data, 11);
	if (err)
		return err;

	err = control_in(handle, CMD_60, 0, dev_data, 20);
	if (err)
		return err;

	sr_dbg("CMD_60 responce (part1): 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x",
					dev_data[0], dev_data[1], dev_data[2], dev_data[3], dev_data[4],
					dev_data[5], dev_data[6], dev_data[7], dev_data[8], dev_data[9]);
	sr_dbg("CMD_60 responce (part1): 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x",
					dev_data[10], dev_data[11], dev_data[12], dev_data[13],
					dev_data[14], dev_data[15], dev_data[16], dev_data[17],
					dev_data[18], dev_data[19]);

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

/*
 * Configure threshold levels.
 * Available from -4.0 to 4.0.
 */
int kingst_la1010_set_logic_level(struct libusb_device_handle *hdl, double level) {
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
										(uint8_t *) &data,
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
	err = control_out(hdl, CMD_CONTROL, CMD_CONTROL_PWM, (uint8_t *) &data, 1);
	if (err)
		return err;

	data[0] = pwm1_freq;
	data[1] = pwm1_duty;
	err = control_out(hdl, CMD_CONTROL, CMD_CONTROL_PWM_1, (uint8_t *) &data, sizeof(data));
	if (err)
		return err;

	data[0] = pwm2_freq;
	data[1] = pwm2_duty;
	err = control_out(hdl, CMD_CONTROL, CMD_CONTROL_PWM_2, (uint8_t *) &data, sizeof(data));
	if (err)
		return err;

	data[0] = 0x01;
	err = control_out(hdl, CMD_CONTROL, CMD_CONTROL_PWM, (uint8_t *) &data, 1);
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
		sr_err("Failed to get device list: %s.", libusb_error_name(device_count));
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
					sr_err("Failed to detach kernel driver: %s.", libusb_error_name(ret));
					ret = SR_ERR;
					break;
				}
			}
		}

		sr_info("Opened device on %d.%d (logical) / %s (physical), " "interface %d",
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

int kingst_la1010_abort_acquisition_request(libusb_device_handle * handle) {
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
				sr_err("Conversion buffer too small! dstcnt %ld, srccnt %ld", destcnt, srccnt);
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
receive_transfer(struct libusb_transfer * transfer) {
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
			kingst_la1010_acquisition_stop(sdi);
			free_transfer(transfer);
		} else {
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
																		(uint8_t *) transfer->buffer,
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
				&& new_samples > devc->limit_samples - devc->sent_samples)
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
					&& num_samples > devc->limit_samples - devc->sent_samples)
				num_samples = devc->limit_samples - devc->sent_samples;
			logic.length = num_samples * 2;
			logic.data = devc->convbuffer + trigger_offset * 2;
			sr_session_send(sdi, &packet);
			devc->sent_samples += num_samples;

			devc->trigger_fired = TRUE;
		}
	}

	if (devc->limit_samples && devc->sent_samples >= devc->limit_samples) {
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
		ch = (struct sr_channel *) l->data;
		if (ch->enabled == FALSE)
			continue;

		channel_bit = 1 << (ch->index);
		devc->cur_channels |= channel_bit;
		devc->channel_masks[devc->num_channels++] = channel_bit;
	}

	return SR_OK;
}

static unsigned int to_bytes_per_ms(unsigned int samplerate, uint8_t num_channels) {
	unsigned long result = samplerate * num_channels * 2;
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
	timeout = total_size / to_bytes_per_ms(devc->cur_samplerate, devc->num_channels);
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
	sr_dbg("Number transfers was calculated: %d (0x%X)", num_transfers, num_transfers);

	size = get_buffer_size(devc);
	sr_dbg("Buffer size for each transfer was calculated: %ld (0x%lX)", size, size);

	devc->submitted_transfers = 0;

	devc->transfers = g_try_malloc0(sizeof(*devc->transfers) * num_transfers);
	if (!devc->transfers) {
		sr_err("USB transfers malloc failed.");
		return SR_ERR_MALLOC;
	}

	timeout = get_timeout(devc);
	sr_dbg("Timeout for each transfer was calculated: %d (0x%X)", timeout, timeout);

	for (i = 0; i < num_transfers; i++) {
		if (!(buf = g_try_malloc(size))) {
			sr_err("USB transfer buffer malloc failed.");
			return SR_ERR_MALLOC;
		}
		transfer = libusb_alloc_transfer(0);
		libusb_fill_bulk_transfer(transfer,
															usb->devhdl,
															USB_SAMPLING_DATA_EP,
															buf,
															size,
															receive_transfer,
															(void *) sdi,
															timeout);

		if ((ret = libusb_submit_transfer(transfer)) != 0) {
			libusb_free_transfer(transfer);
			g_free(buf);
			if (i == 0) {
				sr_err("Failed to submit transfer: %s.", libusb_error_name(ret));
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
	sr_info("%d transfers was submited: data size 0x%lx, timeout %d", devc->num_transfers, size, timeout);

	std_session_send_df_header(sdi);

	return SR_OK;
}

static int control_in(libusb_device_handle * handle,
											uint8_t request,
											uint16_t value,
											uint8_t * data,
											uint16_t size) {
	int actual_length = 0;
  uint8_t empty_data[1] = {0};

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
		sr_err("Wrong response size for 'control in' request: expected %d given %d.",
					 size, actual_length);
		return SR_ERR;
	}

	return SR_OK;
}

int control_out(libusb_device_handle * handle,
								uint8_t request,
								uint16_t value,
								uint8_t * data,
								uint16_t size) {
	int actual_length = 0;
	uint8_t empty_data[1] = {0};

	actual_length = libusb_control_transfer(handle,
																					LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR,
																					request,
																					value,
																					0,
																					data == NULL ? empty_data : data,
																					size,
																					USB_TIMEOUT);
	if (actual_length < 0) {
		sr_err("Failed to send 'control out' request to device: %s.",
					 libusb_error_name(actual_length));
		return actual_length;
	} else if (actual_length != size) {
		sr_err("Wrong response size for 'control out' request: expected %d given %d.",
					 size, actual_length);
		return SR_ERR;
	}

	return SR_OK;
}

int upload_bindata_sync(libusb_device_handle * handle, uint8_t * bindata, int size, int trans_size) {
	int data_len, err, actual_len;

	while (size > 0) {
		data_len = size;
		if (data_len > trans_size) {
			data_len = trans_size;
		}
		err = libusb_bulk_transfer(handle,
															 0x01,
															 bindata,
															 data_len,
															 &actual_len,
															 100
															);
		if (err) {
			sr_err("Failed to upload Spartan firmware: %s.",
						 libusb_error_name(err));
			return err;
		} else if (actual_len != data_len) {
			sr_err("Failed to upload Spartan firmware: sent %d but actual sent %d.",
						 data_len, actual_len);
			return SR_ERR_DATA;
		}
		size -= actual_len;
		bindata += actual_len;
	}
	err = libusb_bulk_transfer(handle,
														 0x01,
														 &data_len, // fake buffer for 0-sized transfer (passing NULL may be not working in some systems)
														 0,
														 &actual_len,
														 100
														);
	if (err) {
		sr_err("Failed to upload Spartan firmware: %s.",
					 libusb_error_name(err));
		return err;
	}
	return size;
}

struct dev_context * kingst_la1010_dev_new(void) {
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


