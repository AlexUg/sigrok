/*
 *
 *
 * Copyright (C) 2017 Alexandr Ugnenko <ugnenko@mail.com>
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
 *
 * This program based on sources of 'fx2lafw' by Bert Vermeulen <bert@biot.com>
 *                                            and Joel Holdsworth <joel@airwebreathe.org.uk>
 */

#include <config.h>
#include <glib.h>
#include <glib/gstdio.h>
#include "protocol.h"


#define USB_TIMEOUT 100

static int
control_in (libusb_device_handle * handle, uint8_t request, uint16_t value,
            uint8_t *data, uint16_t size);
static int
control_out (libusb_device_handle * handle, uint8_t request, uint16_t value,
             uint8_t *data, uint16_t size);
static int
upload_bindata (libusb_device_handle * handle, uint8_t *data, int size);

static void LIBUSB_CALL
abort_acquisition_request_cb(struct libusb_transfer *transfer);



static int
command_start_acquisition (const struct sr_dev_inst *sdi)
{
  struct dev_context *devc;
  struct sr_usb_dev_inst *usb;
  uint64_t samplerate;
  uint16_t division;
  int err;
  uint8_t data[1];

  devc = sdi->priv;
  usb = sdi->conn;
  samplerate = devc->cur_samplerate;
  usb = sdi->conn;

  data[0] = 0;
  err = control_out(usb->devhdl, CMD_CONTROL, CMD_CONTROL_START, data, 1);
  if (err)
    {
      sr_err("Start configure channels failed.");
      return err;
    }

  err = control_out(usb->devhdl, CMD_SAMPLING_CONFIG, CMD_SAMPLING_CONFIG, NULL, 0);
  if (err)
    {
      sr_err("Enter to sampling rate configuration mode failed.");
      return err;
    }

  division = SAMPLING_BASE_FREQUENCY / samplerate;

  sr_dbg("Set sample rate %ld, division %d", samplerate, division);

  err = control_out(usb->devhdl, CMD_CONTROL, CMD_CONTROL_SAMPLE_RATE, (uint8_t *) &division, sizeof(division));
  if (err)
    {
      sr_err("Set sample rate failed.");
      return err;
    }

  sr_dbg("Set channel mask 0x%x", devc->channel_mask);

  err = control_out(usb->devhdl, CMD_CONTROL, CMD_CONTROL_CHAN_SELECT, (uint8_t *) &devc->channel_mask, sizeof(devc->channel_mask));
  if (err)
    {
      sr_err("Set channel mask failed.");
      return err;
    }

  data[0] = 1;
  err = control_out(usb->devhdl, CMD_CONTROL, CMD_CONTROL_START, data, 1);
  if (err)
    {
      sr_err("Commit the channels and sample rate configuration failed.");
      return err;
    }

  err = control_out(usb->devhdl, CMD_SAMPLING_START, CMD_SAMPLING_START, NULL, 0);
  if (err)
    {
      sr_err("Star sampling failed.");
      return err;
    }

  return SR_OK;
}


/*
 * Check if the fx firmware was uploaded to cypress.
 * If FX firmware was loaded the cypress returns 0xFB04 (may be firmware version?).
 */
int
kingst_la1010_has_fx_firmware (struct libusb_device_handle *hdl)
{
  union fx_status status;
  int err;

  status.code = -1;
  err = control_in (hdl,
                    CMD_STATUS,
                    CMD_STATUS_FX_STATUS,
                    status.bytes,
                    sizeof(status.bytes));
  if (err)
    {
      return err;
    }

  if (status.code != 0xFB04)
    {
      return status.code;
    }
  return SR_OK;
}

/*
 * Check if the bitstream was uploaded to spartan.
 * After upload bitstream the device returns 0x00000000.
 * But by next initialization (device wasn't is powered off) the device returns different response.
 */
int
kingst_la1010_has_spartan_firmware (struct libusb_device_handle *hdl)
{
  union spartan_status status;
  int err;

  status.code = -1;
  err = control_in (hdl,
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
int
kingst_la1010_upload_spartan_firmware (struct sr_dev_inst *sdi)
{
  struct drv_context *drvc;
  struct dev_context *devc;
  struct sr_usb_dev_inst *usb;
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
  if (!bindata)
    {
      return SR_ERR_MALLOC;
    }

  usb = sdi->conn;
  for (i = 0; i < 3; i++)
    {
      err = kingst_la1010_has_fx_firmware(usb->devhdl);
      if (err)
        return err;
    }

  err = control_out(usb->devhdl, CMD_SPARTAN_UPLOAD, 0, (uint8_t *) &binsize, sizeof(binsize));
  if (err)
    return err;

  err = upload_bindata(usb->devhdl, bindata, binsize);
  if (err)
    return err;

  err = control_in(usb->devhdl, CMD_CONTROL, CMD_CONTROL_START, status.bytes, sizeof(status.bytes));
  if (err)
    return err;

  if (status.code)
    {
      sr_err("Check Spartan returns wrong status: %d", status.code);
      return SR_ERR;
    }

  return err;
}

/*
 * Init the device for usage.
 */
int
kingst_la1010_init_spartan (struct libusb_device_handle * handle)
{
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
            dev_data[10], dev_data[11], dev_data[12], dev_data[13], dev_data[14],
            dev_data[15], dev_data[16], dev_data[17], dev_data[18], dev_data[19]);

  /*
   * Configure voltage threshold.
   */
  err = kingst_la1010_set_logic_level(handle, 1.58);
  if (err)
    return err;

  /*
   * Configure PWM channels -- two channels.
   */
  err = kingst_la1010_configure_pwm(handle, 0, 1, 0, 1);
  if (err)
    return err;

  return SR_OK;
}

/*
 * Configure threshold levels.
 * Available from -4.0 to 4.0.
 */
int
kingst_la1010_set_logic_level(struct libusb_device_handle *hdl, double level)
{
  uint32_t data;
  int err;

  sr_dbg("kingst_la1010_set_logic_level(): level %f", level);

  if (level > -0.4)
    {
      if (level < 3)
        {
          /*
           * For voltage levels between -0.399 and 2.999 data is 0x00F1YYYY
           * where YYYY -- is .....
           */
          data = ((level + 0.4) * 302);
          data += 0x00F10000;
        }
      else
        {
          /*
           * For voltage levels between 2.999 and 4 data is 0x0000YYYY
           * where YYYY -- is .....
           */
          if (level > 4)
            {
              level = 4;
            }
          data = ((level - 1.2) * 302);
        }
    }
  else
    {
      /*
       * For voltage levels between -4 and -0.4 data is 0x02D4YYYY
       * where YYYY -- is .....
       */
      if (level < -3.6)
        {
          level = -3.6;
        }
      data = ((level + 3.6) * 302);
      data += 0x02D40000;
    }

  err = control_out(hdl, CMD_CONTROL, CMD_CONTROL_LOG_LEVEL, (uint8_t *) &data, sizeof(data));
  if (err)
    return err;

  sr_dbg("kingst_la1010_set_logic_level(): done");

  return SR_OK;
}

/*
 * Configure PWM channels -- two channels.
 * For each channel: frequency between 1 kHz (1000) and 200 MHz (200000000)$
 *                    duty between 1 and 99.
 * Frequency == 0 -- power off PWM channel.
 */
int
kingst_la1010_configure_pwm(struct libusb_device_handle *hdl, int pwm1_freq, int pwm1_duty, int pwm2_freq, int pwm2_duty)
{
  uint32_t data[2];
  int err;

  sr_dbg("kingst_la1010_configure_pwm(): pwm1_freq %d, pwm1_duty %d, pwm2_freq %d, pwm2_duty %d",
         pwm1_freq, pwm1_duty, pwm2_freq, pwm2_duty);

  if ((pwm1_duty < 0)
      || (pwm1_duty > 100))
    {
      sr_err("Wrong PWM1 duty ratio, given %d, but only 0 .. 100 allowed", pwm1_duty);
      pwm1_duty = 50;
    }
  if ((pwm2_duty < 0)
      || (pwm2_duty > 100))
    {
      sr_err("Wrong PWM2 duty ratio, given %d, but only 0 .. 100 allowed", pwm2_duty);
      pwm2_duty = 50;
    }

  if (pwm1_freq)
    {
      /*
       * PWM data is division 800000000 to frequency
       */
      pwm1_freq = PWM_BASE_FREQUENCY / pwm1_freq;
      /*
       * Duty data is between '0' and 'PWM data'
       */
      pwm1_duty = pwm1_freq * pwm1_duty / 100;
    }

  if (pwm2_freq)
    {
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

  sr_dbg("kingst_la1010_configure_pwm(): done");

  return SR_OK;
}

SR_PRIV int
kingst_la1010_dev_open (struct sr_dev_inst *sdi, struct sr_dev_driver *di)
{
  libusb_device **devlist;
  struct sr_usb_dev_inst *usb;
  struct libusb_device_descriptor des;
  struct dev_context *devc;
  struct drv_context *drvc;
  int ret = SR_ERR, i, device_count;
  char connection_id[64];

  drvc = di->context;
  devc = sdi->priv;
  usb = sdi->conn;

  device_count = libusb_get_device_list (drvc->sr_ctx->libusb_ctx, &devlist);
  if (device_count < 0)
    {
      sr_err("Failed to get device list: %s.",
             libusb_error_name (device_count));
      return SR_ERR;
    }

  for (i = 0; i < device_count; i++)
    {
      libusb_get_device_descriptor (devlist[i], &des);

      if (des.idVendor != devc->profile->vid
          || des.idProduct != devc->profile->pid)
        continue;

      if ((sdi->status == SR_ST_INITIALIZING)
          || (sdi->status == SR_ST_INACTIVE))
        {
          /*
           * Check device by its physical USB bus/port address.
           */
          usb_get_port_path (devlist[i], connection_id, sizeof(connection_id));
          if (strcmp (sdi->connection_id, connection_id))
            /* This is not the one. */
            continue;
        }

      if (!(ret = libusb_open (devlist[i], &usb->devhdl)))
        {
          if (usb->address == 0xff)
            /*
             * First time we touch this device after FW
             * upload, so we don't know the address yet.
             */
            usb->address = libusb_get_device_address (devlist[i]);
        }
      else
        {
          sr_err("Failed to open device: %s.", libusb_error_name (ret));
          ret = SR_ERR;
          break;
        }

      if (libusb_has_capability (LIBUSB_CAP_SUPPORTS_DETACH_KERNEL_DRIVER))
        {
          if (libusb_kernel_driver_active (usb->devhdl, USB_INTERFACE) == 1)
            {
              if ((ret = libusb_detach_kernel_driver (usb->devhdl,
                                                      USB_INTERFACE)) < 0)
                {
                  sr_err("Failed to detach kernel driver: %s.",
                         libusb_error_name (ret));
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

  libusb_free_device_list (devlist, 1);

  return ret;
}

SR_PRIV struct dev_context *
kingst_la1010_dev_new (void)
{
  struct dev_context *devc;

  devc = g_malloc0 (sizeof(struct dev_context));
  devc->profile = NULL;
  devc->fw_updated = 0;
  devc->cur_samplerate = 0;
  devc->limit_samples = 0;
  devc->capture_ratio = 0;
  devc->channel_count = 0;
  devc->convbuffer = NULL;
  devc->stl = NULL;

  return devc;
}

static int
abort_acquisition_request(libusb_device_handle * handle)
{
  struct libusb_transfer *transfer;
  unsigned char *buffer;
  int ret;

  transfer = libusb_alloc_transfer (0);
  if (!transfer)
    return LIBUSB_ERROR_NO_MEM;

  buffer = (unsigned char*) malloc (LIBUSB_CONTROL_SETUP_SIZE + 1);
  if (!buffer)
    {
      libusb_free_transfer (transfer);
      return LIBUSB_ERROR_NO_MEM;
    }

  libusb_fill_control_setup (buffer,
                             LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR,
                             CMD_CONTROL,
                             CMD_CONTROL_START,
                             0,
                             1);
  buffer[LIBUSB_CONTROL_SETUP_SIZE] = 0;

  libusb_fill_control_transfer (transfer,
                                handle,
                                buffer,
                                abort_acquisition_request_cb,
                                NULL,
                                1000);
  transfer->flags = LIBUSB_TRANSFER_FREE_BUFFER;

  sr_dbg("abort_acquisition_request(): Submit stop sampling request ...");

  ret = libusb_submit_transfer (transfer);
  if (ret < 0)
    {
      libusb_free_transfer (transfer);
      return ret;
    }

  return ret;
}

static void LIBUSB_CALL
abort_acquisition_request_cb(struct libusb_transfer *transfer)
{
  sr_dbg("abort_acquisition_request(): Stop sampling request done");
  libusb_free_transfer (transfer);
}

SR_PRIV void
kingst_la1010_abort_acquisition (const struct sr_dev_inst *sdi)
{
  int i, ret;
  struct sr_usb_dev_inst *usb;
  struct dev_context *devc;

  usb = sdi->conn;
  /*
   * There are need send request to stop sampling.
   */
  ret = abort_acquisition_request(usb->devhdl);
  if (ret)
    sr_err("kingst_la1010_abort_acquisition(): Stop sampling error %d. libusb err: %s", ret, libusb_error_name (ret));

  devc = sdi->priv;
  devc->acq_aborted = TRUE;

  for (i = devc->num_transfers - 1; i >= 0; i--)
    {
      if (devc->transfers[i])
        libusb_cancel_transfer (devc->transfers[i]);
    }
}

static void
finish_acquisition (struct sr_dev_inst *sdi)
{
  struct dev_context *devc;

  devc = sdi->priv;

  std_session_send_df_end (sdi);

  usb_source_remove (sdi->session, devc->ctx);

  devc->num_transfers = 0;
  g_free (devc->transfers);

  if (devc->convbuffer)
    {
      g_free (devc->convbuffer);
      devc->convbuffer = NULL;
    }

  if (devc->stl)
    {
      soft_trigger_logic_free (devc->stl);
      devc->stl = NULL;
    }

  sr_dbg("finish_acquisition() done");
}

static void
free_transfer (struct libusb_transfer *transfer)
{
  struct sr_dev_inst *sdi;
  struct dev_context *devc;
  unsigned int i;

  sdi = transfer->user_data;
  devc = sdi->priv;

  libusb_free_transfer (transfer);
  if (transfer->buffer)
    {
      g_free (transfer->buffer);
      transfer->buffer = NULL;
    }

  for (i = 0; i < devc->num_transfers; i++)
    {
      if (devc->transfers[i] == transfer)
        {
          devc->transfers[i] = NULL;
          break;
        }
    }

  devc->submitted_transfers--;

  if (devc->submitted_transfers == 0)
    finish_acquisition (sdi);
}

static void
resubmit_transfer (struct libusb_transfer *transfer)
{
  int ret;

  if ((ret = libusb_submit_transfer (transfer)) == LIBUSB_SUCCESS)
    return;

  sr_err("%s: %s", __func__, libusb_error_name (ret));
  free_transfer (transfer);

}

/*
 * Sampling data same as Saleae Logic16.
 * But I found out it after this function was implemented.
 */
static int
convert_data (uint16_t ch_mask,
               uint8_t ch_cnt,
               uint16_t in_width,
               uint8_t *in_data,
               uint64_t in_data_size,
               uint16_t *out_width,
               uint8_t *out_data,
               uint64_t *out_data_size)
{
  uint64_t in_idx = 0, iw, b, out_idx = 0, ch_idx;
  uint64_t pack_size, k, i;
  uint8_t sample;
  uint8_t ch_numbers[ch_cnt];

  if (ch_cnt)
    {
      memset (out_data, 0, *out_data_size);
      k = 0;
      for (ch_idx = 0; ch_idx < 16; ch_idx++)
        {
          if ((1 << ch_idx) & ch_mask)
            {
              ch_numbers[k++] = ch_idx;
            }
        }
      ch_idx = 0;
      *out_width = ch_mask & 0xFF00 ? 2 : 1;
      pack_size = in_width * 8 * (*out_width);
      for (; in_idx < in_data_size;)
        {
          for (iw = 0; iw < in_width; iw++, in_idx++)
            {
              sample = in_data[in_idx];
              for (b = 0; b < 8; b++)
                {
                  k = ch_numbers[ch_idx] < 8 ? 0 : 1;
                  i = out_idx + (iw * 8 * (*out_width)) + (b * (*out_width)) + k;
                  if (i >= *out_data_size)
                    {
                      sr_err("Wrong output buffer length, insert by index = %ld, but length %ld", i, *out_data_size);
                      *out_width = 0;
                      *out_data_size = 0;
                      return -1;
                    }
                  out_data[i] |= (sample & 1) << (ch_numbers[ch_idx] % 8);
                  sample >>= 1;
                }
            }
          ch_idx = (ch_idx + 1) % ch_cnt;
          if (ch_idx == 0)
            {
              out_idx += pack_size;
              if (out_idx > (*out_data_size - pack_size))
                {
                  break;
                }
            }
        }
    }
  else
    {
      *out_width = 0;
    }

  *out_data_size = out_idx;
  return in_idx;
}

static void LIBUSB_CALL
receive_transfer (struct libusb_transfer * transfer)
{
  struct sr_dev_inst *sdi;
  struct dev_context *devc;
  gboolean packet_has_error = FALSE;
  size_t num_samples_length, samples_length;
  int trigger_offset, pre_trigger_samples, ret;
  struct sr_datafeed_logic logic;
  struct sr_datafeed_packet packet;

  sdi = transfer->user_data;
  devc = sdi->priv;

  /*
   * If acquisition has already ended, just free any queued up
   * transfer that come in.
   */
  if (devc->acq_aborted)
    {

      free_transfer (transfer);
      return;
    }

  sr_dbg("receive_transfer(): status %s received %d bytes.",
         libusb_error_name (transfer->status), transfer->actual_length);

  switch (transfer->status)
    {
    case LIBUSB_TRANSFER_NO_DEVICE:
      kingst_la1010_abort_acquisition (sdi);
      free_transfer (transfer);
      return;
    case LIBUSB_TRANSFER_COMPLETED:
    case LIBUSB_TRANSFER_TIMED_OUT: /* We may have received some data though. */
      break;
    default:
      packet_has_error = TRUE;
      break;
    }

  if (transfer->actual_length == 0 || packet_has_error)
    {
      devc->empty_transfer_count++;
      if (devc->empty_transfer_count > MAX_EMPTY_TRANSFERS)
        {
          kingst_la1010_abort_acquisition (sdi);
          free_transfer (transfer);
        }
      else
        {
          resubmit_transfer (transfer);
        }
      return;
    }
  else
    {
      devc->empty_transfer_count = 0;
    }

  logic.data = devc->convbuffer;
  logic.length = devc->convbuffer_size;

  packet.type = SR_DF_LOGIC;
  packet.payload = &logic;

  ret = convert_data(devc->channel_mask,
                       devc->channel_count,
                       2,
                       (uint8_t *) transfer->buffer,
                       transfer->actual_length,
                       &(logic.unitsize),
                       devc->convbuffer,
                       &(logic.length)
                       );

  if (ret < 0)
    {
      sr_err("receive_transfer(): convert data failed");
      kingst_la1010_abort_acquisition (sdi);
      free_transfer (transfer);
    }

  samples_length = (devc->limit_samples - devc->sent_samples) * logic.unitsize;

  if (devc->trigger_fired)
    {
      if (devc->limit_samples
          && logic.length > samples_length)
        {
          logic.length = samples_length;
        }
      logic.data = devc->convbuffer;
      sr_session_send (sdi, &packet);
      devc->sent_samples += logic.length / logic.unitsize;
    }
  else
    {
      devc->stl->unitsize = logic.unitsize;
      trigger_offset = soft_trigger_logic_check (devc->stl,
                                                 devc->convbuffer,
                                                 logic.length,
                                                 &pre_trigger_samples);

      sr_dbg("Trigger offset %d", trigger_offset);

      if (trigger_offset > -1)
        {
          devc->sent_samples += pre_trigger_samples;

          sr_dbg("receive_transfer(): sent samples %d, pre trig samples %d", devc->sent_samples, pre_trigger_samples);

          trigger_offset *= logic.unitsize;

          packet.type = SR_DF_LOGIC;
          packet.payload = &logic;
          num_samples_length = logic.length - trigger_offset;
          if (devc->limit_samples
              && num_samples_length > samples_length)
            {
              num_samples_length = samples_length;
            }
          logic.length = num_samples_length;
          logic.data = devc->convbuffer + trigger_offset;
          sr_session_send (sdi, &packet);
          devc->sent_samples += num_samples_length / logic.unitsize;

          devc->trigger_fired = TRUE;
        }
    }

  if (devc->limit_samples
      && devc->sent_samples >= devc->limit_samples)
    {
      sr_dbg("receive_transfer(): stop after sent samples %d", devc->sent_samples);
      kingst_la1010_abort_acquisition (sdi);
      free_transfer (transfer);
    }
  else
    resubmit_transfer (transfer);
}

static int
configure_channels (const struct sr_dev_inst *sdi)
{
  struct dev_context *devc;
  const GSList *l;
  int p;
  struct sr_channel *ch;

  devc = sdi->priv;
  devc->channel_count = 0;
  devc->channel_mask = 0;

  for (l = sdi->channels, p = 0; l; l = l->next, p++)
    {
      ch = l->data;
      if ((p <= NUM_CHANNELS)
            && ch->enabled)
        {
          devc->channel_mask |= 1 << p;
          devc->channel_count++;
        }
    }

  return SR_OK;
}

static unsigned int
to_bytes_per_ms (unsigned int samplerate, uint8_t channel_count)
{
  unsigned long result = samplerate * channel_count * 2;
  return result / 1000;
}

static size_t
get_buffer_size (struct dev_context *devc)
{
  size_t s;

  /*
   * This transfer size used by KingstVIS
   */
  s = devc->channel_count * devc->cur_samplerate / 128;
  return (s + 511) & ~511;
}

static unsigned int
get_number_of_transfers (struct dev_context *devc)
{
  unsigned int n;

  /* Total buffer size should be able to hold about 500ms of data. */
  n = (500 * to_bytes_per_ms (devc->cur_samplerate, devc->channel_count) / get_buffer_size (devc));

  if (n > NUM_SIMUL_TRANSFERS)
    return NUM_SIMUL_TRANSFERS;

  return n;
}

static unsigned int
get_timeout (struct dev_context *devc)
{
  size_t total_size;
  unsigned int timeout;

  total_size = get_buffer_size (devc) * get_number_of_transfers (devc);
  timeout = total_size / to_bytes_per_ms (devc->cur_samplerate, devc->channel_count);
  return timeout + timeout / 4; /* Leave a headroom of 25% percent. */
}

static int
receive_data (int fd, int revents, void *cb_data)
{
  struct timeval tv;
  struct drv_context *drvc;

  (void) fd;
  (void) revents;

  drvc = (struct drv_context *) cb_data;

  tv.tv_sec = tv.tv_usec = 0;
  libusb_handle_events_timeout (drvc->sr_ctx->libusb_ctx, &tv);

  return TRUE;
}

static int
start_transfers (const struct sr_dev_inst *sdi)
{
  struct dev_context *devc;
  struct sr_usb_dev_inst *usb;
  struct sr_trigger *trigger;
  struct libusb_transfer *transfer;
  unsigned int i, num_transfers;
  int timeout, ret;
  int pre_trigger_samples;
  unsigned char *buf;
  size_t size;

  devc = sdi->priv;
  usb = sdi->conn;

  devc->sent_samples = 0;
  devc->acq_aborted = FALSE;
  devc->empty_transfer_count = 0;

  if ((trigger = sr_session_trigger_get (sdi->session)))
    {
      pre_trigger_samples = 0;
      if (devc->limit_samples > 0)
        pre_trigger_samples = (devc->capture_ratio * devc->limit_samples) / 100;
      devc->stl = soft_trigger_logic_new (sdi, trigger, pre_trigger_samples);
      if (!devc->stl)
        return SR_ERR_MALLOC;
      devc->trigger_fired = FALSE;

      sr_dbg("Enable trigger");
    }
  else
    devc->trigger_fired = TRUE;

  num_transfers = get_number_of_transfers (devc);
  num_transfers = 3;

  sr_dbg("start_transfers(): number of transfers %d", num_transfers);

  size = get_buffer_size (devc);
  devc->submitted_transfers = 0;

  devc->transfers = g_try_malloc0 (sizeof(*devc->transfers) * num_transfers);
  if (!devc->transfers)
    {
      sr_err("USB transfers malloc failed.");
      return SR_ERR_MALLOC;
    }

  timeout = get_timeout (devc) * 10;
  devc->num_transfers = num_transfers;
  for (i = 0; i < num_transfers; i++)
    {
      if (!(buf = g_try_malloc (size)))
        {
          sr_err("USB transfer buffer malloc failed.");
          return SR_ERR_MALLOC;
        }
      transfer = libusb_alloc_transfer (0);
      libusb_fill_bulk_transfer (transfer,
                                 usb->devhdl,
                                 USB_SAMPLING_DATA_EP,
                                 buf,
                                 size,
                                 receive_transfer,
                                 (void *) sdi,
                                 timeout);

      sr_info("submitting transfer: %d, data size 0x%lx, timeout %d", i, size, timeout);

      if ((ret = libusb_submit_transfer (transfer)) != 0)
        {
          sr_err("Failed to submit transfer: %s.", libusb_error_name (ret));
          libusb_free_transfer (transfer);
          g_free (buf);
          kingst_la1010_abort_acquisition (sdi);
          return SR_ERR;
        }
      devc->transfers[i] = transfer;
      devc->submitted_transfers++;
    }

  std_session_send_df_header (sdi);

  return SR_OK;
}

SR_PRIV int
kingst_la1010_start_acquisition (const struct sr_dev_inst *sdi)
{
  struct sr_dev_driver *di;
  struct drv_context *drvc;
  struct dev_context *devc;
  int timeout, ret;

  di = sdi->driver;
  drvc = di->context;
  devc = sdi->priv;

  devc->ctx = drvc->sr_ctx;
  devc->sent_samples = 0;
  devc->empty_transfer_count = 0;
  devc->acq_aborted = FALSE;

  if (configure_channels (sdi) != SR_OK)
    {
      sr_err("Failed to configure channels.");
      return SR_ERR;
    }

  timeout = get_timeout (devc);
  usb_source_add (sdi->session, devc->ctx, timeout, receive_data, drvc);

  devc->convbuffer_size = get_buffer_size (devc);

  devc->convbuffer = g_try_malloc (devc->convbuffer_size);

  if (devc->convbuffer)
    {
      sr_dbg("Allocated logic data buffer size %ld", devc->convbuffer_size);

      if ((ret = command_start_acquisition (sdi)) != SR_OK)
        {
          kingst_la1010_abort_acquisition (sdi);
          if (devc->convbuffer)
              g_free (devc->convbuffer);
          devc->convbuffer = NULL;
          return ret;
        }

      start_transfers (sdi);
    }
  else
    {
      sr_err("Failed to allocate memory for data buffer.");
      return SR_ERR_MALLOC;
    }

  return SR_OK;
}

int
control_in (libusb_device_handle * handle, uint8_t request, uint16_t value,
            uint8_t *data, uint16_t size)
{
  int actual_length = 0;

  actual_length = libusb_control_transfer (handle,
                                           LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR,
                                           request,
                                           value,
                                           0,
                                           data,
                                           size,
                                           1000);
  if (actual_length < 0)
    {
      sr_err("Failed to send 'control in' request to device: %s.", libusb_error_name (actual_length));
      return actual_length;
    }
  else if (actual_length != size)
    {
      sr_err("Wrong response size for 'control in' request: expected %d given %d.", size, actual_length);
      return SR_ERR;
    }

  return SR_OK;
}

int
control_out (libusb_device_handle * handle, uint8_t request, uint16_t value,
             uint8_t *data, uint16_t size)
{
  int actual_length = 0;

  actual_length = libusb_control_transfer (handle,
                                           LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR,
                                           request,
                                           value,
                                           0,
                                           data,
                                           size,
                                           1000);
  if (actual_length < 0)
    {
      sr_err("Failed to send 'control out' request to device: %s.", libusb_error_name (actual_length));
      return actual_length;
    }
  else if (actual_length != size)
    {
      sr_err("Wrong response size for 'control out' request: expected %d given %d.", size, actual_length);
      return SR_ERR;
    }

  return SR_OK;
}

void LIBUSB_CALL cbUploadTransferComplete (struct libusb_transfer * xfr);

void LIBUSB_CALL
cbUploadTransferComplete (struct libusb_transfer * xfr)
{
  int * completed;

  completed = xfr->user_data;
  switch (xfr->status)
    {
    case LIBUSB_TRANSFER_COMPLETED:
      *completed = SR_OK;
      break;
    case LIBUSB_TRANSFER_CANCELLED:
    case LIBUSB_TRANSFER_NO_DEVICE:
    case LIBUSB_TRANSFER_TIMED_OUT:
    case LIBUSB_TRANSFER_ERROR:
    case LIBUSB_TRANSFER_STALL:
    case LIBUSB_TRANSFER_OVERFLOW:
      *completed = xfr->status;
      break;
    }
}

int
upload_bindata (libusb_device_handle * handle, uint8_t * bindata, int size)
{
  struct libusb_transfer * xfr;
  int completed, err;

  completed = 1;
  xfr = libusb_alloc_transfer (0);
  if (xfr)
    {
      libusb_fill_bulk_transfer (xfr,
                                 handle,
                                 0x01, // Endpoint ID
                                 bindata,
                                 size,
                                 cbUploadTransferComplete,
                                 &completed,
                                 60000);
      xfr->flags = LIBUSB_TRANSFER_ADD_ZERO_PACKET;

      err = libusb_submit_transfer (xfr);
      if (err < 0)
        {
          // Error
          libusb_free_transfer (xfr);
          sr_err("Failed to submit transfer for upload Spartan firmware: %s.", libusb_error_name (err));
          return err;
        }

      while (completed)
        {
          if (libusb_handle_events (NULL) != LIBUSB_SUCCESS)
            break;
        }
      libusb_free_transfer (xfr);

      if (completed < 0)
        {
          sr_err("Failed to upload Spartan firmware: %s.", libusb_error_name (completed));
        }

      return completed;
    }
  else
    {
      sr_err("Failed to upload Spartan firmware: out of memory");
      return SR_ERR_MALLOC;
    }

}

