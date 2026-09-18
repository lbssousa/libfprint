// Goodix Tls driver for libfprint
//
// Goodix 538d (GF5298) driver: FpDevice with SIGFM matching.
//
// The capture/transport backend (real TLS-PSK handshake, FDT calibration and
// image read) is kept from the original goodixtls53xd image driver
// (infinytum/libfprint, based on goodix-fp-linux-dev). The enroll / verify /
// identify state machines and the SIGFM-based matching shell are adapted from
// AndyHazz/goodix53x5-libfprint, which is incompatible at the wire level (the
// 53x5 speaks a custom GTLS, the 538d speaks real TLS-PSK) but provides the
// FpDevice + SIGFM architecture this driver reuses.

// Copyright (C) 2021 Alexander Meiler <alex.meiler@protonmail.com>
// Copyright (C) 2021 Matthieu CHARETTE <matthieu.charette@gmail.com>
// Copyright (C) 2021 Michael Teuscher <michael.teuscher@pm.me>
// Copyright (C) 2022 Natasha England-Elbro <ashenglandelbro@protonmail.com>

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA

#include "fp-device.h"
#include "fpi-device.h"
#include "fpi-ssm.h"
#include <stdio.h>
#include <stdlib.h>
#define FP_COMPONENT "goodixtls53xd"

#include <glib.h>
#include <string.h>

#include "drivers_api.h"
#include "goodix.h"
#include "goodix_proto.h"
#include "goodix53xd.h"
#include "sigfm/sigfm.hpp"

#define GOODIX53XD_WIDTH 64
#define GOODIX53XD_HEIGHT 80
#define GOODIX53XD_SCAN_WIDTH 64
#define GOODIX53XD_FRAME_SIZE (GOODIX53XD_WIDTH * GOODIX53XD_HEIGHT)
// For every 4 pixels there are 6 bytes
#define GOODIX53XD_RAW_FRAME_SIZE                                               \
    (GOODIX53XD_HEIGHT * GOODIX53XD_SCAN_WIDTH) / 4 * 6

#define GOODIX53XD_SENSOR_PIXELS GOODIX53XD_FRAME_SIZE

// Matching / enrollment tuning. These start from the AndyHazz 53x5 values but
// the 538d sensor is smaller (64x80 vs 108x88), so fewer SIFT keypoints are
// expected; they likely need tuning against real 538d captures.
#define GOODIX53XD_ENROLL_SAMPLES 8
#define GOODIX53XD_MIN_CAPTURE_KEYPOINTS 8
// Minimum SIGFM match score to accept a verify/identify. Calibrated from 538d
// hardware: impostor fingers scored 0-4, genuine 10-6400+, so 8 sits in the
// gap with margin on both sides. Tunable.
#define GOODIX53XD_SIGFM_BEST_MIN 8

// Finger presence is detected from the number of SIFT keypoints SIGFM finds:
// an empty platen yields ~0, a finger yields many. (The raw dynamic range is
// useless here: fixed saturated/dead pixels keep it pinned high even with no
// finger.) Provisional; tune from the "capture keypoints" debug values.
#define GOODIX53XD_FINGER_MIN_KEYPOINTS 5
// Delay between finger-detection polls (ms).
#define GOODIX53XD_FINGER_POLL_MS 80

typedef unsigned short Goodix53xdPix;

struct _FpiDeviceGoodixTls53XD {
  FpiDeviceGoodixTls parent;

  guint8* otp;

  // Latest decoded capture as an 8-bit grayscale GOODIX53XD_SENSOR_PIXELS buffer
  guint8* captured_image;
  // Raw (12-bit) dynamic range of the latest capture (informational)
  guint last_raw_range;
  // SIGFM keypoint count of the latest capture, used for finger detection
  int last_keypoints;
  // Rotating index for GOODIX53XD_DUMP debug PGM files
  guint dump_counter;

  // Enrollment: array of captured 8-bit images (g_free'd)
  GPtrArray* enroll_images;
  guint enroll_stage;

  FpiSsm* task_ssm;

  // TRUE while a poll_again() delayed state change is armed on task_ssm
  // (i.e. between fpi_ssm_jump_to_state_delayed() and the timeout firing).
  // dev_cancel() needs this to know whether it must disarm the timeout
  // before calling fpi_ssm_mark_failed(), which asserts none is pending.
  gboolean poll_timeout_pending;

  // Deferred verify/identify result reporting
  gboolean verify_wait_finger_up;
  gboolean pending_result_report;
  gboolean action_result_reported;
  FpiDeviceAction pending_result_action;
  FpiMatchResult pending_verify_result;
  FpPrint* pending_identify_match;
  GError* pending_result_error;
};

G_DECLARE_FINAL_TYPE(FpiDeviceGoodixTls53XD, fpi_device_goodixtls53xd, FPI,
                     DEVICE_GOODIXTLS53XD, FpiDeviceGoodixTls);

G_DEFINE_TYPE(FpiDeviceGoodixTls53XD, fpi_device_goodixtls53xd,
              FPI_TYPE_DEVICE_GOODIXTLS);

// ---------------------------------------------------------------------------
// Generic SSM callbacks shared by activation and capture
// ---------------------------------------------------------------------------

static void check_none(FpDevice* dev, gpointer user_data, GError* error)
{
    if (error) {
        fpi_ssm_mark_failed(user_data, error);
        return;
    }
    fpi_ssm_next_state(user_data);
}

static void check_none_cmd(FpDevice* dev, guint8* data, guint16 len,
                           gpointer ssm, GError* err)
{
    if (err) {
        fpi_ssm_mark_failed(ssm, err);
        return;
    }
    fpi_ssm_next_state(ssm);
}

static void check_firmware_version(FpDevice* dev, gchar* firmware,
                                   gpointer user_data, GError* error)
{
    if (error) {
        fpi_ssm_mark_failed(user_data, error);
        return;
    }

    fp_dbg("Device firmware: \"%s\"", firmware);

    if (strcmp(firmware, GOODIX_53XD_FIRMWARE_VERSION)) {
        g_set_error(&error, G_IO_ERROR, G_IO_ERROR_INVALID_DATA,
                    "Invalid device firmware: \"%s\"", firmware);
        fpi_ssm_mark_failed(user_data, error);
        return;
    }

    fpi_ssm_next_state(user_data);
}

static void check_reset(FpDevice* dev, gboolean success, guint16 number,
                        gpointer user_data, GError* error)
{
    if (error) {
        fpi_ssm_mark_failed(user_data, error);
        return;
    }

    if (!success) {
        g_set_error(&error, G_IO_ERROR, G_IO_ERROR_FAILED,
                    "Failed to reset device");
        fpi_ssm_mark_failed(user_data, error);
        return;
    }

    fp_dbg("Device reset number: %d", number);

    if (number != GOODIX_53XD_RESET_NUMBER) {
        g_set_error(&error, G_IO_ERROR, G_IO_ERROR_INVALID_DATA,
                    "Invalid device reset number: %d", number);
        fpi_ssm_mark_failed(user_data, error);
        return;
    }

    fpi_ssm_next_state(user_data);
}

static void check_preset_psk_read(FpDevice* dev, gboolean success,
                                  guint32 flags, guint8* psk, guint16 length,
                                  gpointer user_data, GError* error)
{
    g_autofree gchar* psk_str = data_to_str(psk, length);

    if (error) {
        fpi_ssm_mark_failed(user_data, error);
        return;
    }

    if (!success) {
        g_set_error(&error, G_IO_ERROR, G_IO_ERROR_FAILED,
                    "Failed to read PSK from device");
        fpi_ssm_mark_failed(user_data, error);
        return;
    }

    fp_dbg("Device PSK: 0x%s", psk_str);
    fp_dbg("Device PSK flags: 0x%08x", flags);

    if (flags != GOODIX_53XD_PSK_FLAGS) {
        g_set_error(&error, G_IO_ERROR, G_IO_ERROR_INVALID_DATA,
                    "Invalid device PSK flags: 0x%08x", flags);
        fpi_ssm_mark_failed(user_data, error);
        return;
    }

    if (length != sizeof(goodix_53xd_psk_0)) {
        g_set_error(&error, G_IO_ERROR, G_IO_ERROR_INVALID_DATA,
                    "Invalid device PSK: 0x%s", psk_str);
        fpi_ssm_mark_failed(user_data, error);
        return;
    }

    if (memcmp(psk, goodix_53xd_psk_0, sizeof(goodix_53xd_psk_0))) {
        g_set_error(&error, G_IO_ERROR, G_IO_ERROR_INVALID_DATA,
                    "Invalid device PSK: 0x%s", psk_str);
        fpi_ssm_mark_failed(user_data, error);
        return;
    }

    fpi_ssm_next_state(user_data);
}

static void check_idle(FpDevice* dev, gpointer user_data, GError* err)
{
    if (err) {
        fpi_ssm_mark_failed(user_data, err);
        return;
    }
    fpi_ssm_next_state(user_data);
}

static void check_config_upload(FpDevice* dev, gboolean success,
                                gpointer user_data, GError* error)
{
    if (error) {
        fpi_ssm_mark_failed(user_data, error);
    }
    else if (!success) {
        fpi_ssm_mark_failed(user_data,
                            g_error_new(FP_DEVICE_ERROR, FP_DEVICE_ERROR_PROTO,
                                        "failed to upload mcu config"));
    }
    else {
        fpi_ssm_next_state(user_data);
    }
}

static void read_otp_callback(FpDevice* dev, guint8* data, guint16 len,
                              gpointer ssm, GError* err)
{
    if (err) {
        fpi_ssm_mark_failed(ssm, err);
        return;
    }
    if (len < 64) {
        fpi_ssm_mark_failed(ssm, g_error_new(FP_DEVICE_ERROR,
                                             FP_DEVICE_ERROR_DATA_INVALID,
                                             "OTP is invalid (len: %d)", len));
        return;
    }
    FpiDeviceGoodixTls53XD* self = FPI_DEVICE_GOODIXTLS53XD(dev);
    g_clear_pointer(&self->otp, g_free);
    self->otp = g_malloc(64);
    memcpy(self->otp, data, 64);
    fpi_ssm_next_state(ssm);
}

// ---------------------------------------------------------------------------
// Open SSM: claim + activation + TLS handshake
// ---------------------------------------------------------------------------

enum open_states {
    OPEN_READ_AND_NOP,
    OPEN_ENABLE_CHIP,
    OPEN_NOP,
    OPEN_CHECK_FW_VER,
    OPEN_CHECK_PSK,
    OPEN_RESET,
    OPEN_OTP,
    OPEN_SET_MCU_IDLE,
    OPEN_SET_MCU_CONFIG,
    OPEN_TLS,
    OPEN_NUM_STATES,
};

static void open_tls_complete(FpDevice* dev, gpointer ssm, GError* error)
{
    if (error) {
        fpi_ssm_mark_failed(ssm, error);
        return;
    }
    fpi_ssm_next_state(ssm);
}

static void open_run_state(FpiSsm* ssm, FpDevice* dev)
{
    switch (fpi_ssm_get_cur_state(ssm)) {
    case OPEN_READ_AND_NOP:
        // Nop seems to clear the previous command buffer.
        goodix_start_read_loop(dev);
        goodix_send_nop(dev, check_none, ssm);
        break;

    case OPEN_ENABLE_CHIP:
        goodix_send_enable_chip(dev, TRUE, check_none, ssm);
        break;

    case OPEN_NOP:
        goodix_send_nop(dev, check_none, ssm);
        break;

    case OPEN_CHECK_FW_VER:
        goodix_send_firmware_version(dev, check_firmware_version, ssm);
        break;

    case OPEN_CHECK_PSK:
        goodix_send_preset_psk_read(dev, GOODIX_53XD_PSK_FLAGS, 32,
                                    check_preset_psk_read, ssm);
        break;

    case OPEN_RESET:
        goodix_send_reset(dev, TRUE, 20, check_reset, ssm);
        break;

    case OPEN_OTP:
        goodix_send_read_otp(dev, read_otp_callback, ssm);
        break;

    case OPEN_SET_MCU_IDLE:
        goodix_send_mcu_switch_to_idle_mode(dev, 20, check_idle, ssm);
        break;

    case OPEN_SET_MCU_CONFIG:
        goodix_send_upload_config_mcu(dev, goodix_53xd_config,
                                      sizeof(goodix_53xd_config), NULL,
                                      check_config_upload, ssm);
        break;

    case OPEN_TLS:
        goodix_tls(dev, open_tls_complete, ssm);
        break;
    }
}

static void open_ssm_done(FpiSsm* ssm, FpDevice* dev, GError* error)
{
    FpiDeviceGoodixTls53XD* self = FPI_DEVICE_GOODIXTLS53XD(dev);

    self->task_ssm = NULL;
    fpi_device_open_complete(dev, error);
}

// ---------------------------------------------------------------------------
// Image decoding (12-bit raw -> 8-bit grayscale)
// ---------------------------------------------------------------------------

static void decode_frame(Goodix53xdPix frame[GOODIX53XD_FRAME_SIZE],
                         const guint8* raw_frame)
{
    Goodix53xdPix uncropped[GOODIX53XD_SCAN_WIDTH * GOODIX53XD_HEIGHT];
    Goodix53xdPix* pix = uncropped;
    for (int i = 0; i < GOODIX53XD_RAW_FRAME_SIZE; i += 6) {
        const guint8* chunk = raw_frame + i;
        *pix++ = ((chunk[0] & 0xf) << 8) + chunk[1];
        *pix++ = (chunk[3] << 4) + (chunk[0] >> 4);
        *pix++ = ((chunk[5] & 0xf) << 8) + chunk[2];
        *pix++ = (chunk[4] << 4) + (chunk[5] >> 4);
    }

    for (int y = 0; y != GOODIX53XD_HEIGHT; ++y) {
        for (int x = 0; x != GOODIX53XD_WIDTH; ++x) {
            const int idx = x + y * GOODIX53XD_SCAN_WIDTH;
            frame[x + y * GOODIX53XD_WIDTH] = uncropped[idx];
        }
    }
}

// Borrowed from the elan driver: stretch the 12-bit pixels into the 8-bit
// range based on the per-frame min/max.
static void squash_frame_linear(Goodix53xdPix* frame, guint8* squashed)
{
    Goodix53xdPix min = 0xffff;
    Goodix53xdPix max = 0;

    for (int i = 0; i != GOODIX53XD_FRAME_SIZE; ++i) {
        const Goodix53xdPix pix = frame[i];
        if (pix < min)
            min = pix;
        if (pix > max)
            max = pix;
    }

    for (int i = 0; i != GOODIX53XD_FRAME_SIZE; ++i) {
        const Goodix53xdPix pix = frame[i];
        if (pix - min == 0 || max - min == 0)
            squashed[i] = 0;
        else
            squashed[i] = (pix - min) * 0xff / (max - min);
    }
}

// ---------------------------------------------------------------------------
// Capture sub-SSM: wait for finger (FDT down) + read one image
// ---------------------------------------------------------------------------

static const guint8 fdt_switch_state_mode_53xd[] = {
    0x0d, 0x01, 0x28, 0x01, 0x22, 0x01, 0x28, 0x01,
    0x24, 0x01, 0x91, 0x91, 0x8b, 0x8b, 0x96, 0x96,
    0x91, 0x91, 0x98, 0x98, 0x90, 0x90, 0x92, 0x92,
    0x88, 0x88, 0x00};

enum capture_states {
    CAPTURE_FDT_DOWN,
    CAPTURE_FDT_MODE,
    CAPTURE_WRITE_REG,
    CAPTURE_GET_IMG,
    CAPTURE_NUM_STATES,
};

static void capture_fdt_down_arm(FpDevice* dev, FpiSsm* ssm);

static void on_capture_image(FpDevice* dev, guint8* data, guint16 len,
                             gpointer ssm, GError* err)
{
    if (err) {
        fpi_ssm_mark_failed(ssm, err);
        return;
    }
    if (len < GOODIX53XD_RAW_FRAME_SIZE) {
        fpi_ssm_mark_failed(ssm, g_error_new(FP_DEVICE_ERROR,
                                             FP_DEVICE_ERROR_DATA_INVALID,
                                             "short image frame (len %d)", len));
        return;
    }

    FpiDeviceGoodixTls53XD* self = FPI_DEVICE_GOODIXTLS53XD(dev);
    Goodix53xdPix frame[GOODIX53XD_FRAME_SIZE];
    decode_frame(frame, data);

    // Raw dynamic range (before normalisation) -> finger-presence signal.
    Goodix53xdPix rmin = 0xffff, rmax = 0;
    for (int i = 0; i != GOODIX53XD_FRAME_SIZE; ++i) {
        if (frame[i] < rmin)
            rmin = frame[i];
        if (frame[i] > rmax)
            rmax = frame[i];
    }
    self->last_raw_range = rmax - rmin;

    g_clear_pointer(&self->captured_image, g_free);
    self->captured_image = g_malloc(GOODIX53XD_SENSOR_PIXELS);
    squash_frame_linear(frame, self->captured_image);

    // Keypoint count is our finger-presence / quality signal.
    SigfmImgInfo* info = sigfm_extract(self->captured_image, GOODIX53XD_WIDTH,
                                       GOODIX53XD_HEIGHT);
    self->last_keypoints = sigfm_keypoints_count(info);
    sigfm_free_info(info);
    fp_dbg("capture: raw range %u, keypoints %d (finger min %d)",
           self->last_raw_range, self->last_keypoints,
           GOODIX53XD_FINGER_MIN_KEYPOINTS);

    // Optional debug: dump the latest 8-bit frame as a PGM for visual
    // inspection (set GOODIX53XD_DUMP=/path/prefix).
    const char* dump = g_getenv("GOODIX53XD_DUMP");
    if (dump) {
        g_autofree char* path =
            g_strdup_printf("%s-%03d.pgm", dump, self->dump_counter++ % 20);
        FILE* f = fopen(path, "wb");
        if (f) {
            fprintf(f, "P5\n%d %d\n255\n", GOODIX53XD_WIDTH, GOODIX53XD_HEIGHT);
            fwrite(self->captured_image, 1, GOODIX53XD_SENSOR_PIXELS, f);
            fclose(f);
        }
    }

    fpi_ssm_mark_completed(ssm);
}

static gboolean finger_is_present(FpiDeviceGoodixTls53XD* self)
{
    return self->last_keypoints >= GOODIX53XD_FINGER_MIN_KEYPOINTS;
}

// Re-jump the task SSM to target_state after a short delay.
// Uses fpi_ssm_jump_to_state_delayed so the pending timeout is stored in
// ssm->timeout and automatically cancelled if the SSM is freed (e.g. via
// dev_cancel), preventing the use-after-free crash that occurred when
// fpi_device_add_timeout was used with a raw ssm pointer and no way to cancel.
static void poll_again(FpDevice* dev, FpiSsm* ssm, gint target_state)
{
    FpiDeviceGoodixTls53XD* self = FPI_DEVICE_GOODIXTLS53XD(dev);

    self->poll_timeout_pending = TRUE;
    fpi_ssm_jump_to_state_delayed(ssm, target_state, GOODIX53XD_FINGER_POLL_MS);
}

static void capture_get_img(FpDevice* dev, FpiSsm* ssm)
{
    FpiDeviceGoodixTls53XD* self = FPI_DEVICE_GOODIXTLS53XD(dev);
    guint8 payload[] = {0x41,           0x03, self->otp[26], 0x00,
                        self->otp[26] - 6, 0x00, self->otp[45], 0x00,
                        self->otp[45] - 4, 0x00};
    goodix_tls_read_image(dev, payload, sizeof(payload), on_capture_image, ssm);
}

// FDT-down calibration sequence, patched per-OTP. The first arm carries no
// reply; the second blocks until the device reports the finger-down event.
static void on_fdt_down_second(FpDevice* dev, guint8* data, guint16 len,
                               gpointer ssm, GError* err)
{
    if (err) {
        // Treat a transfer timeout as "no finger yet" and re-arm so the user
        // gets more than one USB timeout window to press. (Needs hardware
        // tuning; a real finger-up/finger-down FDT event would be cleaner.)
        if (g_error_matches(err, G_USB_DEVICE_ERROR,
                            G_USB_DEVICE_ERROR_TIMED_OUT)) {
            g_clear_error(&err);
            capture_fdt_down_arm(dev, ssm);
            return;
        }
        fpi_ssm_mark_failed(ssm, err);
        return;
    }
    // Finger detected -> advance the capture SSM.
    fpi_ssm_next_state(ssm);
}

static void on_fdt_down_first(FpDevice* dev, guint8* data, guint16 len,
                              gpointer ssm, GError* err)
{
    if (err) {
        fpi_ssm_mark_failed(ssm, err);
        return;
    }

    FpiDeviceGoodixTls53XD* self = FPI_DEVICE_GOODIXTLS53XD(dev);
    guint8 seq[] = {0x8c, 0x01, self->otp[33], 0x01, self->otp[41], 0x01,
                    self->otp[42], 0x01, self->otp[43], 0x01, 0x91, 0x91,
                    0x8b, 0x8b, 0x96, 0x96, 0x91, 0x91, 0x98, 0x98, 0x90,
                    0x90, 0x92, 0x92, 0x88, 0x88, 0x01};
    // Second arm asks for a reply: blocks until the finger-down event fires.
    goodix_send_mcu_switch_to_fdt_down(dev, seq, sizeof(seq), TRUE, NULL,
                                       on_fdt_down_second, ssm);
}

static void capture_fdt_down_arm(FpDevice* dev, FpiSsm* ssm)
{
    FpiDeviceGoodixTls53XD* self = FPI_DEVICE_GOODIXTLS53XD(dev);
    guint8 seq[] = {0x8c, 0x01, self->otp[33], 0x01, self->otp[41], 0x01,
                    self->otp[42], 0x01, self->otp[43], 0x01, 0x91, 0x91,
                    0x8b, 0x8b, 0x96, 0x96, 0x91, 0x91, 0x98, 0x98, 0x90,
                    0x90, 0x92, 0x92, 0x88, 0x88, 0x00};
    // First arm: no reply.
    goodix_send_mcu_switch_to_fdt_down(dev, seq, sizeof(seq), FALSE, NULL,
                                       on_fdt_down_first, ssm);
}

static void capture_run_state(FpiSsm* ssm, FpDevice* dev)
{
    switch (fpi_ssm_get_cur_state(ssm)) {
    case CAPTURE_FDT_DOWN:
        capture_fdt_down_arm(dev, ssm);
        break;

    case CAPTURE_FDT_MODE:
        goodix_send_mcu_switch_to_fdt_mode(dev,
                                           (guint8*) fdt_switch_state_mode_53xd,
                                           sizeof(fdt_switch_state_mode_53xd),
                                           FALSE, NULL, check_none_cmd, ssm);
        break;

    case CAPTURE_WRITE_REG:
        // Write reg 0x022c (556) <- bytes {0x05, 0x03} (value 0x0305, LE);
        // confirmed against goodix-fp-dump driver_53xd.py before image read.
        goodix_send_write_sensor_register(dev, 556, 0x0305, check_none, ssm);
        break;

    case CAPTURE_GET_IMG:
        capture_get_img(dev, ssm);
        break;
    }
}

// ---------------------------------------------------------------------------
// Enroll SSM
//
// The FDT "down" command does not block until a finger is present on this
// sensor, so finger presence/absence is detected by polling captured frames
// and looking at their raw dynamic range (finger_is_present()).
// ---------------------------------------------------------------------------

enum enroll_states {
    ENROLL_CAPTURE,
    ENROLL_CAPTURE_CHECK,
    ENROLL_PROCESS,
    ENROLL_WAIT_UP,
    ENROLL_WAIT_UP_CHECK,
    ENROLL_NEXT,
    ENROLL_NUM_STATES,
};

static void enroll_run_state(FpiSsm* ssm, FpDevice* dev)
{
    FpiDeviceGoodixTls53XD* self = FPI_DEVICE_GOODIXTLS53XD(dev);

    // Any state entry means a previously armed poll_again() timeout (if any)
    // has already fired and cleared itself; keep our bookkeeping in sync.
    self->poll_timeout_pending = FALSE;

    switch (fpi_ssm_get_cur_state(ssm)) {
    case ENROLL_CAPTURE:
    case ENROLL_WAIT_UP: {
        FpiSsm* sub = fpi_ssm_new(dev, capture_run_state, CAPTURE_NUM_STATES);
        fpi_ssm_start_subsm(ssm, sub);
        break;
    }

    case ENROLL_CAPTURE_CHECK:
        // Poll until a finger is actually on the sensor.
        if (!finger_is_present(self)) {
            g_clear_pointer(&self->captured_image, g_free);
            poll_again(dev, ssm, ENROLL_CAPTURE);
            return;
        }
        fpi_ssm_next_state(ssm);
        break;

    case ENROLL_PROCESS: {
        SigfmImgInfo* info = sigfm_extract(self->captured_image,
                                           GOODIX53XD_WIDTH, GOODIX53XD_HEIGHT);
        int keypoints = sigfm_keypoints_count(info);
        sigfm_free_info(info);
        fp_dbg("enroll: capture keypoints %d (min %d)", keypoints,
               GOODIX53XD_MIN_CAPTURE_KEYPOINTS);

        if (keypoints < GOODIX53XD_MIN_CAPTURE_KEYPOINTS) {
            g_clear_pointer(&self->captured_image, g_free);
            fpi_device_enroll_progress(
                dev, self->enroll_stage, NULL,
                fpi_device_retry_new(FP_DEVICE_RETRY_CENTER_FINGER));
            // Wait for finger up, then retry this stage.
            fpi_ssm_next_state(ssm);
            return;
        }

        g_ptr_array_add(self->enroll_images, self->captured_image);
        self->captured_image = NULL;
        self->enroll_stage++;

        fp_dbg("Enrollment stage %d/%d complete", self->enroll_stage,
               GOODIX53XD_ENROLL_SAMPLES);
        fpi_device_enroll_progress(dev, self->enroll_stage, NULL, NULL);
        fpi_ssm_next_state(ssm);
        break;
    }

    case ENROLL_WAIT_UP_CHECK:
        // Poll until the finger is lifted before the next sample.
        if (finger_is_present(self)) {
            g_clear_pointer(&self->captured_image, g_free);
            poll_again(dev, ssm, ENROLL_WAIT_UP);
            return;
        }
        g_clear_pointer(&self->captured_image, g_free);
        fpi_ssm_next_state(ssm);
        break;

    case ENROLL_NEXT:
        if (self->enroll_stage < GOODIX53XD_ENROLL_SAMPLES)
            fpi_ssm_jump_to_state(ssm, ENROLL_CAPTURE);
        else
            fpi_ssm_mark_completed(ssm);
        break;
    }
}

static void enroll_ssm_done(FpiSsm* ssm, FpDevice* dev, GError* error)
{
    FpiDeviceGoodixTls53XD* self = FPI_DEVICE_GOODIXTLS53XD(dev);

    self->task_ssm = NULL;

    if (error) {
        g_clear_pointer(&self->enroll_images, g_ptr_array_unref);
        g_clear_pointer(&self->captured_image, g_free);
        fpi_device_enroll_complete(dev, NULL, error);
        return;
    }

    FpPrint* print = NULL;
    fpi_device_get_enroll_data(dev, &print);
    fpi_print_set_type(print, FPI_PRINT_RAW);

    GVariantBuilder builder;
    g_variant_builder_init(&builder, G_VARIANT_TYPE("aay"));
    for (guint i = 0; i < self->enroll_images->len; i++) {
        guint8* img = g_ptr_array_index(self->enroll_images, i);
        g_variant_builder_add(
            &builder, "@ay",
            g_variant_new_fixed_array(G_VARIANT_TYPE_BYTE, img,
                                      GOODIX53XD_SENSOR_PIXELS, 1));
    }
    GVariant* data = g_variant_builder_end(&builder);
    g_object_set(G_OBJECT(print), "fpi-data", data, NULL);

    g_clear_pointer(&self->enroll_images, g_ptr_array_unref);

    fp_info("Enrollment complete with %d samples", GOODIX53XD_ENROLL_SAMPLES);
    fpi_device_enroll_complete(dev, g_object_ref(print), NULL);
}

// ---------------------------------------------------------------------------
// Verify / Identify SSM
// ---------------------------------------------------------------------------

static void clear_pending_result(FpiDeviceGoodixTls53XD* self)
{
    self->pending_result_report = FALSE;
    self->pending_result_action = 0;
    self->pending_verify_result = 0;
    g_clear_object(&self->pending_identify_match);
    g_clear_error(&self->pending_result_error);
}

static void queue_verify_report(FpiDeviceGoodixTls53XD* self,
                                FpiMatchResult result, GError* error)
{
    clear_pending_result(self);
    self->pending_result_report = TRUE;
    self->pending_result_action = FPI_DEVICE_ACTION_VERIFY;
    self->pending_verify_result = result;
    self->pending_result_error = error;
}

static void queue_identify_report(FpiDeviceGoodixTls53XD* self, FpPrint* match,
                                  GError* error)
{
    clear_pending_result(self);
    self->pending_result_report = TRUE;
    self->pending_result_action = FPI_DEVICE_ACTION_IDENTIFY;
    if (match != NULL)
        self->pending_identify_match = g_object_ref(match);
    self->pending_result_error = error;
}

static void flush_pending_result(FpDevice* dev)
{
    FpiDeviceGoodixTls53XD* self = FPI_DEVICE_GOODIXTLS53XD(dev);

    if (!self->pending_result_report)
        return;

    self->action_result_reported = TRUE;

    if (self->pending_result_action == FPI_DEVICE_ACTION_IDENTIFY) {
        g_autoptr(FpPrint) match = g_steal_pointer(&self->pending_identify_match);
        fpi_device_identify_report(dev, match, NULL,
                                   g_steal_pointer(&self->pending_result_error));
    }
    else {
        fpi_device_verify_report(dev, self->pending_verify_result, NULL,
                                 g_steal_pointer(&self->pending_result_error));
    }

    self->pending_result_report = FALSE;
    self->pending_result_action = 0;
    self->pending_verify_result = 0;
}

// Match the probe against one stored template ("aay" of raw images).
//
// Returns the *second*-highest per-sample SIGFM score across the template's
// GOODIX53XD_ENROLL_SAMPLES stored samples, not the single best one. Taking
// the max of many independent pairwise comparisons (8 samples per enrolled
// finger, times the gallery size on identify) inflates the false-accept
// rate well beyond what a single-pair threshold calibration suggests: a
// lone coincidentally-high score against one stored sample is enough to
// accept with max-pooling, and on this small 64x80 sensor with few SIFT
// keypoints such one-off collisions are common enough to cause cross-finger
// false accepts (e.g. left index confused for an already-enrolled right
// index). Requiring two independent stored samples to each clear the
// threshold keeps genuine matches (which score highly consistently) intact
// while making a single spurious high score insufficient.
static int match_against_template(SigfmImgInfo* probe, GVariant* tmpl_data)
{
    int best = 0;
    int second_best = 0;
    GVariantIter iter;
    GVariant* child;

    g_variant_iter_init(&iter, tmpl_data);
    while ((child = g_variant_iter_next_value(&iter))) {
        gsize len;
        const guint8* img = g_variant_get_fixed_array(child, &len, 1);
        if (len == GOODIX53XD_SENSOR_PIXELS) {
            SigfmImgInfo* tmpl_info =
                sigfm_extract(img, GOODIX53XD_WIDTH, GOODIX53XD_HEIGHT);
            int score = sigfm_match_score(probe, tmpl_info);
            sigfm_free_info(tmpl_info);
            if (score > best) {
                second_best = best;
                best = score;
            }
            else if (score > second_best) {
                second_best = score;
            }
        }
        g_variant_unref(child);
    }
    return second_best;
}

enum verify_states {
    VERIFY_CAPTURE,
    VERIFY_CAPTURE_CHECK,
    VERIFY_MATCH,
    VERIFY_WAIT_UP,
    VERIFY_WAIT_UP_CHECK,
    VERIFY_NUM_STATES,
};

static void verify_run_state(FpiSsm* ssm, FpDevice* dev)
{
    FpiDeviceGoodixTls53XD* self = FPI_DEVICE_GOODIXTLS53XD(dev);

    // Any state entry means a previously armed poll_again() timeout (if any)
    // has already fired and cleared itself; keep our bookkeeping in sync.
    self->poll_timeout_pending = FALSE;

    switch (fpi_ssm_get_cur_state(ssm)) {
    case VERIFY_CAPTURE: {
        FpiSsm* sub = fpi_ssm_new(dev, capture_run_state, CAPTURE_NUM_STATES);
        fpi_ssm_start_subsm(ssm, sub);
        break;
    }

    case VERIFY_CAPTURE_CHECK:
        if (!finger_is_present(self)) {
            g_clear_pointer(&self->captured_image, g_free);
            poll_again(dev, ssm, VERIFY_CAPTURE);
            return;
        }
        fpi_ssm_next_state(ssm);
        break;

    case VERIFY_MATCH: {
        FpiDeviceAction action = fpi_device_get_current_action(dev);
        SigfmImgInfo* probe = sigfm_extract(self->captured_image,
                                            GOODIX53XD_WIDTH, GOODIX53XD_HEIGHT);
        int keypoints = sigfm_keypoints_count(probe);
        fp_dbg("verify: capture keypoints %d (min %d)", keypoints,
               GOODIX53XD_MIN_CAPTURE_KEYPOINTS);

        if (keypoints < GOODIX53XD_MIN_CAPTURE_KEYPOINTS) {
            if (action == FPI_DEVICE_ACTION_IDENTIFY)
                queue_identify_report(
                    self, NULL,
                    fpi_device_retry_new(FP_DEVICE_RETRY_REMOVE_FINGER));
            else
                queue_verify_report(
                    self, FPI_MATCH_ERROR,
                    fpi_device_retry_new(FP_DEVICE_RETRY_REMOVE_FINGER));

            self->verify_wait_finger_up = TRUE;
            sigfm_free_info(probe);
            g_clear_pointer(&self->captured_image, g_free);
            flush_pending_result(dev);
            fpi_ssm_next_state(ssm);
            return;
        }

        if (action == FPI_DEVICE_ACTION_IDENTIFY) {
            GPtrArray* gallery = NULL;
            FpPrint* match = NULL;
            int best_score = 0;

            fpi_device_get_identify_data(dev, &gallery);
            for (guint i = 0; i < gallery->len; i++) {
                FpPrint* tmpl = g_ptr_array_index(gallery, i);
                GVariant* tmpl_data = NULL;
                g_object_get(G_OBJECT(tmpl), "fpi-data", &tmpl_data, NULL);
                if (tmpl_data == NULL)
                    continue;
                int score = match_against_template(probe, tmpl_data);
                g_variant_unref(tmpl_data);
                fp_dbg("identify: gallery[%u] sigfm corroborated %d", i, score);
                if (score >= GOODIX53XD_SIGFM_BEST_MIN && score > best_score) {
                    best_score = score;
                    match = tmpl;
                }
            }

            if (match != NULL) {
                queue_identify_report(self, match, NULL);
                self->verify_wait_finger_up = FALSE;
            }
            else {
                queue_identify_report(self, NULL, NULL);
                self->verify_wait_finger_up = TRUE;
            }
        }
        else {
            FpPrint* print = NULL;
            GVariant* data = NULL;
            int best_score = 0;

            fpi_device_get_verify_data(dev, &print);
            g_object_get(G_OBJECT(print), "fpi-data", &data, NULL);
            if (data != NULL) {
                best_score = match_against_template(probe, data);
                g_variant_unref(data);
            }
            fp_dbg("verify: corroborated sigfm %d (min %d)", best_score,
                   GOODIX53XD_SIGFM_BEST_MIN);

            if (best_score >= GOODIX53XD_SIGFM_BEST_MIN) {
                queue_verify_report(self, FPI_MATCH_SUCCESS, NULL);
                self->verify_wait_finger_up = FALSE;
            }
            else {
                queue_verify_report(self, FPI_MATCH_FAIL, NULL);
                self->verify_wait_finger_up = TRUE;
            }
        }

        sigfm_free_info(probe);
        g_clear_pointer(&self->captured_image, g_free);

        if (self->verify_wait_finger_up)
            flush_pending_result(dev);

        fpi_ssm_next_state(ssm);
        break;
    }

    case VERIFY_WAIT_UP:
        // On a confirmed match we are done; otherwise wait for finger up
        // before completing so the user can retry cleanly.
        if (!self->verify_wait_finger_up) {
            fpi_ssm_mark_completed(ssm);
            break;
        }
        {
            FpiSsm* sub =
                fpi_ssm_new(dev, capture_run_state, CAPTURE_NUM_STATES);
            fpi_ssm_start_subsm(ssm, sub);
        }
        break;

    case VERIFY_WAIT_UP_CHECK:
        if (finger_is_present(self)) {
            g_clear_pointer(&self->captured_image, g_free);
            poll_again(dev, ssm, VERIFY_WAIT_UP);
            return;
        }
        g_clear_pointer(&self->captured_image, g_free);
        fpi_ssm_mark_completed(ssm);
        break;
    }
}

static void verify_ssm_done(FpiSsm* ssm, FpDevice* dev, GError* error)
{
    FpiDeviceGoodixTls53XD* self = FPI_DEVICE_GOODIXTLS53XD(dev);
    FpiDeviceAction action = fpi_device_get_current_action(dev);

    self->task_ssm = NULL;
    g_clear_pointer(&self->captured_image, g_free);

    if (error == NULL)
        flush_pending_result(dev);
    else
        clear_pending_result(self);

    self->action_result_reported = FALSE;
    self->verify_wait_finger_up = FALSE;

    if (action == FPI_DEVICE_ACTION_IDENTIFY)
        fpi_device_identify_complete(dev, error);
    else
        fpi_device_verify_complete(dev, error);
}

// ---------------------------------------------------------------------------
// FpDevice virtual methods
// ---------------------------------------------------------------------------

static void dev_open(FpDevice* dev)
{
    FpiDeviceGoodixTls53XD* self = FPI_DEVICE_GOODIXTLS53XD(dev);
    GError* error = NULL;

    // goodix_dev_init() claims the USB interface; it returns TRUE on success.
    if (!goodix_dev_init(dev, &error)) {
        fpi_device_open_complete(dev, error);
        return;
    }

    self->task_ssm = fpi_ssm_new(dev, open_run_state, OPEN_NUM_STATES);
    fpi_ssm_start(self->task_ssm, open_ssm_done);
}

static void dev_close(FpDevice* dev)
{
    FpiDeviceGoodixTls53XD* self = FPI_DEVICE_GOODIXTLS53XD(dev);
    GError* error = NULL;

    g_clear_pointer(&self->captured_image, g_free);
    g_clear_pointer(&self->enroll_images, g_ptr_array_unref);
    g_clear_pointer(&self->otp, g_free);
    clear_pending_result(self);

    goodix_reset_state(dev);
    goodix_dev_deinit(dev, &error);
    fpi_device_close_complete(dev, error);
}

static void dev_enroll(FpDevice* dev)
{
    FpiDeviceGoodixTls53XD* self = FPI_DEVICE_GOODIXTLS53XD(dev);

    self->enroll_stage = 0;
    g_clear_pointer(&self->enroll_images, g_ptr_array_unref);
    self->enroll_images = g_ptr_array_new_with_free_func(g_free);

    self->task_ssm = fpi_ssm_new(dev, enroll_run_state, ENROLL_NUM_STATES);
    fpi_ssm_start(self->task_ssm, enroll_ssm_done);
}

static void dev_verify(FpDevice* dev)
{
    FpiDeviceGoodixTls53XD* self = FPI_DEVICE_GOODIXTLS53XD(dev);

    clear_pending_result(self);
    self->action_result_reported = FALSE;
    self->verify_wait_finger_up = FALSE;

    self->task_ssm = fpi_ssm_new(dev, verify_run_state, VERIFY_NUM_STATES);
    fpi_ssm_start(self->task_ssm, verify_ssm_done);
}

static void dev_cancel(FpDevice* dev)
{
    FpiDeviceGoodixTls53XD* self = FPI_DEVICE_GOODIXTLS53XD(dev);

    if (self->task_ssm) {
        // A poll_again() delayed state change may still be armed on
        // task_ssm (we're waiting out GOODIX53XD_FINGER_POLL_MS between
        // finger-presence polls). fpi_ssm_mark_failed() asserts no timeout
        // is pending, so disarm it first or we hit
        // "BUG: (machine->timeout != NULL)" and leave the timeout dangling.
        if (self->poll_timeout_pending) {
            fpi_ssm_cancel_delayed_state_change(self->task_ssm);
            self->poll_timeout_pending = FALSE;
        }
        goodix_reset_state(dev);

        // A FDT_DOWN "wait for finger" (0x32) may still be outstanding on
        // the MCU itself when we get here: goodix_reset_state() only clears
        // our own ack/reply/timeout bookkeeping, it can't un-send bytes
        // already on the wire. Left alone, the MCU eventually replies to
        // that stale FDT_DOWN once a *new* command is already in flight, and
        // its cmd byte (0x32) no longer matches priv->cmd -> permanent
        // "Invalid protocol/ACK command" desync for the rest of this
        // fprintd process's life (confirmed in the field: only a full
        // fprintd restart recovers). Explicitly switching the MCU to idle
        // closes out any pending FDT sequence on its side before the next
        // verify attempt starts. Fire-and-forget: priv state is already
        // clear so it can't collide with the next command, and it
        // self-resolves via ack-or-GOODIX_TIMEOUT either way.
        goodix_send_mcu_switch_to_idle_mode(dev, 20, NULL, NULL);

        fpi_ssm_mark_failed(self->task_ssm,
                            g_error_new(G_IO_ERROR, G_IO_ERROR_CANCELLED,
                                        "Cancelled"));
    }
}

// ---------------------------------------------------------------------------

static void fpi_device_goodixtls53xd_init(FpiDeviceGoodixTls53XD* self) {}

static void fpi_device_goodixtls53xd_class_init(
    FpiDeviceGoodixTls53XDClass* class)
{
    FpiDeviceGoodixTlsClass* gx_class = FPI_DEVICE_GOODIXTLS_CLASS(class);
    FpDeviceClass* dev_class = FP_DEVICE_CLASS(class);

    gx_class->interface = GOODIX_53XD_INTERFACE;
    gx_class->ep_in = GOODIX_53XD_EP_IN;
    gx_class->ep_out = GOODIX_53XD_EP_OUT;

    dev_class->id = "goodixtls53xd";
    dev_class->full_name = "Goodix TLS Fingerprint Sensor 53XD";
    dev_class->type = FP_DEVICE_TYPE_USB;
    dev_class->id_table = id_table;
    dev_class->scan_type = FP_SCAN_TYPE_PRESS;
    dev_class->nr_enroll_stages = GOODIX53XD_ENROLL_SAMPLES;
    dev_class->temp_hot_seconds = -1;

    dev_class->open = dev_open;
    dev_class->close = dev_close;
    dev_class->enroll = dev_enroll;
    dev_class->verify = dev_verify;
    dev_class->identify = dev_verify;
    dev_class->cancel = dev_cancel;

    dev_class->features = FP_DEVICE_FEATURE_VERIFY | FP_DEVICE_FEATURE_IDENTIFY;
}
