// Goodix 538d image preprocessing and SIGFM template matching.
//
// Kept separate from the driver state machines so the exact same pipeline can
// be run offline over recorded frames (libfprint/sigfm/sigfm-eval.c) to
// measure false accept / false reject rates before changing any tuning.

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

#pragma once

#include <glib.h>

#include "sigfm/sigfm.h"

#define GOODIX53XD_WIDTH 64
#define GOODIX53XD_HEIGHT 80
#define GOODIX53XD_FRAME_SIZE (GOODIX53XD_WIDTH * GOODIX53XD_HEIGHT)

// Minimum SIGFM match score (as returned by goodix53xd_match_raw_template())
// to accept a verify/identify. Calibrated offline with sigfm-eval over 538d
// captures of three fingers (16-sample templates, fixed pattern from other
// sessions): impostors scored at most 11, while genuine rejections stayed at
// 8% anywhere between 4 and 30. 24 keeps a 2x margin over the worst impostor.
#define GOODIX53XD_SIGFM_BEST_MIN 24

// Enrollment sessions kept in the fixed-pattern calibration (oldest dropped).
#define GOODIX53XD_CALIB_MAX_SESSIONS 16

// Raw 12-bit sensor pixel, as decoded from the image read reply.
typedef guint16 Goodix53xdPix;

// Stretch the 12-bit pixels into the 8-bit range based on the per-frame
// min/max.
void goodix53xd_squash_frame_linear(const Goodix53xdPix* frame,
                                    guint8* squashed);

// Ridge-enhancing preprocessing. The 538d frames are dominated by a fixed
// per-pixel/per-row pattern that is identical for every finger; left in, it
// gives SIFT keypoints at the same positions for any finger. With fpn (the
// sensor's fixed pattern, GOODIX53XD_FRAME_SIZE floats) it is subtracted
// exactly; with fpn == NULL it is approximated per frame by removing row and
// column offsets and smoothing. The result is then locally normalised so the
// ridges have uniform contrast, and the dead 1-pixel border is replaced by
// its neighbours.
void goodix53xd_enhance_frame(const Goodix53xdPix* frame, const float* fpn,
                              guint8* out);

// Score a probe against the stored 8-bit samples of one enrolled template.
int goodix53xd_match_template(SigfmImgInfo* probe,
                              const guint8* const* samples, guint n_samples);

// Score a raw probe frame against the raw frames of one enrolled template,
// enhancing all of them with the same fixed pattern first.
int goodix53xd_match_raw_template(const Goodix53xdPix* probe,
                                  const Goodix53xdPix* const* samples,
                                  guint n_samples, const float* fpn);

// Highest SIGFM score of `probe` against any single one of `samples` (all
// raw, enhanced with fpn, which may be NULL). Used at enrollment to spot a
// press on the same spot as an earlier one.
int goodix53xd_match_raw_best_single(const Goodix53xdPix* probe,
                                     const Goodix53xdPix* const* samples,
                                     guint n_samples, const float* fpn);

void goodix53xd_mean_frame(const Goodix53xdPix* const* frames, guint n_frames,
                           float* mean);

// Fixed-pattern calibration.
//
// The sensor's fixed pattern can't be measured on a bare sensor (every pixel
// saturates) and can't be estimated from one finger's frames alone (the mean
// of a few presses of the same finger still holds its ridges, and subtracting
// them wrecks genuine matches). So it is learned from the mean frame of each
// enrollment session: when scoring against a template, only the *other*
// sessions are used, which on 538d data correlates at ~0.95 with the true
// pattern after a single other finger.
typedef struct {
  guint64 session;
  float mean[GOODIX53XD_FRAME_SIZE];
} Goodix53xdCalibSession;

// Returns a GArray of Goodix53xdCalibSession; empty if path doesn't exist.
GArray* goodix53xd_calib_load(const char* path, GError** error);
gboolean goodix53xd_calib_save(GArray* calib, const char* path,
                               GError** error);
// Add (or replace) the mean frame of an enrollment session, dropping the
// oldest ones past GOODIX53XD_CALIB_MAX_SESSIONS.
void goodix53xd_calib_add(GArray* calib, guint64 session, const float* mean);
// Mean of every session but `session`. FALSE (fpn untouched) if there is
// none.
gboolean goodix53xd_calib_fpn_excluding(GArray* calib, guint64 session,
                                        float* fpn);
