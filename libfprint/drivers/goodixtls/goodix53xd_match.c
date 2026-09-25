// Goodix 538d image preprocessing and SIGFM template matching.

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

#include "goodix53xd_match.h"

#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// Borrowed from the elan driver: stretch the 12-bit pixels into the 8-bit
// range based on the per-frame min/max.
void goodix53xd_squash_frame_linear(const Goodix53xdPix* frame,
                                    guint8* squashed)
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

// Pixels of the outer ring that read as dead (0) on every frame.
#define BORDER 2

static int cmp_float(const void* a, const void* b)
{
    float x = *(const float*) a, y = *(const float*) b;
    return (x > y) - (x < y);
}

static float median(float* v, int n)
{
    qsort(v, n, sizeof(float), cmp_float);
    return v[n / 2];
}

// Separable Gaussian blur with edge clamping.
static void gauss_blur(const float* in, float* out, double sigma)
{
    const int r = (int) (3 * sigma + 0.5);
    float k[32];
    float tmp[GOODIX53XD_FRAME_SIZE];
    double sum = 0;

    g_assert(2 * r + 1 <= (int) G_N_ELEMENTS(k));
    for (int i = -r; i <= r; i++)
        sum += k[i + r] = exp(-(double) i * i / (2 * sigma * sigma));
    for (int i = 0; i <= 2 * r; i++)
        k[i] /= sum;

    for (int y = 0; y < GOODIX53XD_HEIGHT; y++)
        for (int x = 0; x < GOODIX53XD_WIDTH; x++) {
            float acc = 0;
            for (int i = -r; i <= r; i++) {
                int xx = CLAMP(x + i, 0, GOODIX53XD_WIDTH - 1);
                acc += k[i + r] * in[y * GOODIX53XD_WIDTH + xx];
            }
            tmp[y * GOODIX53XD_WIDTH + x] = acc;
        }
    for (int y = 0; y < GOODIX53XD_HEIGHT; y++)
        for (int x = 0; x < GOODIX53XD_WIDTH; x++) {
            float acc = 0;
            for (int i = -r; i <= r; i++) {
                int yy = CLAMP(y + i, 0, GOODIX53XD_HEIGHT - 1);
                acc += k[i + r] * tmp[yy * GOODIX53XD_WIDTH + x];
            }
            out[y * GOODIX53XD_WIDTH + x] = acc;
        }
}

void goodix53xd_enhance_frame(const Goodix53xdPix* frame, const float* fpn,
                              guint8* out)
{
    const int w = GOODIX53XD_WIDTH, h = GOODIX53XD_HEIGHT;
    float f[GOODIX53XD_FRAME_SIZE];
    float mean[GOODIX53XD_FRAME_SIZE];
    float var[GOODIX53XD_FRAME_SIZE];
    float line[MAX(GOODIX53XD_WIDTH, GOODIX53XD_HEIGHT)];

    for (int i = 0; i < GOODIX53XD_FRAME_SIZE; i++)
        f[i] = frame[i] - (fpn ? fpn[i] : 0);

    if (!fpn) {
        // Row, then column offsets, over the live area only.
        for (int y = BORDER; y < h - BORDER; y++) {
            int n = 0;
            for (int x = BORDER; x < w - BORDER; x++)
                line[n++] = f[y * w + x];
            float m = median(line, n);
            for (int x = 0; x < w; x++)
                f[y * w + x] -= m;
        }
        for (int x = BORDER; x < w - BORDER; x++) {
            int n = 0;
            for (int y = BORDER; y < h - BORDER; y++)
                line[n++] = f[y * w + x];
            float m = median(line, n);
            for (int y = 0; y < h; y++)
                f[y * w + x] -= m;
        }
    }

    // Replace the dead border by the nearest live pixel so it doesn't show
    // up as a strong edge at the same place in every frame.
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++) {
            int yy = CLAMP(y, BORDER, h - 1 - BORDER);
            int xx = CLAMP(x, BORDER, w - 1 - BORDER);
            f[y * w + x] = f[yy * w + xx];
        }

    // Suppress what is left of the per-pixel noise (much more of it without
    // the fixed pattern), keeping ridges (period ~8 px).
    gauss_blur(f, f, fpn ? 0.8 : 1.2);

    // Local normalisation: zero mean, unit contrast in a ~ridge-period
    // neighbourhood.
    gauss_blur(f, mean, 4.0);
    for (int i = 0; i < GOODIX53XD_FRAME_SIZE; i++) {
        f[i] -= mean[i];
        var[i] = f[i] * f[i];
    }
    gauss_blur(var, var, 4.0);
    for (int i = 0; i < GOODIX53XD_FRAME_SIZE; i++) {
        float v = 128 + 48 * f[i] / (sqrtf(var[i]) + 1e-3f);
        out[i] = CLAMP(v, 0, 255);
    }
}

// Returns the *second*-highest per-sample SIGFM score across the template's
// stored samples, not the single best one. Taking the max of many independent
// pairwise comparisons (8 samples per enrolled finger, times the gallery size
// on identify) inflates the false-accept rate well beyond what a single-pair
// threshold calibration suggests: a lone coincidentally-high score against
// one stored sample is enough to accept with max-pooling, and on this small
// 64x80 sensor with few SIFT keypoints such one-off collisions are common
// enough to cause cross-finger false accepts (e.g. left index confused for an
// already-enrolled right index). Requiring two independent stored samples to
// each clear the threshold keeps genuine matches (which score highly
// consistently) intact while making a single spurious high score
// insufficient.
int goodix53xd_match_template(SigfmImgInfo* probe,
                              const guint8* const* samples, guint n_samples)
{
    int best = 0;
    int second_best = 0;

    for (guint i = 0; i < n_samples; i++) {
        SigfmImgInfo* tmpl_info =
            sigfm_extract(samples[i], GOODIX53XD_WIDTH, GOODIX53XD_HEIGHT);
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
    return second_best;
}

int goodix53xd_match_raw_template(const Goodix53xdPix* probe,
                                  const Goodix53xdPix* const* samples,
                                  guint n_samples, const float* fpn)
{
    g_autofree guint8* imgs = g_malloc(n_samples * GOODIX53XD_FRAME_SIZE);
    g_autofree const guint8** ptrs = g_new(const guint8*, n_samples);
    guint8 probe_img[GOODIX53XD_FRAME_SIZE];

    for (guint i = 0; i < n_samples; i++) {
        guint8* img = imgs + i * GOODIX53XD_FRAME_SIZE;
        goodix53xd_enhance_frame(samples[i], fpn, img);
        ptrs[i] = img;
    }
    goodix53xd_enhance_frame(probe, fpn, probe_img);

    SigfmImgInfo* info =
        sigfm_extract(probe_img, GOODIX53XD_WIDTH, GOODIX53XD_HEIGHT);
    int score = goodix53xd_match_template(info, ptrs, n_samples);
    sigfm_free_info(info);
    return score;
}

int goodix53xd_match_raw_best_single(const Goodix53xdPix* probe,
                                     const Goodix53xdPix* const* samples,
                                     guint n_samples, const float* fpn)
{
    guint8 img[GOODIX53XD_FRAME_SIZE];
    int best = 0;

    goodix53xd_enhance_frame(probe, fpn, img);
    SigfmImgInfo* info = sigfm_extract(img, GOODIX53XD_WIDTH, GOODIX53XD_HEIGHT);
    for (guint i = 0; i < n_samples; i++) {
        goodix53xd_enhance_frame(samples[i], fpn, img);
        SigfmImgInfo* s = sigfm_extract(img, GOODIX53XD_WIDTH, GOODIX53XD_HEIGHT);
        best = MAX(best, sigfm_match_score(info, s));
        sigfm_free_info(s);
    }
    sigfm_free_info(info);
    return best;
}

void goodix53xd_mean_frame(const Goodix53xdPix* const* frames, guint n_frames,
                           float* mean)
{
    for (int k = 0; k < GOODIX53XD_FRAME_SIZE; k++) {
        double sum = 0;
        for (guint i = 0; i < n_frames; i++)
            sum += frames[i][k];
        mean[k] = n_frames ? sum / n_frames : 0;
    }
}

// On-disk calibration: GVariant "(ua(taq))" holding a format version and,
// per session, its id and mean frame in 1/16 pixel units (12-bit * 16 still
// fits a guint16).
#define CALIB_VERSION 1
#define CALIB_TYPE "(ua(taq))"
#define CALIB_SCALE 16

GArray* goodix53xd_calib_load(const char* path, GError** error)
{
    GArray* calib = g_array_new(FALSE, FALSE, sizeof(Goodix53xdCalibSession));
    g_autofree char* contents = NULL;
    g_autoptr(GError) local_error = NULL;
    gsize len;

    if (!g_file_get_contents(path, &contents, &len, &local_error)) {
        if (g_error_matches(local_error, G_FILE_ERROR, G_FILE_ERROR_NOENT))
            return calib;
        g_propagate_error(error, g_steal_pointer(&local_error));
        g_array_unref(calib);
        return NULL;
    }

    g_autoptr(GBytes) bytes = g_bytes_new_take(g_steal_pointer(&contents), len);
    g_autoptr(GVariant) v = g_variant_ref_sink(
        g_variant_new_from_bytes(G_VARIANT_TYPE(CALIB_TYPE), bytes, FALSE));
    guint32 version;
    g_variant_get_child(v, 0, "u", &version);
    if (version == GUINT32_SWAP_LE_BE(CALIB_VERSION)) {
        GVariant* swapped = g_variant_byteswap(v);
        g_variant_unref(v);
        v = swapped;
        version = CALIB_VERSION;
    }
    if (version != CALIB_VERSION || !g_variant_is_normal_form(v)) {
        g_set_error(error, G_FILE_ERROR, G_FILE_ERROR_INVAL,
                    "%s: not a goodix53xd calibration file", path);
        g_array_unref(calib);
        return NULL;
    }

    g_autoptr(GVariant) sessions = g_variant_get_child_value(v, 1);
    for (gsize i = 0; i < g_variant_n_children(sessions); i++) {
        g_autoptr(GVariant) child = g_variant_get_child_value(sessions, i);
        g_autoptr(GVariant) mean = g_variant_get_child_value(child, 1);
        gsize n;
        const guint16* px = g_variant_get_fixed_array(mean, &n, sizeof(guint16));
        Goodix53xdCalibSession s;

        if (n != GOODIX53XD_FRAME_SIZE)
            continue;
        g_variant_get_child(child, 0, "t", &s.session);
        for (int k = 0; k < GOODIX53XD_FRAME_SIZE; k++)
            s.mean[k] = (float) px[k] / CALIB_SCALE;
        g_array_append_val(calib, s);
    }
    return calib;
}

gboolean goodix53xd_calib_save(GArray* calib, const char* path, GError** error)
{
    GVariantBuilder builder;
    guint16 px[GOODIX53XD_FRAME_SIZE];

    g_variant_builder_init(&builder, G_VARIANT_TYPE("a(taq)"));
    for (guint i = 0; i < calib->len; i++) {
        const Goodix53xdCalibSession* s =
            &g_array_index(calib, Goodix53xdCalibSession, i);
        for (int k = 0; k < GOODIX53XD_FRAME_SIZE; k++)
            px[k] = CLAMP(s->mean[k] * CALIB_SCALE + 0.5f, 0, G_MAXUINT16);
        g_variant_builder_add(
            &builder, "(t@aq)", s->session,
            g_variant_new_fixed_array(G_VARIANT_TYPE_UINT16, px,
                                      GOODIX53XD_FRAME_SIZE, sizeof(guint16)));
    }
    g_autoptr(GVariant) v = g_variant_ref_sink(
        g_variant_new("(u@a(taq))", CALIB_VERSION,
                      g_variant_builder_end(&builder)));
    g_autofree char* dir = g_path_get_dirname(path);

    if (g_mkdir_with_parents(dir, 0700) != 0) {
        g_set_error(error, G_FILE_ERROR, g_file_error_from_errno(errno),
                    "cannot create %s: %s", dir, g_strerror(errno));
        return FALSE;
    }
    return g_file_set_contents(path, g_variant_get_data(v),
                               g_variant_get_size(v), error);
}

void goodix53xd_calib_add(GArray* calib, guint64 session, const float* mean)
{
    Goodix53xdCalibSession s = {.session = session};

    for (guint i = 0; i < calib->len; i++)
        if (g_array_index(calib, Goodix53xdCalibSession, i).session == session)
            g_array_remove_index(calib, i--);
    memcpy(s.mean, mean, sizeof(s.mean));
    g_array_append_val(calib, s);
    while (calib->len > GOODIX53XD_CALIB_MAX_SESSIONS)
        g_array_remove_index(calib, 0);
}

gboolean goodix53xd_calib_fpn_excluding(GArray* calib, guint64 session,
                                        float* fpn)
{
    guint n = 0;

    for (guint i = 0; i < calib->len; i++) {
        const Goodix53xdCalibSession* s =
            &g_array_index(calib, Goodix53xdCalibSession, i);
        if (s->session == session)
            continue;
        if (n++ == 0)
            memcpy(fpn, s->mean, sizeof(s->mean));
        else
            for (int k = 0; k < GOODIX53XD_FRAME_SIZE; k++)
                fpn[k] += s->mean[k];
    }
    if (n == 0)
        return FALSE;
    for (int k = 0; k < GOODIX53XD_FRAME_SIZE; k++)
        fpn[k] /= n;
    return TRUE;
}
