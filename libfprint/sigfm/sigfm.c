// SIGFM algorithm for libfprint

// Copyright (C) 2022 Matthieu CHARETTE <matthieu.charette@gmail.com>
// Copyright (c) 2022 Natasha England-Elbro <ashenglandelbro@protonmail.com>
// Copyright (c) 2022 Timur Mangliev <tigrmango@gmail.com>

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
//

// Self-contained implementation (no OpenCV). CLAHE, SIFT and the brute-force
// matcher follow OpenCV's algorithms and parameters (imgproc CLAHE,
// features2d SIFT with a doubled base image, BFMatcher NORM_L2) so keypoints
// and scores stay comparable with the original OpenCV-based SIGFM.

#include "sigfm.h"

#include <float.h>
#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DESC_LEN 128

typedef struct {
  float x, y;
  float size;
  float angle;
  int octave; // OpenCV packing: octave | layer << 8 | sub-layer << 16
} Keypoint;

struct SigfmImgInfo {
  int n;
  Keypoint* kps;
  float* desc; // n * DESC_LEN
};

// ---------------------------------------------------------------------------
// Tuning (same values the OpenCV-based SIGFM used)
// ---------------------------------------------------------------------------

#define CLAHE_CLIP 4.0
#define CLAHE_TILES 4

#define SIFT_OCTAVE_LAYERS 3
#define SIFT_CONTRAST_THRESHOLD 0.04
#define SIFT_EDGE_THRESHOLD 18.0
#define SIFT_SIGMA 2.0

#define SIFT_INIT_SIGMA 0.5
#define SIFT_IMG_BORDER 5
#define SIFT_MAX_INTERP_STEPS 5
#define SIFT_ORI_HIST_BINS 36
#define SIFT_ORI_SIG_FCTR 1.5
#define SIFT_ORI_RADIUS (3 * SIFT_ORI_SIG_FCTR)
#define SIFT_ORI_PEAK_RATIO 0.8
#define SIFT_DESCR_WIDTH 4
#define SIFT_DESCR_HIST_BINS 8
#define SIFT_DESCR_SCL_FCTR 3.0
#define SIFT_DESCR_MAG_THR 0.2
#define SIFT_INT_DESCR_FCTR 512.0

#define DISTANCE_MATCH 0.85
#define LENGTH_MATCH 0.05
#define ANGLE_MATCH 0.05
#define MIN_MATCH 5

// ---------------------------------------------------------------------------
// Float images
// ---------------------------------------------------------------------------

typedef struct {
  int w, h;
  float* px;
} Image;

static Image image_new(int w, int h)
{
    Image img = {w, h, calloc((size_t) w * h, sizeof(float))};
    return img;
}

#define AT(img, x, y) ((img).px[(y) * (img).w + (x)])

// BORDER_REFLECT_101: ... 2 1 | 0 1 2 ... n-1 | n-2 ...
static int reflect101(int i, int n)
{
    if (n == 1)
        return 0;
    while (i < 0 || i >= n) {
        if (i < 0)
            i = -i;
        if (i >= n)
            i = 2 * n - 2 - i;
    }
    return i;
}

// One separable pass: dst[i] = sum k[t] * src[reflect101(i + t - r)], for
// `count` lines of `n` samples each, `stride` apart within a line and `step`
// apart between lines.
static void blur_pass(const float* src, float* dst, int n, int count,
                      int stride, int step, const float* k, int r,
                      const int* idx)
{
    for (int l = 0; l < count; l++) {
        const float* s = src + (size_t) l * step;
        float* d = dst + (size_t) l * step;
        for (int i = 0; i < n; i++) {
            const int* ix = idx + i;
            // Symmetric kernel: pair up taps t and 2r - t.
            float acc = k[r] * s[ix[r] * stride];
            for (int t = 0; t < r; t++)
                acc += k[t] * (s[ix[t] * stride] + s[ix[2 * r - t] * stride]);
            d[i * stride] = acc;
        }
    }
}

// cv::GaussianBlur(src, dst, Size(), sigma, sigma) for float images.
static void gaussian_blur(const Image* src, Image* dst, double sigma)
{
    int ksize = ((int) lrint(sigma * 8 + 1)) | 1;
    int r = ksize / 2;
    float* k = malloc(ksize * sizeof(float));
    float* tmp = malloc((size_t) src->w * src->h * sizeof(float));
    // Reflected source index for every output index + tap offset.
    int* xidx = malloc((src->w + 2 * r) * sizeof(int));
    int* yidx = malloc((src->h + 2 * r) * sizeof(int));
    double sum = 0;

    for (int i = 0; i < ksize; i++) {
        double x = i - r;
        k[i] = exp(-x * x / (2 * sigma * sigma));
        sum += k[i];
    }
    for (int i = 0; i < ksize; i++)
        k[i] /= sum;
    for (int i = 0; i < src->w + 2 * r; i++)
        xidx[i] = reflect101(i - r, src->w);
    for (int i = 0; i < src->h + 2 * r; i++)
        yidx[i] = reflect101(i - r, src->h);

    blur_pass(src->px, tmp, src->w, src->h, 1, src->w, k, r, xidx);
    blur_pass(tmp, dst->px, src->h, src->w, src->w, 1, k, r, yidx);

    free(yidx);
    free(xidx);
    free(tmp);
    free(k);
}

// cv::resize(INTER_LINEAR) to twice the size.
static Image upscale2(const Image* src)
{
    Image dst = image_new(src->w * 2, src->h * 2);

    for (int y = 0; y < dst.h; y++) {
        float sy = (y + 0.5f) * 0.5f - 0.5f;
        int y0 = (int) floorf(sy);
        float fy = sy - y0;
        if (y0 < 0) {
            y0 = 0;
            fy = 0;
        }
        if (y0 >= src->h - 1) {
            y0 = src->h - 1;
            fy = 0;
        }
        int y1 = y0 + (fy > 0);
        for (int x = 0; x < dst.w; x++) {
            float sx = (x + 0.5f) * 0.5f - 0.5f;
            int x0 = (int) floorf(sx);
            float fx = sx - x0;
            if (x0 < 0) {
                x0 = 0;
                fx = 0;
            }
            if (x0 >= src->w - 1) {
                x0 = src->w - 1;
                fx = 0;
            }
            int x1 = x0 + (fx > 0);
            AT(dst, x, y) =
                (AT(*src, x0, y0) * (1 - fx) + AT(*src, x1, y0) * fx) * (1 - fy) +
                (AT(*src, x0, y1) * (1 - fx) + AT(*src, x1, y1) * fx) * fy;
        }
    }
    return dst;
}

// cv::resize(INTER_NEAREST) to half the size.
static Image downscale2(const Image* src)
{
    Image dst = image_new(src->w / 2, src->h / 2);

    for (int y = 0; y < dst.h; y++)
        for (int x = 0; x < dst.w; x++)
            AT(dst, x, y) = AT(*src, 2 * x, 2 * y);
    return dst;
}

// ---------------------------------------------------------------------------
// CLAHE
// ---------------------------------------------------------------------------

static unsigned char saturate_u8(double v)
{
    long r = lrint(v);
    return r < 0 ? 0 : r > 255 ? 255 : (unsigned char) r;
}

// cv::createCLAHE(CLAHE_CLIP, Size(CLAHE_TILES, CLAHE_TILES))->apply(). The
// image dimensions must be multiples of CLAHE_TILES (OpenCV pads otherwise).
static int clahe(const SigfmPix* src, int w, int h, unsigned char* dst)
{
    const int tw = w / CLAHE_TILES, th = h / CLAHE_TILES;
    const int tile_area = tw * th;
    int clip = (int) (CLAHE_CLIP * tile_area / 256);
    unsigned char lut[CLAHE_TILES][CLAHE_TILES][256];

    if (tw * CLAHE_TILES != w || th * CLAHE_TILES != h)
        return 0;
    if (clip < 1)
        clip = 1;

    for (int ty = 0; ty < CLAHE_TILES; ty++)
        for (int tx = 0; tx < CLAHE_TILES; tx++) {
            int hist[256] = {0};
            for (int y = ty * th; y < (ty + 1) * th; y++)
                for (int x = tx * tw; x < (tx + 1) * tw; x++)
                    hist[src[y * w + x]]++;

            int clipped = 0;
            for (int i = 0; i < 256; i++)
                if (hist[i] > clip) {
                    clipped += hist[i] - clip;
                    hist[i] = clip;
                }
            int batch = clipped / 256;
            int residual = clipped - batch * 256;
            for (int i = 0; i < 256; i++)
                hist[i] += batch;
            if (residual > 0) {
                int step = 256 / residual > 1 ? 256 / residual : 1;
                for (int i = 0; i < 256 && residual > 0; i += step, residual--)
                    hist[i]++;
            }

            const double scale = 255.0 / tile_area;
            int sum = 0;
            for (int i = 0; i < 256; i++) {
                sum += hist[i];
                lut[ty][tx][i] = saturate_u8(sum * scale);
            }
        }

    for (int y = 0; y < h; y++) {
        float tyf = (float) y / th - 0.5f;
        int ty1 = (int) floorf(tyf), ty2 = ty1 + 1;
        float ya = tyf - ty1;
        if (ty1 < 0)
            ty1 = 0;
        if (ty2 > CLAHE_TILES - 1)
            ty2 = CLAHE_TILES - 1;
        for (int x = 0; x < w; x++) {
            float txf = (float) x / tw - 0.5f;
            int tx1 = (int) floorf(txf), tx2 = tx1 + 1;
            float xa = txf - tx1;
            if (tx1 < 0)
                tx1 = 0;
            if (tx2 > CLAHE_TILES - 1)
                tx2 = CLAHE_TILES - 1;
            int v = src[y * w + x];
            float res =
                (lut[ty1][tx1][v] * (1 - xa) + lut[ty1][tx2][v] * xa) * (1 - ya) +
                (lut[ty2][tx1][v] * (1 - xa) + lut[ty2][tx2][v] * xa) * ya;
            dst[y * w + x] = saturate_u8(res);
        }
    }
    return 1;
}

// ---------------------------------------------------------------------------
// SIFT
// ---------------------------------------------------------------------------

typedef struct {
  int n_octaves;
  Image* gauss; // n_octaves * (SIFT_OCTAVE_LAYERS + 3)
  Image* dog;   // n_octaves * (SIFT_OCTAVE_LAYERS + 2)
  // Gradient magnitude and orientation (degrees) of the Gaussian images
  // keypoints can live on (layers 1..SIFT_OCTAVE_LAYERS), indexed like gauss.
  Image* mag;
  Image* ori;
} Pyramid;

#define GAUSS(p, o, i) ((p)->gauss[(o) * (SIFT_OCTAVE_LAYERS + 3) + (i)])
#define DOG(p, o, i) ((p)->dog[(o) * (SIFT_OCTAVE_LAYERS + 2) + (i)])
#define MAG(p, o, i) ((p)->mag[(o) * (SIFT_OCTAVE_LAYERS + 3) + (i)])
#define ORI(p, o, i) ((p)->ori[(o) * (SIFT_OCTAVE_LAYERS + 3) + (i)])

typedef struct {
  Keypoint* v;
  int n, cap;
} KeypointArray;

static void kp_push(KeypointArray* a, Keypoint kp)
{
    if (a->n == a->cap) {
        a->cap = a->cap ? a->cap * 2 : 64;
        a->v = realloc(a->v, a->cap * sizeof(Keypoint));
    }
    a->v[a->n++] = kp;
}

// Degrees in [0, 360), like cv::fastAtan2.
static float atan2_deg(float y, float x)
{
    float a = atan2f(y, x) * (float) (180.0 / M_PI);
    return a < 0 ? a + 360 : a;
}

static void build_pyramid(const Image* base, Pyramid* p)
{
    const int layers = SIFT_OCTAVE_LAYERS;
    double sig[SIFT_OCTAVE_LAYERS + 3];
    double k = pow(2.0, 1.0 / layers);
    int min_dim = base->w < base->h ? base->w : base->h;

    // Doubled base image: firstOctave = -1.
    p->n_octaves = (int) lrint(log((double) min_dim) / log(2.0) - 2) + 1;
    p->gauss = calloc(p->n_octaves * (layers + 3), sizeof(Image));
    p->dog = calloc(p->n_octaves * (layers + 2), sizeof(Image));

    sig[0] = SIFT_SIGMA;
    for (int i = 1; i < layers + 3; i++) {
        double sig_prev = pow(k, (double) (i - 1)) * SIFT_SIGMA;
        double sig_total = sig_prev * k;
        sig[i] = sqrt(sig_total * sig_total - sig_prev * sig_prev);
    }

    for (int o = 0; o < p->n_octaves; o++)
        for (int i = 0; i < layers + 3; i++) {
            Image* dst = &GAUSS(p, o, i);
            if (o == 0 && i == 0) {
                *dst = image_new(base->w, base->h);
                memcpy(dst->px, base->px, (size_t) base->w * base->h * sizeof(float));
            }
            else if (i == 0)
                *dst = downscale2(&GAUSS(p, o - 1, layers));
            else {
                const Image* src = &GAUSS(p, o, i - 1);
                *dst = image_new(src->w, src->h);
                gaussian_blur(src, dst, sig[i]);
            }
        }

    // Gradients, computed once here instead of per keypoint sample. Only
    // interior pixels are ever read.
    p->mag = calloc(p->n_octaves * (layers + 3), sizeof(Image));
    p->ori = calloc(p->n_octaves * (layers + 3), sizeof(Image));
    for (int o = 0; o < p->n_octaves; o++)
        for (int i = 1; i <= layers; i++) {
            const Image* g = &GAUSS(p, o, i);
            Image* mag = &MAG(p, o, i);
            Image* ori = &ORI(p, o, i);
            *mag = image_new(g->w, g->h);
            *ori = image_new(g->w, g->h);
            for (int y = 1; y < g->h - 1; y++)
                for (int x = 1; x < g->w - 1; x++) {
                    float dx = AT(*g, x + 1, y) - AT(*g, x - 1, y);
                    float dy = AT(*g, x, y - 1) - AT(*g, x, y + 1);
                    AT(*mag, x, y) = sqrtf(dx * dx + dy * dy);
                    AT(*ori, x, y) = atan2_deg(dy, dx);
                }
        }

    for (int o = 0; o < p->n_octaves; o++)
        for (int i = 0; i < layers + 2; i++) {
            const Image* a = &GAUSS(p, o, i);
            const Image* b = &GAUSS(p, o, i + 1);
            Image* d = &DOG(p, o, i);
            *d = image_new(a->w, a->h);
            for (int j = 0; j < a->w * a->h; j++)
                d->px[j] = b->px[j] - a->px[j];
        }
}

static void free_pyramid(Pyramid* p)
{
    for (int i = 0; i < p->n_octaves * (SIFT_OCTAVE_LAYERS + 3); i++) {
        free(p->gauss[i].px);
        free(p->mag[i].px);
        free(p->ori[i].px);
    }
    free(p->mag);
    free(p->ori);
    for (int i = 0; i < p->n_octaves * (SIFT_OCTAVE_LAYERS + 2); i++)
        free(p->dog[i].px);
    free(p->gauss);
    free(p->dog);
}

// Solve the 3x3 system H x = b (Gaussian elimination, partial pivoting).
static int solve3(double H[3][3], const double b[3], double x[3])
{
    double a[3][4];

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++)
            a[i][j] = H[i][j];
        a[i][3] = b[i];
    }
    for (int c = 0; c < 3; c++) {
        int piv = c;
        for (int r = c + 1; r < 3; r++)
            if (fabs(a[r][c]) > fabs(a[piv][c]))
                piv = r;
        if (fabs(a[piv][c]) < DBL_EPSILON)
            return 0;
        if (piv != c)
            for (int j = 0; j < 4; j++) {
                double t = a[c][j];
                a[c][j] = a[piv][j];
                a[piv][j] = t;
            }
        for (int r = c + 1; r < 3; r++) {
            double f = a[r][c] / a[c][c];
            for (int j = c; j < 4; j++)
                a[r][j] -= f * a[c][j];
        }
    }
    for (int r = 2; r >= 0; r--) {
        double s = a[r][3];
        for (int j = r + 1; j < 3; j++)
            s -= a[r][j] * x[j];
        x[r] = s / a[r][r];
    }
    return 1;
}

// Refine an extremum's position by fitting a quadratic, and reject low
// contrast and edge responses (OpenCV adjustLocalExtrema()).
static int adjust_local_extrema(const Pyramid* p, Keypoint* kp, int o,
                                int* layer, int* r, int* c)
{
    const double img_scale = 1.0 / 255;
    const double deriv_scale = img_scale * 0.5;
    const double second_deriv_scale = img_scale;
    const double cross_deriv_scale = img_scale * 0.25;
    double xi = 0, xr = 0, xc = 0, contr;
    int i;

    for (i = 0; i < SIFT_MAX_INTERP_STEPS; i++) {
        const Image* img = &DOG(p, o, *layer);
        const Image* prev = &DOG(p, o, *layer - 1);
        const Image* next = &DOG(p, o, *layer + 1);
        int y = *r, x = *c;

        double dD[3] = {
            (AT(*img, x + 1, y) - AT(*img, x - 1, y)) * deriv_scale,
            (AT(*img, x, y + 1) - AT(*img, x, y - 1)) * deriv_scale,
            (AT(*next, x, y) - AT(*prev, x, y)) * deriv_scale,
        };
        double v2 = (double) AT(*img, x, y) * 2;
        double dxx = (AT(*img, x + 1, y) + AT(*img, x - 1, y) - v2) * second_deriv_scale;
        double dyy = (AT(*img, x, y + 1) + AT(*img, x, y - 1) - v2) * second_deriv_scale;
        double dss = (AT(*next, x, y) + AT(*prev, x, y) - v2) * second_deriv_scale;
        double dxy = (AT(*img, x + 1, y + 1) - AT(*img, x - 1, y + 1) -
                      AT(*img, x + 1, y - 1) + AT(*img, x - 1, y - 1)) *
                     cross_deriv_scale;
        double dxs = (AT(*next, x + 1, y) - AT(*next, x - 1, y) -
                      AT(*prev, x + 1, y) + AT(*prev, x - 1, y)) *
                     cross_deriv_scale;
        double dys = (AT(*next, x, y + 1) - AT(*next, x, y - 1) -
                      AT(*prev, x, y + 1) + AT(*prev, x, y - 1)) *
                     cross_deriv_scale;
        double H[3][3] = {{dxx, dxy, dxs}, {dxy, dyy, dys}, {dxs, dys, dss}};
        double X[3];

        if (!solve3(H, dD, X))
            return 0;
        xi = -X[2];
        xr = -X[1];
        xc = -X[0];

        if (fabs(xi) < 0.5 && fabs(xr) < 0.5 && fabs(xc) < 0.5)
            break;
        if (fabs(xi) > (double) (INT32_MAX / 3) || fabs(xr) > (double) (INT32_MAX / 3) ||
            fabs(xc) > (double) (INT32_MAX / 3))
            return 0;

        *c += (int) lrint(xc);
        *r += (int) lrint(xr);
        *layer += (int) lrint(xi);

        if (*layer < 1 || *layer > SIFT_OCTAVE_LAYERS || *c < SIFT_IMG_BORDER ||
            *c >= img->w - SIFT_IMG_BORDER || *r < SIFT_IMG_BORDER ||
            *r >= img->h - SIFT_IMG_BORDER)
            return 0;
    }
    if (i >= SIFT_MAX_INTERP_STEPS)
        return 0;

    {
        const Image* img = &DOG(p, o, *layer);
        const Image* prev = &DOG(p, o, *layer - 1);
        const Image* next = &DOG(p, o, *layer + 1);
        int y = *r, x = *c;
        double dD[3] = {
            (AT(*img, x + 1, y) - AT(*img, x - 1, y)) * deriv_scale,
            (AT(*img, x, y + 1) - AT(*img, x, y - 1)) * deriv_scale,
            (AT(*next, x, y) - AT(*prev, x, y)) * deriv_scale,
        };
        double t = dD[0] * xc + dD[1] * xr + dD[2] * xi;

        contr = AT(*img, x, y) * img_scale + t * 0.5;
        if (fabs(contr) * SIFT_OCTAVE_LAYERS < SIFT_CONTRAST_THRESHOLD)
            return 0;

        double v2 = AT(*img, x, y) * 2.0;
        double dxx = (AT(*img, x + 1, y) + AT(*img, x - 1, y) - v2) * second_deriv_scale;
        double dyy = (AT(*img, x, y + 1) + AT(*img, x, y - 1) - v2) * second_deriv_scale;
        double dxy = (AT(*img, x + 1, y + 1) - AT(*img, x - 1, y + 1) -
                      AT(*img, x + 1, y - 1) + AT(*img, x - 1, y - 1)) *
                     cross_deriv_scale;
        double tr = dxx + dyy;
        double det = dxx * dyy - dxy * dxy;

        if (det <= 0 || tr * tr * SIFT_EDGE_THRESHOLD >=
                            (SIFT_EDGE_THRESHOLD + 1) * (SIFT_EDGE_THRESHOLD + 1) * det)
            return 0;
    }

    kp->x = (float) ((*c + xc) * (1 << o));
    kp->y = (float) ((*r + xr) * (1 << o));
    kp->octave = o + (*layer << 8) + ((int) lrint((xi + 0.5) * 255) << 16);
    kp->size = (float) (SIFT_SIGMA * pow(2.0, (*layer + xi) / SIFT_OCTAVE_LAYERS) *
                        (1 << o) * 2);
    return 1;
}

static float orientation_hist(const Image* mag_img, const Image* ori_img,
                              int px, int py, int radius, float sigma,
                              float* hist)
{
    const int n = SIFT_ORI_HIST_BINS;
    float temp[SIFT_ORI_HIST_BINS + 4] = {0};
    float* t = temp + 2;
    const float expf_scale = -1.f / (2.f * sigma * sigma);

    for (int i = -radius; i <= radius; i++) {
        int y = py + i;
        if (y <= 0 || y >= mag_img->h - 1)
            continue;
        for (int j = -radius; j <= radius; j++) {
            int x = px + j;
            if (x <= 0 || x >= mag_img->w - 1)
                continue;
            float w = expf((i * i + j * j) * expf_scale);
            float ori = AT(*ori_img, x, y);
            float mag = AT(*mag_img, x, y);
            int bin = (int) lrintf((n / 360.f) * ori);
            if (bin >= n)
                bin -= n;
            if (bin < 0)
                bin += n;
            t[bin] += w * mag;
        }
    }

    t[-1] = t[n - 1];
    t[-2] = t[n - 2];
    t[n] = t[0];
    t[n + 1] = t[1];
    float maxval = 0;
    for (int i = 0; i < n; i++) {
        hist[i] = (t[i - 2] + t[i + 2]) * (1.f / 16) +
                  (t[i - 1] + t[i + 1]) * (4.f / 16) + t[i] * (6.f / 16);
        if (hist[i] > maxval)
            maxval = hist[i];
    }
    return maxval;
}

static void find_extrema(const Pyramid* p, KeypointArray* out)
{
    const int threshold =
        (int) floor(0.5 * SIFT_CONTRAST_THRESHOLD / SIFT_OCTAVE_LAYERS * 255);
    const int n = SIFT_ORI_HIST_BINS;
    float hist[SIFT_ORI_HIST_BINS];

    for (int o = 0; o < p->n_octaves; o++)
        for (int i = 1; i <= SIFT_OCTAVE_LAYERS; i++) {
            const Image* img = &DOG(p, o, i);
            const Image* prev = &DOG(p, o, i - 1);
            const Image* next = &DOG(p, o, i + 1);

            for (int r = SIFT_IMG_BORDER; r < img->h - SIFT_IMG_BORDER; r++)
                for (int c = SIFT_IMG_BORDER; c < img->w - SIFT_IMG_BORDER; c++) {
                    float val = AT(*img, c, r);
                    int is_ext = fabsf(val) > threshold;

                    for (int dz = -1; is_ext && dz <= 1; dz++) {
                        const Image* s = dz < 0 ? prev : dz > 0 ? next : img;
                        for (int dy = -1; is_ext && dy <= 1; dy++)
                            for (int dx = -1; is_ext && dx <= 1; dx++) {
                                if (!dz && !dy && !dx)
                                    continue;
                                float v = AT(*s, c + dx, r + dy);
                                is_ext = val > 0 ? val >= v : val <= v;
                            }
                    }
                    if (!is_ext)
                        continue;

                    Keypoint kp;
                    int r1 = r, c1 = c, layer = i;
                    if (!adjust_local_extrema(p, &kp, o, &layer, &r1, &c1))
                        continue;

                    float scl_octv = kp.size * 0.5f / (1 << o);
                    float omax = orientation_hist(
                        &MAG(p, o, layer), &ORI(p, o, layer), c1, r1,
                        (int) lrintf(SIFT_ORI_RADIUS * scl_octv),
                        SIFT_ORI_SIG_FCTR * scl_octv, hist);
                    float mag_thr = omax * SIFT_ORI_PEAK_RATIO;

                    for (int j = 0; j < n; j++) {
                        int l = j > 0 ? j - 1 : n - 1;
                        int r2 = j < n - 1 ? j + 1 : 0;
                        if (hist[j] > hist[l] && hist[j] > hist[r2] &&
                            hist[j] >= mag_thr) {
                            float bin = j + 0.5f * (hist[l] - hist[r2]) /
                                                (hist[l] - 2 * hist[j] + hist[r2]);
                            bin = bin < 0 ? n + bin : bin >= n ? bin - n : bin;
                            kp.angle = 360.f - (360.f / n) * bin;
                            if (fabsf(kp.angle - 360.f) < FLT_EPSILON)
                                kp.angle = 0;
                            kp_push(out, kp);
                        }
                    }
                }
        }
}

static void calc_descriptor(const Image* mag_img, const Image* ori_img,
                            float ptx, float pty, float ori, float scl,
                            float* dst)
{
    const int d = SIFT_DESCR_WIDTH, n = SIFT_DESCR_HIST_BINS;
    int px = (int) lrintf(ptx), py = (int) lrintf(pty);
    float cos_t = cosf(ori * (float) (M_PI / 180));
    float sin_t = sinf(ori * (float) (M_PI / 180));
    const float bins_per_rad = n / 360.f;
    const float exp_scale = -1.f / (d * d * 0.5f);
    float hist_width = SIFT_DESCR_SCL_FCTR * scl;
    int radius = (int) lrintf(hist_width * 1.4142135623730951f * (d + 1) * 0.5f);
    float hist[(SIFT_DESCR_WIDTH + 2) * (SIFT_DESCR_WIDTH + 2) *
               (SIFT_DESCR_HIST_BINS + 2)] = {0};

    {
        double diag = sqrt((double) mag_img->w * mag_img->w +
                           (double) mag_img->h * mag_img->h);
        if (radius > diag)
            radius = (int) diag;
    }
    cos_t /= hist_width;
    sin_t /= hist_width;

    // Only samples strictly inside the image contribute (below); skip the
    // rest of the window up front, it is mostly outside for large keypoints.
    const int i_min = radius < py - 1 ? -radius : 1 - py;
    const int i_max = radius < mag_img->h - 2 - py ? radius : mag_img->h - 2 - py;
    const int j_min = radius < px - 1 ? -radius : 1 - px;
    const int j_max = radius < mag_img->w - 2 - px ? radius : mag_img->w - 2 - px;

    // The sample weight exp((c_rot^2 + r_rot^2) * exp_scale) only depends on
    // the rotation-invariant distance (i^2 + j^2) / hist_width^2, so it
    // separates into per-row and per-column factors.
    float* wexp = malloc((2 * radius + 1) * sizeof(float));
    for (int t = -radius; t <= radius; t++)
        wexp[t + radius] = expf(t * t * exp_scale / (hist_width * hist_width));

    for (int i = i_min; i <= i_max; i++)
        for (int j = j_min; j <= j_max; j++) {
            float c_rot = j * cos_t - i * sin_t;
            float r_rot = j * sin_t + i * cos_t;
            float rbin = r_rot + d / 2 - 0.5f;
            float cbin = c_rot + d / 2 - 0.5f;
            int r = py + i, c = px + j;

            if (!(rbin > -1 && rbin < d && cbin > -1 && cbin < d && r > 0 &&
                  r < mag_img->h - 1 && c > 0 && c < mag_img->w - 1))
                continue;

            float w = wexp[i + radius] * wexp[j + radius];
            float obin = (AT(*ori_img, c, r) - ori) * bins_per_rad;
            float mag = AT(*mag_img, c, r) * w;

            int r0 = (int) floorf(rbin);
            int c0 = (int) floorf(cbin);
            int o0 = (int) floorf(obin);
            rbin -= r0;
            cbin -= c0;
            obin -= o0;
            if (o0 < 0)
                o0 += n;
            if (o0 >= n)
                o0 -= n;

            float v_r1 = mag * rbin, v_r0 = mag - v_r1;
            float v_rc11 = v_r1 * cbin, v_rc10 = v_r1 - v_rc11;
            float v_rc01 = v_r0 * cbin, v_rc00 = v_r0 - v_rc01;
            float v_rco111 = v_rc11 * obin, v_rco110 = v_rc11 - v_rco111;
            float v_rco101 = v_rc10 * obin, v_rco100 = v_rc10 - v_rco101;
            float v_rco011 = v_rc01 * obin, v_rco010 = v_rc01 - v_rco011;
            float v_rco001 = v_rc00 * obin, v_rco000 = v_rc00 - v_rco001;

            int idx = ((r0 + 1) * (d + 2) + c0 + 1) * (n + 2) + o0;
            hist[idx] += v_rco000;
            hist[idx + 1] += v_rco001;
            hist[idx + (n + 2)] += v_rco010;
            hist[idx + (n + 3)] += v_rco011;
            hist[idx + (d + 2) * (n + 2)] += v_rco100;
            hist[idx + (d + 2) * (n + 2) + 1] += v_rco101;
            hist[idx + (d + 3) * (n + 2)] += v_rco110;
            hist[idx + (d + 3) * (n + 2) + 1] += v_rco111;
        }

    free(wexp);

    for (int i = 0; i < d; i++)
        for (int j = 0; j < d; j++) {
            int idx = ((i + 1) * (d + 2) + (j + 1)) * (n + 2);
            hist[idx] += hist[idx + n];
            hist[idx + 1] += hist[idx + n + 1];
            for (int k = 0; k < n; k++)
                dst[(i * d + j) * n + k] = hist[idx + k];
        }

    const int len = d * d * n;
    float nrm2 = 0;
    for (int k = 0; k < len; k++)
        nrm2 += dst[k] * dst[k];
    float thr = sqrtf(nrm2) * SIFT_DESCR_MAG_THR;
    nrm2 = 0;
    for (int k = 0; k < len; k++) {
        if (dst[k] > thr)
            dst[k] = thr;
        nrm2 += dst[k] * dst[k];
    }
    nrm2 = SIFT_INT_DESCR_FCTR / fmaxf(sqrtf(nrm2), FLT_EPSILON);
    for (int k = 0; k < len; k++)
        dst[k] = saturate_u8(dst[k] * nrm2);
}

static int kp_cmp(const void* a, const void* b)
{
    const Keypoint* k1 = a;
    const Keypoint* k2 = b;

    if (k1->x != k2->x)
        return k1->x < k2->x ? -1 : 1;
    if (k1->y != k2->y)
        return k1->y < k2->y ? -1 : 1;
    if (k1->size != k2->size)
        return k1->size > k2->size ? -1 : 1;
    if (k1->angle != k2->angle)
        return k1->angle < k2->angle ? -1 : 1;
    return 0;
}

SigfmImgInfo* sigfm_extract(const SigfmPix* pix, int width, int height)
{
    SigfmImgInfo* info = calloc(1, sizeof(SigfmImgInfo));
    unsigned char* enhanced = malloc((size_t) width * height);
    Image img = image_new(width, height);
    KeypointArray kps = {0};
    Pyramid pyr;

    // Enhance local contrast for better SIFT detection.
    if (!clahe(pix, width, height, enhanced))
        memcpy(enhanced, pix, (size_t) width * height);
    for (int i = 0; i < width * height; i++)
        img.px[i] = enhanced[i];
    free(enhanced);

    // Base image: doubled, blurred to SIFT_SIGMA assuming the input already
    // has SIFT_INIT_SIGMA.
    Image base = upscale2(&img);
    free(img.px);
    {
        double sig_diff = sqrt(fmax(SIFT_SIGMA * SIFT_SIGMA -
                                        SIFT_INIT_SIGMA * SIFT_INIT_SIGMA * 4,
                                    0.01));
        gaussian_blur(&base, &base, sig_diff);
    }

    build_pyramid(&base, &pyr);
    free(base.px);
    find_extrema(&pyr, &kps);

    // KeyPointsFilter::removeDuplicatedSorted()
    if (kps.n > 1) {
        qsort(kps.v, kps.n, sizeof(Keypoint), kp_cmp);
        int j = 0;
        for (int i = 1; i < kps.n; i++)
            if (kps.v[i].x != kps.v[j].x || kps.v[i].y != kps.v[j].y ||
                kps.v[i].size != kps.v[j].size || kps.v[i].angle != kps.v[j].angle)
                kps.v[++j] = kps.v[i];
        kps.n = j + 1;
    }

    info->n = kps.n;
    info->kps = kps.v;
    info->desc = calloc((size_t) (kps.n ? kps.n : 1) * DESC_LEN, sizeof(float));
    for (int i = 0; i < kps.n; i++) {
        Keypoint* kp = &info->kps[i];
        int o = kp->octave & 255;
        int layer = (kp->octave >> 8) & 255;
        float angle = 360.f - kp->angle;
        if (fabsf(angle - 360.f) < FLT_EPSILON)
            angle = 0;
        // Keypoints were found on the doubled image (octave index o there
        // is octave o - 1 of the input).
        float scale = 1.f / (1 << o);
        calc_descriptor(&MAG(&pyr, o, layer), &ORI(&pyr, o, layer),
                        kp->x * scale, kp->y * scale,
                        angle, kp->size * scale * 0.5f,
                        info->desc + (size_t) i * DESC_LEN);
        kp->x *= 0.5f;
        kp->y *= 0.5f;
        kp->size *= 0.5f;
    }
    free_pyramid(&pyr);
    return info;
}

void sigfm_free_info(SigfmImgInfo* info)
{
    if (!info)
        return;
    free(info->kps);
    free(info->desc);
    free(info);
}

int sigfm_keypoints_count(SigfmImgInfo* info) { return info->n; }

// ---------------------------------------------------------------------------
// Matching
// ---------------------------------------------------------------------------

typedef struct {
  int x1, y1, x2, y2;
} Match;

typedef struct {
  double cos, sin;
} Angle;

static int match_cmp(const void* a, const void* b)
{
    const Match* m1 = a;
    const Match* m2 = b;

    if (m1->y1 != m2->y1)
        return m1->y1 < m2->y1 ? -1 : 1;
    if (m1->x1 != m2->x1)
        return m1->x1 < m2->x1 ? -1 : 1;
    if (m1->y2 != m2->y2)
        return m1->y2 < m2->y2 ? -1 : 1;
    if (m1->x2 != m2->x2)
        return m1->x2 < m2->x2 ? -1 : 1;
    return 0;
}

static float l2(const float* a, const float* b)
{
    float s = 0;
    for (int k = 0; k < DESC_LEN; k++) {
        float d = a[k] - b[k];
        s += d * d;
    }
    return sqrtf(s);
}

// Ratio-test matches of frame keypoints against enrolled ones, as integer
// point pairs, sorted and deduplicated. Returns the number of ratio-test
// matches before deduplication; *out holds *n_out unique ones.
static int ratio_matches(SigfmImgInfo* frame, SigfmImgInfo* enrolled,
                         Match** out, int* n_out)
{
    Match* m = malloc((size_t) (frame->n ? frame->n : 1) * sizeof(Match));
    int nb = 0;

    for (int i = 0; i < frame->n; i++) {
        float d1 = INFINITY, d2 = INFINITY;
        int best = -1;
        for (int j = 0; j < enrolled->n; j++) {
            float d = l2(frame->desc + (size_t) i * DESC_LEN,
                         enrolled->desc + (size_t) j * DESC_LEN);
            if (d < d1) {
                d2 = d1;
                d1 = d;
                best = j;
            }
            else if (d < d2)
                d2 = d;
        }
        if (best < 0 || enrolled->n < 2)
            continue;
        if (d1 < DISTANCE_MATCH * d2) {
            m[nb].x1 = (int) lrintf(frame->kps[i].x);
            m[nb].y1 = (int) lrintf(frame->kps[i].y);
            m[nb].x2 = (int) lrintf(enrolled->kps[best].x);
            m[nb].y2 = (int) lrintf(enrolled->kps[best].y);
            nb++;
        }
    }

    int n = 0;
    if (nb > 0) {
        qsort(m, nb, sizeof(Match), match_cmp);
        n = 1;
        for (int i = 1; i < nb; i++)
            if (match_cmp(&m[i], &m[n - 1]) != 0)
                m[n++] = m[i];
    }
    *out = m;
    *n_out = n;
    return nb;
}

int sigfm_match_score(SigfmImgInfo* frame, SigfmImgInfo* enrolled)
{
    Match* matches;
    int n;
    int nb_matched = ratio_matches(frame, enrolled, &matches, &n);

    if (nb_matched < MIN_MATCH) {
        free(matches);
        return 0;
    }

    Angle* angles = NULL;
    size_t n_angles = 0, cap = 0;
    for (int j = 0; j < n; j++)
        for (int k = j + 1; k < n; k++) {
            int v1x = matches[j].x1 - matches[k].x1;
            int v1y = matches[j].y1 - matches[k].y1;
            int v2x = matches[j].x2 - matches[k].x2;
            int v2y = matches[j].y2 - matches[k].y2;
            double len1 = sqrt((double) v1x * v1x + (double) v1y * v1y);
            double len2 = sqrt((double) v2x * v2x + (double) v2y * v2y);

            if (1 - fmin(len1, len2) / fmax(len1, len2) <= LENGTH_MATCH) {
                double product = len1 * len2;
                if (n_angles == cap) {
                    cap = cap ? cap * 2 : 64;
                    angles = realloc(angles, cap * sizeof(Angle));
                }
                angles[n_angles].cos =
                    M_PI / 2 + asin((v1x * v2x + v1y * v2y) / product);
                angles[n_angles].sin = acos((v1x * v2y - v1y * v2x) / product);
                n_angles++;
            }
        }
    free(matches);

    if (n_angles < MIN_MATCH) {
        free(angles);
        return 0;
    }

    int count = 0;
    for (size_t j = 0; j < n_angles; j++)
        for (size_t k = j + 1; k < n_angles; k++)
            if (1 - fmin(angles[j].sin, angles[k].sin) /
                        fmax(angles[j].sin, angles[k].sin) <=
                    ANGLE_MATCH &&
                1 - fmin(angles[j].cos, angles[k].cos) /
                        fmax(angles[j].cos, angles[k].cos) <=
                    ANGLE_MATCH)
                count++;
    free(angles);
    return count;
}
