// Offline accuracy evaluation of the goodixtls53xd matching pipeline.
//
// Runs the driver's own preprocessing and template matching
// (drivers/goodixtls/goodix53xd_match.c) over frames recorded with
// GOODIX53XD_DUMP and reports genuine / impostor score distributions, FAR/FRR
// per threshold and an identify simulation, so tuning changes can be measured
// instead of guessed.
//
// Dataset layout: one directory per finger, holding the *.raw16 files dumped
// by the driver (e.g. run the enroll example with
// GOODIX53XD_DUMP=dataset/right-index/s). Files are taken in name order; the
// first --enroll samples of each finger form its template and the rest are
// probes. "*-empty-*" frames (bare sensor) are counted but not matched.
//
// --pipeline selects the preprocessing: "linear" (min/max stretch), "enhance"
// (per-frame fixed-pattern approximation) or "fpn" (subtract the sensor's
// fixed pattern, estimated for each finger as the mean frame of all the
// *other* fingers so no finger's own ridges leak into its estimate) or
// "fpn-tmpl" (estimate it as the mean of each template's own enrollment
// frames, as a driver storing it in the print would, and preprocess every
// probe with the estimate of the template it is compared to) or "driver"
// (exactly what the driver does: each template is one enrollment session in
// the calibration, and a probe is scored with goodix53xd_match_raw_template()
// using the mean of every *other* session).

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

#include <glib.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "drivers/goodixtls/goodix53xd_match.h"

typedef struct {
    char* name;
    // Decoded 12-bit frames, then 8-bit preprocessed images, in file name
    // order
    GPtrArray* raws;
    GPtrArray* images;
    GPtrArray* files;
    guint n_empty;
    // Number of leading images forming the template (0: probes only)
    guint n_template;
    // Fixed pattern estimated from this finger's template ("fpn-tmpl")
    float* fpn;
} Finger;

typedef struct {
    Finger* finger;
    const char* file;
    const Goodix53xdPix* raw;
    SigfmImgInfo* info;
} Probe;

static int opt_enroll = 8;
static int opt_threshold = GOODIX53XD_SIGFM_BEST_MIN;
static char* opt_csv = NULL;
static char* opt_pipeline = NULL;
static gboolean opt_pairs = FALSE;

static GOptionEntry entries[] = {
    {"enroll", 'e', 0, G_OPTION_ARG_INT, &opt_enroll,
     "Samples per finger used as its template (default 8)", "N"},
    {"threshold", 't', 0, G_OPTION_ARG_INT, &opt_threshold,
     "Accept threshold for the per-pair and identify reports", "T"},
    {"csv", 'c', 0, G_OPTION_ARG_FILENAME, &opt_csv,
     "Write every probe x template score to FILE", "FILE"},
    {"pairs", 0, 0, G_OPTION_ARG_NONE, &opt_pairs,
     "Print, for each template sample, its best score against the earlier "
     "samples of the same template (enrollment duplicate detection)",
     NULL},
    {"pipeline", 'p', 0, G_OPTION_ARG_STRING, &opt_pipeline,
     "Preprocessing: linear (default), enhance, fpn, fpn-tmpl or driver",
     "NAME"},
    {NULL},
};

static void finger_free(Finger* f)
{
    g_free(f->name);
    g_free(f->fpn);
    g_ptr_array_unref(f->raws);
    g_ptr_array_unref(f->images);
    g_ptr_array_unref(f->files);
    g_free(f);
}

static Goodix53xdPix* load_raw16(const char* path, GError** error)
{
    g_autofree char* contents = NULL;
    gsize len;
    const gsize size = GOODIX53XD_FRAME_SIZE * sizeof(Goodix53xdPix);

    if (!g_file_get_contents(path, &contents, &len, error))
        return NULL;
    if (len != size) {
        g_set_error(error, G_FILE_ERROR, G_FILE_ERROR_INVAL,
                    "%s: expected %zu bytes, got %zu", path, size, len);
        return NULL;
    }
    Goodix53xdPix* frame = g_malloc(size);
    memcpy(frame, contents, len);
    for (int i = 0; i != GOODIX53XD_FRAME_SIZE; ++i)
        frame[i] = GUINT16_FROM_LE(frame[i]);
    return frame;
}

// Mean of the first n frames of f.
static float* fixed_pattern_of(Finger* f, guint n)
{
    float* fpn = g_new0(float, GOODIX53XD_FRAME_SIZE);

    for (guint j = 0; j < n; j++) {
        const Goodix53xdPix* raw = g_ptr_array_index(f->raws, j);
        for (int k = 0; k < GOODIX53XD_FRAME_SIZE; k++)
            fpn[k] += raw[k];
    }
    for (int k = 0; k < GOODIX53XD_FRAME_SIZE; k++)
        fpn[k] /= MAX(n, 1);
    return fpn;
}

// Mean frame of every finger except `skip`.
static float* fixed_pattern_without(GPtrArray* fingers, Finger* skip)
{
    float* fpn = g_new0(float, GOODIX53XD_FRAME_SIZE);
    guint n = 0;

    for (guint i = 0; i < fingers->len; i++) {
        Finger* f = g_ptr_array_index(fingers, i);
        if (f == skip)
            continue;
        for (guint j = 0; j < f->raws->len; j++, n++) {
            const Goodix53xdPix* raw = g_ptr_array_index(f->raws, j);
            for (int k = 0; k < GOODIX53XD_FRAME_SIZE; k++)
                fpn[k] += raw[k];
        }
    }
    for (int k = 0; k < GOODIX53XD_FRAME_SIZE; k++)
        fpn[k] /= MAX(n, 1);
    return fpn;
}

static gboolean preprocess(GPtrArray* fingers, const char* pipeline)
{
    for (guint i = 0; i < fingers->len; i++) {
        Finger* f = g_ptr_array_index(fingers, i);
        g_autofree float* fpn = NULL;

        if (g_str_equal(pipeline, "fpn"))
            fpn = fixed_pattern_without(fingers, f);
        else if (g_str_equal(pipeline, "driver")) {
            // Matching works on raw frames; images are unused.
            continue;
        }
        else if (g_str_equal(pipeline, "fpn-tmpl")) {
            f->fpn = fixed_pattern_of(f, f->n_template);
            // Probes get preprocessed per template at match time.
            for (guint j = 0; j < f->raws->len; j++) {
                guint8* img = g_malloc(GOODIX53XD_FRAME_SIZE);
                goodix53xd_enhance_frame(g_ptr_array_index(f->raws, j),
                                         f->fpn, img);
                g_ptr_array_add(f->images, img);
            }
            continue;
        }
        else if (!g_str_equal(pipeline, "linear") &&
                 !g_str_equal(pipeline, "enhance"))
            return FALSE;

        for (guint j = 0; j < f->raws->len; j++) {
            const Goodix53xdPix* raw = g_ptr_array_index(f->raws, j);
            guint8* img = g_malloc(GOODIX53XD_FRAME_SIZE);
            if (g_str_equal(pipeline, "linear"))
                goodix53xd_squash_frame_linear(raw, img);
            else
                goodix53xd_enhance_frame(raw, fpn, img);
            g_ptr_array_add(f->images, img);
        }
    }
    return TRUE;
}

static gint compare_strings(gconstpointer a, gconstpointer b)
{
    return strcmp(*(const char* const*) a, *(const char* const*) b);
}

static Finger* load_finger(const char* dir_path, const char* name,
                           GError** error)
{
    g_autoptr(GDir) dir = g_dir_open(dir_path, 0, error);
    if (!dir)
        return NULL;

    g_autoptr(GPtrArray) names = g_ptr_array_new_with_free_func(g_free);
    const char* entry;
    guint n_empty = 0;
    while ((entry = g_dir_read_name(dir))) {
        if (!g_str_has_suffix(entry, ".raw16"))
            continue;
        if (strstr(entry, "-empty-")) {
            n_empty++;
            continue;
        }
        g_ptr_array_add(names, g_strdup(entry));
    }
    g_ptr_array_sort(names, compare_strings);

    Finger* f = g_new0(Finger, 1);
    f->name = g_strdup(name);
    f->raws = g_ptr_array_new_with_free_func(g_free);
    f->images = g_ptr_array_new_with_free_func(g_free);
    f->files = g_ptr_array_new_with_free_func(g_free);
    f->n_empty = n_empty;
    for (guint i = 0; i < names->len; i++) {
        g_autofree char* path =
            g_build_filename(dir_path, g_ptr_array_index(names, i), NULL);
        Goodix53xdPix* raw = load_raw16(path, error);
        if (!raw) {
            finger_free(f);
            return NULL;
        }
        g_ptr_array_add(f->raws, raw);
        g_ptr_array_add(f->files, g_strdup(g_ptr_array_index(names, i)));
    }
    return f;
}

static gint compare_ints(gconstpointer a, gconstpointer b)
{
    return *(const int*) a - *(const int*) b;
}

static void print_distribution(const char* label, GArray* scores)
{
    if (scores->len == 0) {
        printf("  %-10s n=0\n", label);
        return;
    }
    g_array_sort(scores, compare_ints);
    printf("  %-10s n=%-5u min=%-6d p50=%-6d p95=%-6d max=%d\n", label,
           scores->len, g_array_index(scores, int, 0),
           g_array_index(scores, int, scores->len / 2),
           g_array_index(scores, int, (scores->len * 95) / 100),
           g_array_index(scores, int, scores->len - 1));
}

int main(int argc, char** argv)
{
    g_autoptr(GError) error = NULL;
    g_autoptr(GOptionContext) ctx =
        g_option_context_new("DATASET_DIR - evaluate goodixtls53xd matching");
    g_option_context_add_main_entries(ctx, entries, NULL);
    if (!g_option_context_parse(ctx, &argc, &argv, &error)) {
        g_printerr("%s\n", error->message);
        return 1;
    }
    if (argc != 2 || opt_enroll < 2) {
        g_printerr("usage: %s [--enroll N>=2] [--threshold T] DATASET_DIR\n",
                   argv[0]);
        return 1;
    }

    // Load fingers (one subdirectory each).
    g_autoptr(GPtrArray) fingers =
        g_ptr_array_new_with_free_func((GDestroyNotify) finger_free);
    {
        g_autoptr(GDir) dir = g_dir_open(argv[1], 0, &error);
        if (!dir) {
            g_printerr("%s\n", error->message);
            return 1;
        }
        g_autoptr(GPtrArray) names = g_ptr_array_new_with_free_func(g_free);
        const char* entry;
        while ((entry = g_dir_read_name(dir))) {
            g_autofree char* path = g_build_filename(argv[1], entry, NULL);
            if (g_file_test(path, G_FILE_TEST_IS_DIR))
                g_ptr_array_add(names, g_strdup(entry));
        }
        g_ptr_array_sort(names, compare_strings);
        for (guint i = 0; i < names->len; i++) {
            const char* name = g_ptr_array_index(names, i);
            g_autofree char* path = g_build_filename(argv[1], name, NULL);
            Finger* f = load_finger(path, name, &error);
            if (!f) {
                g_printerr("%s\n", error->message);
                return 1;
            }
            if (f->raws->len > (guint) opt_enroll)
                f->n_template = opt_enroll;
            printf("finger %-20s %3u samples (%u template, %u probes), "
                   "%u empty frames\n",
                   f->name, f->raws->len, f->n_template,
                   f->raws->len - f->n_template, f->n_empty);
            if (f->raws->len > 0)
                g_ptr_array_add(fingers, f);
            else
                finger_free(f);
        }
    }

    if (opt_pairs) {
        // Same calibration as "driver": each other template's mean frame.
        g_autoptr(GArray) calib =
            g_array_new(FALSE, FALSE, sizeof(Goodix53xdCalibSession));
        for (guint i = 0; i < fingers->len; i++) {
            Finger* f = g_ptr_array_index(fingers, i);
            float mean[GOODIX53XD_FRAME_SIZE];
            goodix53xd_mean_frame((const Goodix53xdPix* const*) f->raws->pdata,
                                  f->n_template, mean);
            goodix53xd_calib_add(calib, i, mean);
        }
        for (guint i = 0; i < fingers->len; i++) {
            Finger* f = g_ptr_array_index(fingers, i);
            const Goodix53xdPix* const* raws =
                (const Goodix53xdPix* const*) f->raws->pdata;
            float fpn[GOODIX53XD_FRAME_SIZE];
            goodix53xd_calib_fpn_excluding(calib, i, fpn);
            printf("%s: best vs earlier samples (calibrated / uncalibrated):",
                   f->name);
            for (guint k = 1; k < f->n_template; k++)
                printf(" %d/%d",
                       goodix53xd_match_raw_best_single(raws[k], raws, k, fpn),
                       goodix53xd_match_raw_best_single(raws[k], raws, k, NULL));
            printf("\n");
        }
        return 0;
    }

    if (!preprocess(fingers, opt_pipeline ? opt_pipeline : "linear")) {
        g_printerr("unknown pipeline %s\n", opt_pipeline);
        return 1;
    }

    // Extract every probe once.
    g_autoptr(GArray) probes = g_array_new(FALSE, FALSE, sizeof(Probe));
    for (guint i = 0; i < fingers->len; i++) {
        Finger* f = g_ptr_array_index(fingers, i);
        for (guint j = f->n_template; j < f->raws->len; j++) {
            Probe p = {
                .finger = f,
                .file = g_ptr_array_index(f->files, j),
                .raw = g_ptr_array_index(f->raws, j),
                // No preprocessed images in "driver" mode.
                .info = j < f->images->len
                            ? sigfm_extract(g_ptr_array_index(f->images, j),
                                            GOODIX53XD_WIDTH, GOODIX53XD_HEIGHT)
                            : NULL,
            };
            g_array_append_val(probes, p);
        }
    }

    g_autoptr(GPtrArray) templates = g_ptr_array_new();
    for (guint i = 0; i < fingers->len; i++) {
        Finger* f = g_ptr_array_index(fingers, i);
        if (f->n_template > 0)
            g_ptr_array_add(templates, f);
    }
    if (templates->len == 0 || probes->len == 0) {
        g_printerr("need at least one finger with more than %d samples\n",
                   opt_enroll);
        return 1;
    }

    FILE* csv = NULL;
    if (opt_csv) {
        csv = fopen(opt_csv, "w");
        if (!csv) {
            g_printerr("cannot open %s\n", opt_csv);
            return 1;
        }
        fprintf(csv, "probe_finger,probe_file,template_finger,score\n");
    }

    // "driver": one calibration session per template, as enrolled.
    g_autoptr(GArray) calib = NULL;
    const gboolean driver = g_strcmp0(opt_pipeline, "driver") == 0;
    if (driver) {
        calib = g_array_new(FALSE, FALSE, sizeof(Goodix53xdCalibSession));
        for (guint t = 0; t < templates->len; t++) {
            Finger* tf = g_ptr_array_index(templates, t);
            float mean[GOODIX53XD_FRAME_SIZE];
            goodix53xd_mean_frame((const Goodix53xdPix* const*) tf->raws->pdata,
                                  tf->n_template, mean);
            goodix53xd_calib_add(calib, t, mean);
        }
    }

    // scores[p * templates->len + t]
    int* scores = g_new0(int, probes->len * templates->len);
    for (guint p = 0; p < probes->len; p++) {
        Probe* pr = &g_array_index(probes, Probe, p);
        for (guint t = 0; t < templates->len; t++) {
            Finger* tf = g_ptr_array_index(templates, t);
            if (driver) {
                const Goodix53xdPix* const* frames =
                    (const Goodix53xdPix* const*) tf->raws->pdata;
                float fpn[GOODIX53XD_FRAME_SIZE];
                if (!goodix53xd_calib_fpn_excluding(calib, t, fpn))
                    goodix53xd_mean_frame(frames, tf->n_template, fpn);
                int s = goodix53xd_match_raw_template(pr->raw, frames,
                                                      tf->n_template, fpn);
                scores[p * templates->len + t] = s;
                if (csv)
                    fprintf(csv, "%s,%s,%s,%d\n", pr->finger->name, pr->file,
                            tf->name, s);
                continue;
            }
            SigfmImgInfo* info = pr->info;
            if (tf->fpn) {
                guint8 img[GOODIX53XD_FRAME_SIZE];
                goodix53xd_enhance_frame(pr->raw, tf->fpn, img);
                info = sigfm_extract(img, GOODIX53XD_WIDTH, GOODIX53XD_HEIGHT);
            }
            int s = goodix53xd_match_template(
                info, (const guint8* const*) tf->images->pdata,
                tf->n_template);
            if (info != pr->info)
                sigfm_free_info(info);
            scores[p * templates->len + t] = s;
            if (csv)
                fprintf(csv, "%s,%s,%s,%d\n", pr->finger->name, pr->file,
                        tf->name, s);
        }
    }
    if (csv)
        fclose(csv);

    // Per template x probe-finger distributions.
    printf("\nscores per template (rows: probe finger), accept at >= %d\n",
           opt_threshold);
    g_autoptr(GArray) genuine = g_array_new(FALSE, FALSE, sizeof(int));
    g_autoptr(GArray) impostor = g_array_new(FALSE, FALSE, sizeof(int));
    for (guint t = 0; t < templates->len; t++) {
        Finger* tf = g_ptr_array_index(templates, t);
        printf("template %s:\n", tf->name);
        for (guint i = 0; i < fingers->len; i++) {
            Finger* pf = g_ptr_array_index(fingers, i);
            g_autoptr(GArray) cell = g_array_new(FALSE, FALSE, sizeof(int));
            guint accepted = 0;
            for (guint p = 0; p < probes->len; p++) {
                Probe* pr = &g_array_index(probes, Probe, p);
                if (pr->finger != pf)
                    continue;
                int s = scores[p * templates->len + t];
                g_array_append_val(cell, s);
                g_array_append_val(pf == tf ? genuine : impostor, s);
                if (s >= opt_threshold)
                    accepted++;
            }
            if (cell->len == 0)
                continue;
            g_autofree char* label = g_strdup_printf(
                "%s%s", pf->name, pf == tf ? " (gen)" : "");
            printf("  accepted %u/%u ", accepted, cell->len);
            print_distribution(label, cell);
        }
    }

    printf("\noverall:\n");
    print_distribution("genuine", genuine);
    print_distribution("impostor", impostor);

    printf("\nthreshold sweep (per probe x template comparison):\n");
    printf("  %9s %8s %8s\n", "threshold", "FRR%", "FAR%");
    static const int sweep[] = {1,  2,  4,  6,   8,   10,  12,  16,
                                20, 30, 50, 100, 200, 500, 1000};
    for (guint k = 0; k < G_N_ELEMENTS(sweep); k++) {
        guint fr = 0, fa = 0;
        for (guint i = 0; i < genuine->len; i++)
            fr += g_array_index(genuine, int, i) < sweep[k];
        for (guint i = 0; i < impostor->len; i++)
            fa += g_array_index(impostor, int, i) >= sweep[k];
        printf("  %9d %8.2f %8.2f\n", sweep[k],
               genuine->len ? 100.0 * fr / genuine->len : 0.0,
               impostor->len ? 100.0 * fa / impostor->len : 0.0);
    }

    // Identify simulation, mirroring the driver: best-scoring template at or
    // above the threshold wins.
    guint correct = 0, wrong = 0, rejected = 0, false_accept = 0;
    for (guint p = 0; p < probes->len; p++) {
        Probe* pr = &g_array_index(probes, Probe, p);
        Finger* match = NULL;
        int best = 0;
        for (guint t = 0; t < templates->len; t++) {
            int s = scores[p * templates->len + t];
            if (s >= opt_threshold && s > best) {
                best = s;
                match = g_ptr_array_index(templates, t);
            }
        }
        gboolean enrolled = pr->finger->n_template > 0;
        if (match == NULL)
            rejected++;
        else if (match == pr->finger)
            correct++;
        else if (enrolled)
            wrong++;
        else
            false_accept++;
        if (match != NULL && match != pr->finger)
            printf("  identify error: %s/%s -> %s (score %d)\n",
                   pr->finger->name, pr->file, match->name, best);
    }
    printf("\nidentify at >= %d over %u probes: %u correct, %u wrong finger, "
           "%u unenrolled accepted, %u rejected\n",
           opt_threshold, probes->len, correct, wrong, false_accept,
           rejected);

    for (guint p = 0; p < probes->len; p++)
        sigfm_free_info(g_array_index(probes, Probe, p).info);
    g_free(scores);
    return 0;
}
