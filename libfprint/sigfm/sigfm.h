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

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef unsigned char SigfmPix;

/**
 * @brief Keypoints and descriptors of one image, used for matching
 * @details Get one from sigfm_extract() and release it with sigfm_free_info()
 */
typedef struct SigfmImgInfo SigfmImgInfo;

/**
 * @brief Extract SIFT keypoints and descriptors from a grayscale image
 *
 * @param pix Pixels of the image, width * height bytes, row-major
 * @param width Width of the image
 * @param height Height of the image
 * @return SigfmImgInfo* Info to pass to sigfm_match_score()
 */
SigfmImgInfo* sigfm_extract(const SigfmPix* pix, int width, int height);

/**
 * @brief Destroy an SigfmImgInfo
 */
void sigfm_free_info(SigfmImgInfo* info);

/**
 * @brief Score how closely a frame matches another
 *
 * @param frame Print to be checked
 * @param enrolled Canonical print to verify against
 * @return int Score of how closely they match, 0 means always reject
 */
int sigfm_match_score(SigfmImgInfo* frame, SigfmImgInfo* enrolled);

/**
 * @brief Keypoints for an image. Low keypoints generally means the image is
 * low quality for matching
 */
int sigfm_keypoints_count(SigfmImgInfo* info);

#ifdef __cplusplus
}
#endif
