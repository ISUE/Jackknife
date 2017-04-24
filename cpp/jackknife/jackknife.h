/**
 * Copyright 2017 the University of Central Florida Research Foundation, Inc.
 * All rights reserved.
 * 
 *     Eugene M. Taranta II <etaranta@gmail.com>
 *     Amirreza Samiei <samiei@knights.ucf.edu>
 *     Mehran Maghoumi <mehran@cs.ucf.edu>
 *     Pooya Khaloo <pooya@cs.ucf.edu>
 *     Corey R. Pittman <cpittman@knights.ucf.edu>
 *     Joseph J. LaViola Jr. <jjl@cs.ucf.edu>
 * 
 * Subject to the terms and conditions of the Florida Public Educational
 * Institution non-exclusive software license, this software is distributed 
 * under a non-exclusive, royalty-free, non-sublicensable, non-commercial, 
 * non-exclusive, academic research license, and is distributed without warranty
 * of any kind express or implied. 
 *
 * The Florida Public Educational Institution non-exclusive software license
 * is located at <https://github.com/ISUE/Jackknife/blob/master/LICENSE>.
 */

#pragma once

namespace Jackknife {

/**
 * Jackknife supports a number of different DTW-based measurement techniques.
 * Select an approach that is appropriate for your situation by setting the
 * associated options in this structure.
 */
struct jackknife_blades {

    /**
     * No matter which measure is used, the trajectory is first resampled
     * to a fixed number of points.
     */
    unsigned int resample_cnt        : 8;

    /**
     * Sakoe-Chiba band size. This is one type of constraint that specifies
     * how much warping is allowed between two time series. A common value
     * is 10% of the resample_cnt.
     */
    unsigned int radius              : 8;

    /**
     * Utilize the squared Euclidean distance measure. This flag is mutually
     * exclusive with the inner product flag.
     */
    unsigned int euclidean_distance  : 1;

    /**
     * Z-score normalize the resampled points. If using Euclidean distance,
     * you will normally need to do this. Not recommended for inner product
     * measures though.
     */
    unsigned int z_normalize         : 1;

    /**
     * Extract and normalize the direction vectors between the resampled
     * points, and use the inner product of the direction vectors. Note,
     * this flag is mutually exclusive with the Euclidean distance flag.
     */
    unsigned int inner_product       : 1;

    /**
     * Use lower bounding to cull full DTW evaluations.
     */
    unsigned int lower_bound         : 1;

    /**
     * Utilize the absolute distance correction factor -- the distance
     * traversed by each component in a time series.
     */
    unsigned int cf_abs_distance     : 1;

    /**
     * Utilize the bounding box width correction factor -- the difference
     * between the maximum and minimum value of each component.
     */
    unsigned int cf_bb_widths        : 1;

    /**
     * Setup defaults for inner product measure.
     */
    void set_ip_defaults()
    {
        resample_cnt = 16;
        radius = 2;
        euclidean_distance = 0;
        z_normalize = 0;
        inner_product = 1;
        lower_bound = 1;
        cf_abs_distance = 1;
        cf_bb_widths = 1;
    }

    /**
     * Setup defaults for Euclidean distance measure.
     */
    void set_ed_defaults()
    {
        resample_cnt = 16;
        radius = 2;
        euclidean_distance = 1;
        z_normalize = 1;
        inner_product = 0;
        lower_bound = 1;
        cf_abs_distance = 1;
        cf_bb_widths = 1;
    }
};

typedef struct jackknife_blades jackknife_blades_t;

} // Jackknife namespace

#include <memory>
#include "jackknife_recognizer.h"
