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

/**
 * Jackknife supports a number of different DTW-based measurement techniques.
 * Select an approach that is appropriate for your situation by setting the
 * associated options in this structure.
 */
function jackknife_blades() {

    /**
     * No matter which measure is used, the trajectory is first resampled
     * to a fixed number of points.
     */
    this.resample_cnt = 8;

    /**
     * Sakoe-Chiba band size. This is one type of constraint that specifies
     * how much warping is allowed between two time series. A common value
     * is 10% of the resample_cnt.
     */
    this.radius = 8;

    /**
     * Utilize the squared Euclidean distance measure. This flag is mutually
     * exclusive with the inner product flag.
     */
    this.euclidean_distance = 1;

    /**
     * Z-score normalize the resampled points. If using Euclidean distance,
     * you will normally need to do this. Not recommended for inner product
     * measures though.
     */
    this.z_normalize = 1;

    /**
     * Extract and normalize the direction vectors between the resampled
     * points, and use the inner product of the direction vectors. Note,
     * this flag is mutually exclusive with the Euclidean distance flag.
     */
    this.inner_product = 1;

    /**
     * Use lower bounding to cull full DTW evaluations.
     */
    this.lower_bound = 1;

    /**
     * Utilize the absolute distance correction factor -- the distance
     * traversed by each component in a time series.
     */
    this.cf_abs_distance = 1;

    /**
     * Utilize the bounding box width correction factor -- the difference
     * between the maximum and minimum value of each component.
     */
    this.cf_bb_widths = 1;

}

/**
 * Setup defaults for inner product measure.
 */
jackknife_blades.prototype.set_ip_defaults = function() {
    this.resample_cnt = 16;
    this.radius = 2;
    this.euclidean_distance = 0;
    this.z_normalize = 0;
    this.inner_product = 1;
    this.lower_bound = 1;
    this.cf_abs_distance = 1;
    this.cf_bb_widths = 1;
}

/**
 * Setup defaults for Euclidean distance measure.
 */
jackknife_blades.prototype.set_ed_defaults = function() {
    this.resample_cnt = 16;
    this.radius = 2;
    this.euclidean_distance = 1;
    this.z_normalize = 1;
    this.inner_product = 0;
    this.lower_bound = 1;
    this.cf_abs_distance = 1;
    this.cf_bb_widths = 1;
}
