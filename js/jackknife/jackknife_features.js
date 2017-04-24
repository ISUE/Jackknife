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
 * Place to store extracted information and
 * features from a given sample.
 */

/**
 * The main data extraction work is done in the constructor.
 *
 * This is where we:
 *
 *      1) Resample the trajectory to a fixed number
 *         of points (resample_cnt).
 *
 *      2) Calculate the normalized direction vector
 *         between each resampled point or the z-score
 *         normalized points.
 *
 *      3) Extract correction factor related data.
 *
 * It is highly recommended that the sample trajectory points
 * have been smoothed with a low pass filter before calling
 * this function. A simple exponential smoothing filter will
 * probably be adequate for most case.
 */
function JackknifeFeatures(blades, points) {
    /**
     * A trajectory is first resampled to N points,
     * and the points are stored here.
     */
    this.pts = [];

    /**
     * Vecs is named in a general sense. It can mean
     * either m-dimensional points or direction vectors,
     * depending on which DTW measure is being used.
     * In both cases, it's the processed trajectory.
     */
    this.vecs = [];

    // Number of components per point.
    var m = points[0].data.length;

    // resample the trajectory to a fixed number of points
    resample(
        points,
        this.pts,
        blades.resample_cnt);

    // To track the bounding box widths,
    // start with one point and expand.
    var minimum = new Vector(this.pts[0].data);
    var maximum = new Vector(this.pts[0].data);

    // The abs distance traversed starts with zeros.
    this.abs = new Vector(0.0, m);

    // Incrementally extract information.
    for (var ii = 1; ii < blades.resample_cnt; ii++) {
        // In-between point direction vector.
        var vec = this.pts[ii].subtract(this.pts[ii - 1]);

        // Update correction factor features.
        for (var jj = 0; jj < m; jj++) {
            this.abs.data[jj] += Math.abs(vec.data[jj]);

            minimum.data[jj] = Math.min(
                minimum.data[jj],
                this.pts[ii].data[jj]);

            maximum.data[jj] = Math.max(
                maximum.data[jj],
                this.pts[ii].data[jj]);
        }

        // Save the points or direction vectors,
        // depending on the selected measure.
        if (blades.inner_product) {
            this.vecs.push(new Vector(vec.normalize()));
        }
        else if (blades.euclidean_distance) {
            // In ED scenario, make sure not to forget first point as
            // loop starts at 1.
            if (ii == 1) {
                this.vecs.push(new Vector(this.pts[0]));
            }

            this.vecs.push(new Vector(this.pts[ii]));
        } else {
            console.assert(0);
        }
    }

    // Z-score normalize the vecs if required,
    // typically only if using euclidean distance
    if (blades.z_normalize) {
        z_normalize(this.vecs);
    }

    // normalize the correction factor vectors
    this.abs.normalize();
    this.bb = (maximum.subtract(minimum)).normalize();
}
