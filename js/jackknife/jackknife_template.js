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

function JackknifeTemplate(blades, sample) {
    this.sample = sample;
    this.gesture_id = sample.gesture_id;

    this.lower = [];
    this.upper = [];

    // Any default value less than zero will ensure if
    // lower bounding is disabled, we don't cull any
    // instance.
    this.lb = -1.0;

    // A default value of one will ensure that the DTW
    // score is not modified when all correction factors
    // are disabled.
    this.cf = 1.0;

    // A default value that will never cause rejection.
    // So if the recognizer is not trained, things will
    // still work as expected.
    this.rejection_threshold = Number.POSITIVE_INFINITY;

    // Extract import information about the sample.
    this.features = new JackknifeFeatures(
        blades,
        sample.trajectory);

    // Now we find the envelop for being able to
    // calculate lower bounds. Go ahead and do this
    // even if the feature is disabled, since it's
    // just a one time thing at training time and
    // is fast.
    var vecs = this.features.vecs;
    var component_cnt = vecs[0].data.length;

    for (var ii = 0; ii < vecs.length; ii++) {
        var maximum = new Vector(Number.NEGATIVE_INFINITY, component_cnt);
        var minimum = new Vector(Number.POSITIVE_INFINITY, component_cnt);
        // For each component find the min and max value
        // within the radius (Sakoe-Chiba band).
        for (var jj = Math.max(0, ii - Math.floor(blades.radius)); jj < Math.min(ii + blades.radius + 1, vecs.length); jj++) {
            for (var kk = 0; kk < component_cnt; kk++) {
                maximum.data[kk] = Math.max(
                    maximum.data[kk],
                    vecs[jj].data[kk]);

                minimum.data[kk] = Math.min(
                    minimum.data[kk],
                    vecs[jj].data[kk]);
            }
        }

        this.upper.push(maximum);
        this.lower.push(minimum);
    }
}

function compareTemplates(t1, t2) {
    return (t1.lb < t2.lb);
}
