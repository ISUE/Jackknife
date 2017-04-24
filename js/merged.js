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
 * An effective, easy to use low pass filter that is good
 * for continuous data.
 */

/**
 * Constructor that allows you to select the cut off frequency.
 * One Hz may not be optimal but is probably a very good
 * starting point.
 */
function ExponentialMovingAverage(initial_pt, cut_off_frequency_hz) {
    this.pt = new Vector(initial_pt);
    this.cut_off_frequency_hz = cut_off_frequency_hz || 1;
}

ExponentialMovingAverage.prototype.filter = function(pt, duration_s) {
    var tau = 1.0 / (2.0 * Math.PI * this.cut_off_frequency_hz);
    var alpha = 1.0 / (1.0 + tau / duration_s);

    this.pt = (pt.multiply(alpha)).add(this.pt.multiply((1.0 - alpha)));
    return this.pt;
}
/**
 * Copyright 2017 the University of Central Florida Research Foundation, Inc.
 * All rights reserved.
 *
 * Eugene M. Taranta II <etaranta@gmail.com>
 * Amirreza Samiei <samiei@knights.ucf.edu>
 * Mehran Maghoumi <mehran@cs.ucf.edu>
 * Pooya Khaloo <pooya@cs.ucf.edu>
 * Corey R. Pittman <cpittman@knights.ucf.edu>
 * Joseph J. LaViola Jr. <jjl@cs.ucf.edu>
 *
 * Subject to the terms and conditions of this academic license, the Licensor
 * hereby grants, royalty-free, non-sublicensable, non-commercial,
 * non-exclusive, academic research license to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, but not to distribute, and to permit
 * persons to whom the Software is furnished to do so, subject to the following
 * conditions:
 *
 *    * The above copyright notice and this permission notice shall be included
 *      in all copies or substantial portions of the Software. Publications
 *      resulting from use of the software shall cite the authors of the
 *      publication titled, “Jackknife: A Reliable Recognizer with Few Samples
 *      and Many Modalities.”
 *
 *    * Identification of the authors and copyright holder of the software shall
 *      receive attribution, in any reasonable manner.
 *
 *    * Patent and trademark rights of the copyright holder are not licensed
 *      under this academic license.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" BY THE COPYRIGHT HOLDER, WITHOUT WARRANTY OF
 * ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN
 * NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDER BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, OR CONSEQUENTIAL DAMAGES (INCLUDING BUT NOT
 * LIMITED TO LOSS OF USE, DATA OR BUSINESS INTERRUPTION), CLAIM, OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
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
/**
 * Copyright 2017 the University of Central Florida Research Foundation, Inc.
 * All rights reserved.
 *
 * Eugene M. Taranta II <etaranta@gmail.com>
 * Amirreza Samiei <samiei@knights.ucf.edu>
 * Mehran Maghoumi <mehran@cs.ucf.edu>
 * Pooya Khaloo <pooya@cs.ucf.edu>
 * Corey R. Pittman <cpittman@knights.ucf.edu>
 * Joseph J. LaViola Jr. <jjl@cs.ucf.edu>
 *
 * Subject to the terms and conditions of this academic license, the Licensor
 * hereby grants, royalty-free, non-sublicensable, non-commercial,
 * non-exclusive, academic research license to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, but not to distribute, and to permit
 * persons to whom the Software is furnished to do so, subject to the following
 * conditions:
 *
 *    * The above copyright notice and this permission notice shall be included
 *      in all copies or substantial portions of the Software. Publications
 *      resulting from use of the software shall cite the authors of the
 *      publication titled, “Jackknife: A Reliable Recognizer with Few Samples
 *      and Many Modalities.”
 *
 *    * Identification of the authors and copyright holder of the software shall
 *      receive attribution, in any reasonable manner.
 *
 *    * Patent and trademark rights of the copyright holder are not licensed
 *      under this academic license.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" BY THE COPYRIGHT HOLDER, WITHOUT WARRANTY OF
 * ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN
 * NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDER BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, OR CONSEQUENTIAL DAMAGES (INCLUDING BUT NOT
 * LIMITED TO LOSS OF USE, DATA OR BUSINESS INTERRUPTION), CLAIM, OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
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
/**
 * Copyright 2017 the University of Central Florida Research Foundation, Inc.
 * All rights reserved.
 *
 * Eugene M. Taranta II <etaranta@gmail.com>
 * Amirreza Samiei <samiei@knights.ucf.edu>
 * Mehran Maghoumi <mehran@cs.ucf.edu>
 * Pooya Khaloo <pooya@cs.ucf.edu>
 * Corey R. Pittman <cpittman@knights.ucf.edu>
 * Joseph J. LaViola Jr. <jjl@cs.ucf.edu>
 *
 * Subject to the terms and conditions of this academic license, the Licensor
 * hereby grants, royalty-free, non-sublicensable, non-commercial,
 * non-exclusive, academic research license to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, but not to distribute, and to permit
 * persons to whom the Software is furnished to do so, subject to the following
 * conditions:
 *
 *    * The above copyright notice and this permission notice shall be included
 *      in all copies or substantial portions of the Software. Publications
 *      resulting from use of the software shall cite the authors of the
 *      publication titled, “Jackknife: A Reliable Recognizer with Few Samples
 *      and Many Modalities.”
 *
 *    * Identification of the authors and copyright holder of the software shall
 *      receive attribution, in any reasonable manner.
 *
 *    * Patent and trademark rights of the copyright holder are not licensed
 *      under this academic license.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" BY THE COPYRIGHT HOLDER, WITHOUT WARRANTY OF
 * ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN
 * NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDER BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, OR CONSEQUENTIAL DAMAGES (INCLUDING BUT NOT
 * LIMITED TO LOSS OF USE, DATA OR BUSINESS INTERRUPTION), CLAIM, OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * Constructor.
 */
function Jackknife(blades) {
    /**
     * The set of measures and features used in this instance
     * of Jackknife.
     */
    this.blades = blades;

    /**
     * Array of all gesture templates.
     */
    this.templates = [];
}

/**
 * Turn sample into template.
 */
Jackknife.prototype.add_template = function(sample) {
    this.templates.push(new JackknifeTemplate(this.blades, sample));
}

/**
 * Determine gesture class of the given sample.
 * The gesture id is returned.
 *
 * This function can be modified to return an n-best list, but
 * because of early rejection, such a list may not make sense.
 *
 */
Jackknife.prototype.classify = function(trajectory) {

    // Handle non-sample data
    if (trajectory instanceof Sample) {
        return this.classify(trajectory.trajectory);
    }

    // extract important information from candidate sample
    var features = new JackknifeFeatures(this.blades, trajectory);

    // now find the lower bound for this candidate
    // compared against each template.
    // also cache correction factor scores
    var template_cnt = this.templates.length;

    for (var tt = 0; tt < template_cnt; tt++) {
        var cf = 1.0;

        if (this.blades.cf_abs_distance > 0) {
            cf *= 1.0 / Math.max(
                0.01,
                features.abs.dot(this.templates[tt].features.abs));
        }

        if (this.blades.cf_bb_widths > 0) {
            cf *= 1.0 / Math.max(
                0.01,
                features.bb.dot(this.templates[tt].features.bb));
        }

        this.templates[tt].cf = cf;

        if (this.blades.lower_bound > 0) {
            this.templates[tt].lb = cf * this.lower_bound(
                features.vecs,
                this.templates[tt]);
        }
    }

    // Now sort all templates based on their lower bound
    // results. This helps to improve culling later on.
    this.templates.sort(compareTemplates);

    // track best result so far
    var best = Number.POSITIVE_INFINITY;
    var ret = -1;

    for (var tt = 0; tt < template_cnt; tt++) {
        // cull based on lower bound vs rejection threshold
        if (this.templates[tt].lb > this.templates[tt].rejection_threshold)
            continue;

        // cull based on lower bound vs best score
        if (this.templates[tt].lb > best)
            continue;

        // forced to do full evaluation
        var score = this.templates[tt].cf;

        // note that the vecs can be either direction vectors
        // or points, depending on selected measure
        score *= this.DTW(
            features.vecs,
            this.templates[tt].features.vecs);
        // only accept if below rejection threshold
        if (score > this.templates[tt].rejection_threshold)
            continue;

        // save, if best result
        if (score < best) {
            best = score;
            ret = this.templates[tt].gesture_id;
        }
    }

    return ret;
}


/**
 * Learn a rejection threshold for each template.
 * Call after all templates have been added.
 *
 */
Jackknife.prototype.train = function(gpsr_n, gpsr_r, beta) {
    var template_cnt = this.templates.length;
    var distributions = [];
    var synthetic = [];

    var worst_score = 0.0;

    //
    // Create negative samples.
    //
    for (var ii = 0; ii < 1000; ii++) {
        synthetic.length = 0
        // Splice two samples together
        // to create one negative sample.
        for (var jj = 0; jj < 2; jj++) {
            var tt = Math.floor(Math.random() * template_cnt % template_cnt);

            var s = this.templates[tt].sample;
            var len = s.trajectory.length;

            var start = Math.floor(Math.random() * (len / 2) % (len / 2));

            for (var kk = 0; kk < len / 2; kk++) {
                synthetic.push(new Vector(s.trajectory[start + kk]));
            }
        }

        var features = new JackknifeFeatures(
            this.blades,
            synthetic);

        // and score it
        for (var tt = 0; tt < template_cnt; tt++) {
            var score = this.DTW(
                features.vecs,
                this.templates[tt].features.vecs);

            if (worst_score < score)
                worst_score = score;

            if (ii > 50)
                distributions[tt].add_negative_score(score);
        }

        // Generate a few samples to get an estimate
        // of worst possible score.
        if (ii != 50)
            continue;

        // allocate distributions
        for (var tt = 0; tt < template_cnt; tt++) {
            distributions.push(
                new Distributions(
                    worst_score,
                    1000));
        }
    }

    //
    // Create positive examples.
    //
    for (var tt = 0; tt < template_cnt; tt++) {
        for (var ii = 0; ii < 1000; ii++) {
            synthetic.length = 0;

            // Create a synthetic variation of the sample.
            gpsr(
                this.templates[tt].sample.trajectory,
                synthetic,
                gpsr_n,
                0.25,
                gpsr_r);

            var features = new JackknifeFeatures(this.blades, synthetic);
            // and score it
            var score = this.DTW(features.vecs, this.templates[tt].features.vecs);
            distributions[tt].add_positive_score(score);
        }
    }

    //
    // Now extract the rejection thresholds.
    //
    for (var tt = 0; tt < template_cnt; tt++) {
        var threshold = distributions[tt].rejection_threshold(beta);
        this.templates[tt].rejection_threshold = threshold;
    }
}

/**
 * The DTW algorithm. Awesome!
 */
Jackknife.prototype.DTW = function(v1, v2) {
    var cost = new Array(v1.length + 1);
    for (var i = 0; i < cost.length; i++)
        cost[i] = new Array(v2.length + 1);


    // initialize matrix
    for (var ii = 0; ii <= v1.length; ii++) {
        for (var jj = 0; jj <= v2.length; jj++) {
            cost[ii][jj] = Number.POSITIVE_INFINITY;

        }
    }

    cost[0][0] = 0.0;

    // using DP to find solution
    for (var ii = 1; ii <= v1.length; ii++) {
        for (var jj = Math.max(1, ii - Math.floor(this.blades.radius));
            jj <= Math.min(Math.floor(v2.length), ii + Math.floor(this.blades.radius));
            jj++) {
            // pick minimum cost path (neighbor) to
            // extend to this ii, jj element
            var minimum = Math.min(
                Math.min(
                    cost[ii - 1][jj], // repeat v1 element
                    cost[ii][jj - 1]), // repeat v2 element
                cost[ii - 1][jj - 1]); // don't repeat either

            cost[ii][jj] = minimum;

            if (this.blades.inner_product) {
                cost[ii][jj] += 1.0 - v1[ii - 1].dot(v2[jj - 1]);
            } else if (this.blades.euclidean_distance) {
                cost[ii][jj] += v1[ii - 1].l2norm2(v2[jj - 1]);
            } else {
                assert(0);
            }
        }
    }
    //console.log(cost[v1.length][v2.length])
    return cost[v1.length][v2.length];
}

/**
 * Find the lower bound score for a candidate
 * against a given template.
 */
Jackknife.prototype.lower_bound = function(vecs, t) {
    var lb = 0.0; // lower bound
    var component_cnt = vecs[0].data.length;

    for (var ii = 0; ii < vecs.length; ii++) {
        var cost = 0.0;

        for (var jj = 0; jj < component_cnt; jj++) {
            if (this.blades.inner_product) {
                if (vecs[ii].data[jj] < 0.0) {
                    cost += vecs[ii].data[jj] * t.lower[ii].data[jj];
                } else {
                    cost += vecs[ii].data[jj] * t.upper[ii].data[jj];
                }
            } else if (this.blades.euclidean_distance) {
                var diff = 0.0;

                if (vecs[ii].data[jj] < t.lower[ii].data[jj]) {
                    diff = vecs[ii].data[jj] - t.lower[ii].data[jj];
                } else if (vecs[ii].data[jj] > t.upper[ii].data[jj]) {
                    diff = vecs[ii].data[jj] - t.upper[ii].data[jj];
                }

                cost += (diff * diff);
            } else {
                assert(0);
            }
        }

        // inner products are bounded
        if (this.blades.inner_product) {
            cost = 1.0 - Math.min(1.0, Math.max(-1.0, cost));
        }

        lb += cost;
    }

    return lb;
}
/**
 * Copyright 2017 the University of Central Florida Research Foundation, Inc.
 * All rights reserved.
 *
 * Eugene M. Taranta II <etaranta@gmail.com>
 * Amirreza Samiei <samiei@knights.ucf.edu>
 * Mehran Maghoumi <mehran@cs.ucf.edu>
 * Pooya Khaloo <pooya@cs.ucf.edu>
 * Corey R. Pittman <cpittman@knights.ucf.edu>
 * Joseph J. LaViola Jr. <jjl@cs.ucf.edu>
 *
 * Subject to the terms and conditions of this academic license, the Licensor
 * hereby grants, royalty-free, non-sublicensable, non-commercial,
 * non-exclusive, academic research license to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, but not to distribute, and to permit
 * persons to whom the Software is furnished to do so, subject to the following
 * conditions:
 *
 *    * The above copyright notice and this permission notice shall be included
 *      in all copies or substantial portions of the Software. Publications
 *      resulting from use of the software shall cite the authors of the
 *      publication titled, “Jackknife: A Reliable Recognizer with Few Samples
 *      and Many Modalities.”
 *
 *    * Identification of the authors and copyright holder of the software shall
 *      receive attribution, in any reasonable manner.
 *
 *    * Patent and trademark rights of the copyright holder are not licensed
 *      under this academic license.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" BY THE COPYRIGHT HOLDER, WITHOUT WARRANTY OF
 * ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN
 * NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDER BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, OR CONSEQUENTIAL DAMAGES (INCLUDING BUT NOT
 * LIMITED TO LOSS OF USE, DATA OR BUSINESS INTERRUPTION), CLAIM, OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
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
/**
 * Copyright 2017 the University of Central Florida Research Foundation, Inc.
 * All rights reserved.
 *
 * Eugene M. Taranta II <etaranta@gmail.com>
 * Amirreza Samiei <samiei@knights.ucf.edu>
 * Mehran Maghoumi <mehran@cs.ucf.edu>
 * Pooya Khaloo <pooya@cs.ucf.edu>
 * Corey R. Pittman <cpittman@knights.ucf.edu>
 * Joseph J. LaViola Jr. <jjl@cs.ucf.edu>
 *
 * Subject to the terms and conditions of this academic license, the Licensor
 * hereby grants, royalty-free, non-sublicensable, non-commercial,
 * non-exclusive, academic research license to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, but not to distribute, and to permit
 * persons to whom the Software is furnished to do so, subject to the following
 * conditions:
 *
 *    * The above copyright notice and this permission notice shall be included
 *      in all copies or substantial portions of the Software. Publications
 *      resulting from use of the software shall cite the authors of the
 *      publication titled, “Jackknife: A Reliable Recognizer with Few Samples
 *      and Many Modalities.”
 *
 *    * Identification of the authors and copyright holder of the software shall
 *      receive attribution, in any reasonable manner.
 *
 *    * Patent and trademark rights of the copyright holder are not licensed
 *      under this academic license.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" BY THE COPYRIGHT HOLDER, WITHOUT WARRANTY OF
 * ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN
 * NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDER BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, OR CONSEQUENTIAL DAMAGES (INCLUDING BUT NOT
 * LIMITED TO LOSS OF USE, DATA OR BUSINESS INTERRUPTION), CLAIM, OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
 
/**
 * Helper structure to manage statistics when calculating
 * thresholds.
 */

/**
 * The range [0.0, max_score] is divided into bin_cnt bins.
 */
function Distributions(max_score, bin_cnt) {
    /**
     * Distribution of negative sample scores
     */
    this.neg = new Vector(0, bin_cnt);

    /**
     * Distribution of positive sample scores
     */
    this.pos = new Vector(0, bin_cnt);

    /**
     * Specifies the range of the histogram: [0.0, max_score].
     */
    this.max_score = max_score;
}

/**
 * Convert score into a bin number.
 */
Distributions.prototype.bin = function(score) {
    return Math.min(Math.floor(score * (this.neg.length / this.max_score)), this.neg.length - 1);
}

/**
 * Increment negative score histogram.
 */
Distributions.prototype.add_negative_score = function(score) {
    this.neg.data[this.bin(score)] += 1;
}

/**
 * Increment positive score histogram.
 */
Distributions.prototype.add_positive_score = function(score) {
    this.pos.data[this.bin(score)] += 1;
}

/**
 * Estimate a rejection threshold based on a target
 * F-score. Beta is the parameter that controls the
 * balance between false positives and false negatives.
 *
 * More information can be found on Wikipedia:
 *      https://en.wikipedia.org/wiki/F1_score
 *
 * Specifically F_Beta "measures the effectiveness of retrieval with
 * respect to a user who attaches Beta times as much importance to
 * recall as precision."
 */
Distributions.prototype.rejection_threshold = function(beta) {
    // Convert negative distribution into CDF.
    this.neg = this.neg.divide(this.neg.sum());
    this.neg.cumulative_sum();
    console.assert(Math.abs(this.neg.data[this.neg.data.length - 1] - 1.0) < .00001);

    // Convert positive distribution into CDF.
    this.pos = this.pos.divide(this.pos.sum());
    this.pos.cumulative_sum();
    console.assert(Math.abs(this.pos.data[this.pos.data.length - 1] - 1.0) < .00001);

    var alpha = 1.0 / (1.0 + beta * beta);

    // Percentage of positives
    // that are actually true positive.
    var precision = this.pos.divide((this.pos.add(this.neg)));

    // Percentage of true positives.
    var recall = this.pos;

    var best_score = 0.0;
    var best_idx = -1;

    for (var ii = 0; ii < this.neg.length; ii++) {
        var E, f_score;

        // This equation comes from Wikipedia
        E = (alpha / precision.data[ii]) + ((1.0 - alpha) / recall.data[ii]);
        f_score = 1.0 / E;

        // save best f-score result
        if (f_score > best_score) {
            best_score = f_score;
            best_idx = ii;
        }
    }

    var ret = best_idx + 0.5;
    ret *= this.max_score / this.neg.length;
    return ret;
}
/**
 * Copyright 2017 the University of Central Florida Research Foundation, Inc.
 * All rights reserved.
 *
 * Eugene M. Taranta II <etaranta@gmail.com>
 * Amirreza Samiei <samiei@knights.ucf.edu>
 * Mehran Maghoumi <mehran@cs.ucf.edu>
 * Pooya Khaloo <pooya@cs.ucf.edu>
 * Corey R. Pittman <cpittman@knights.ucf.edu>
 * Joseph J. LaViola Jr. <jjl@cs.ucf.edu>
 *
 * Subject to the terms and conditions of this academic license, the Licensor
 * hereby grants, royalty-free, non-sublicensable, non-commercial,
 * non-exclusive, academic research license to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, but not to distribute, and to permit
 * persons to whom the Software is furnished to do so, subject to the following
 * conditions:
 *
 *    * The above copyright notice and this permission notice shall be included
 *      in all copies or substantial portions of the Software. Publications
 *      resulting from use of the software shall cite the authors of the
 *      publication titled, “Jackknife: A Reliable Recognizer with Few Samples
 *      and Many Modalities.”
 *
 *    * Identification of the authors and copyright holder of the software shall
 *      receive attribution, in any reasonable manner.
 *
 *    * Patent and trademark rights of the copyright holder are not licensed
 *      under this academic license.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" BY THE COPYRIGHT HOLDER, WITHOUT WARRANTY OF
 * ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN
 * NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDER BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, OR CONSEQUENTIAL DAMAGES (INCLUDING BUT NOT
 * LIMITED TO LOSS OF USE, DATA OR BUSINESS INTERRUPTION), CLAIM, OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * Component-wise z-score normalize each point of a time series.
 */
function z_normalize(points) {
    var n = points.length;
    var m = points[0].size();

    var mean = new Vector(0.0, m);
    var variance = new Vector(0.0, m);

    // Estimate the component-wise mean.
    for (var ii = 0; ii < n; ii++) {
        mean = mean.add(points[ii]);

    }

    mean = mean.divide(n);

    // Estimate the component-wise variance
    for (var ii = 0; ii < n; ii++) {
        for (var jj = 0; jj < m; jj++) {
            var diff = points[ii].data[jj] - mean.data[jj];
            variance.data[jj] += (diff * diff);
        }
    }

    variance = variance.divide(n - 1);

    // now convert variance to standard deviation
    for (var ii = 0; ii < m; ii++) {
        variance.data[ii] = Math.sqrt(variance.data[ii]);
    }

    // last, z-score normalize all points
    for (var ii = 0; ii < n; ii++) {
        points[ii] = (points[ii].subtract(mean)).divide(variance);

    }

}

/**
 * Given a time series trajectory, determine the path length
 * through the m-dimensional space, where m is the number of
 * components per point.
 */
function path_length(points) {
    var ret = 0.0;

    for (var ii = 1; ii < points.length; ii++) {
        ret += points[ii].l2norm(points[ii - 1]);
    }
    return ret;
}

/**
 * Resample a trajectory either uniformly or stochastically. To perform uniform
 * resampling, set variance to zero. To perform stochastic resampling, set
 * variance greater than zero.
 */
function resample(points, ret, n, variance) {
    variance = variance || 0;

    var path_distance = path_length(points);
    var intervals = new Vector(n - 1);

    var interval;
    var ii;

    // uniform resampling
    if (variance == 0.0) {
        intervals.setAllElementsTo(1.0 / (n  - 1));
    }
    // stochastic resampling
    else {
        for (ii = 0; ii < n - 1; ii++) {
            var b = Math.sqrt(12 * variance);
            var rr = Math.random();
            intervals.data[ii] = 1.0 + rr * b;
        }
        intervals = intervals.divide(intervals.sum());
    }

    console.assert(Math.abs(intervals.sum() - 1) < 0.00001);

    //
    // Core resampling logic.
    //
    var remaining_distance = path_distance * intervals.elementAt(0);
    var prev = points[0];

    ret.push(new Vector(points[0]));
    ii = 1;

    while(ii < points.length)
    {
        var distance = points[ii].l2norm(prev);

        if(distance < remaining_distance)
        {
            prev = points[ii];
            remaining_distance -= distance;
            ii ++;
            continue;
        }

        // Now we need to interpolate between the last point
        // and the current point.
        var ratio = remaining_distance / distance;

        // If two points are close together, the distance
        // may be close to zero and the ratio becomes inf.
        // Also, if the remaining_distance is close to zero
        // and the distance is close to zero, then 0/0 = nan.
        if(ratio > 1.0 || isNaN(ratio)) ratio = 1.0;

        ret.push(
            InterpolateVectors(
                prev,
                points[ii],
                ratio));

        // Because of rounding errors, we may hit the end
        // just a bit too soon.
        if(ret.length == n)
        {
            return;
        }

        // The previous point becomes the new
        // point we just created.
        prev = ret[ret.length - 1];

        // Setup for the next interval.
        remaining_distance = path_distance * intervals.elementAt(ret.length - 1);
    }

    // Because of rounding errors, we may not reach the
    // last point, which is very common.
    if (ret.length < n)
    {
        ret.push(points[ii - 1]);
    }

    console.assert(ret.length == n);
}

function gpsr(points, ret, n, variance, remove_cnt) {
    var resampled = [];
    resample(points, resampled, n + remove_cnt, variance);

    // Remove random points to simulate cutting corners.
    for (var ii = 0; ii < remove_cnt; ii++) {
        var remove_idx = Math.random()*65535;
        remove_idx = Math.floor(remove_idx % Math.floor(n + remove_cnt - ii));
        resampled.splice(remove_idx, 1);
    }

    // Construct synthetic variation.
    var m = resampled[0].data.length;
    ret.push(new Vector(0, m));

    for (var ii = 1; ii < resampled.length; ii++) {
        var delta = resampled[ii].subtract(resampled[ii - 1]);
        ret.push(delta.normalize());
    }

}
/**
 * Copyright 2017 the University of Central Florida Research Foundation, Inc.
 * All rights reserved.
 *
 * Eugene M. Taranta II <etaranta@gmail.com>
 * Amirreza Samiei <samiei@knights.ucf.edu>
 * Mehran Maghoumi <mehran@cs.ucf.edu>
 * Pooya Khaloo <pooya@cs.ucf.edu>
 * Corey R. Pittman <cpittman@knights.ucf.edu>
 * Joseph J. LaViola Jr. <jjl@cs.ucf.edu>
 *
 * Subject to the terms and conditions of this academic license, the Licensor
 * hereby grants, royalty-free, non-sublicensable, non-commercial,
 * non-exclusive, academic research license to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, but not to distribute, and to permit
 * persons to whom the Software is furnished to do so, subject to the following
 * conditions:
 *
 *    * The above copyright notice and this permission notice shall be included
 *      in all copies or substantial portions of the Software. Publications
 *      resulting from use of the software shall cite the authors of the
 *      publication titled, “Jackknife: A Reliable Recognizer with Few Samples
 *      and Many Modalities.”
 *
 *    * Identification of the authors and copyright holder of the software shall
 *      receive attribution, in any reasonable manner.
 *
 *    * Patent and trademark rights of the copyright holder are not licensed
 *      under this academic license.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" BY THE COPYRIGHT HOLDER, WITHOUT WARRANTY OF
 * ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN
 * NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDER BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, OR CONSEQUENTIAL DAMAGES (INCLUDING BUT NOT
 * LIMITED TO LOSS OF USE, DATA OR BUSINESS INTERRUPTION), CLAIM, OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * A class to hold one sample, perhaps loaded from a file.
 */
var Sample = function(subject_id, gesture_id, example_id) {
    this.subject_id = subject_id || 0;
    this.gesture_id = gesture_id || 0;
    this.example_id = example_id || 0;

    this.trajectory = [];
}

/**
 * Multiple series are combined into a single trajectory.
 * In most cases this isn't relevant, but it may impact
 * multistroke gesture recognition, such as for hand
 * written pen or touch gestures. For those cases, you may
 * want to consider using Vatavu et al.'s $P recognizer.
 */
Sample.prototype.add_trajectory = function(trajectory) {
    for (var ii = 0; ii < trajectory.length; ii++) {
        this.trajectory.push(trajectory[ii]);
    }
}
/**
 * Copyright 2017 the University of Central Florida Research Foundation, Inc.
 * All rights reserved.
 *
 * Eugene M. Taranta II <etaranta@gmail.com>
 * Amirreza Samiei <samiei@knights.ucf.edu>
 * Mehran Maghoumi <mehran@cs.ucf.edu>
 * Pooya Khaloo <pooya@cs.ucf.edu>
 * Corey R. Pittman <cpittman@knights.ucf.edu>
 * Joseph J. LaViola Jr. <jjl@cs.ucf.edu>
 *
 * Subject to the terms and conditions of this academic license, the Licensor
 * hereby grants, royalty-free, non-sublicensable, non-commercial,
 * non-exclusive, academic research license to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, but not to distribute, and to permit
 * persons to whom the Software is furnished to do so, subject to the following
 * conditions:
 *
 *    * The above copyright notice and this permission notice shall be included
 *      in all copies or substantial portions of the Software. Publications
 *      resulting from use of the software shall cite the authors of the
 *      publication titled, “Jackknife: A Reliable Recognizer with Few Samples
 *      and Many Modalities.”
 *
 *    * Identification of the authors and copyright holder of the software shall
 *      receive attribution, in any reasonable manner.
 *
 *    * Patent and trademark rights of the copyright holder are not licensed
 *      under this academic license.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" BY THE COPYRIGHT HOLDER, WITHOUT WARRANTY OF
 * ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN
 * NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDER BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, OR CONSEQUENTIAL DAMAGES (INCLUDING BUT NOT
 * LIMITED TO LOSS OF USE, DATA OR BUSINESS INTERRUPTION), CLAIM, OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * This class is a wrapper around the std::vector class that
 * provides some mathematics support for points and vectors.
 */
var Vector = function(x, size) {
    this.length = size || 0;

    /**
     * Initialize an m-component vector, setting each
     * component to a constant value.
     */
    if ((size > 0) && (Number.isInteger(x) ||
            x == Number.POSITIVE_INFINITY ||
            x == Number.NEGATIVE_INFINITY)) {
        this.data = new Array(size)
        for (var ii = 0; ii < size; ii++) {
            this.data[ii] = x;
        }
    }
    /**
     * Create an uninitialized m-component vector.
     */
    else if (Number.isInteger(x)) {
        this.data = new Array(x);
        this.length = x;
    }
    /**
     * Initialize a new vector as a copy of another vector.
     */
    else if (x instanceof Vector) {
        this.data = x.data;
        this.length = x.data.length;
    }
    /**
     * Initialize a new vector from an existing data array.
     */
    else if (x instanceof Array) {
        this.data = new Array(x.length);
        for (var ii = 0; ii < x.length; ii++) {
            this.data[ii] = x[ii];
        }
        this.length = x.length;
    }
}

/**
 * Interpolate between two vectors, where 0 <= t <= 1,
 * t = 0 = a, and t = 1 = b.
 */
function InterpolateVectors(a, b, t) {
    var m = a.length;
    var n = b.length;

    console.assert(m == n, 'Different sized arrays to interpolate');

    var data = new Array(m);
    for (var ii = 0; ii < m; ii++) {
        data[ii] = (1.0 - t) * a.data[ii];
        data[ii] += t * b.data[ii];
    }

    return new Vector(data);
}

Vector.prototype.size = function() {
    return this.length;
}

Vector.prototype.elementAt = function(idx) {
    return this.data[idx];
}

Vector.prototype.setAllElementsTo = function(rhs) {
    for (var ii = 0; ii < this.length; ii++) {
        this.data[ii] = rhs;
    }
}

Vector.prototype.negative = function() {
    var m = this.length;
    var vec = new Vector(m);

    for (var ii = 0; ii < m; ii++) {
        vec.data[ii] = -this.data[ii];
    }

    return vec;
}

Vector.prototype.add = function(rhs) {
    var m = this.length;
    var vec = new Vector(m);

    for (var ii = 0; ii < m; ii++) {
        vec.data[ii] = this.data[ii] + rhs.data[ii];
    }

    return vec;
}

Vector.prototype.subtract = function(rhs) {
    var m = this.length;
    var vec = new Vector(m);

    for (var ii = 0; ii < m; ii++) {
        vec.data[ii] = this.data[ii] - rhs.data[ii];
    }

    return vec;
}

Vector.prototype.divide = function(rhs) {
    if (rhs instanceof Vector) {
        var m = this.length;
        var vec = new Vector(m);

        for (var ii = 0; ii < m; ii++) {
            vec.data[ii] = this.data[ii] / rhs.data[ii];
        }

        return vec;
    } else {
        var m = this.length;
        var vec = new Vector(m);

        for (var ii = 0; ii < m; ii++) {
            vec.data[ii] = this.data[ii] / rhs;
        }

        return vec;
    }
}

Vector.prototype.multiply = function(rhs) {
    if (rhs instanceof Vector) {
        var m = this.length;
        var vec = new Vector(m);

        for (var ii = 0; ii < m; ii++) {
            vec.data[ii] = this.data[ii] * rhs.data[ii];
        }

        return vec;
    } else {
        var m = this.length;
        var vec = new Vector(m);

        for (var ii = 0; ii < m; ii++) {
            vec.data[ii] = this.data[ii] * rhs;
        }

        return vec;
    }
}

Vector.prototype.equals = function(rhs) {
    var m = this.length;
    var ret = 1;

    for (var ii = 0; ii < m; ii++) {
        ret &= (this.data[ii] == rhs.data[ii]);
    }

    return ret;
}

Vector.prototype.l2norm2 = function(other) {
    var ret = 0;

    for (var ii = 0; ii < this.length; ii++) {
        var delta = this.data[ii] - other.data[ii];
        ret += delta * delta;
    }

    return ret;
}

Vector.prototype.l2norm = function(other) {
    return Math.sqrt(this.l2norm2(other));
}

Vector.prototype.magnitude = function() {
    var ret = 0;
    for (var ii = 0; ii < this.length; ii++) {
        ret += this.data[ii] * this.data[ii];
    }

    return Math.sqrt(ret);
}

Vector.prototype.normalize = function() {
    var len = this.magnitude();

    for (var ii = 0; ii < this.length; ii++) {
        this.data[ii] = this.data[ii] / len;
    }
    return this;
}

Vector.prototype.dot = function(rhs) {
    var m = this.length;
    var ret = 0;

    for (var ii = 0; ii < m; ii++) {
        ret += this.data[ii] * rhs.data[ii];
    }

    return ret;
}

Vector.prototype.sum = function() {
    var ret = 0;

    for (var ii = 0; ii < this.length; ii++) {
        ret += this.data[ii];
    }

    return ret;
}

Vector.prototype.cumulative_sum = function() {
    var ret = 0;

    for (var ii = 0; ii < this.data.length; ii++) {
        ret += this.data[ii];
        this.data[ii] = ret;
    }

}
/**
 * Copyright 2017 the University of Central Florida Research Foundation, Inc.
 * All rights reserved.
 *
 * Eugene M. Taranta II <etaranta@gmail.com>
 * Amirreza Samiei <samiei@knights.ucf.edu>
 * Mehran Maghoumi <mehran@cs.ucf.edu>
 * Pooya Khaloo <pooya@cs.ucf.edu>
 * Corey R. Pittman <cpittman@knights.ucf.edu>
 * Joseph J. LaViola Jr. <jjl@cs.ucf.edu>
 *
 * Subject to the terms and conditions of this academic license, the Licensor
 * hereby grants, royalty-free, non-sublicensable, non-commercial,
 * non-exclusive, academic research license to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, but not to distribute, and to permit
 * persons to whom the Software is furnished to do so, subject to the following
 * conditions:
 *
 *    * The above copyright notice and this permission notice shall be included
 *      in all copies or substantial portions of the Software. Publications
 *      resulting from use of the software shall cite the authors of the
 *      publication titled, “Jackknife: A Reliable Recognizer with Few Samples
 *      and Many Modalities.”
 *
 *    * Identification of the authors and copyright holder of the software shall
 *      receive attribution, in any reasonable manner.
 *
 *    * Patent and trademark rights of the copyright holder are not licensed
 *      under this academic license.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" BY THE COPYRIGHT HOLDER, WITHOUT WARRANTY OF
 * ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN
 * NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDER BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, OR CONSEQUENTIAL DAMAGES (INCLUDING BUT NOT
 * LIMITED TO LOSS OF USE, DATA OR BUSINESS INTERRUPTION), CLAIM, OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * Enumeration of error types.
 */
var TRUE_POSITIVE = 0;
var FALSE_NEGATIVE = 1;
var FALSE_POSITIVE = 2;
var TRUE_NEGATIVE = 3;

function results_t() {
    this.recall = 0;
    this.precision = 0;
    this.fall_out = 0;
    this.f1 = 0;

    /**
     * Place to store confusion matrices analysis.
     */
    this.print = function() {
        console.log("Recall:    " + (this.recall * 100.0).toFixed(2)+ "\%");
        console.log("Precision: " + (this.precision * 100.0).toFixed(2)+ "\%");
        console.log("Fall Out:  " + (this.fall_out * 100.0).toFixed(2)+ "\%");
        console.log("F1 score:  " + (this.f1 * 100.0).toFixed(2) + "\%");
    }
}

/**
 * Helper structure for tracking confusion matrices.
 */
function ConfusionMatrices(ds) {
    /**
     * A confusion matrix for each gesture.
     */
    this.ConfusionMatrices = [];

    /**
     * Number of results added;
     */
    this.entries = 0;

    var gesture_cnt = ds.gesture_cnt();
    for (var ii = 0; ii < gesture_cnt; ii++) {
        this.ConfusionMatrices.push(new Vector(0, 4));
    }

    /**
     * Add a single result to confusion matrices.
     */
    this.add_result = function(expected_id, detected_id) {

        if (expected_id instanceof ConfusionMatrices) {
            this.add_cm(expected_id);
            return;
        }
        console.assert(expected_id < this.ConfusionMatrices.length);
        console.assert(detected_id < this.ConfusionMatrices.length);

        this.entries++;
        for (var gesture_id = 0; gesture_id < this.ConfusionMatrices.length; gesture_id++) {
            if (gesture_id == expected_id) {
                // This is a special case for our sessions logic,
                // where if a gesture was already detected,
                // we treat a second detection as a false positive.
                if (detected_id == -2)
                    this.ConfusionMatrices[gesture_id].data[FALSE_POSITIVE] += 1.0;
                else if (gesture_id == detected_id)
                    this.ConfusionMatrices[gesture_id].data[TRUE_POSITIVE] += 1.0;
                else
                    this.ConfusionMatrices[gesture_id].data[FALSE_NEGATIVE] += 1.0;
            } else {
                if (gesture_id == detected_id)
                    this.ConfusionMatrices[gesture_id].data[FALSE_POSITIVE] += 1.0;
                else
                    this.ConfusionMatrices[gesture_id].data[TRUE_NEGATIVE] += 1.0;
            }
        }
    }

    /**
     * Use this to average together confusion matrices
     * over different tests.
     */
    this.add_cm = function(other) {
        console.assert(this.ConfusionMatrices.length == other.ConfusionMatrices.length);
        this.entries++;

        for (var ii = 0; ii < other.ConfusionMatrices.length; ii++) {
            var sum = other.ConfusionMatrices[ii].sum();
            this.ConfusionMatrices[ii] = this.ConfusionMatrices[ii].add(other.ConfusionMatrices[ii].divide(sum));
        }
    }
    /**
     * Extract statistics from the confusion matrices.
     */
    this.results = function() {
        var cm = new Vector(0, 4);

        for (var ii = 0; ii < this.ConfusionMatrices.length; ii++) {
            cm = cm.add(this.ConfusionMatrices[ii]);
        }
        var ret = new results_t();
        ret.recall = cm.data[TRUE_POSITIVE];
        ret.recall /= (cm.data[TRUE_POSITIVE] + cm.data[FALSE_NEGATIVE]);

        ret.precision = cm.data[TRUE_POSITIVE];
        ret.precision /= (cm.data[TRUE_POSITIVE] + cm.data[FALSE_POSITIVE]);

        ret.fall_out = cm.data[FALSE_POSITIVE];
        ret.fall_out /= (cm.data[FALSE_POSITIVE] + cm.data[TRUE_NEGATIVE]);

        ret.f1 = 2.0 * cm.data[TRUE_POSITIVE];
        ret.f1 /= (2.0 * cm.data[TRUE_POSITIVE] + cm.data[FALSE_POSITIVE] + cm.data[FALSE_NEGATIVE]);

        return ret;
    }
}
/**
 * Copyright 2017 the University of Central Florida Research Foundation, Inc.
 * All rights reserved.
 *
 * Eugene M. Taranta II <etaranta@gmail.com>
 * Amirreza Samiei <samiei@knights.ucf.edu>
 * Mehran Maghoumi <mehran@cs.ucf.edu>
 * Pooya Khaloo <pooya@cs.ucf.edu>
 * Corey R. Pittman <cpittman@knights.ucf.edu>
 * Joseph J. LaViola Jr. <jjl@cs.ucf.edu>
 *
 * Subject to the terms and conditions of this academic license, the Licensor
 * hereby grants, royalty-free, non-sublicensable, non-commercial,
 * non-exclusive, academic research license to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, but not to distribute, and to permit
 * persons to whom the Software is furnished to do so, subject to the following
 * conditions:
 *
 *    * The above copyright notice and this permission notice shall be included
 *      in all copies or substantial portions of the Software. Publications
 *      resulting from use of the software shall cite the authors of the
 *      publication titled, “Jackknife: A Reliable Recognizer with Few Samples
 *      and Many Modalities.”
 *
 *    * Identification of the authors and copyright holder of the software shall
 *      receive attribution, in any reasonable manner.
 *
 *    * Patent and trademark rights of the copyright holder are not licensed
 *      under this academic license.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" BY THE COPYRIGHT HOLDER, WITHOUT WARRANTY OF
 * ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN
 * NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDER BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, OR CONSEQUENTIAL DAMAGES (INCLUDING BUT NOT
 * LIMITED TO LOSS OF USE, DATA OR BUSINESS INTERRUPTION), CLAIM, OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * A class to help manage all samples of a dataset.
 */
function Dataset() {
    /**
     * Enumeration of gesture string names.
     */
    this.gestures = [];

    /**
     * Enumeration of subject names.
     */
    this.subjects = [];

    /**
     * List of all samples from the dataset.
     */
    this.samples = [];

    /**
     * Use samples_by_gesture[gesture_id] to get all the samples
     * from a particular gesture class.
     */
    this.samples_by_gesture = [];
}

/**
 * Add gesture name to database if it does not
 * already exist and returns its enumerated id.
 */
Dataset.prototype.add_gesture = function(gname) {
    var ii;

    for (ii = 0; ii < this.gestures.length; ii++) {
        if (this.gestures[ii] == gname)
            return ii;
    }

    // push gesture name into list and make room
    // for sample list in samples_by_gesture, which
    // will be populated later add sample is called
    this.gestures.push(gname);
    this.samples_by_gesture.push([]);
    return ii;
}

/**
 * Add subject to database if it does not already exist
 * and returns their enumerated id.
 */
Dataset.prototype.add_subject = function(sname) {
    var ii;

    for (ii = 0; ii < this.subjects.length; ii++) {
        if (this.subjects[ii] == sname)
            return ii;
    }

    this.subjects.push(sname);
    return ii;
}

/**
 * Add a new sample to the dataset. The gesture name
 * and subject must have already been added.
 */
Dataset.prototype.add_sample = function(sample, subject_id, gesture_id) {
    this.samples.push(sample);
    console.assert(gesture_id < this.samples_by_gesture.length);
    this.samples_by_gesture[gesture_id].push(sample);
}

Dataset.prototype.sample_cnt = function() {
    return this.samples.length;
}

Dataset.prototype.gesture_cnt = function() {
    return this.gestures.length;
}

Dataset.prototype.subject_cnt = function() {
    return this.subjects.length;
}

Dataset.prototype.gesture_name_to_id = function(gname) {
    for (var ii = 0; ii < this.gestures.length; ii++) {
        if (this.gestures[ii] == gname) {
            return ii;
        }
    }

    console.assert(0);
    return -1;
}

/**
 *
 */
Dataset.prototype.dump_catalog = function() {
    console.log("Subject Count: " + this.subjects.length +
        "\nSample Count: " + this.samples.length +
        "\nGesture Count " + this.samples_by_gesture.length + "\n");

    for (var ii = 0; ii < this.samples_by_gesture.length; ii++) {
        var gname = gestures[ii];
        console.log(gname + ": " + this.samples_by_gesture[ii].length + "\n");
    }
}


/**
 *
 */
load_dataset = function(path) {
    var fs = require('fs');

    var subject_paths = [];

    var ret = new Dataset();
    var directories = fs.readdirSync(path);
    console.assert(directories.length != 0);

    for (var ii = 0; ii < directories.length; ii++) {
        if (directories[ii].substring(0, 4) != "Sub_") {
            continue;
        }

        var spath = path;
        spath = spath + directories[ii];
        subject_paths.push(spath)
    }

    //
    // We want the subject names in order.
    //
    subject_paths.sort();

    //
    // Load the subject directories.
    //
    for (var ii = 0; ii < subject_paths.length; ii++) {
        load_subject_dataset(
            ret,
            subject_paths[ii]);
    }

    return ret;
}

/**
 * Recurse through a *subject* directory and load up all of the samples:
 * <path>/sub_* /gesture name/ex_*
 *
 * Also, all subject and gesture names are recorded and enumerated.
 *
 * This function can be called multiple times with to build a single data set.
 * First call with dataset=NULL to allocate a new Dataset and then pass in
 * on subsequent calls.
 */
load_subject_dataset = function(ret, subject_path) {
    var fs = require('fs');
    var gestures = [];

    //
    // Create a new data set if required.
    //
    if (ret == null) {
        ret = new Dataset();
    }

    //
    // Get subject name from path string ".../Sub_<subject_name>".
    //
    var idx;
    idx = subject_path.search("Sub_");

    if (idx < 0) {
        console.log("exit 0\n");
        return ret;
    }

    idx += 4;


    //
    // Add subject to database, get sub id.
    //
    var tmp = subject_path.substring(idx);
    var subject_name = tmp;
    var subject_id = ret.add_subject(subject_name);

    //
    // Find all gesture directories.
    //
    directories = fs.readdirSync(subject_path);
    console.assert(directories.length != 0);

    for (var ii = 0; ii < directories.length; ii++) {
        var gname = directories[ii];
        var gesture_path = subject_path + "/" + gname;

        var entry_stat = fs.statSync(gesture_path);

        if (!entry_stat.isDirectory()) {
            continue;
        }

        gestures.push({
            path: gesture_path,
            gesturename: gname
        });
    }
    //
    // Ensure gesture names are in order.
    //
    gestures.sort(function(a, b) {
        a.gesturename, b.gesturename
    });

    for (var ii = 0; ii < gestures.length; ii++) {
        var sample_paths = [];

        // Get a gesture ID.
        var gesture_id = ret.add_gesture(gestures[ii].gesturename);

        //
        // Now find all examples.
        //
        directories = fs.readdirSync(gestures[ii].path);
        console.assert(directories.length != 0);

        for (var jj = 0; jj < directories.length; jj++) {
            var sname = directories[jj];
            var sample_path = gestures[ii].path + "/" + sname;

            if (!fs.existsSync(sample_path))
            {
                console.log("HERE")
                continue;
            }
            if (sname.substring(0, 2) != "ex")
            {
                console.log("No, HERE")
                continue;
            }
            sample_paths.push(sample_path);
        }


        //
        // Again, we prefer to have samples sorted.
        //
        sample_paths.sort();
        for (var sample_no = 0; sample_no < sample_paths.length; sample_no++) {
            //
            // finally! load a sample file
            //
            var sample = load_sample_file(
                sample_paths[sample_no],
                subject_id,
                gesture_id);

            if (sample == null)
                continue;

            ret.add_sample(
                sample,
                subject_id,
                gesture_id);
        }
    }

    return;
}

/**
 *
 */
load_sample_file = function(path, subject_id, gesture_id) {
    //Load the file into the lines object
    var lines = require('fs').readFileSync(path, 'utf-8')
        .split('\n')
        .filter(Boolean);

    var gname = "";
    var pt_cnt_str;
    var hash_line;

    gname = lines.shift();
    pt_cnt_str = lines.shift();
    hash_line = lines.shift();

    console.assert(hash_line.trim() ==  '####');

    var pt_cnt = parseInt(pt_cnt_str);

    var ret = new Sample(subject_id, gesture_id, 0);

    pt = [];
    points = [];

    while (true) {
        // read next line in file
        var line = lines.shift();

        // check for empty line or separator
        if (!line || line.trim() == "####") {
            // add pt to trajectory
            //console.log(pt);
            points.push(new Vector(pt));
            pt.length = 0;
            //console.log(points[points.length-1].data);

            // end of file
            if (!line)
                break;

            continue;
        }


        // else read in x, y, z
        var parts = line.split(',');
        pt.push(parseFloat(parts[0]));
        pt.push(parseFloat(parts[1]));
        pt.push(parseFloat(parts[2]));
    }

    ret.add_trajectory(points);
    return ret;
}
/**
 * Copyright 2017 the University of Central Florida Research Foundation, Inc.
 * All rights reserved.
 *
 * Eugene M. Taranta II <etaranta@gmail.com>
 * Amirreza Samiei <samiei@knights.ucf.edu>
 * Mehran Maghoumi <mehran@cs.ucf.edu>
 * Pooya Khaloo <pooya@cs.ucf.edu>
 * Corey R. Pittman <cpittman@knights.ucf.edu>
 * Joseph J. LaViola Jr. <jjl@cs.ucf.edu>
 *
 * Subject to the terms and conditions of this academic license, the Licensor
 * hereby grants, royalty-free, non-sublicensable, non-commercial,
 * non-exclusive, academic research license to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, but not to distribute, and to permit
 * persons to whom the Software is furnished to do so, subject to the following
 * conditions:
 *
 *    * The above copyright notice and this permission notice shall be included
 *      in all copies or substantial portions of the Software. Publications
 *      resulting from use of the software shall cite the authors of the
 *      publication titled, “Jackknife: A Reliable Recognizer with Few Samples
 *      and Many Modalities.”
 *
 *    * Identification of the authors and copyright holder of the software shall
 *      receive attribution, in any reasonable manner.
 *
 *    * Patent and trademark rights of the copyright holder are not licensed
 *      under this academic license.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" BY THE COPYRIGHT HOLDER, WITHOUT WARRANTY OF
 * ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN
 * NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDER BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, OR CONSEQUENTIAL DAMAGES (INCLUDING BUT NOT
 * LIMITED TO LOSS OF USE, DATA OR BUSINESS INTERRUPTION), CLAIM, OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

var device_type_t = {
    KINECT: 0,
    LEAP_MOTION: 1
};

/**
 * A simple user independent test.
 */
function user_indepedent_test(device) {
    // First, load all training data for one device type.
    var path;

    if (device == device_type_t.KINECT)
        path = "../datasets/jk2017/kinect/training/";
    else if (device == device_type_t.LEAP_MOTION)
        path = "../datasets/jk2017/leap_motion/training/";
    else
        assert(0);
    console.log("Loading Dataset");
    var ds = load_dataset(path);
    console.log("Loaded")
    var subject_cnt = ds.subject_cnt();
    var sample_cnt = ds.sample_cnt();
    console.log(subject_cnt + " " + sample_cnt)

    var cm = new ConfusionMatrices(ds);

    // iterate through all subjects
    for (var subject_id = 0; subject_id < subject_cnt; subject_id++) {
        var cm_individual = new ConfusionMatrices(ds);
        console.log("Participant: " + subject_id + "\n");

        // train a recognizer with the selected subject
        var blades = new jackknife_blades();

        //blades.set_ed_defaults();
        blades.set_ip_defaults();


        var jk = new Jackknife(blades);

        var template_cnt = 0.0;

        for (var sample_id = 0; sample_id < sample_cnt; sample_id++) {
            var sample = ds.samples[sample_id];

            if (sample.subject_id != subject_id)
                continue;

            jk.add_template(ds.samples[sample_id]);
        }

        // only train the recognize if you need
        // to set a rejection criteria
        // jk.train(4, 2, 1.0);

        // test the recognizer with all other samples
        for (var sample_id = 0; sample_id < sample_cnt; sample_id++) {
            var sample = ds.samples[sample_id];
            if (sample.subject_id == subject_id)
                continue;
            //gettimeofday(&start, NULL);
            var gid = jk.classify(sample);
            //gettimeofday(&end, NULL);

            //double seconds  = (double)(end.tv_sec  - start.tv_sec);
            //double useconds = (double)(end.tv_usec - start.tv_usec);
            //time_us += (seconds*1.0e6 + useconds);
            cm_individual.add_result(
                sample.gesture_id,
                gid);
        }

        cm.add_result(cm_individual);

        var results = cm_individual.results();
        results.print();
        console.log("\n");

        // std::cout << "result: " << accuracy << "%, ";
        // std::cout << "time: " << time_us / test_cnt;
        // std::cout << " us" << std::endl;
        // std::cout << std::endl;
    }

    console.log("Aggregate Results: \n");
    var results = cm.results();
    results.print();
    console.log("\n");
}
/**
 * Copyright 2017 the University of Central Florida Research Foundation, Inc.
 * All rights reserved.
 *
 * Eugene M. Taranta II <etaranta@gmail.com>
 * Amirreza Samiei <samiei@knights.ucf.edu>
 * Mehran Maghoumi <mehran@cs.ucf.edu>
 * Pooya Khaloo <pooya@cs.ucf.edu>
 * Corey R. Pittman <cpittman@knights.ucf.edu>
 * Joseph J. LaViola Jr. <jjl@cs.ucf.edu>
 *
 * Subject to the terms and conditions of this academic license, the Licensor
 * hereby grants, royalty-free, non-sublicensable, non-commercial,
 * non-exclusive, academic research license to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, but not to distribute, and to permit
 * persons to whom the Software is furnished to do so, subject to the following
 * conditions:
 *
 *    * The above copyright notice and this permission notice shall be included
 *      in all copies or substantial portions of the Software. Publications
 *      resulting from use of the software shall cite the authors of the
 *      publication titled, “Jackknife: A Reliable Recognizer with Few Samples
 *      and Many Modalities.”
 *
 *    * Identification of the authors and copyright holder of the software shall
 *      receive attribution, in any reasonable manner.
 *
 *    * Patent and trademark rights of the copyright holder are not licensed
 *      under this academic license.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" BY THE COPYRIGHT HOLDER, WITHOUT WARRANTY OF
 * ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN
 * NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDER BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, OR CONSEQUENTIAL DAMAGES (INCLUDING BUT NOT
 * LIMITED TO LOSS OF USE, DATA OR BUSINESS INTERRUPTION), CLAIM, OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * Size of each frame stored in a session file in bytes.
 */
var g_frame_size_b = (5 + 63) * 4;

/**
 *
 */
function configuartion_parameters_t(device) {
    /**
     * The input device sampling rate.
     */
    this.fps = 30;

    /**
     * The maximum amount of data (in seconds) that is collected
     * and passed to the recognizer. Since the buffer is cleared
     * when a gesture is recognized, the buffer may be shorter.
     */
    this.sliding_window_s = 2.0;

    /**
     * Sliding window converted into frames based on FPS.
     */
    this.sliding_window_frame_cnt = 60.0;

    /**
     * The recognizer is called once per this many frames.
     */
    this.update_interval = 5.0;

    /**
     * A gesture has to have the best Jackknife score this many
     * times before being officially recognized by this application.
     */
    this.repeat_cnt = 3.0;

    if (device == device_type_t.KINECT) {
        this.fps = 30;
        this.sliding_window_s = 2.0;
        this.update_interval = 5;
        this.repeat_cnt = 3;
    } else if (device == device_type_t.LEAP_MOTION) {
        this.fps = 30;
        this.sliding_window_s = 4.0;
        this.update_interval = 10;
        this.repeat_cnt = 4;
    } else {
        assert(0);
    }

    this.sliding_window_frame_cnt = Math.floor(this.fps * this.sliding_window_s);
}

/**
 * The gesture ID in the dataset does not match the gesture
 * IDs used in the session files. So this helper function
 * does a conversion.
 */
function convert_gesture_id(ds, gesture_id) {
    var gname = [
        "jab_right",
        "jab_left",
        "kick_right",
        "kick_left",
        "hook_right",
        "hook_left",
        "uppercut_right",
        "uppercut_left",
        "cartwheel_right",
        "cartwheel_left",
        "push",
        "sidekick_right",
        "sidekick_left",
        "duck",
        "explode",
        "fist_2_circles",
        "index_2_circles",
        "knock_x3",
        "scissors",
        "sideways",
        "rock_out",
        "love",
        "redrum",
    ];

    console.assert(gesture_id >= 1);
    console.assert(gesture_id <= 23);

    return ds.gesture_name_to_id(gname[gesture_id - 1]);
}

/**
 *
 */
function bad_gesture(gesture_id) {
    // redrum
    if (gesture_id == 23 || gesture_id === "redrum")
        return true;

    return false;
}

/**
 *
 */
function get_participant_list(device) {
    if (device == device_type_t.KINECT) {
        var ret = [
            100, 101, 102, 103,
            104, 105, 106, 107,
            108, 109, 110, 111,
            200, 201, 202, 203,
            204, 205, 206, 207,
            000,
        ];

        return ret;
    } else if (device == device_type_t.LEAP_MOTION) {
        var ret = [
            300, 301, 302, 303,
            304, 305, 306, 307,
            400, 401, 402, 403,
            404, 500, 501, 502,
            503, 504, 505, 506,
            000,
        ];

        return ret;
    }

    assert(0);
    return NULL;
}

/**
 * Read in the next frame from the binary input session file.
 */
function Frame(text, position) {
    /**
     * The expected gesture: the gesture that the
     * participant should execute.
     */
    this.gesture_id = -1;

    /**
     * Enumerated command id.
     */
    this.cmd_id = -1;

    /**
     * Currently not used (originates from another project
     * but takes up space in output file).
     */
    this.time_remaining_s = -1;

    /**
     * Position of command on screen.
     */
    this.text_pos_x = -1;

    /**
     * Position of command on screen.
     */
    this.text_pos_y = -1;

    /**
     * True if any component in vector is NAN or infinity.
     * This can happen if the input device loses tracking.
     */
    this.bad_pt = false;

    this.gesture_id = text.getInt32(position.index,true);
    position.index+=4;

    this.cmd_id = text.getInt32(position.index,true);
    position.index+=4;

    this.time_remaining_s = text.getFloat32(position.index,true);
    position.index+=4;

    this.text_pos_x = text.getFloat32(position.index,true);
    position.index+=4;

    this.text_pos_y = text.getFloat32(position.index,true);
    position.index+=4;

    /**
     * The actual input! :)
     */
    this.pt = new Vector(63);
    for (var ii = 0; ii < 63; ii++) {
        var component;
        this.pt.data[ii] = text.getFloat32(position.index,true);
        position.index+=4;
        if (!Number.isFinite(this.pt.data[ii])) {
            this.bad_pt = true;
        }
    }
}

/**
 *
 */
function CommandResults(command_id, expected_id) {
    /**
     * The enumerated command ID.
     */
    this.command_id = command_id;

    /**
     * Gesture that is expected.
     */
    this.expected_id = expected_id;

    /**
     * List of gestures that are detected during this
     * command window.
     */
    this.detected_ids = []
    /**
     * If the participant made a mistake or tracking was lost,
     * we repeat the gesture request in the next command window.
     * So this command should be ignored.
     */
    this.ignore = false;
}

/**
 * Collect any gestures detection during this
 * command duration.
 */
CommandResults.prototype.add = function(detected_id) {
    console.assert(detected_id >= 0);
    this.detected_ids.push(detected_id);
}

/**
 * After the command is complete, update the matrices
 * with detected gestures.
 */
CommandResults.prototype.update_confusion_matrices = function(cm) {
    var found = false;

    for (var ii = 0; ii < this.detected_ids.length; ii++) {
        var detected_id = this.detected_ids[ii];

        if (found && this.expected_id == detected_id) {
            // treat as false positive
            // because we've already detected
            // the gesture once
            cm.add_result(
                this.expected_id, -2);
            continue;
        }

        cm.add_result(
            this.expected_id,
            detected_id);

        // mark that we've found this gesture
        found = found || (detected_id == this.expected_id);
    }

    // false negative
    if (!found) {
        cm.add_result(
            this.expected_id, -1);
    }
}

/**
 * Local overload of version in dataset.cpp.
 */
function load_subject_dataset_session(device, subject_id) {
    // build the subject's dataset path
    var padded_sid = ("00000000" + subject_id).substr(-3);
    var subject_path = "../datasets/jk2017/" +
        (device == device_type_t.KINECT ? "kinect" : "leap_motion") +
        "/training/Sub_U" + padded_sid;
    // load the subject's dataset
    var ds = new Dataset();
    load_subject_dataset(
        ds,
        subject_path);
    return ds;
}

/**
 * Read in all frames from a binary session file.
 */
function load_session(device, subject_id, frames) {
    // Build the subject's session path.
    var padded_sid = ("00000000" + subject_id).substr(-3);
    var session_path = "../datasets/jk2017/" +
        (device == device_type_t.KINECT ? "kinect" : "leap_motion") +
        "/sessions/U" + padded_sid;

    // Open file at end.
    var fs = require("fs");

    var contents = new DataView(toArrayBuffer(fs.readFileSync(session_path)));
    var size = contents.byteLength;

    var position = {index: 0};
    // convert to frame_cnt
    var frame_cnt = size / g_frame_size_b;
    for (var ii = 0; ii < frame_cnt; ii++) {

        var frame = new Frame(contents,position);
        // There is an issue where the main program ran at 60fps,
        // but the Kinect only samples at 30fps. Some a number of
        // readings are duplicates and need to be removed.
        if (ii > 0) {
            var idx = frames.length - 1;
            var distance = frame.pt.l2norm(frames[idx].pt);
            if (distance == 0.0)
                continue;
        }
        frames.push(frame);
    }

}

function toArrayBuffer(buf) {
    var ab = new ArrayBuffer(buf.length);
    var view = new Uint8Array(ab);
    for (var i = 0; i < buf.length; ++i) {
        view[i] = buf[i];
    }
    return ab;
}

/**
 * Load a participant's dataset and session.
 * Train the recognizer with the training data
 * and run the video through. See what happens...
 */
function evaluate_session(
    device,
    subject_id) {
    // Load up the training dataset.
    var ds = load_subject_dataset_session(
        device,
        subject_id);

    // Load up the session.
    var frames = [];
    load_session(
        device,
        subject_id,
        frames);

    // Create a new recognizer.
    var blades = new jackknife_blades();
    blades.set_ip_defaults();
    var jk = new Jackknife(blades);

    // Train the recognizer, without 'bad' gestures.
    for (var ii = 0; ii < ds.samples.length; ii++) {
        var gesture_id = ds.samples[ii].gesture_id;
        var gesture_name = ds.gestures[gesture_id];
        if (bad_gesture(gesture_name))
            continue;
        jk.add_template(ds.samples[ii]);
    }

    // Get device and application parameters
    // based on the device type.
    var params = new configuartion_parameters_t(device);

    // We originally used n=4, r=2 for Kinect data
    // and n=6, r=2 for Leap Motion data, but
    // here we just set the average. There is barely
    // any effect on the results.
    jk.train(6, 2, 1.00);

    // Play session video through
    // the recognizer.
    var buffer = [];
    var detections = [];
    var cmds = [];
    var last_cmd_id = -1;
    var next_update = params.update_interval;

    var frame_no = 0;

    var filter = new ExponentialMovingAverage(frames[0].pt);
    var pt;

    for (var ii = 0; ii < frames.length; ii++) {
        // skip this frame if its bad
        if (frames[ii].bad_pt) {
            continue;
        }

        // Low pass filter the input.
        // Note, we originally didn't smooth the data,
        // so results now are a little higher than in
        // the paper.
        pt = filter.filter(
            frames[ii].pt,
            1 / params.fps);
        //pt = frames[ii].pt;
        frame_no += 1;

        // start a new command
        if (frames[ii].cmd_id != last_cmd_id) {
            last_cmd_id = frames[ii].cmd_id;

            var gid = convert_gesture_id(
                ds,
                frames[ii].gesture_id);

            var cmd = new CommandResults(
                frames[ii].cmd_id,
                gid);

            if (bad_gesture(frames[ii].gesture_id))
                cmd.ignore = true;

            cmds.push(cmd);
        }

        // This buffering approach is really
        // inefficient, but since this off-line,
        // performance is not important.
        buffer.push(pt);
        if (buffer.length > params.sliding_window_frame_cnt)
            buffer.shift();

        // We need to have a couple points before
        // calling the recognizer.
        if (buffer.length < 2)
            continue;

        // Wait a few frames again before trying
        // to recognize again.
        if (frame_no < next_update)
            continue;

        next_update = frame_no + params.update_interval;

        // Run the recognizer.
        var gesture_id = jk.classify(buffer);

        // Add recognition result.
        detections.push(gesture_id);
        if (detections.length > params.repeat_cnt)
            detections.shift();

        // Count how many times this gesture was recognized.
        var winner_cnt = 0;
        for (var jj = 0; jj < detections.length; jj++) {
            winner_cnt += (detections[jj] == gesture_id);
        }

        // Ensure we have enough recognitions.
        if (winner_cnt < params.repeat_cnt)
            continue;

        // If nothing was detected, skip rest.
        if (gesture_id == -1)
            continue;

        // Hopefully it's the right one too!!
        cmds[cmds.length - 1].add(gesture_id);
        detections.length = 0;
        buffer.length = 0;
    }

    // Mark bad commands, situations where the participant
    // made a mistake or tracking was lost. We know the
    // command was bad because the protector asked the
    // participant to repeat the gesture, but a new command
    // ID is assigned to the sequence.
    for (var ii = 1; ii < cmds.length; ii++) {
        if (cmds[ii].expected_id == cmds[ii - 1].expected_id)
            cmds[ii - 1].ignore = true;
    }

    // Put all results in confusion matrices.
    var ret = new ConfusionMatrices(ds);

    for (var ii = 0; ii < cmds.length; ii++) {
        if (cmds[ii].ignore)
            continue;

        cmds[ii].update_confusion_matrices(ret);
    }

    return ret;
}

/**
 * Evaluate all user study sessions for a given device type.
 */
function evaluate_sessions(device) {
    var confusion_matrices = [];
    var participants = get_participant_list(device).slice();
    for (var i = 0; i < participants.length - 1; i++) {
        var cm = evaluate_session(device, participants[i])
        confusion_matrices.push(cm);

        var idx = confusion_matrices.length - 1;

        console.log("Participant: " + participants[i]);


        var results = confusion_matrices[idx].results();
        results.print();
        console.log();
    }

    // put all results into first confusion
    // matrix
    for (var ii = 1; ii < confusion_matrices.length; ii++) {
        confusion_matrices[0].add_result(confusion_matrices[ii]);
    }

    console.log("Aggregate results:");

    var results = confusion_matrices[0].results();
    results.print();
    console.log();
}
/**
 * Copyright 2017 the University of Central Florida Research Foundation, Inc.
 * All rights reserved.
 *
 * Eugene M. Taranta II <etaranta@gmail.com>
 * Amirreza Samiei <samiei@knights.ucf.edu>
 * Mehran Maghoumi <mehran@cs.ucf.edu>
 * Pooya Khaloo <pooya@cs.ucf.edu>
 * Corey R. Pittman <cpittman@knights.ucf.edu>
 * Joseph J. LaViola Jr. <jjl@cs.ucf.edu>
 *
 * Subject to the terms and conditions of this academic license, the Licensor
 * hereby grants, royalty-free, non-sublicensable, non-commercial,
 * non-exclusive, academic research license to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, but not to distribute, and to permit
 * persons to whom the Software is furnished to do so, subject to the following
 * conditions:
 *
 *    * The above copyright notice and this permission notice shall be included
 *      in all copies or substantial portions of the Software. Publications
 *      resulting from use of the software shall cite the authors of the
 *      publication titled, “Jackknife: A Reliable Recognizer with Few Samples
 *      and Many Modalities.”
 *
 *    * Identification of the authors and copyright holder of the software shall
 *      receive attribution, in any reasonable manner.
 *
 *    * Patent and trademark rights of the copyright holder are not licensed
 *      under this academic license.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" BY THE COPYRIGHT HOLDER, WITHOUT WARRANTY OF
 * ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN
 * NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDER BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, OR CONSEQUENTIAL DAMAGES (INCLUDING BUT NOT
 * LIMITED TO LOSS OF USE, DATA OR BUSINESS INTERRUPTION), CLAIM, OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
 
console.log("---------[ KINECT UI TEST ] ---------\n\n")
user_indepedent_test(device_type_t.KINECT);

console.log("---------[ LEAP MOTION SESSIONS TEST ] ---------\n\n")
evaluate_sessions(device_type_t.LEAP_MOTION);

console.log("---------[ KINECT SESSIONS TEST ] ---------\n\n")
evaluate_sessions(device_type_t.KINECT);
