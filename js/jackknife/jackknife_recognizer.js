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
