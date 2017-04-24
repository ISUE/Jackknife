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
