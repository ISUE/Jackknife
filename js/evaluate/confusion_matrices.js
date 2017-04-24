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
