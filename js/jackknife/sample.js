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
