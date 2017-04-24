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
