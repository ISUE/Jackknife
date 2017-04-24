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
