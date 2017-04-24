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

#pragma once

#include <cmath>
#include <vector>
#include <cstdlib>
#include <algorithm>
#include "vector.h"

namespace Jackknife {

/**
 * Component-wise z-score normalize each point of a time series.
 */
static inline void z_normalize(std::vector<Vector> &points)
{
    int n = points.size();
    int m = points[0].data.size();
    Vector mean(0.0, m);
    Vector var(0.0, m);

    // Estimate the component-wise mean.
    for(int ii = 0;
        ii < n;
        ii ++)
    {
        mean += points[ii];
    }

    mean /= (double) n;

    // Estimate the component-wise variance
    for(int ii = 0;
        ii < n;
        ii++)
    {
        for(int jj = 0;
            jj < m;
            jj ++)
        {
            double diff = points[ii].data[jj] - mean.data[jj];
            var.data[jj] += (diff * diff);
        }
    }

    var /= (double)(n-1);

    // now convert variance to standard deviation
    for (int ii = 0;
        ii < m;
        ii++)
    {
        var.data[ii] = std::sqrt(var.data[ii]);
    }

    // last, z-score normalize all points
    for (int ii = 0;
        ii < n;
        ii++)
    {
        points[ii] = (points[ii] - mean) / var;
    }
}

/**
 * Given a time series trajectory, determine the path length
 * through the m-dimensional space, where m is the number of
 * components per point.
 */
static inline double path_length(const std::vector<Vector> &points)
{
    double ret = 0.0;

    for(int ii = 1;
        ii < points.size();
        ii ++)
    {
        ret += points[ii].l2norm(points[ii-1]);
    }

    return ret;
}

/**
 * Resample a trajectory either uniformly or stochastically. To perform uniform
 * resampling, set variance to zero. To perform stochastic resampling, set
 * variance greater than zero.
 */
static void resample(
    const std::vector<Vector> &points,
    std::vector<Vector> &ret,
    int n,
    double variance = 0.0)
{
    double path_distance = path_length(points);
    Vector intervals(n-1);
    double interval;
    int ii;

    // uniform resampling
    if(variance == 0.0)
    {
        intervals = 1.0 / (double)(n - 1);
    }
    // stochastic resampling
    else
    {
        // Create a set of random intervals
        // that are drawn from the uniform
        // random distribution
        for(ii = 0;
            ii < n - 1;
            ii++)
        {
            double b = std::sqrt(12.0 * variance);
            double rr = static_cast<double>(std::rand());
            rr /= static_cast<float> (RAND_MAX);
            intervals[ii] = 1.0 + rr * b;
        }

        intervals /= intervals.sum();
    }

    assert(std::abs(intervals.sum() - 1.0) < 0.00001);

    //
    // Core resampling logic.
    //
    double remaining_distance = path_distance * intervals[0];
    const Vector *prev = &points[0];

    ret.reserve(n);
    ret.push_back(points[0]);
    ii = 1;

    while(ii < points.size())
    {
        double distance = points[ii].l2norm(*prev);

        if(distance < remaining_distance)
        {
            prev = &points[ii];
            remaining_distance -= distance;
            ii ++;
            continue;
        }

        // Now we need to interpolate between the last point
        // and the current point.
        double ratio = remaining_distance / distance;

        // If two points are close together, the distance
        // may be close to zero and the ratio becomes inf.
        // Also, if the remaining_distance is close to zero
        // and the distance is close to zero, then 0/0 = nan.
        if(ratio > 1.0 || std::isnan(ratio)) ratio = 1.0;

        ret.push_back(
            Vector(
                *prev,
                points[ii],
                ratio));

        // Because of rounding errors, we may hit the end
        // just a bit too soon.
        if(ret.size() == n)
        {
            return;
        }

        // The previous point becomes the new
        // point we just created.
        prev = &ret[ret.size() - 1];

        // Setup for the next interval.
        remaining_distance = path_distance * intervals[ret.size() - 1];
    }

    // Because of rounding errors, we may not reach the
    // last point, which is very common.
    if (ret.size() < n)
    {
        ret.push_back(points[ii - 1]);
    }

    assert(ret.size() == n);
}

/**
 * Perform gesture path stochastic resampling (GPSR) to create a synthetic
 * variation of the given trajectory:
 *
 * Eugene M. Taranta II, Mehran Maghoumi, Corey R. Pittman, Joseph J. LaViola Jr.
 * "A Rapid Prototyping Approach to Synthetic Data Generation For Improved 2D Gesture Recognition"
 * Proceedings of the 29th Annual Symposium on User Interface Software and Technology
 * 2016
 */
static void gpsr(
    std::vector<Vector> &points,
    std::vector<Vector> &ret,
    int n,
    double variance,
    int remove_cnt)
{
    std::vector<Vector> resampled;

    resample(
        points,
        resampled,
        n + remove_cnt,
        variance);

    // Remove random points to simulate cutting corners.
    for (int ii = 0;
         ii < remove_cnt;
         ii++)
    {
        int remove_idx = std::rand();
        remove_idx %= (int)n + remove_cnt - ii;
        resampled.erase(resampled.begin() + remove_idx);
    }

    // Construct synthetic variation.
    int m = resampled[0].data.size();
    ret.reserve(n);
    ret.push_back(Vector(m));

    for(int ii = 1;
        ii < resampled.size();
        ii++)
    {
        Vector delta = resampled[ii] - resampled[ii - 1];
        ret.push_back(delta.normalize());
    }
}

} // Jackknife namespace
