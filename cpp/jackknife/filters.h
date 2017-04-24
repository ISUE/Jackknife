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

namespace Jackknife {

/**
 * An effective, easy to use low pass filter that is good
 * for continuous data.
 */
class ExponentialMovingAverage
{

public:

    /**
     *
     */
    Vector pt;

    /**
     *
     */
    double cut_off_frequency_hz;

    /**
     * Constructor that allows you to select the cut off frequency.
     * One Hz may not be optimal but is probably a very good
     * starting point.
     */
    ExponentialMovingAverage(
        Vector initial_pt,
        double cut_off_frequency_hz = 1.0)
    {
        pt = initial_pt;
        this->cut_off_frequency_hz = cut_off_frequency_hz;
    }

    /**
     * The duration_s parameter is the duration between the previous
     * and current points in seconds. If the exact time stamps are
     * unavailable, you can estimate using the input device sampling
     * rate, e.g., 1/30.
     */
    Vector& operator()(
        const Vector &pt,
        double duration_s)
    {
        double tau = 1.0 / (2.0 * M_PI * cut_off_frequency_hz);
        double alpha = 1.0 / (1.0 + tau/duration_s);

        this->pt = (pt * alpha) + (this->pt * (1.0 - alpha));

        return this->pt;
    }
};

} // Jackknife namespace
