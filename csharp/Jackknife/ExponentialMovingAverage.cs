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

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Jackknife
{
    /**
     * An effective, easy to use low pass filter that is good
     * for continuous data.
     */
    public class ExponentialMovingAverage
    {
        public Vector Pt { get; set; }

        public double CutOffFrequencyHz { get; set; }

        /**
         * Constructor that allows you to select the cut off frequency.
         * One Hz may not be optimal but is probably a very good
         * starting point.
         */
        public ExponentialMovingAverage(Vector initialPt, double cuttoff = 1)
        {
            Pt = initialPt.Clone() as Vector;
            CutOffFrequencyHz = cuttoff;
        }

        /**
         * The duration_s parameter is the duration between the previous
         * and current points in seconds. If the exact time stamps are
         * unavailable, you can estimate using the input device sampling
         * rate, e.g., 1/30.
         */
        public Vector Filter(Vector pt, double durationS)
        {
            double tau = 1.0 / (2.0 * Math.PI * CutOffFrequencyHz);
            double alpha = 1.0 / (1.0 + tau / durationS);
            this.Pt = (pt * alpha) + (this.Pt * (1 - alpha));

            return this.Pt;
        }
    }
}
