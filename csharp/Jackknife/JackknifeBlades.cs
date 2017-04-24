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
     * Jackknife supports a number of different DTW-based measurement techniques.
     * Select an approach that is appropriate for your situation by setting the
     * associated options in this structure.
     */
    public class JackknifeBlades
    {
        /**
         * No matter which measure is used, the trajectory is first resampled
         * to a fixed number of points.
         */
        public int ResampleCnt { get; set; }

        /**
         * Sakoe-Chiba band size. This is one type of constraint that specifies
         * how much warping is allowed between two time series. A common value
         * is 10% of the resample_cnt.
         */
        public int Radius { get; set; }

        /**
         * Utilize the squared Euclidean distance measure. This flag is mutually
         * exclusive with the inner product flag.
         */
        public bool EuclideanDistance { get; set; }

        /**
         * Z-score normalize the resampled points. If using Euclidean distance,
         * you will normally need to do this. Not recommended for inner product
         * measures though.
         */
        public bool ZNormalize { get; set; }

        /**
         * Extract and normalize the direction vectors between the resampled
         * points, and use the inner product of the direction vectors. Note,
         * this flag is mutually exclusive with the Euclidean distance flag.
         */
        public bool InnerProduct { get; set; }

        /**
         * Use lower bounding to cull full DTW evaluations.
         */
        public bool LowerBound { get; set; }

        /**
         * Utilize the absolute distance correction factor -- the distance
         * traversed by each component in a time series.
         */
        public bool CFAbsDistance { get; set; }

        /**
        * Utilize the bounding box width correction factor -- the difference
        * between the maximum and minimum value of each component.
        */
        public bool CFBbWidths { get; set; }

        /**
         * Setup defaults for inner product measure.
         */
        public void SetIPDefaults()
        {
            ResampleCnt = 16;
            Radius = 2;
            EuclideanDistance = false;
            ZNormalize = false;
            InnerProduct = true;
            LowerBound = true;
            CFAbsDistance = true;
            CFBbWidths = true;
        }

        /**
         * Setup defaults for Euclidean distance measure.
         */
        public void SetEDDefaults()
        {
            ResampleCnt = 16;
            Radius = 2;
            EuclideanDistance = true;
            ZNormalize = true;
            InnerProduct = false;
            LowerBound = true;
            CFAbsDistance = true;
            CFBbWidths = true;
        }
    }
}
