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
     * All information about a single template, including
     * some cached result used by the recognizer.
     */
    public struct JackknifeTemplate
    {
        /**
         * Original sample used to create this template.
         */
        public Sample Sample { get; set; }

        /**
         * The enumerate gesture class id, which matches the
         * gesture id stored in the sample.
         */
        public int GestureId { get; set; }

        /**
         * Extracted important information about gesture trajectory.
         */
        public JackknifeFeatures Features { get; set; }

        /**
         * Envelop used to calculate the lower bounds.
         */
        public List<Vector> Lower { get; set; }

        public List<Vector> Upper { get; set; }

        /**
         * Cached lower bound result used by the recognizer.
         */
        public double LB { get; set; }

        /**
         * Cached correction factor result used by the recognizer.
         */
        public double CF { get; set; }

        /**
         * If score is worse than this value, reject it.
         */
        public double RejectionThreshold { get; set; }

        /**
         * Constructor.
         */
        public JackknifeTemplate(JackknifeBlades blades, Sample sample)
        {
            this.Sample = sample;
            GestureId = sample.GestureId;
            // Any default value less than zero will ensure if
            // lower bounding is disabled, we don't cull any
            // instance.
            LB = -1;

            // A default value of one will ensure that the DTW
            // score is not modified when all correction factors
            // are disabled.
            CF = 1;

            // A default value that will never cause rejection.
            // So if the recognizer is not trained, things will
            // still work as expected.
            RejectionThreshold = double.PositiveInfinity;
            Lower = new List<Vector>();
            Upper = new List<Vector>();

            // Extract import information about the sample.
            Features = new JackknifeFeatures(blades, sample.Trajectory);
            List<Vector> vecs = Features.Vecs;
            int componentCnt = vecs[0].Size;

            // For each component find the min and max value
            // within the radius (Sakoe-Chiba band).
            for (int i = 0; i < vecs.Count; i++)
            {
                Vector maximum = new Vector(double.NegativeInfinity, componentCnt);
                Vector minimum = new Vector(double.PositiveInfinity, componentCnt);

                for (int j = Math.Max(0, i - blades.Radius); j < Math.Min(i + blades.Radius + 1, vecs.Count); j++)
                {
                    for (int k = 0; k < componentCnt; k++)
                    {
                        maximum[k] = Math.Max(maximum[k], vecs[j][k]);
                        minimum[k] = Math.Min(minimum[k], vecs[j][k]);
                    }
                }

                Upper.Add(maximum);
                Lower.Add(minimum);
            }
        }

    }

    /**
     * Helper for being able to sort templates based on
     * lower bound results.
     */
    public class SortTemplate : IComparer<JackknifeTemplate>
    {
        public int Compare(JackknifeTemplate x, JackknifeTemplate y)
        {
            if (x.LB < y.LB)
                return -1;
            else if (x.LB > y.LB)
                return 1;
            return 0;
        }
    }
}
