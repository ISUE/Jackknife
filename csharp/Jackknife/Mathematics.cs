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
    public class Mathematics
    {
        /**
         * Component-wise z-score normalize each point of a time series.
         */
        public static void ZNormalize(List<Vector> points)
        {
            int n = points.Count;

            int m = points[0].Size;
            Vector mean = new Vector(0, m);
            Vector var = new Vector(0, m);

            // Estimate the component-wise mean.
            for (int i = 0; i < n; i++)
                mean += points[i];

            mean /= (double)n;

            // Estimate the component-wise variance
            for (int i = 0; i < n; i++)
            {
                for (int j = 0; j < m; j++)
                {
                    double diff = points[i][j] - mean[j];
                    var[j] += diff * diff;
                }
            }

            var /= (double)(n - 1);

            // now convert variance to standard deviation
            for (int i = 0; i < m; i++)
                var[i] = Math.Sqrt(var[i]);

            // last, z-score normalize all points
            for (int i = 0; i < n; i++)
                points[i] = (points[1] - mean) / var;
        }

        /**
         * Given a time series trajectory, determine the path length
         * through the m-dimensional space, where m is the number of
         * components per point.
         */
        public static double PathLength(List<Vector> points)
        {
            double ret = 0;

            for (int i = 1; i < points.Count; i++)
                ret += points[i].L2Norm(points[i - 1]);

            return ret;
        }

        /**
         * Resample a trajectory either uniformly or stochastically. To perform uniform
         * resampling, set variance to zero. To perform stochastic resampling, set
         * variance greater than zero.
         */
        public static List<Vector> Resample(List<Vector> points, int n, double variance = 0)
        {
            Random rnd = new Random();

            double pathDistance = PathLength(points);

            Vector intervals = new Vector(n - 1);
            double interval;
            int i;

            // uniform resampling
            if (variance == 0)
                intervals.Set(1 / (double)(n - 1));
            // stochastic resampling
            else
            {
                // Create a set of random intervals
                // that are drawn from the uniform
                // random distribution
                for (i = 0; i < n - 1; i++)
                {
                    double b = Math.Sqrt(12 * variance);
                    double rr = rnd.NextDouble();

                    intervals[i] = 1 + rr * b;
                }

                intervals /= intervals.Sum();
            }

            if (Math.Abs(intervals.Sum() - 1) >= 0.00001)
                throw new Exception("Error in resample function.");

            //
            // Core resampling logic.
            //
            double remaining_distance = pathDistance * intervals[0];
            Vector prev = points[0];

            List<Vector> ret = new List<Vector>();

            ret.Add(points[0]);
            i = 1;

            while (i < points.Count)
            {
                double distance = points[i].L2Norm(prev);

                if (distance < remaining_distance)
                {
                    prev = points[i];
                    remaining_distance -= distance;
                    i++;
                    continue;
                }

                // Now we need to interpolate between the last point
                // and the current point.
                double ratio = remaining_distance / distance;

                // If two points are close together, the distance
                // may be close to zero and the ratio becomes inf.
                // Also, if the remaining_distance is close to zero
                // and the distance is close to zero, then 0/0 = nan.
                if (ratio > 1.0 || double.IsNaN(ratio)) ratio = 1.0;

                ret.Add(new Vector(prev, points[i], ratio));

                // Because of rounding errors, we may hit the end
                // just a bit too soon.
                if (ret.Count == n)
                {
                    return ret;
                }

                // The previous point becomes the new
                // point we just created.
                prev = ret[ret.Count - 1];

                // Setup for the next interval.
                remaining_distance = pathDistance * intervals[ret.Count - 1];
            }

            // sometimes we make it a little past the end due
            // to precision errors
            if (ret.Count < n)
                ret.Add(points[i - 1]);

            if (ret.Count != n)
                throw new Exception("ret.Count != n");

            // exit if not making a synthetic sample
            return ret;
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
        public static List<Vector> GPSR(List<Vector> points, int n, double variance, int removeCnt)
        {
            List<Vector> resampled = Resample(points, n + removeCnt, variance);
            Random rnd = new Random();

            // Remove random points to simulate cutting corners.
            for (int i = 0; i < removeCnt; i++)
            {
                int removeIndex = rnd.Next(0, 32767);
                removeIndex %= n + removeCnt - i;

                resampled.RemoveAt(removeIndex);
            }

            // Construct synthetic variation.
            int m = resampled[0].Size;
            List<Vector> ret = new List<Vector>(n);

            ret.Add(new Vector(m));

            for (int i = 1; i < resampled.Count; i++)
            {
                Vector delta = resampled[i] - resampled[i - 1];

                ret.Add(delta.Normalize());
            }

            return ret;
        }
    }
}
