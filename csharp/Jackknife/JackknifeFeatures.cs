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
     * Place to store extracted information and
     * features from a given sample.
     */
    public struct JackknifeFeatures
    {
        /**
         * A trajectory is first resampled to N points,
         * and the points are stored here.
         */
        public List<Vector> Pts { get; set; }

        /**
         * Vecs is named in a general sense. It can mean
         * either m-dimensional points or direction vectors,
         * depending on which DTW measure is being used.
         * In both cases, it's the processed trajectory.
         */
        public List<Vector> Vecs { get; set; }

        /**
         * The normalized absolute distance traversed
         * by each component. This is used as a correction
         * factor to augment the DTW score.
         */
        public Vector Abs { get; set; }

        /**
         * The per component bounding box width, which is
         * normalized. This is also used as a correction
         * factor to augment the DTW score.
         */
        public Vector Bb { get; set; }

        /**
         * The main data extraction work is done in the constructor.
         *
         * This is where we:
         *
         *      1) Resample the trajectory to a fixed number
         *         of points (resample_cnt).
         *
         *      2) Calculate the normalized direction vector
         *         between each resampled point or the z-score
         *         normalized points.
         *
         *      3) Extract correction factor related data.
         *
         * It is highly recommended that the sample trajectory points
         * have been smoothed with a low pass filter before calling
         * this function. A simple exponential smoothing filter will
         * probably be adequate for most case.
         */
        public JackknifeFeatures(JackknifeBlades blades, List<Vector> points)
        {
            this.Vecs = new List<Vector>();

            // Number of components per point.
            int m = points[0].Size;

            // resample the trajectory to a fixed number of points
            this.Pts = Mathematics.Resample(points, blades.ResampleCnt);

            // To track the bounding box widths,
            // start with one point and expand.
            Vector minimum = new Vector(Pts[0].Data);
            Vector maximum = new Vector(Pts[0].Data);

            // The abs distance traversed starts with zeros.
            this.Abs = new Vector(0, m);

            // Incrementally extract information.
            for (int i = 1; i < blades.ResampleCnt; i++)
            {
                // In-between point direction vector.
                Vector vec = Pts[i] - Pts[i - 1];

                // Update correction factor features.
                for (int j = 0; j < m; j++)
                {
                    this.Abs[j] += Math.Abs(vec[j]);
                    minimum[j] = Math.Min(minimum[j], Pts[i][j]);
                    maximum[j] = Math.Max(maximum[j], Pts[i][j]);
                }

                // Save the points or direction vectors,
                // depending on the selected measure.
                if (blades.InnerProduct)
                    this.Vecs.Add(vec.Normalize());
                else if (blades.EuclideanDistance)
                {
                    // In ED scenario, make sure not to forget first point as
                    // loop starts at 1.
                    if (i == 1)
                        this.Vecs.Add(Pts[0]);

                    this.Vecs.Add(Pts[i]);
                }
                else
                    throw new Exception("This should not happen!");
            }

            // Z-score normalize the vecs if required,
            // typically only if using euclidean distance
            if (blades.ZNormalize)
                Mathematics.ZNormalize(this.Vecs);

            // normalize the correction factor vectors
            this.Abs.Normalize();
            this.Bb = (maximum - minimum).Normalize();
        }
    }
}
