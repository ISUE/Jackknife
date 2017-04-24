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

#include "sample.h"
#include "mathematics.h"
#include "jackknife.h"

namespace Jackknife {

/**
 * Place to store extracted information and
 * features from a given sample.
 */
struct JackknifeFeatures
{
    /**
     * A trajectory is first resampled to N points,
     * and the points are stored here.
     */
    std::vector<Vector> pts;

    /**
     * Vecs is named in a general sense. It can mean
     * either m-dimensional points or direction vectors,
     * depending on which DTW measure is being used.
     * In both cases, it's the processed trajectory.
     */
    std::vector<Vector> vecs;

    /**
     * The normalized absolute distance traversed
     * by each component. This is used as a correction
     * factor to augment the DTW score.
     */
    Vector abs;

    /**
     * The per component bounding box width, which is
     * normalized. This is also used as a correction
     * factor to augment the DTW score.
     */
    Vector bb;

    /**
     * Don't use this.
     */
    JackknifeFeatures() {}

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
    JackknifeFeatures(
        jackknife_blades_t blades,
        const std::vector<Vector> &points)
    {
        // Number of components per point.
        int m = points[0].data.size();

        // resample the trajectory to a fixed number of points
        resample(
            points,
            this->pts,
            blades.resample_cnt);

        // To track the bounding box widths,
        // start with one point and expand.
        Vector minimum = Vector(pts[0].data);
        Vector maximum = Vector(pts[0].data);

        // The abs distance traversed starts with zeros.
        this->abs = Vector(0.0, m);

        // Incrementally extract information.
        for (int ii = 1;
             ii < blades.resample_cnt;
             ii++)
        {
            // In-between point direction vector.
            Vector vec = pts[ii] - pts[ii-1];

            // Update correction factor features.
            for(int jj = 0;
                jj < m;
                jj++)
            {
                this->abs.data[jj] += fabs(vec.data[jj]);

                minimum.data[jj] = std::min(
                    minimum.data[jj],
                    pts[ii].data[jj]);

                maximum.data[jj] = std::max(
                    maximum.data[jj],
                    pts[ii].data[jj]);
            }

            // Save the points or direction vectors,
            // depending on the selected measure.
            if(blades.inner_product)
            {
                this->vecs.push_back(vec.normalize());
            }
            else if(blades.euclidean_distance)
            {
                // In ED scenario, make sure not to forget first point as
                // loop starts at 1.
                if(ii == 1)
                {
                    this->vecs.push_back(pts[0]);
                }

                this->vecs.push_back(pts[ii]);
            }
            else
            {
                assert(0);
            }
        }

        // Z-score normalize the vecs if required,
        // typically only if using euclidean distance
        if(blades.z_normalize)
        {
            z_normalize(this->vecs);
        }

        // normalize the correction factor vectors
        this->abs.normalize();
        this->bb = (maximum - minimum).normalize();
    }
};

} // Jackknife namespace
