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

#include <limits>
#include <algorithm>

#include "jackknife.h"
#include "jackknife_features.h"

namespace Jackknife {

/**
 * All information about a single template, including
 * some cached result used by the recognizer.
 */
struct JackknifeTemplate
{
    /**
     * JackknifeTemplate shared pointer type.
     */
    typedef std::shared_ptr<JackknifeTemplate> Ptr;

    /**
     * Original sample used to create this template.
     */
    Sample::Ptr sample;

    /**
     * The enumerate gesture class id, which matches the
     * gesture id stored in the sample.
     */
    int gesture_id;

    /**
     * Extracted important information about gesture trajectory.
     */
    JackknifeFeatures features;

    /**
     * Envelop used to calculate the lower bounds.
     */
    std::vector<Vector> lower;
    std::vector<Vector> upper;

    /**
     * Cached lower bound result used by the recognizer.
     */
    double lb;

    /**
     * Cached correction factor result used by the recognizer.
     */
    double cf;

    /**
     * If score is worse than this value, reject it.
     */
    double rejection_threshold;

    /**
     * Constructor.
     */
    JackknifeTemplate(
        jackknife_blades_t blades,
        Sample::Ptr sample):
            sample(sample),
            gesture_id(sample->gesture_id)
    {
        this->sample = sample;
        this->gesture_id = sample->gesture_id;

        // Any default value less than zero will ensure if
        // lower bounding is disabled, we don't cull any
        // instance.
        this->lb = -1.0;

        // A default value of one will ensure that the DTW
        // score is not modified when all correction factors
        // are disabled.
        this->cf = 1.0;

        // A default value that will never cause rejection.
        // So if the recognizer is not trained, things will
        // still work as expected.
        this->rejection_threshold = std::numeric_limits<double>::infinity();

        // Extract import information about the sample.
        this->features = JackknifeFeatures(
            blades,
            sample->trajectory);

        // Now we find the envelop for being able to
        // calculate lower bounds. Go ahead and do this
        // even if the feature is disabled, since it's
        // just a one time thing at training time and
        // is fast.
        std::vector<Vector> &vecs = features.vecs;
        int component_cnt = vecs[0].data.size();

        for(int ii = 0;
            ii < vecs.size();
            ii++)
        {
            Vector maximum(
                -std::numeric_limits<double>::infinity(),
                component_cnt);

            Vector minimum(
                std::numeric_limits<double>::infinity(),
                component_cnt);

            // For each component find the min and max value
            // within the radius (Sakoe-Chiba band).
            for (int jj = std::max(0, ii - (int)blades.radius);
                 jj < std::min((size_t)(ii + blades.radius + 1), vecs.size());
                 jj ++)
            {
                for(int kk = 0;
                    kk < component_cnt;
                    kk++)
                {
                    maximum.data[kk] = std::max(
                        maximum.data[kk],
                        vecs[jj].data[kk]);

                    minimum.data[kk] = std::min(
                        minimum.data[kk],
                        vecs[jj].data[kk]);
                }
            }

            upper.push_back(maximum);
            lower.push_back(minimum);
        }
    }
};

/**
 * Helper for being able to sort templates based on
 * lower bound results.
 */
struct SortTemplates
{
    inline bool operator() (
        const JackknifeTemplate::Ptr &t1,
        const JackknifeTemplate::Ptr &t2)
    {
        return (t1->lb < t2->lb);
    }
};

} // Jackknife namespace
