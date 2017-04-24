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

#include <memory>
#include "vector.h"

namespace Jackknife {

/**
 * A class to hold one sample, perhaps loaded from a file.
 */
class Sample
{
public:

    /**
     * Sample shared pointer type.
     */
    typedef std::shared_ptr<Sample> Ptr;

    /**
     * Subject ID.
     * This is not used by the recognizer.
     */
    int subject_id;

    /**
     * Gesture class ID.
     */
    int gesture_id;

    /**
     * If using multiple samples of a gesture,
     * this can be used to enumerate the
     * different versions. However, this is not
     * used by the recognizer.
     */
    int example_id;

    /**
     * List of points in the gesture path.
     */
    std::vector<Vector> trajectory;

    /**
     * Constructor that initializes all public variables,
     * except the trajectory. Use add_trajectory for that.
     */
    Sample(
        int subject_id,
        int gesture_id,
        int example_id):
        subject_id(subject_id),
        gesture_id(gesture_id),
        example_id(example_id)
    {
    }

    /**
     *
     */
    ~Sample()
    {
        trajectory.clear();
    }

    /**
     * Multiple series are combined into a single trajectory.
     * In most cases this isn't relevant, but it may impact
     * multistroke gesture recognition, such as for hand
     * written pen or touch gestures. For those cases, you may
     * want to consider using Vatavu et al.'s $P recognizer.
     */
    void add_trajectory(const std::vector<Vector> &trajectory)
    {
        for(int ii = 0;
            ii < trajectory.size();
            ii++)
        {
            this->trajectory.push_back(trajectory[ii]);
        }
    }
};

} // Jackknife namespace
