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
     * A class to hold one sample, perhaps loaded from a file.
     */
    public class Sample
    {
        /**
        * Subject ID.
        * This is not used by the recognizer.
        */
        public int SubjectId { get; set; }

        /**
         * Gesture class ID.
         */
        public int GestureId { get; set; }

        /**
         * If using multiple samples of a gesture,
         * this can be used to enumerate the
         * different versions. However, this is not
         * used by the recognizer.
         */
        public int ExampleId { get; set; }

        /**
         * List of points in the gesture path.
         */
        public List<Vector> Trajectory { get; set; }

        /**
         * Constructor that initializes all public variables,
         * except the trajectory. Use AddTrajectory for that.
         */
        public Sample(int subjectId, int gestureId, int exampleId)
        {
            SubjectId = subjectId;
            GestureId = gestureId;
            ExampleId = exampleId;

            Trajectory = new List<Vector>();
        }

        /**
         * Multiple series are combined into a single trajectory.
         * In most cases this isn't relevant, but it may impact
         * multistroke gesture recognition, such as for hand
         * written pen or touch gestures. For those cases, you may
         * want to consider using Vatavu et al.'s $P recognizer.
         */
        public void AddTrajectory(List<Vector> trajectory)
        {
            for (int i = 0; i < trajectory.Count; i++)
                Trajectory.Add(trajectory[i].Clone() as Vector);
        }
    }
}
