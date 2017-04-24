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

namespace Jackknife.Evaluate
{
    /**
 *
 */
    public struct CommandResults
    {
        /**
         * The enumerated command ID.
         */
       public int command_id;

        /**
         * Gesture that is expected.
         */
        public int expected_id;

        /**
         * List of gestures that are detected during this
         * command window.
         */
        public List<int> detected_ids;

        /**
         * If the participant made a mistake or tracking was lost,
         * we repeat the gesture request in the next command window.
         * So this command should be ignored.
         */
        public bool ignore;

        /**
         *
         */
        public CommandResults(
            int command_id,
            int expected_id)
        {
            detected_ids = new List<int>();
            this.command_id = command_id;
            this.expected_id = expected_id;
            this.ignore = false;
        }

        /**
         * Collect any gestures detection during this
         * command duration.
         */
        public void add(int detected_id)
        {
            //assert(detected_id >= 0);
            detected_ids.Add(detected_id);
        }

        /**
         * After the command is complete, update the matrices
         * with detected gestures.
         */
        public void update_confusion_matrices(ConfusionMatrices  cm)
        {
            bool found = false;

            for (int ii = 0;
                 ii < detected_ids.Count;
                 ii++)
            {
                int detected_id = detected_ids[ii];

                if (found && expected_id == detected_id)
                {
                    // treat as false positive
                    // because we've already detected
                    // the gesture once
                    cm.AddResult(
                        expected_id,
                        -2);
                    continue;
                }

                cm.AddResult(
                    expected_id,
                    detected_id);

                // mark that we've found this gesture
                found = found || (detected_id == expected_id);
            }

            // false negative
            if (!found)
            {
                cm.AddResult(
                    expected_id,
                    -1);
            }
        }
    };
}
