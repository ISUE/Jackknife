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
using System.IO;
using System.Linq;
using System.Text;

namespace Jackknife.Evaluate
{
    /**
     *
     */
    public struct Frame
    {
        /**
         * The expected gesture: the gesture that the
         * participant should execute.
         */
        public int gesture_id;

        /**
         * Enumerated command id.
         */
        public int cmd_id;

        /**
         * Currently not used (originates from another project
         * but takes up space in output file).
         */
        public float time_remaining_s;

        /**
         * Position of command on screen.
         */
        public float text_pos_x;

        /**
         * Position of command on screen.
         */
        public float text_pos_y;

        /**
         * True if any component in vector is NAN or infinity.
         * This can happen if the input device loses tracking.
         */
        public bool bad_pt;

        /**
         * The actual input! :)
         */
        public Vector pt;

        /**
         *
         */

        public Frame(RecordingReader reader)
        {
            this.gesture_id = reader.ReadInt();
            this.cmd_id = reader.ReadInt();

            time_remaining_s = reader.ReadFloat(); // remaining time
            text_pos_x = reader.ReadFloat(); // text position x
            text_pos_y = reader.ReadFloat(); // text position y

            this.pt = new Vector(63);

            this.bad_pt = false;

            for (int i = 0; i < 63; i++)
            {
                this.pt[i] = reader.ReadFloat();

                if (double.IsNaN(this.pt[i]) == true)
                    this.bad_pt = true;

                if (double.IsInfinity(this.pt[i]) == true)
                    this.bad_pt = true;
            }
        }
    };

}
