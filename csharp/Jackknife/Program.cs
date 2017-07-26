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

using Jackknife.Evaluate;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Jackknife
{
    class Program
    {
        static void Main(string[] args)
        {
            /**
             * IMPORTANT:
             * It is important that the files containing training data are parsed with "en-US" locale setting.
             * Other regional formats (e.g. German) may use different decimal separators, which would cause problems
             * when parsing our data files.
             * 
             * The line below forces .NET to use "en-US" locale when converting decimal strings to double
             * values
             */
            System.Threading.Thread.CurrentThread.CurrentCulture = new System.Globalization.CultureInfo("en-US", false);

            Console.WriteLine("---------[ KINECT UI TEST ] ---------");

            UserIndependent.user_indepedent_test(DeviceType.KINECT);

            Console.WriteLine("---------[ LEAP MOTION SESSIONS TEST] ---------");

            Global.evaluate_sessions(DeviceType.LEAP_MOTION);

            Console.WriteLine("---------[ KINECT SESSIONS TEST] ---------");

            Global.evaluate_sessions(DeviceType.KINECT);
        }
    }
}
