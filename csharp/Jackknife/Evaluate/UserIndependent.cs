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
using System.IO;
using System.Linq;
using System.Text;

namespace Jackknife.Evaluate
{

    public enum DeviceType
    {

        KINECT = 0,
        LEAP_MOTION = 1,

    };


    public static class UserIndependent
    {

        /**
         * A simple user independent test.
         */
        public static void user_indepedent_test(DeviceType device)
        {
            // First, load all training data for one device type.
            string path = "";

            if (device == DeviceType.KINECT)
                path = Global.GetRootDirectory() + "datasets/jk2017/kinect/training/";
            else if (device == DeviceType.LEAP_MOTION)
                path = Global.GetRootDirectory() + "datasets/jk2017/leap_motion/training/";


            Dataset ds = Dataset.LoadDataset(path);

            int subject_cnt = ds.Subjects.Count;
            int sample_cnt = ds.Samples.Count;
            ConfusionMatrices cm = new ConfusionMatrices(ds);

            // iterate through all subjects
            for (int subject_id = 0; subject_id < subject_cnt; subject_id++)
            {
                ConfusionMatrices cm_individual = new ConfusionMatrices(ds);

                Console.WriteLine("Participant: " + subject_id);

                // train a recognizer with the selected subject
                JackknifeBlades blades = new JackknifeBlades();

                blades.SetIPDefaults();

                Jackknife jk = new Jackknife(blades);

                for (int sample_id = 0; sample_id < sample_cnt; sample_id++)
                {
                    Sample sample = ds.Samples[sample_id];

                    if (sample.SubjectId != subject_id)
                        continue;

                    jk.AddTemplate(ds.Samples[sample_id]);
                }

                // only train the recognize if you need

                // test the recognizer with all other samples
                for (int sample_id = 0; sample_id < sample_cnt; sample_id++)
                {
                    Sample sample = ds.Samples[sample_id];

                    if (sample.SubjectId == subject_id)
                        continue;

                    int gid = jk.Classify(sample);

                    cm_individual.AddResult(
                        sample.GestureId,
                        gid);
                }

                cm.AddResult(cm_individual);

                ResultT resultss = cm_individual.Results();
                resultss.Print();
            }

            Console.WriteLine("Aggregate Results: ");
            ResultT results = cm.Results();
            results.Print();
        }
    }
}
