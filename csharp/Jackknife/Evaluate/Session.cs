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


    public class Global
    {
        public static string GetRootDirectory()
        {
            return Directory.GetCurrentDirectory() + "/../../../../";
        }

        public static int g_frame_size_b = (5 + 63) * 4;

        /**
         * The gesture ID in the dataset does not match the gesture
         * IDs used in the session files. So this helper function
         * does a conversion.
         */
        public static int convert_gesture_id(Dataset ds, int gesture_id)
        {
            string[] gname = {
                                "jab_right",
                                "jab_left",
                                "kick_right",
                                "kick_left",
                                "hook_right",
                                "hook_left",
                                "uppercut_right",
                                "uppercut_left",
                                "cartwheel_right",
                                "cartwheel_left",
                                "push",
                                "sidekick_right",
                                "sidekick_left",
                                "duck",
                                "explode",
                                "fist_2_circles",
                                "index_2_circles",
                                "knock_x3",
                                "scissors",
                                "sideways",
                                "rock_out",
                                "love",
                                "redrum",
                            };

            //assert(gesture_id >= 1);
            //assert(gesture_id <= 23);

            int ret = ds.GestureNameToId(gname[gesture_id - 1]);
            return ret;
        }

        public static bool bad_gesture(int gesture_id)
        {
            // redrum
            if (gesture_id == 23)
                return true;

            return false;
        }

        /**
         *
         */
        public static bool bad_gesture(string gesture_name)
        {
            if (gesture_name == "redrum")
                return true;

            return false;
        }

        public static int[] get_participant_list(DeviceType device)
        {
            if (device == DeviceType.KINECT)
            {
                int[] ret = {
            100, 101, 102, 103,
            104, 105, 106, 107,
            108, 109, 110, 111,
            200, 201, 202, 203,
            204, 205, 206, 207,
            000,
        };

                return ret;
            }
            else if (device == DeviceType.LEAP_MOTION)
            {
                int[] ret = {
            300, 301, 302, 303,
            304, 305, 306, 307,
            400, 401, 402, 403,
            404, 500, 501, 502,
            503, 504, 505, 506,
            000,
        };

                return ret;
            }

            return null;
        }

        /**
         * Local overload of version in dataset.cpp.
         */
        public static Dataset load_subject_dataset(
            DeviceType device,
            int subject_id)
        {
            // build the subject's dataset path
            string subject_path = Global.GetRootDirectory() + string.Format("datasets/jk2017/{0}/training/Sub_U{1:D3}",
                device == DeviceType.KINECT ? "kinect" : "leap_motion", subject_id);

            //assert(ret != -1);

            // load the subject's dataset
            Dataset ds = new Dataset();
            Dataset.LoadSubjectDataset(
                ds,
                subject_path);
            return ds;
        }


        /**
         * Read in all frames from a binary session file.
         */
        public static void load_session(
              DeviceType device,
              int subject_id,
              List<Frame> frames)
        {

            // Build the subject's session path.
            string session_path = Global.GetRootDirectory() +
                string.Format("datasets/jk2017/{0}/sessions/U{1:D3}",
                device == DeviceType.KINECT ? "kinect" : "leap_motion", subject_id);

            RecordingReader r = new RecordingReader(session_path);

            //// Read position and extract file size.
            //// Size should be end position, but
            //// just in case, do this extract work.

            // convert to frame_cnt
            int frame_cnt = (int)r.FrameCount();

            for (int ii = 0; ii < frame_cnt; ii++)
            {
                Frame frame = new Frame(r);

                // There is an issue where the main program ran at 60fps,
                // but the Kinect only samples at 30fps. Some a number of
                // readings are duplicates and need to be removed.
                if (ii > 0)
                {
                    int idx = frames.Count - 1;
                    double distance = frame.pt.L2Norm(frames[idx].pt);
                    if (distance == 0.0)
                        continue;
                }

                frames.Add(frame);
            }

            r.Close();
        }


        /**
         * Load a participant's dataset and session.
         * Train the recognizer with the training data
         * and run the video through. See what happens...
         */
        public static ConfusionMatrices evaluate_session(DeviceType device, int subject_id)
        {
            // Load up the training dataset.
            Dataset ds = load_subject_dataset(
                device,
                subject_id);

            // Load up the session.
            List<Frame> frames = new List<Frame>();
            load_session(
                device,
                subject_id,
                frames);

            // Create a new recognizer.
            JackknifeBlades blades = new JackknifeBlades();
            blades.SetIPDefaults();
            Jackknife jk = new Jackknife(blades);

            // Train the recognizer, without 'bad' gestures.
            for (int ii = 0;
                ii < ds.Samples.Count;
                 ii++)
            {
                int gesture_id = ds.Samples[ii].GestureId;
                string gesture_name = ds.Gestures[gesture_id];
                if (bad_gesture(gesture_name))
                    continue;
                jk.AddTemplate(ds.Samples[ii]);
            }


            // Get device and application parameters
            // based on the device type.
            configuartion_parameters_t parameters = new configuartion_parameters_t(device);

            // We originally used n=4, r=2 for Kinect data
            // and n=6, r=2 for Leap Motion data, but
            // here we just set the average. There is barely
            // any effect on the results.
            jk.Train(6, 2, 1.00);

            // Play session video through
            // the recognizer.
            List<Vector> buffer = new List<Vector>();
            List<int> detections = new List<int>();
            List<CommandResults> cmds = new List<CommandResults>();
            int last_cmd_id = -1;
            int next_update = parameters.update_interval;

            int frame_no = 0;

            ExponentialMovingAverage filter = new ExponentialMovingAverage(frames[0].pt);
            Vector pt;

            for (int ii = 0;
                ii < frames.Count;
                 ii++)
            {
                // skip this frame if its bad
                if (frames[ii].bad_pt)
                {
                    continue;
                }

                // Low pass filter the input.
                // Note, we originally didn't smooth the data,
                // so results now are a little higher than in
                // the paper.
                pt = filter.Filter(
                    frames[ii].pt,
                            1 / (double)parameters.fps);

                //pt = frames[ii].pt;

                frame_no += 1;

                // start a new command
                if (frames[ii].cmd_id != last_cmd_id)
                {
                    last_cmd_id = frames[ii].cmd_id;

                    int gid = convert_gesture_id(
                        ds,
                        frames[ii].gesture_id);

                    CommandResults cmd = new CommandResults(
                        frames[ii].cmd_id,
                        gid);

                    if (bad_gesture(frames[ii].gesture_id))
                        cmd.ignore = true;

                    cmds.Add(cmd);
                }

                // This buffering approach is really
                // inefficient, but since this off-line,
                // performance is not important.
                buffer.Add(pt);
                if (buffer.Count > parameters.sliding_window_frame_cnt)
                    buffer.RemoveAt(0);

                // We need to have a couple points before
                // calling the recognizer.
                if (buffer.Count < 2)
                    continue;

                // Wait a few frames again before trying
                // to recognize again.
                if (frame_no < next_update)
                    continue;

                next_update = frame_no + parameters.update_interval;

                // Run the recognizer.
                int gesture_id = jk.Classify(buffer);

                // Add recognition result.
                detections.Add(gesture_id);
                if (detections.Count > parameters.repeat_cnt)
                    detections.RemoveAt(0);

                // Count how many times this gesture was recognized.
                int winner_cnt = 0;
                for (int jj = 0;
            jj < detections.Count;
                     jj++)
                {
                    if (detections[jj] == gesture_id)
                        winner_cnt += 1;
                }

                // Ensure we have enough recognitions.
                if (winner_cnt < parameters.repeat_cnt)
                    continue;

                // If nothing was detected, skip rest.
                if (gesture_id == -1)
                    continue;

                // Hurray! A gesture is recognized!
                // Hopefully it's the right one too!!
                cmds[cmds.Count - 1].add(gesture_id);
                detections.Clear();
                buffer.Clear();
            }

            // Mark bad commands, situations where the participant
            // made a mistake or tracking was lost. We know the
            // command was bad because the protector asked the
            // participant to repeat the gesture, but a new command
            // ID is assigned to the sequence.
            for (int ii = 1;
        ii < cmds.Count;
                 ii++)
            {
                if (cmds[ii].expected_id == cmds[ii - 1].expected_id)
                {
                    CommandResults temp = cmds[ii - 1];
                    temp.ignore = true;
                    cmds[ii - 1] = temp;
                }
            }

            // Put all results in confusion matrices.
            ConfusionMatrices ret = new ConfusionMatrices(ds);

            for (int ii = 0; ii < cmds.Count; ii++)
            {
                if (cmds[ii].ignore)
                    continue;

                cmds[ii].update_confusion_matrices(ret);
            }

            return ret;
        }


        /**
         * Evaluate all user study sessions for a given device type.
         */
        public static void evaluate_sessions(DeviceType device)
        {
            List<ConfusionMatrices> confusion_matrices = new List<ConfusionMatrices>();
            int[] participants = get_participant_list(device);

            for (int i = 0; i < participants.Length - 1; i++)
            {
                ConfusionMatrices cm = evaluate_session(
                   device,
                   participants[i]);

                confusion_matrices.Add(cm);

                int idx = confusion_matrices.Count - 1;

                Console.WriteLine("Participant: " + participants[i]);

                ResultT resultss = confusion_matrices[idx].Results();
                resultss.Print();
                Console.WriteLine();
            }


            // put all results into first confusion
            // matrix
            for (int ii = 1; ii < confusion_matrices.Count; ii++)
                confusion_matrices[0].AddResult(confusion_matrices[ii]);

            Console.WriteLine("Aggregate results:");

            ResultT result = confusion_matrices[0].Results();
            result.Print();
            Console.WriteLine();
        }
    }


    public struct configuartion_parameters_t
    {
        /**
         * The input device sampling rate.
         */
        public int fps;

        /**
         * The maximum amount of data (in seconds) that is collected
         * and passed to the recognizer. Since the buffer is cleared
         * when a gesture is recognized, the buffer may be shorter.
         */
        public double sliding_window_s;

        /**
         * Sliding window converted into frames based on FPS.
         */
        public int sliding_window_frame_cnt;

        /**
         * The recognizer is called once per this many frames.
         */
        public int update_interval;

        /**
         * A gesture has to have the best Jackknife score this many
         * times before being officially recognized by this application.
         */
        public int repeat_cnt;

        /**
         *
         */
        public configuartion_parameters_t(DeviceType device)
        {
            sliding_window_s = -1;
            fps = -1;
            update_interval = -1;
            sliding_window_frame_cnt = -1;
            repeat_cnt = -1;

            if (device == DeviceType.KINECT)
            {
                fps = 30;
                sliding_window_s = 2.0;
                update_interval = 5;
                repeat_cnt = 3;
            }
            else if (device == DeviceType.LEAP_MOTION)
            {
                fps = 30;
                sliding_window_s = 4.0;
                update_interval = 10;
                repeat_cnt = 4;
            }

            sliding_window_frame_cnt = (int)((double)fps * sliding_window_s);
        }
    };



}
