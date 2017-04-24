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
    public class Dataset
    {
        /**
         * Enumeration of gesture string names.
         */
        public List<string> Gestures { get; set; }

        /**
         * Enumeration of subject names.
         */
        public List<string> Subjects { get; set; }

        /**
         * List of all samples from the dataset.
         */
        public List<Sample> Samples { get; set; }

        /**
         * Use samples_by_gesture[gesture_id] to get all the samples
         * from a particular gesture class.
         */
        public List<List<Sample>> SamplesByGesture { get; set; }

        /**
         *
         */
        public Dataset()
        {
            Gestures = new List<string>();
            Subjects = new List<string>();
            Samples = new List<Sample>();
            SamplesByGesture = new List<List<Sample>>();
        }

        /**
         * Add gesture name to database if it does not
         * already exist and returns its enumerated id.
         */
        public int AddGesture(string gname)
        {
            int i;

            for (i = 0; i < Gestures.Count; i++)
            {
                if (Gestures[i] == gname)
                    return i;
            }

            // push gesture name into list and make room
            // for sample list in samples_by_gesture, which
            // will be populated later add sample is called
            Gestures.Add(gname);
            SamplesByGesture.Add(new List<Sample>());
            return i;
        }

        /**
        * Add subject to database if it does not already exist
        * and returns their enumerated id.
        */
        public int AddSubject(string sname)
        {
            int i;

            for (i = 0; i < Subjects.Count; i++)
                if (Subjects[i] == sname)
                    return i;

            Subjects.Add(sname);
            return i;
        }

        /**
         * Add a new sample to the dataset. The gesture name
         * and subject must have already been added.
         */
        public void AddSample(Sample sample, int subject_id, int gesture_id)
        {
            Samples.Add(sample);

            if (gesture_id >= SamplesByGesture.Count)
                throw new Exception("gesture_id >= SamplesByGesture.Count");

            SamplesByGesture[gesture_id].Add(sample);
        }

        /**
         *
         */
        public int GestureNameToId(string gname)
        {
            for (int i = 0; i < Gestures.Count; i++)
            {
                if (Gestures[i] == gname)
                    return i;
            }

            throw new Exception("Gesture not found");
        }

        /**
         *
         */
        public void DumpCatalog()
        {
            Console.WriteLine("Subject Count: " + Subjects.Count);
            Console.WriteLine("Sample Count: " + Samples.Count);
            Console.WriteLine("Gesture Count: " + SamplesByGesture.Count);

            for (int i = 0; i < SamplesByGesture.Count; i++)
            {
                string gname = Gestures[i];
                Console.WriteLine(gname + ": " + SamplesByGesture[i].Count);
            }
        }

        /**
         *
         */
        public static Sample LoadSampleFile(string path, int subject_id, int gesture_id)
        {
            Sample ret;

            var lines = File.ReadAllLines(path).ToList();

            if (lines.Last() != "####")
                lines.Add("####");

            string gname = lines[0];
            string pt_cnt_str = lines[1];
            string hash_line = lines[2];

            if (hash_line != "####")
                throw new Exception("hash_line == \"####\"");

            string line;

            // convert point count to integer
            int pt_cnt = int.Parse(pt_cnt_str);

            //
            // create a new sample object
            //
            ret = new Sample(subject_id, gesture_id, 0);

            List<double> pt = new List<double>();
            List<Vector> points = new List<Vector>();

            for (int i = 3; i < lines.Count; i++)
            {
                // read next line in file
                line = lines[i];

                // check for empty line or separator
                if (string.IsNullOrEmpty(line) || line == "####")
                {
                    // add pt to trajectory
                    points.Add(new Vector(pt));
                    pt.Clear();

                    // end of file
                    if (string.IsNullOrEmpty(line))
                        break;

                    continue;
                }

                // else read in x, y, z
                var pos = line.Split(',');
                pt.Add(double.Parse(pos[0]));
                pt.Add(double.Parse(pos[1]));
                pt.Add(double.Parse(pos[2]));

            }

            // add trajectory and return
            ret.AddTrajectory(points);
            return ret;
        }

        /**
         * Helper tuple class.
         */
        public class Tuple<T1, T2>
        {
            public T1 First { get; private set; }
            public T2 Second { get; private set; }
            internal Tuple(T1 first, T2 second)
            {
                First = first;
                Second = second;
            }
        }

        /**
         * Recurse through a *subject* directory and load up all of the samples:
         * <path>/sub_* /gesture name/ex_*
         *
         * Also, all subject and gesture names are recorded and enumerated.
         *
         * This function can be called multiple times with to build a single data set.
         * First call with dataset=NULL to allocate a new Dataset and then pass in
         * on subsequent calls.
         */
        public static Dataset LoadSubjectDataset(Dataset dataset, string subject_path)
        {
            // the path and gesture name
            List<Tuple<string, string>> gestures = new List<Tuple<string, string>>();
            Dataset ret = dataset; // alias

            //
            // Create a new data set if required.
            //
            if (ret == null)
            {
                ret = new Dataset();
            }

            //
            // Get subject name from path string ".../Sub_<subject_name>".
            //
            int idx;
            idx = subject_path.IndexOf("Sub_");

            if (idx == -1)
            {
                Console.WriteLine("exit 0\n");
                return ret;
            }

            idx += 4;

            //
            // Add subject to database, get sub id.
            //
            string tmp = subject_path.Substring(idx);
            string subject_name = tmp;
            int subject_id = ret.AddSubject(subject_name);

            //
            // Find all gesture directories.
            //
            if (!Directory.Exists(subject_path))
                throw new Exception("Directory does not exist");

            var dirs = Directory.GetDirectories(subject_path);

            foreach (var dir in dirs)
            {
                string gname = new DirectoryInfo(dir).Name;

                gestures.Add(new Tuple<string, string>(dir, gname));
            }

            //
            // Load each gesture directory.
            //
            for (int ii = 0; ii < gestures.Count; ii++)
            {
                List<string> sample_paths = new List<string>();

                // Get a gesture ID.
                int gesture_id = ret.AddGesture(gestures[ii].Second);

                //
                // Now find all examples.
                //
                var path = gestures[ii].First;
                var files = Directory.GetFiles(path);
                foreach (var file in files)
                {
                    string sname = Path.GetFileNameWithoutExtension(file);

                    // Fixme
                    if (!sname.StartsWith("ex"))
                        continue;

                    sample_paths.Add(file);
                }

                //
                // Again, we prefer to have samples sorted.
                //
                sample_paths.Sort();

                for (int sample_no = 0; sample_no < sample_paths.Count; sample_no++)
                {
                    //
                    // finally! load a sample file
                    //
                    Sample sample;
                    sample = LoadSampleFile(
                        sample_paths[sample_no],
                        subject_id,
                        gesture_id);

                    if (sample == null)
                        continue;

                    ret.AddSample(
                        sample,
                        subject_id,
                        gesture_id);
                }
            }

            return ret;
        }

        /**
         * Largely follows example from:
         */
        public static Dataset LoadDataset(string path)
        {
            List<string> subject_paths = new List<string>();
            Dataset ret = new Dataset();

            //
            // Find all subject directories.
            //
            if (!Directory.Exists(path))
                throw new Exception("Path does not exist");

            var dirs = Directory.GetDirectories(path);

            foreach (var dir in dirs)
            {
                string folderName = new DirectoryInfo(dir).Name;
                if (!folderName.StartsWith("Sub_"))
                    continue;
                subject_paths.Add(dir);
            }

            subject_paths.Sort();

            //
            // Load the subject directories.
            //
            for (int ii = 0; ii < subject_paths.Count; ii++)
            {
                ret = LoadSubjectDataset(ret, subject_paths[ii]);
            }

            return ret;
        }
    }
}
