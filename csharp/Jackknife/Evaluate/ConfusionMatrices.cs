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

    public enum ConfusionMatrixEntry : int
    {
        TRUE_POSITIVE = 0,
        FALSE_NEGATIVE = 1,
        FALSE_POSITIVE = 2,
        TRUE_NEGATIVE = 3,
    };

    public struct ResultT
    {
        public double Recall { get; set; }
        public double Precision { get; set; }
        public double FallOut { get; set; }
        public double F1 { get; set; }

        public void Print()
        {
            Console.WriteLine("Recall:    {0:P2}", Recall);
            Console.WriteLine("Precision: {0:P2}", Precision);
            Console.WriteLine("Fall Out:  {0:P2}", FallOut);
            Console.WriteLine("F1 score:  {0:P2}", F1);
        }

    }

    public class ConfusionMatrices
    {

        List<Vector> confusion_matrices;

        /**
         * Number of results added;
         */
        int entries;

        public ConfusionMatrices(Dataset ds)
        {
            int gesture_cnt = ds.Gestures.Count;
            confusion_matrices = new List<Vector>();
            entries = 0;

            for (int i = 0; i < gesture_cnt; i++)
                confusion_matrices.Add(new Vector(0, 4));
        }

        /**
         * Add a single result to confusion matrices.
         */
        public void AddResult(int expected_id, int detected_id)
        {
            if (expected_id >= confusion_matrices.Count)
                throw new ArgumentException("expected_id >= confusion_matrices.Count");

            if (detected_id >= confusion_matrices.Count)
                throw new ArgumentException("detected_id >= confusion_matrices.Count");

            entries++;

            for (int gesture_id = 0; gesture_id < confusion_matrices.Count; gesture_id++)
            {
                if (gesture_id == expected_id)
                {
                    // This is a special case for our sessions logic,
                    // where if a gesture was already detected,
                    // we treat a second detection as a false positive.
                    if (detected_id == -2)
                        confusion_matrices[gesture_id][(int)ConfusionMatrixEntry.FALSE_POSITIVE] += 1.0;
                    else if (gesture_id == detected_id)
                        confusion_matrices[gesture_id][(int)ConfusionMatrixEntry.TRUE_POSITIVE] += 1.0;
                    else
                        confusion_matrices[gesture_id][(int)ConfusionMatrixEntry.FALSE_NEGATIVE] += 1.0;
                }
                else
                {
                    if (gesture_id == detected_id)
                        confusion_matrices[gesture_id][(int)ConfusionMatrixEntry.FALSE_POSITIVE] += 1.0;
                    else
                        confusion_matrices[gesture_id][(int)ConfusionMatrixEntry.TRUE_NEGATIVE] += 1.0;
                }
            }
        }

        /**
         * Use this to average together confusion matrices
         * over different tests.
         */
        public void AddResult(ConfusionMatrices other)
        {
            if (confusion_matrices.Count != other.confusion_matrices.Count)
                throw new Exception("confusion_matrices.Count == other.confusion_matrices.Count");

            entries++;

            for (int ii = 0; ii < other.confusion_matrices.Count; ii++)
            {
                double sum = other.confusion_matrices[ii].Sum();
                confusion_matrices[ii] += other.confusion_matrices[ii] / sum;
            }
        }

        /**
        * Extract statistics from the confusion matrices.
        */
        public ResultT Results()
        {
            // aggregate confusion matrix
            Vector cm = new Vector(0.0, 4);

            for (int ii = 0; ii < confusion_matrices.Count; ii++)
            {
                cm += confusion_matrices[ii];
            }

            ResultT ret = new ResultT();

            ret.Recall = cm[(int)ConfusionMatrixEntry.TRUE_POSITIVE];
            ret.Recall /= (cm[(int)ConfusionMatrixEntry.TRUE_POSITIVE] + cm[(int)ConfusionMatrixEntry.FALSE_NEGATIVE]);

            ret.Precision = cm[(int)ConfusionMatrixEntry.TRUE_POSITIVE];
            ret.Precision /= (cm[(int)ConfusionMatrixEntry.TRUE_POSITIVE] + cm[(int)ConfusionMatrixEntry.FALSE_POSITIVE]);

            ret.FallOut = cm[(int)ConfusionMatrixEntry.FALSE_POSITIVE];
            ret.FallOut /= (cm[(int)ConfusionMatrixEntry.FALSE_POSITIVE] + cm[(int)ConfusionMatrixEntry.TRUE_NEGATIVE]);

            ret.F1 = 2.0 * cm[(int)ConfusionMatrixEntry.TRUE_POSITIVE];
            ret.F1 /= (2.0 * cm[(int)ConfusionMatrixEntry.TRUE_POSITIVE] + cm[(int)ConfusionMatrixEntry.FALSE_POSITIVE] + cm[(int)ConfusionMatrixEntry.FALSE_NEGATIVE]);

            return ret;
        }
    }
}
