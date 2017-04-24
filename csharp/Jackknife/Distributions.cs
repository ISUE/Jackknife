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

﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Jackknife
{
    /**
     * Helper structure to manage statistics when calculating
     * thresholds.
     */
    public struct Distributions
    {
        /**
         * Specifies the range of the histogram: [0.0, max_score].
         */
        public double MaxScore { get; set; }

        /**
         * Distribution of negative sample scores
         */
        public Vector Neg { get; set; }

        /**
         * Distribution of positive sample scores
         */
        public Vector Pos { get; set; }

        /**
         * The range [0.0, max_score] is divided into bin_cnt bins.
         */
        public Distributions(double maxScore, int binCount)
        {
            Neg = new Vector(0, binCount);
            Pos = new Vector(0, binCount);

            this.MaxScore = maxScore;
        }

        /**
         * Convert score into a bin number.
         */
        public int Bin(double score)
        {
            return Math.Min((int)(score * Neg.Size / MaxScore), Neg.Size - 1);
        }

        /**
        * Increment negative score histogram.
        */
        public void AddNegativeScore(double score)
        {
            Neg[Bin(score)] += 1;
        }

        /**
         * Increment positive score histogram.
         */
        public void AddPositiveScore(double score)
        {
            Pos[Bin(score)] += 1;
        }

        /**
         * Estimate a rejection threshold based on a target
         * F-score. Beta is the parameter that controls the
         * balance between false positives and false negatives.
         *
         * More information can be found on Wikipedia:
         *      https://en.wikipedia.org/wiki/F1_score
         *
         * Specifically F_Beta "measures the effectiveness of retrieval with
         * respect to a user who attaches Beta times as much importance to
         * recall as precision."
         */
        public double RejectionThreshold(double beta)
        {
            Neg /= Neg.Sum();
            Neg.CumulativeSum();

            if (Math.Abs(Neg[Neg.Size - 1]) - 1 >= 0.00001)
                throw new Exception("Assertion failed: Math.Abs(Neg[Neg.Size - 1]) - 1 >= 0.00001");

            Pos /= Pos.Sum();
            Pos.CumulativeSum();

            if (Math.Abs(Pos[Pos.Size - 1]) - 1 >= 0.00001)
                throw new Exception("Assertion failed: Math.Abs(Pos[Pos.Size - 1]) - 1 >= 0.00001");

            double alpha = 1 / (1 + beta * beta);

            Vector precision = Pos / (Pos + Neg);
            Vector recall = Pos;

            double bestScore = 0;
            int bestIndex = -1;

            for (int i = 0; i < Neg.Size; i++)
            {
                double E, FScore;
                E = (alpha / precision[i]) + ((1 - alpha) / recall[i]);
                FScore = 1 / E;

                if (FScore > bestScore)
                {
                    bestScore = FScore;
                    bestIndex = i;
                }
            }

            double ret = bestIndex + 0.5;
            ret *= MaxScore / Neg.Size;

            return ret;
        }
    }
}
