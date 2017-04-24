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

#include <cstring>
#include "mathematics.h"
#include "jackknife.h"

namespace Jackknife {

/**
 * Helper structure to manage statistics when calculating
 * thresholds.
 */
struct Distributions
{
    /**
     * Specifies the range of the histogram: [0.0, max_score].
     */
    double max_score;

    /**
     * Distribution of negative sample scores
     */
    Vector neg;

    /**
     * Distribution of positive sample scores
     */
    Vector pos;

    /**
     * The range [0.0, max_score] is divided into bin_cnt bins.
     */
    Distributions(
        double max_score,
        int bin_cnt)
    {
        neg = Vector(0.0, bin_cnt);
        pos = Vector(0.0, bin_cnt);
        this->max_score = max_score;
    }

    /**
     * Convert score into a bin number.
     */
    int bin(double score)
    {
        int ret = std::min(
            (int)(score * ((double)neg.size()) / max_score),
            neg.size() - 1);

        return ret;
    }

    /**
     * Increment negative score histogram.
     */
    void add_negative_score(double score)
    {
        neg[bin(score)] += 1.0;
    }

    /**
     * Increment positive score histogram.
     */
    void add_positive_score(double score)
    {
        pos[bin(score)] += 1.0;
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
    double rejection_threshold(double beta)
    {
        // Convert negative distribution into CDF.
        neg /= neg.sum();
        neg.cumulative_sum();
        assert(std::abs(neg[neg.size() - 1] - 1.0) < .00001);

        // Convert positive distribution into CDF.
        pos /= pos.sum();
        pos.cumulative_sum();
        assert(std::abs(pos[pos.size() - 1] - 1.0) < .00001);

        double alpha = 1.0 / (1.0 + beta * beta);

        // Percentage of positives
        // that are actually true positive.
        Vector precision = pos / (pos + neg);

        // Percentage of true positives.
        Vector recall = pos;

        double best_score = 0.0;
        int best_idx = -1;

        for (int ii = 0;
             ii < neg.size();
             ii++)
        {
            double E, f_score;

            // This equation comes from Wikipedia
            E = (alpha / precision[ii]) + ((1.0 - alpha) / recall[ii]);
            f_score = 1.0 / E;

            // save best f-score result
            if(f_score > best_score)
            {
                best_score = f_score;
                best_idx = ii;
            }
        }

        double ret = (double)best_idx + 0.5;
        ret *= max_score / (double)neg.size();
        return ret;
    }
};

/**
 *
 */
void Jackknife::train(
    int gpsr_n,
    int gpsr_r,
    double beta)
{
    int template_cnt = templates.size();
    std::vector<struct Distributions> distributions;
    std::vector<Vector> synthetic;

    double worst_score = 0.0;

    //
    // Create negative samples.
    //
    for(int ii = 0;
        ii < 1000;
        ii++)
    {
        synthetic.clear();

        // Splice two samples together
        // to create one negative sample.
        for (int jj = 0;
             jj < 2;
             jj++)
        {
            int tt = std::rand() % template_cnt;
            Sample::Ptr s = templates[tt]->sample;

            int len = s->trajectory.size();
            int start = std::rand() % (len / 2);

            for(int kk = 0;
                kk < len / 2;
                kk++)
            {
                synthetic.push_back(s->trajectory[start+kk]);
            }
        }

        JackknifeFeatures features(
            this->blades,
            synthetic);

        // and score it
        for (int tt = 0;
             tt < template_cnt;
             tt++)
        {
            double score = DTW(
                features.vecs,
                templates[tt]->features.vecs);

            if(worst_score < score)
                worst_score = score;

            if(ii > 50)
                distributions[tt].add_negative_score(score);
        }

        // Generate a few samples to get an estimate
        // of worst possible score.
        if(ii != 50)
            continue;

        // allocate distributions
        for (int tt = 0;
             tt < template_cnt;
             tt++)
        {
            distributions.push_back(
                Distributions(
                    worst_score,
                    1000));
        }
    }

    //
    // Create positive examples.
    //
    for(int tt = 0;
        tt < template_cnt;
        tt++)
    {
        for(int ii = 0;
            ii < 1000;
            ii++)
        {
            synthetic.clear();

            // Create a synthetic variation of the sample.
            gpsr(
                templates[tt]->sample->trajectory,
                synthetic,
                gpsr_n,
                0.25,
                gpsr_r);

            JackknifeFeatures features(
                this->blades,
                synthetic);

            // and score it
            double score = DTW(
                features.vecs,
                templates[tt]->features.vecs);

            distributions[tt].add_positive_score(score);
        }
    }

    //
    // Now extract the rejection thresholds.
    //
    for (int tt = 0;
         tt < template_cnt;
         tt ++)
    {
        double threshold = distributions[tt].rejection_threshold(beta);
        templates[tt]->rejection_threshold = threshold;
    }
}

} // Jackknife namespace
