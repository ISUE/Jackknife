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

#pragma once

#include <iostream>
#include <algorithm>

#include "jackknife.h"
#include "jackknife_template.h"

namespace Jackknife {

/**
 *
 */
class Jackknife
{
private:

    /**
     * Vector of all gesture templates.
     */
    std::vector<JackknifeTemplate::Ptr> templates;

    /**
     * The DTW algorithm. Awesome!
     */
    inline double DTW(
        std::vector<Vector> &v1,
        std::vector<Vector> &v2)
    {
		std::vector <std::vector<double>> cost(v1.size() + 1);
		for (int i = 0; i < cost.size(); i++)
			cost[i] = std::vector<double>(v2.size() + 1);

        // initialize matrix
        for (int ii = 0;
             ii <= v1.size();
             ii++)
        {
            for (int jj = 0;
                 jj <= v2.size();
                 jj++)
            {
                cost[ii][jj] = std::numeric_limits<double>::infinity();
            }
        }

        cost[0][0] = 0.0;

        // using DP to find solution
        for (int ii = 1;
             ii <= v1.size();
             ii++)
        {
            for (int jj = std::max(1, ii - (int)blades.radius);
                 jj <= std::min((int)v2.size(), ii + (int)blades.radius);
                 jj++)
            {
                // pick minimum cost path (neighbor) to
                // extend to this ii, jj element
                double minimum = std::min(
                    std::min(
                        cost[ii-1][jj],    // repeat v1 element
                        cost[ii][jj-1]),   // repeat v2 element
                    cost[ii-1][jj-1]);     // don't repeat either

                cost[ii][jj] = minimum;

                if(blades.inner_product)
                {
                    cost[ii][jj] += 1.0 - v1[ii-1].dot(v2[jj-1]);
                }
                else if(blades.euclidean_distance)
                {
                    cost[ii][jj] += v1[ii-1].l2norm2(v2[jj-1]);
                }
                else
                {
                    assert(0);
                }
            }
        }

        return cost[v1.size()][v2.size()];
    }

    /**
     * Find the lower bound score for a candidate
     * against a given template.
     */
    double lower_bound(
        std::vector<Vector> &vecs,
        JackknifeTemplate::Ptr &t)
    {
        double lb = 0.0; // lower bound
        int component_cnt = vecs[0].data.size();

        for (int ii = 0;
             ii < vecs.size();
             ii++)
        {
            double cost = 0.0;

            for (int jj = 0;
                jj < component_cnt;
                jj++)
            {
                if(blades.inner_product)
                {
                    if(vecs[ii].data[jj] < 0.0)
                    {
                        cost += vecs[ii].data[jj] * t->lower[ii].data[jj];
                    }
                    else
                    {
                        cost += vecs[ii].data[jj] * t->upper[ii].data[jj];
                    }
                }
                else if(blades.euclidean_distance)
                {
                    double diff = 0.0;

                    if(vecs[ii].data[jj] < t->lower[ii].data[jj])
                    {
                        diff = vecs[ii].data[jj] - t->lower[ii].data[jj];
                    }
                    else if(vecs[ii].data[jj] > t->upper[ii].data[jj])
                    {
                        diff = vecs[ii].data[jj] - t->upper[ii].data[jj];
                    }

                    cost += (diff * diff);
                }
                else
                {
                    assert(0);
                }
            }

            // inner products are bounded
            if(blades.inner_product)
            {
                cost = 1.0 - std::min(1.0, std::max(-1.0, cost));
            }

            lb += cost;
        }

        return lb;
    }

public:

    /**
     * Jackknife recognizer shared pointer type.
     */
    typedef std::shared_ptr<Jackknife> Ptr;

    /**
     * The set of measures and features used in this instance
     * of Jackknife.
     */
    jackknife_blades_t blades;

    /**
     * Constructor.
     */
    Jackknife(jackknife_blades_t blades)
    {
        assert(blades.inner_product ^ blades.euclidean_distance);
        this->blades = blades;
    }

    /**
     * Turn sample into template.
     */
    void add_template(Sample::Ptr sample)
    {
        JackknifeTemplate::Ptr t = JackknifeTemplate::Ptr(
            new JackknifeTemplate(
                blades,
                sample));
        templates.push_back(t);
    }

    /**
     * Learn a rejection threshold for each template.
     * Call after all templates have been added.
     *
     * See explanation in jackknife_train to understand
     * the input parameters.
     */
    void train(
        int gpsr_n,
        int gpsr_r,
        double beta);

    /**
     * Determine gesture class of the given sample.
     * The gesture id is returned.
     *
     * This function can be modified to return an n-best list, but
     * because of early rejection, such a list may not make sense.
     *
     */
    int classify(const std::vector<Vector> &trajectory)
    {
        // extract important information from candidate sample
        JackknifeFeatures features(
            blades,
            trajectory);

        // now find the lower bound for this candidate
        // compared against each template.
        // also cache correction factor scores
        int template_cnt = templates.size();
        for(int tt = 0;
            tt < template_cnt;
            tt++)
        {
            double cf = 1.0;

            if(blades.cf_abs_distance)
            {
                cf *= 1.0 / std::max(
                    0.01,
                    features.abs.dot(templates[tt]->features.abs));
            }

            if(blades.cf_bb_widths)
            {
                cf *= 1.0 / std::max(
                    0.01,
                    features.bb.dot(templates[tt]->features.bb));
            }

            templates[tt]->cf = cf;

            if(blades.lower_bound)
            {
                templates[tt]->lb = cf * lower_bound(
                    features.vecs,
                    templates[tt]);
            }
        }

        // Now sort all templates based on their lower bound
        // results. This helps to improve culling later on.
        std::sort(
            templates.begin(),
            templates.end(),
            SortTemplates());

        // track best result so far
        double best = std::numeric_limits<double>::infinity();
        int ret = -1;

        for(int tt = 0;
            tt < template_cnt;
            tt++)
        {
            // cull based on lower bound vs rejection threshold
            if(templates[tt]->lb > templates[tt]->rejection_threshold)
                continue;

            // cull based on lower bound vs best score
            if(templates[tt]->lb > best)
                continue;

            // forced to do full evaluation
            double score = templates[tt]->cf;

            // note that the vecs can be either direction vectors
            // or points, depending on selected measure
            score *= DTW(
                features.vecs,
                templates[tt]->features.vecs);

            // only accept if below rejection threshold
            if(score > templates[tt]->rejection_threshold)
                continue;

            // save, if best result
            if(score < best)
            {
                best = score;
                ret = templates[tt]->gesture_id;
            }
        }

        return ret;
    }

    /**
     *
     */
    int classify(const Sample::Ptr &sample)
    {
        int ret = classify(sample->trajectory);
        return ret;
    }
};

} // Jackknife namespace
