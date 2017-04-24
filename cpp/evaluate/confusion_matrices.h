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

#include "evaluate.h"

/**
 * Enumeration of error types.
 */
enum {

    TRUE_POSITIVE = 0,
    FALSE_NEGATIVE = 1,
    FALSE_POSITIVE = 2,
    TRUE_NEGATIVE = 3,
};

/**
 * Place to store confusion matrices analysis.
 */
typedef struct {

    double recall;
    double precision;
    double fall_out;
    double f1;

    void print()
    {
        printf("Recall:    %3.2lf%%\n", recall * 100.0);
        printf("Precision: %3.2lf%%\n", precision * 100.0);
        printf("Fall Out:  %3.2lf%%\n", fall_out * 100.0);
        printf("F1 score:  %3.2lf%%\n", f1 * 100.0);
    }

} results_t;

/**
 * Helper structure for tracking confusion matrices.
 */
class ConfusionMatrices
{
public:

    /**
     * A confusion matrix for each gesture.
     */
    std::vector<Jackknife::Vector> confusion_matrices;

    /**
     * Number of results added;
     */
    int entries;

    ConfusionMatrices(const Dataset::Ptr &ds)
    {
        int gesture_cnt = ds->gesture_cnt();

        entries = 0;

        for (int ii = 0;
             ii < gesture_cnt;
             ii++)
        {
            confusion_matrices.push_back(Jackknife::Vector(0, 4));
        }
    }

    /**
     * Add a single result to confusion matrices.
     */
    void add_result(
        int expected_id,
        int detected_id)
    {
        assert(expected_id < (int)confusion_matrices.size());
        assert(detected_id < (int)confusion_matrices.size());

        entries ++;

        for (int gesture_id = 0;
            gesture_id < confusion_matrices.size();
            gesture_id++)
        {
            if(gesture_id == expected_id)
            {
                // This is a special case for our sessions logic,
                // where if a gesture was already detected,
                // we treat a second detection as a false positive.
                if(detected_id == -2)
                    confusion_matrices[gesture_id][FALSE_POSITIVE] += 1.0;
                else if(gesture_id == detected_id)
                    confusion_matrices[gesture_id][TRUE_POSITIVE] += 1.0;
                else
                    confusion_matrices[gesture_id][FALSE_NEGATIVE] += 1.0;
            }
            else
            {
                if(gesture_id == detected_id)
                    confusion_matrices[gesture_id][FALSE_POSITIVE] += 1.0;
                else
                    confusion_matrices[gesture_id][TRUE_NEGATIVE] += 1.0;
            }
        }
    }

    /**
     * Use this to average together confusion matrices
     * over different tests.
     */
    void add_result(const ConfusionMatrices &other)
    {
        assert(confusion_matrices.size() == other.confusion_matrices.size());
        entries ++;

        for (int ii = 0;
             ii < other.confusion_matrices.size();
             ii ++)
        {
            double sum = other.confusion_matrices[ii].sum();
            confusion_matrices[ii] += other.confusion_matrices[ii] / sum;
        }
    }

    /**
     * Extract statistics from the confusion matrices.
     */
    results_t results(void) const
    {
        // aggregate confusion matrix
        Jackknife::Vector cm(0.0, 4);

        for (int ii = 0;
             ii < confusion_matrices.size();
             ii ++)
        {
            cm += confusion_matrices[ii];
        }

        results_t ret;

        ret.recall = cm[TRUE_POSITIVE];
        ret.recall /= (cm[TRUE_POSITIVE] + cm[FALSE_NEGATIVE]);

        ret.precision = cm[TRUE_POSITIVE];
        ret.precision /= (cm[TRUE_POSITIVE] + cm[FALSE_POSITIVE]);

        ret.fall_out = cm[FALSE_POSITIVE];
        ret.fall_out /= (cm[FALSE_POSITIVE] + cm[TRUE_NEGATIVE]);

        ret.f1 = 2.0 * cm[TRUE_POSITIVE];
        ret.f1 /= (2.0 * cm[TRUE_POSITIVE] + cm[FALSE_POSITIVE] + cm[FALSE_NEGATIVE]);

        return ret;
    }
};
