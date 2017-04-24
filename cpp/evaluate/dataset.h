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

#include <string>
#include <iostream>
#include <cstring>
#include "vector.h"
#include "sample.h"

/**
 * Forward declaration.
 */
class Dataset;

/**
 * Recurse through a directory and load up all of the samples:
 * <path>/sub_* /gesture name/ex_*
 *
 * Also, all subject and gesture names are recorded and
 # enumerated automatically.
 */
extern std::shared_ptr<Dataset> load_dataset(std::string path);

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
extern std::shared_ptr<Dataset> load_subject_dataset(
    std::shared_ptr<Dataset> dataset,
    std::string subject_path);

/**
 * A class to help manage all samples of a dataset.
 */
class Dataset
{

public:

    /**
     * Dataset shared pointer type.
     */
    typedef std::shared_ptr<Dataset> Ptr;

    /**
     * Enumeration of gesture string names.
     */
    std::vector<std::shared_ptr<std::string>> gestures;


    /**
     * Enumeration of subject names.
     */
    std::vector<std::shared_ptr<std::string>> subjects;


    /**
     * List of all samples from the dataset.
     */
    std::vector<Jackknife::Sample::Ptr> samples;


    /**
     * Use samples_by_gesture[gesture_id] to get all the samples
     * from a particular gesture class.
     */
    std::vector< std::vector<Jackknife::Sample::Ptr> > samples_by_gesture;

    /**
     *
     */
    Dataset()
    {
    }

    /**
     *
     */
    ~Dataset()
    {
        //
        // Clean up gesture list.
        //
        gestures.clear();

        //
        // Clean up subject list.
        //
        subjects.clear();

        //
        // Clean up samples.
        //
        samples.clear();

        //
        // Clean up partitioned sample lists.
        //
        std::vector< std::vector<Jackknife::Sample::Ptr> >::iterator it;
        for(it = samples_by_gesture.begin();
            it != samples_by_gesture.end();
            it ++)
        {
            (*it).clear();
        }
        samples_by_gesture.clear();
    }

    /**
     * Add gesture name to database if it does not
     * already exist and returns its enumerated id.
     */
    int add_gesture(std::string gname)
    {
        int ii;

        for(ii = 0;
            ii < gestures.size();
            ii ++)
        {
            int result = gestures[ii]->compare(gname);
            if(result == 0)
                return ii;
        }

        // push gesture name into list and make room
        // for sample list in samples_by_gesture, which
        // will be populated later add sample is called
        gestures.push_back(std::shared_ptr<std::string>(new std::string(gname)));
        samples_by_gesture.push_back(std::vector<Jackknife::Sample::Ptr>());
        return ii;
    }

    /**
     * Add subject to database if it does not already exist
     * and returns their enumerated id.
     */
    int add_subject(std::shared_ptr<std::string> sname)
    {
        int ii;

        for(ii = 0;
            ii < subjects.size();
            ii ++)
        {
            int result = subjects[ii]->compare(*sname.get());
            if(result == 0)
                return ii;
        }

        subjects.push_back(sname);
        return ii;
    }

    /**
     * Add a new sample to the dataset. The gesture name
     * and subject must have already been added.
     */
    void add_sample(
        Jackknife::Sample::Ptr sample,
        int subject_id,
        int gesture_id)
    {
        samples.push_back(sample);

        assert(gesture_id < samples_by_gesture.size());
        samples_by_gesture[gesture_id].push_back(sample);
    }

    /**
     *
     */
    int sample_cnt() const
    {
        return samples.size();
    }

    /**
     *
     */
    int gesture_cnt() const
    {
        return gestures.size();
    }

    /**
     *
     */
    int subject_cnt() const
    {
        return subjects.size();
    }

    /**
     *
     */
    int gesture_name_to_id(std::string gname)
    {
        for (int ii = 0;
             ii < gestures.size();
             ii++)
        {
            if(gestures[ii]->compare(gname) == 0)
            {
                return ii;
            }
        }

        assert(0);
        return -1;
    }

    /**
     *
     */
    void dump_catalog(void)
    {
		std::cout << "Subject Count: " << subjects.size() << std::endl
			      << "Sample Count: "  << samples.size() << std::endl
			      << "Gesture Count: " << samples_by_gesture.size() << std::endl;

		for (int ii=0;
             ii < samples_by_gesture.size();
             ii++)
        {
            std::string gname = *gestures[ii];
			std::cout <<
				gname.c_str() << ": " << samples_by_gesture[ii].size() << std::endl;
        }
    }
};
