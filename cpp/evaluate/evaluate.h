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

#include <assert.h>
#include <cmath>
#include "dataset.h"
#include "vector.h"
#include "jackknife.h"
#include "confusion_matrices.h"

/**
 * Enumeration of device types used in user study.
 */
typedef enum {

    KINECT = 0,
    LEAP_MOTION = 1,

} device_type_t;

/**
 * Run a simple user independent test on segmented training data
 * collected during the user study.
 */
void user_indepedent_test(device_type_t device);

/**
 * Evaluate the Jackknife recognizer on videos collected during the
 * user study. Details are in the paper.
 */
void evaluate_sessions(device_type_t device);
