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

#include "dataset.h"
#include "jackknife.h"
#include "evaluate.h"

/**
 *
 */
int main(
    int argc,
    char *argv[])
{
    // This test doesn't make sense given
    // that there is a mix or left handed
    // and right handed participants and
    // we don't do any xforms.
    // user_indepedent_test(LEAP_MOTION);

    // These are okay though.

    std::cout << "---------[ KINECT UI TEST ] ---------"
              << std::endl
              << std::endl;

    user_indepedent_test(KINECT);

    std::cout << "---------[ LEAP MOTION SESSIONS TEST] ---------"
              << std::endl
              << std::endl;

    evaluate_sessions(LEAP_MOTION);

    std::cout << "---------[ KINECT SESSIONS TEST] ---------"
              << std::endl
              << std::endl;

    evaluate_sessions(KINECT);
}
