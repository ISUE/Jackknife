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

var device_type_t = {
    KINECT: 0,
    LEAP_MOTION: 1
};

/**
 * A simple user independent test.
 */
function user_indepedent_test(device) {
    // First, load all training data for one device type.
    var path;

    if (device == device_type_t.KINECT)
        path = "../datasets/jk2017/kinect/training/";
    else if (device == device_type_t.LEAP_MOTION)
        path = "../datasets/jk2017/leap_motion/training/";
    else
        assert(0);
    console.log("Loading Dataset");
    var ds = load_dataset(path);
    console.log("Loaded")
    var subject_cnt = ds.subject_cnt();
    var sample_cnt = ds.sample_cnt();
    console.log(subject_cnt + " " + sample_cnt)

    var cm = new ConfusionMatrices(ds);

    // iterate through all subjects
    for (var subject_id = 0; subject_id < subject_cnt; subject_id++) {
        var cm_individual = new ConfusionMatrices(ds);
        console.log("Participant: " + subject_id + "\n");

        // train a recognizer with the selected subject
        var blades = new jackknife_blades();

        //blades.set_ed_defaults();
        blades.set_ip_defaults();


        var jk = new Jackknife(blades);

        var template_cnt = 0.0;

        for (var sample_id = 0; sample_id < sample_cnt; sample_id++) {
            var sample = ds.samples[sample_id];

            if (sample.subject_id != subject_id)
                continue;

            jk.add_template(ds.samples[sample_id]);
        }

        // only train the recognize if you need
        // to set a rejection criteria
        // jk.train(4, 2, 1.0);

        // test the recognizer with all other samples
        for (var sample_id = 0; sample_id < sample_cnt; sample_id++) {
            var sample = ds.samples[sample_id];
            if (sample.subject_id == subject_id)
                continue;
            //gettimeofday(&start, NULL);
            var gid = jk.classify(sample);
            //gettimeofday(&end, NULL);

            //double seconds  = (double)(end.tv_sec  - start.tv_sec);
            //double useconds = (double)(end.tv_usec - start.tv_usec);
            //time_us += (seconds*1.0e6 + useconds);
            cm_individual.add_result(
                sample.gesture_id,
                gid);
        }

        cm.add_result(cm_individual);

        var results = cm_individual.results();
        results.print();
        console.log("\n");

        // std::cout << "result: " << accuracy << "%, ";
        // std::cout << "time: " << time_us / test_cnt;
        // std::cout << " us" << std::endl;
        // std::cout << std::endl;
    }

    console.log("Aggregate Results: \n");
    var results = cm.results();
    results.print();
    console.log("\n");
}
