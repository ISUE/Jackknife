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

/**
 * A class to help manage all samples of a dataset.
 */
function Dataset() {
    /**
     * Enumeration of gesture string names.
     */
    this.gestures = [];

    /**
     * Enumeration of subject names.
     */
    this.subjects = [];

    /**
     * List of all samples from the dataset.
     */
    this.samples = [];

    /**
     * Use samples_by_gesture[gesture_id] to get all the samples
     * from a particular gesture class.
     */
    this.samples_by_gesture = [];
}

/**
 * Add gesture name to database if it does not
 * already exist and returns its enumerated id.
 */
Dataset.prototype.add_gesture = function(gname) {
    var ii;

    for (ii = 0; ii < this.gestures.length; ii++) {
        if (this.gestures[ii] == gname)
            return ii;
    }

    // push gesture name into list and make room
    // for sample list in samples_by_gesture, which
    // will be populated later add sample is called
    this.gestures.push(gname);
    this.samples_by_gesture.push([]);
    return ii;
}

/**
 * Add subject to database if it does not already exist
 * and returns their enumerated id.
 */
Dataset.prototype.add_subject = function(sname) {
    var ii;

    for (ii = 0; ii < this.subjects.length; ii++) {
        if (this.subjects[ii] == sname)
            return ii;
    }

    this.subjects.push(sname);
    return ii;
}

/**
 * Add a new sample to the dataset. The gesture name
 * and subject must have already been added.
 */
Dataset.prototype.add_sample = function(sample, subject_id, gesture_id) {
    this.samples.push(sample);
    console.assert(gesture_id < this.samples_by_gesture.length);
    this.samples_by_gesture[gesture_id].push(sample);
}

Dataset.prototype.sample_cnt = function() {
    return this.samples.length;
}

Dataset.prototype.gesture_cnt = function() {
    return this.gestures.length;
}

Dataset.prototype.subject_cnt = function() {
    return this.subjects.length;
}

Dataset.prototype.gesture_name_to_id = function(gname) {
    for (var ii = 0; ii < this.gestures.length; ii++) {
        if (this.gestures[ii] == gname) {
            return ii;
        }
    }

    console.assert(0);
    return -1;
}

/**
 *
 */
Dataset.prototype.dump_catalog = function() {
    console.log("Subject Count: " + this.subjects.length +
        "\nSample Count: " + this.samples.length +
        "\nGesture Count " + this.samples_by_gesture.length + "\n");

    for (var ii = 0; ii < this.samples_by_gesture.length; ii++) {
        var gname = gestures[ii];
        console.log(gname + ": " + this.samples_by_gesture[ii].length + "\n");
    }
}


/**
 *
 */
load_dataset = function(path) {
    var fs = require('fs');

    var subject_paths = [];

    var ret = new Dataset();
    var directories = fs.readdirSync(path);
    console.assert(directories.length != 0);

    for (var ii = 0; ii < directories.length; ii++) {
        if (directories[ii].substring(0, 4) != "Sub_") {
            continue;
        }

        var spath = path;
        spath = spath + directories[ii];
        subject_paths.push(spath)
    }

    //
    // We want the subject names in order.
    //
    subject_paths.sort();

    //
    // Load the subject directories.
    //
    for (var ii = 0; ii < subject_paths.length; ii++) {
        load_subject_dataset(
            ret,
            subject_paths[ii]);
    }

    return ret;
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
load_subject_dataset = function(ret, subject_path) {
    var fs = require('fs');
    var gestures = [];

    //
    // Create a new data set if required.
    //
    if (ret == null) {
        ret = new Dataset();
    }

    //
    // Get subject name from path string ".../Sub_<subject_name>".
    //
    var idx;
    idx = subject_path.search("Sub_");

    if (idx < 0) {
        console.log("exit 0\n");
        return ret;
    }

    idx += 4;


    //
    // Add subject to database, get sub id.
    //
    var tmp = subject_path.substring(idx);
    var subject_name = tmp;
    var subject_id = ret.add_subject(subject_name);

    //
    // Find all gesture directories.
    //
    directories = fs.readdirSync(subject_path);
    console.assert(directories.length != 0);

    for (var ii = 0; ii < directories.length; ii++) {
        var gname = directories[ii];
        var gesture_path = subject_path + "/" + gname;

        var entry_stat = fs.statSync(gesture_path);

        if (!entry_stat.isDirectory()) {
            continue;
        }

        gestures.push({
            path: gesture_path,
            gesturename: gname
        });
    }
    //
    // Ensure gesture names are in order.
    //
    gestures.sort(function(a, b) {
        a.gesturename, b.gesturename
    });

    for (var ii = 0; ii < gestures.length; ii++) {
        var sample_paths = [];

        // Get a gesture ID.
        var gesture_id = ret.add_gesture(gestures[ii].gesturename);

        //
        // Now find all examples.
        //
        directories = fs.readdirSync(gestures[ii].path);
        console.assert(directories.length != 0);

        for (var jj = 0; jj < directories.length; jj++) {
            var sname = directories[jj];
            var sample_path = gestures[ii].path + "/" + sname;

            if (!fs.existsSync(sample_path))
            {
                console.log("HERE")
                continue;
            }
            if (sname.substring(0, 2) != "ex")
            {
                console.log("No, HERE")
                continue;
            }
            sample_paths.push(sample_path);
        }


        //
        // Again, we prefer to have samples sorted.
        //
        sample_paths.sort();
        for (var sample_no = 0; sample_no < sample_paths.length; sample_no++) {
            //
            // finally! load a sample file
            //
            var sample = load_sample_file(
                sample_paths[sample_no],
                subject_id,
                gesture_id);

            if (sample == null)
                continue;

            ret.add_sample(
                sample,
                subject_id,
                gesture_id);
        }
    }

    return;
}

/**
 *
 */
load_sample_file = function(path, subject_id, gesture_id) {
    //Load the file into the lines object
    var lines = require('fs').readFileSync(path, 'utf-8')
        .split('\n')
        .filter(Boolean);

    var gname = "";
    var pt_cnt_str;
    var hash_line;

    gname = lines.shift();
    pt_cnt_str = lines.shift();
    hash_line = lines.shift();

    console.assert(hash_line.trim() ==  '####');

    var pt_cnt = parseInt(pt_cnt_str);

    var ret = new Sample(subject_id, gesture_id, 0);

    pt = [];
    points = [];

    while (true) {
        // read next line in file
        var line = lines.shift();

        // check for empty line or separator
        if (!line || line.trim() == "####") {
            // add pt to trajectory
            //console.log(pt);
            points.push(new Vector(pt));
            pt.length = 0;
            //console.log(points[points.length-1].data);

            // end of file
            if (!line)
                break;

            continue;
        }


        // else read in x, y, z
        var parts = line.split(',');
        pt.push(parseFloat(parts[0]));
        pt.push(parseFloat(parts[1]));
        pt.push(parseFloat(parts[2]));
    }

    ret.add_trajectory(points);
    return ret;
}
