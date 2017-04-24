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
 * Size of each frame stored in a session file in bytes.
 */
var g_frame_size_b = (5 + 63) * 4;

/**
 *
 */
function configuartion_parameters_t(device) {
    /**
     * The input device sampling rate.
     */
    this.fps = 30;

    /**
     * The maximum amount of data (in seconds) that is collected
     * and passed to the recognizer. Since the buffer is cleared
     * when a gesture is recognized, the buffer may be shorter.
     */
    this.sliding_window_s = 2.0;

    /**
     * Sliding window converted into frames based on FPS.
     */
    this.sliding_window_frame_cnt = 60.0;

    /**
     * The recognizer is called once per this many frames.
     */
    this.update_interval = 5.0;

    /**
     * A gesture has to have the best Jackknife score this many
     * times before being officially recognized by this application.
     */
    this.repeat_cnt = 3.0;

    if (device == device_type_t.KINECT) {
        this.fps = 30;
        this.sliding_window_s = 2.0;
        this.update_interval = 5;
        this.repeat_cnt = 3;
    } else if (device == device_type_t.LEAP_MOTION) {
        this.fps = 30;
        this.sliding_window_s = 4.0;
        this.update_interval = 10;
        this.repeat_cnt = 4;
    } else {
        assert(0);
    }

    this.sliding_window_frame_cnt = Math.floor(this.fps * this.sliding_window_s);
}

/**
 * The gesture ID in the dataset does not match the gesture
 * IDs used in the session files. So this helper function
 * does a conversion.
 */
function convert_gesture_id(ds, gesture_id) {
    var gname = [
        "jab_right",
        "jab_left",
        "kick_right",
        "kick_left",
        "hook_right",
        "hook_left",
        "uppercut_right",
        "uppercut_left",
        "cartwheel_right",
        "cartwheel_left",
        "push",
        "sidekick_right",
        "sidekick_left",
        "duck",
        "explode",
        "fist_2_circles",
        "index_2_circles",
        "knock_x3",
        "scissors",
        "sideways",
        "rock_out",
        "love",
        "redrum",
    ];

    console.assert(gesture_id >= 1);
    console.assert(gesture_id <= 23);

    return ds.gesture_name_to_id(gname[gesture_id - 1]);
}

/**
 *
 */
function bad_gesture(gesture_id) {
    // redrum
    if (gesture_id == 23 || gesture_id === "redrum")
        return true;

    return false;
}

/**
 *
 */
function get_participant_list(device) {
    if (device == device_type_t.KINECT) {
        var ret = [
            100, 101, 102, 103,
            104, 105, 106, 107,
            108, 109, 110, 111,
            200, 201, 202, 203,
            204, 205, 206, 207,
            000,
        ];

        return ret;
    } else if (device == device_type_t.LEAP_MOTION) {
        var ret = [
            300, 301, 302, 303,
            304, 305, 306, 307,
            400, 401, 402, 403,
            404, 500, 501, 502,
            503, 504, 505, 506,
            000,
        ];

        return ret;
    }

    assert(0);
    return NULL;
}

/**
 * Read in the next frame from the binary input session file.
 */
function Frame(text, position) {
    /**
     * The expected gesture: the gesture that the
     * participant should execute.
     */
    this.gesture_id = -1;

    /**
     * Enumerated command id.
     */
    this.cmd_id = -1;

    /**
     * Currently not used (originates from another project
     * but takes up space in output file).
     */
    this.time_remaining_s = -1;

    /**
     * Position of command on screen.
     */
    this.text_pos_x = -1;

    /**
     * Position of command on screen.
     */
    this.text_pos_y = -1;

    /**
     * True if any component in vector is NAN or infinity.
     * This can happen if the input device loses tracking.
     */
    this.bad_pt = false;

    this.gesture_id = text.getInt32(position.index,true);
    position.index+=4;

    this.cmd_id = text.getInt32(position.index,true);
    position.index+=4;

    this.time_remaining_s = text.getFloat32(position.index,true);
    position.index+=4;

    this.text_pos_x = text.getFloat32(position.index,true);
    position.index+=4;

    this.text_pos_y = text.getFloat32(position.index,true);
    position.index+=4;

    /**
     * The actual input! :)
     */
    this.pt = new Vector(63);
    for (var ii = 0; ii < 63; ii++) {
        var component;
        this.pt.data[ii] = text.getFloat32(position.index,true);
        position.index+=4;
        if (!Number.isFinite(this.pt.data[ii])) {
            this.bad_pt = true;
        }
    }
}

/**
 *
 */
function CommandResults(command_id, expected_id) {
    /**
     * The enumerated command ID.
     */
    this.command_id = command_id;

    /**
     * Gesture that is expected.
     */
    this.expected_id = expected_id;

    /**
     * List of gestures that are detected during this
     * command window.
     */
    this.detected_ids = []
    /**
     * If the participant made a mistake or tracking was lost,
     * we repeat the gesture request in the next command window.
     * So this command should be ignored.
     */
    this.ignore = false;
}

/**
 * Collect any gestures detection during this
 * command duration.
 */
CommandResults.prototype.add = function(detected_id) {
    console.assert(detected_id >= 0);
    this.detected_ids.push(detected_id);
}

/**
 * After the command is complete, update the matrices
 * with detected gestures.
 */
CommandResults.prototype.update_confusion_matrices = function(cm) {
    var found = false;

    for (var ii = 0; ii < this.detected_ids.length; ii++) {
        var detected_id = this.detected_ids[ii];

        if (found && this.expected_id == detected_id) {
            // treat as false positive
            // because we've already detected
            // the gesture once
            cm.add_result(
                this.expected_id, -2);
            continue;
        }

        cm.add_result(
            this.expected_id,
            detected_id);

        // mark that we've found this gesture
        found = found || (detected_id == this.expected_id);
    }

    // false negative
    if (!found) {
        cm.add_result(
            this.expected_id, -1);
    }
}

/**
 * Local overload of version in dataset.cpp.
 */
function load_subject_dataset_session(device, subject_id) {
    // build the subject's dataset path
    var padded_sid = ("00000000" + subject_id).substr(-3);
    var subject_path = "../datasets/jk2017/" +
        (device == device_type_t.KINECT ? "kinect" : "leap_motion") +
        "/training/Sub_U" + padded_sid;
    // load the subject's dataset
    var ds = new Dataset();
    load_subject_dataset(
        ds,
        subject_path);
    return ds;
}

/**
 * Read in all frames from a binary session file.
 */
function load_session(device, subject_id, frames) {
    // Build the subject's session path.
    var padded_sid = ("00000000" + subject_id).substr(-3);
    var session_path = "../datasets/jk2017/" +
        (device == device_type_t.KINECT ? "kinect" : "leap_motion") +
        "/sessions/U" + padded_sid;

    // Open file at end.
    var fs = require("fs");

    var contents = new DataView(toArrayBuffer(fs.readFileSync(session_path)));
    var size = contents.byteLength;

    var position = {index: 0};
    // convert to frame_cnt
    var frame_cnt = size / g_frame_size_b;
    for (var ii = 0; ii < frame_cnt; ii++) {

        var frame = new Frame(contents,position);
        // There is an issue where the main program ran at 60fps,
        // but the Kinect only samples at 30fps. Some a number of
        // readings are duplicates and need to be removed.
        if (ii > 0) {
            var idx = frames.length - 1;
            var distance = frame.pt.l2norm(frames[idx].pt);
            if (distance == 0.0)
                continue;
        }
        frames.push(frame);
    }

}

function toArrayBuffer(buf) {
    var ab = new ArrayBuffer(buf.length);
    var view = new Uint8Array(ab);
    for (var i = 0; i < buf.length; ++i) {
        view[i] = buf[i];
    }
    return ab;
}

/**
 * Load a participant's dataset and session.
 * Train the recognizer with the training data
 * and run the video through. See what happens...
 */
function evaluate_session(
    device,
    subject_id) {
    // Load up the training dataset.
    var ds = load_subject_dataset_session(
        device,
        subject_id);

    // Load up the session.
    var frames = [];
    load_session(
        device,
        subject_id,
        frames);

    // Create a new recognizer.
    var blades = new jackknife_blades();
    blades.set_ip_defaults();
    var jk = new Jackknife(blades);

    // Train the recognizer, without 'bad' gestures.
    for (var ii = 0; ii < ds.samples.length; ii++) {
        var gesture_id = ds.samples[ii].gesture_id;
        var gesture_name = ds.gestures[gesture_id];
        if (bad_gesture(gesture_name))
            continue;
        jk.add_template(ds.samples[ii]);
    }

    // Get device and application parameters
    // based on the device type.
    var params = new configuartion_parameters_t(device);

    // We originally used n=4, r=2 for Kinect data
    // and n=6, r=2 for Leap Motion data, but
    // here we just set the average. There is barely
    // any effect on the results.
    jk.train(6, 2, 1.00);

    // Play session video through
    // the recognizer.
    var buffer = [];
    var detections = [];
    var cmds = [];
    var last_cmd_id = -1;
    var next_update = params.update_interval;

    var frame_no = 0;

    var filter = new ExponentialMovingAverage(frames[0].pt);
    var pt;

    for (var ii = 0; ii < frames.length; ii++) {
        // skip this frame if its bad
        if (frames[ii].bad_pt) {
            continue;
        }

        // Low pass filter the input.
        // Note, we originally didn't smooth the data,
        // so results now are a little higher than in
        // the paper.
        pt = filter.filter(
            frames[ii].pt,
            1 / params.fps);
        //pt = frames[ii].pt;
        frame_no += 1;

        // start a new command
        if (frames[ii].cmd_id != last_cmd_id) {
            last_cmd_id = frames[ii].cmd_id;

            var gid = convert_gesture_id(
                ds,
                frames[ii].gesture_id);

            var cmd = new CommandResults(
                frames[ii].cmd_id,
                gid);

            if (bad_gesture(frames[ii].gesture_id))
                cmd.ignore = true;

            cmds.push(cmd);
        }

        // This buffering approach is really
        // inefficient, but since this off-line,
        // performance is not important.
        buffer.push(pt);
        if (buffer.length > params.sliding_window_frame_cnt)
            buffer.shift();

        // We need to have a couple points before
        // calling the recognizer.
        if (buffer.length < 2)
            continue;

        // Wait a few frames again before trying
        // to recognize again.
        if (frame_no < next_update)
            continue;

        next_update = frame_no + params.update_interval;

        // Run the recognizer.
        var gesture_id = jk.classify(buffer);

        // Add recognition result.
        detections.push(gesture_id);
        if (detections.length > params.repeat_cnt)
            detections.shift();

        // Count how many times this gesture was recognized.
        var winner_cnt = 0;
        for (var jj = 0; jj < detections.length; jj++) {
            winner_cnt += (detections[jj] == gesture_id);
        }

        // Ensure we have enough recognitions.
        if (winner_cnt < params.repeat_cnt)
            continue;

        // If nothing was detected, skip rest.
        if (gesture_id == -1)
            continue;

        // Hopefully it's the right one too!!
        cmds[cmds.length - 1].add(gesture_id);
        detections.length = 0;
        buffer.length = 0;
    }

    // Mark bad commands, situations where the participant
    // made a mistake or tracking was lost. We know the
    // command was bad because the protector asked the
    // participant to repeat the gesture, but a new command
    // ID is assigned to the sequence.
    for (var ii = 1; ii < cmds.length; ii++) {
        if (cmds[ii].expected_id == cmds[ii - 1].expected_id)
            cmds[ii - 1].ignore = true;
    }

    // Put all results in confusion matrices.
    var ret = new ConfusionMatrices(ds);

    for (var ii = 0; ii < cmds.length; ii++) {
        if (cmds[ii].ignore)
            continue;

        cmds[ii].update_confusion_matrices(ret);
    }

    return ret;
}

/**
 * Evaluate all user study sessions for a given device type.
 */
function evaluate_sessions(device) {
    var confusion_matrices = [];
    var participants = get_participant_list(device).slice();
    for (var i = 0; i < participants.length - 1; i++) {
        var cm = evaluate_session(device, participants[i])
        confusion_matrices.push(cm);

        var idx = confusion_matrices.length - 1;

        console.log("Participant: " + participants[i]);


        var results = confusion_matrices[idx].results();
        results.print();
        console.log();
    }

    // put all results into first confusion
    // matrix
    for (var ii = 1; ii < confusion_matrices.length; ii++) {
        confusion_matrices[0].add_result(confusion_matrices[ii]);
    }

    console.log("Aggregate results:");

    var results = confusion_matrices[0].results();
    results.print();
    console.log();
}
