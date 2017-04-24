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
 * It seems there isn't a good cross platform way
 * to search directories in C++:
 *
 * 1) There will be licensing issues if we include Boost for MSVC.
 * 2) The std::experimental FS interface isn't available on OS X.
 * 3) The dirent interface isn't available on MSVS.
 *
 * But at least there is a port of dirent for MSVS we can use.
 * The port is available at: https://github.com/tronkko/dirent
 */
#ifdef _MSC_VER
    #include "dirent/dirent.h"
#else
    #include <dirent.h>
    #include <sys/stat.h>
#endif

#include <cstdlib>
#include <algorithm>
#include <fstream>
#include <tuple>
#include "dataset.h"

/**
 *
 */
static Jackknife::Sample::Ptr load_sample_file(
    std::string path,
    int subject_id,
    int gesture_id);

/**
 * An alternative to std::getline to workaround issues encountered in Windows.
 * Code adapted from http://stackoverflow.com/a/6089413/398316
 */
static std::istream& safe_get_line(
    std::istream& is,
    std::string& t);

/**
 * Largely follows example from:
 * http://www.boost.org/doc/libs/1_36_0/libs/filesystem/doc/index.htm
 */
Dataset::Ptr load_dataset(std::string path)
{
    std::vector<std::string> subject_paths;
    Dataset::Ptr ret(new Dataset());
    struct dirent *entry;
    DIR *directory;

    //
    // Find all subject directories.
    //
    directory = opendir(path.c_str());
    assert(directory != NULL);

    while((entry = readdir(directory)))
    {
        int cmp = strncmp(
            entry->d_name,
            "Sub_",
            4);

        if(cmp != 0)
            continue;

        std::string spath = path;
        spath += entry->d_name;
        subject_paths.push_back(spath);
    }

    closedir(directory);

    //
    // We want the subject names in order.
    //
    sort(
        subject_paths.begin(),
        subject_paths.end());

    //
    // Load the subject directories.
    //
    for (int ii = 0;
         ii < subject_paths.size();
         ii ++)
    {
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
Dataset::Ptr load_subject_dataset(
    Dataset::Ptr dataset,
    std::string subject_path)
{
    // the path and gesture name
    std::vector<std::tuple<std::string, std::string>> gestures;
    Dataset::Ptr &ret = dataset; // alias
    struct dirent *entry;
    DIR *directory;

    //
    // Create a new data set if required.
    //
    if(ret == NULL)
    {
        ret = Dataset::Ptr(new Dataset());
    }

    //
    // Get subject name from path string ".../Sub_<subject_name>".
    //
    std::string::size_type idx;
    idx = subject_path.find("Sub_");

    if(idx == std::string::npos)
    {
        printf("exit 0\n");
        return ret;
    }

    idx += 4;

    //
    // Add subject to database, get sub id.
    //
    std::string *tmp = new std::string(subject_path.substr(idx));
    std::shared_ptr<std::string> subject_name(tmp);
    int subject_id = ret->add_subject(subject_name);

    //
    // Find all gesture directories.
    //
    directory = opendir(subject_path.c_str());
    assert(directory != NULL);

    while((entry = readdir(directory)))
    {
        std::string gname = std::string(entry->d_name);
        std::string gesture_path = subject_path;
        gesture_path += "/";
        gesture_path += entry->d_name;
        struct stat entry_stat;

        if(stat(gesture_path.c_str(), &entry_stat))
            continue;

        if(!S_ISDIR(entry_stat.st_mode))
            continue;

        if(gname == ".")
            continue;

        if(gname == "..")
            continue;

        gestures.push_back(
            std::make_tuple(
                gesture_path,
                gname));
    }

    closedir(directory);

    //
    // Ensure gesture names are in order.
    //
    sort(
        gestures.begin(),
        gestures.end());

    //
    //
    //
    for (int ii = 0;
         ii < gestures.size();
         ii++)
    {
        std::vector<std::string> sample_paths;

        // Get a gesture ID.
        int gesture_id = ret->add_gesture(std::get<1>(gestures[ii]));

        //
        // Now find all examples.
        //
        directory = opendir(std::get<0>(gestures[ii]).c_str());
        assert(directory != NULL);

        while((entry = readdir(directory)))
        {
            std::string sname = std::string(entry->d_name);
            std::string sample_path = std::get<0>(gestures[ii]);
            sample_path += "/";
            sample_path += entry->d_name;
            struct stat entry_stat;

            if(stat(sample_path.c_str(), &entry_stat))
                continue;

            if(!S_ISREG(entry_stat.st_mode))
                continue;

            if(sname.compare(0, 2, "ex"))
                continue;

            sample_paths.push_back(sample_path);
        }

        closedir(directory);

        //
        // Again, we prefer to have samples sorted.
        //
        sort(
            sample_paths.begin(),
            sample_paths.end());

        for (int sample_no = 0;
             sample_no < sample_paths.size();
             sample_no++)
        {
            //
            // finally! load a sample file
            //
            Jackknife::Sample::Ptr sample;
            sample = load_sample_file(
                sample_paths[sample_no],
                subject_id,
                gesture_id);

            if(sample == NULL)
                continue;

            ret->add_sample(
                sample,
                subject_id,
                gesture_id);
        }
    }

    return NULL;
}

/**
 *
 */
static Jackknife::Sample::Ptr load_sample_file(
    std::string path,
    int subject_id,
    int gesture_id)
{
    Jackknife::Sample::Ptr ret;
	std::ifstream in(path);

    std::string gname;
    std::string pt_cnt_str;
    std::string hash_line;
    std::string line;

    // read gesture name
    safe_get_line(
        in,
        gname);

    // read how many points are in file
    safe_get_line(
        in,
        pt_cnt_str);

    // read hashes
    safe_get_line(
        in,
        hash_line);

    assert(hash_line.compare("####") == 0);

    // convert point count to integer
    int pt_cnt = strtol(
        pt_cnt_str.c_str(),
        NULL,
        10);

    //
    // create a new sample object
    //
    ret = Jackknife::Sample::Ptr(new Jackknife::Sample(
            subject_id,
            gesture_id,
            0));

    std::vector<double> pt;
    std::vector<Jackknife::Vector> points;

    while(true)
    {
        // read next line in file
        safe_get_line(
            in,
            line);

        // check for empty line or separator
        if (line.empty() || line.compare("####") == 0)
        {
            // add pt to trajectory
            points.push_back(Jackknife::Vector(pt));
            pt.clear();

            // end of file
            if(line.empty())
                break;

            continue;
        }

        // else read in x, y, z
        char *next = (char *)line.c_str();
        pt.push_back((double)strtod(next, &next)); next ++;
        pt.push_back((double)strtod(next, &next)); next ++;
        pt.push_back((double)strtod(next, &next));
    }

    // clean up
    in.close();

    // add trajectory and return
    ret->add_trajectory(points);
    return ret;
}

/**
 * An alternative to std::getline to workaround issues encountered in Windows.
 * Code adapted from http://stackoverflow.com/a/6089413/398316
 */
static std::istream& safe_get_line(
    std::istream& is,
    std::string& t)
{
    t.clear();

    // The characters in the stream are read one-by-one using a std::streambuf.
    // That is faster than reading them one-by-one using the std::istream.
    // Code that uses streambuf this way must be guarded by a sentry object.
    // The sentry object performs various tasks,
    // such as thread synchronization and updating the stream state.

    std::istream::sentry se(is, true);
    std::streambuf* sb = is.rdbuf();

    for(;;)
    {
        int c = sb->sbumpc();
        switch (c)
        {
            case '\n':
                return is;
            case '\r':
                if(sb->sgetc() == '\n')
                    sb->sbumpc();
                return is;
            case EOF:
                // Also handle the case when the last line has no line ending
                if(t.empty())
                    is.setstate(std::ios::eofbit);
                return is;
            default:
                t += (char)c;
        }
    }
}
