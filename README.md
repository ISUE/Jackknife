# Jackknife

Jackknife is a general purpose gesture recognizer for C++, C#, and JavaScript that is designed to work with a variety of input devices including Kinect, WiiMote, Leap Motion, as well as touch and stylus devices. Our recognizer can even be used to detect hand gestures passing through inaudible sound waves due to the doppler effect. If the input can be represented as a sequence of points through time, there is a good chance that our recognizer can be used to detect input patterns with high accuracy. Jackknife is also designed for gesture customization. This means that a user or developer only needs to provide one or two examples of a gesture pattern to work, whereas other recognizers require lots of training data, which may be quite difficult to collect.

## Dataset

We also include our Kinect v2 and Leap Motion datasets in this repository, which we refer to as the JK2017 dataset.  The following image shows the gestures available in the data set (Kinect v2 gestures are on the top row whereas Leap Motion gestures are on the bottom row).

<p align="center"><img src="http://www.eecs.ucf.edu/isuelab/research/jackknife/teaser.jpg" width="800"></p>

Documentation in the dataset directory is available that explains the directory structure and file formats. However, the software references also include routines for loading the various datasets: see, for example, `cpp/evaluate/dataset.h` and `dataset.cpp` for working with segmented training data, and `load_session()` in `cpp/evaluate/session.cpp` for working with our continuous (session) data.

## Usage

Platform | C++ | C# | JavaScript| Java | Python |
-------- | ----- | -------- | ---- | ---- | ---- |
Linux and OS X | &nbsp;&nbsp;✔ | &nbsp;✔ |&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;✔  |WIP | WIP |
Windows | &nbsp;&nbsp;✔ | &nbsp;✔ |&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;✔  | WIP | WIP |

Jackknife is available in a variety of langugages and supports different platforms. One can simply drop the source into their own project, (e.g., `cpp/jackknife/*`), and start using Jackknife right away without too much trouble. The interface is best described through source examples; so please have a look at the `user_indepedent_test()` function in the implementation langugage of your choice.

### C++ Code

Code for C++ implementation is available under the `cpp` directory. The code does not have any dependencies and should build out of the box with Visual Studio 2015 and above, GCC v5.0 and above and also LLVM. Simply clone the repository, configure using CMake and run the included example:  

    $ git clone https://github.com/ISUE/Jackknife
    $ cd Jackknife/cpp
    $ mkdir build
    $ cd build
    $ cmake ../
    $ make
    $ ./Jackknife

For Visual Studio 2015 and above, it's easier to use CMake GUI tool to generate the solution file.

### C# Code

The solution file for C# implementation is available under `csharp` directory. The code does not have any dependencies and should build out of the box with Visual Studio 2015 and above. After cloning the repository, simply open `Jackknife.sln` and build. Running the built executable will run the default examples on our dataset.

#### A Note About Locale Settings
Some of our data files are stored in text format and contain decimal values stored as strings. We have noticed that depending on the operating system's regional settings, some data files could get parsed incorrectly by .NET framework. This is due to the fact that decimal separators vary by region (for instance, the German format uses "," as the decimal separator, so 50.7 is shown as 50,7 in the German format).  

As a result, at the beginning of `Program.cs` of our bundled example, we have included the following line:  

    System.Threading.Thread.CurrentThread.CurrentCulture = new System.Globalization.CultureInfo("en-US", false);

This effectively forces the current thread to use "en-US" locale when parsing string data.

### JavaScript Code

The JavaScript implementation is available under `js` directory. The JS implementation depends on node.js if running the default examples from our dataset. After cloning the repository, run merged.js (the concatenation of all the files from the subdirectories and main.js) from the node command line to test our example datasets. If only the recognizer is needed, there is no node.js dependency and only the js/jackknife subdirectory needs to be included.

# Some Technical Details

Jackknife is a collection of dynamic time warping (DTW) based techniques tailored for gesture customization. That is, Jackknife is designed to work well with only one or two samples per gesture class for many different input devices.

As an overview of the underlying methods and process, we first resample an input gesture to a fixed number of points, similar to $1 [1]. We then measure the similarity of the given input to each gesture class template using DTW, where the local cost function measures the inner product of corresponding gesture path direction vectors, per Penny Pincher [2]. Motivated by the complexity-invariant distance (CID) [3], we further augment the DTW score with a set of new correction factors designed specifically for gestures. Finally, the input sample is said to belong to the gesture class whose template yields the best score. When dealing with continuous data, we also find an appropriate rejection criteria by selecting the threshold which maximizes the F<sub>1</sub> score of synthetically generated distributions. Negative samples are generated by splicing together template gestures and positive samples are generated with gesture path stochastic resampling (GPSR) [4].

Further details, including information on various optimizations, can be found in the Jackknife paper.

[1] Jacob O. Wobbrock, Andrew D. Wilson, and Yang Li. "[Gestures without libraries, toolkits or training: a $1 recognizer for user interface prototypes.](http://faculty.washington.edu/wobbrock/pubs/uist-07.01.pdf)" Proceedings of the 20th annual ACM symposium on User interface software and technology. ACM, 2007.

[2] Eugene M. Taranta II, Andrés N. Vargas, and Joseph J. LaViola Jr. "[Streamlined and accurate gesture recognition with Penny Pincher.](http://www.sciencedirect.com/science/article/pii/S0097849315001788)" Computers & Graphics 55 (2016): 130-142.

[3] Gustavo E.A.P.A. Batista, Xiaoyue Wang, and Eamonn J. Keogh. "[A complexity-invariant distance measure for time series.](http://www.cs.ucr.edu/~eamonn/Complexity-Invariant%20Distance%20Measure.pdf)" Proceedings of the 2011 SIAM International Conference on Data Mining. Society for Industrial and Applied Mathematics, 2011. 

[4] Eugene M. Taranta II, Mehran Maghoumi, Corey R. Pittman, and Joseph J. LaViola Jr. "[A Rapid Prototyping Approach to Synthetic Data Generation For Improved 2D Gesture Recognition](https://www.cs.ucf.edu/~jjl/pubs/uist2016-taranta.pdf)." Proceedings of the 29th Annual Symposium on User Interface Software and Technology. ACM, 2016.

# Citing

When using Jackknife or the JK2017 dataset, please reference the following paper:

    @inproceedings{taranta17jackknife,
     author = {Taranta II, Eugene M. and Samiei, Amirreza and Maghoumi, Mehran and Khaloo, Pooya and Pittman, Corey R. and LaViola Jr., Joseph J.},
     title = {Jackknife: A Reliable Recognizer with Few Samples and Many Modalities},
     booktitle = {Proceedings of the 2017 CHI Conference on Human Factors in Computing Systems},
     series = {CHI '17},
     year = {2017},
     location = {Denver, Colorado, USA},
     pages = {5850--5861},
     url = {http://doi.acm.org/10.1145/3025453.3026002},
     publisher = {ACM},
    }

# Contributions and Bug Reports

Contributions are welcome. Please submit your contributions as pull requests and we will incorporate them. Also, if you find any bugs, please report them via the issue tracker.	

# License

Jackknife can be used freely for academic research purposes. More details are [available in our license file](https://raw.githubusercontent.com/ISUE/Jackknife/master/LICENSE).
