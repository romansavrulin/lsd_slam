# LSD-SLAM: Large-Scale Direct Monocular SLAM

[![wercker status](https://app.wercker.com/status/4c30e195acc92af03c75e1f3451b6916/m/master "wercker status")](https://app.wercker.com/project/byKey/4c30e195acc92af03c75e1f3451b6916)

See my [Development Blog](https://faculty.washington.edu/amarburg/press/category/lsdslam/) for current status.

> __December 2017__   Not as much time as I would like to work on this
over the last year (clearly).   One thing I've discovered is I'm not a huge
fan of Conan.   I ended up making a lot of infrastructure to get what I
wanted out of it --- which was the ability to define dependencies and
have them all built locally.

> So if you've got here, I've thrown out Conan and moved to [fips](http://floooh.github.io/fips/index.html).   fips ain't perfect --
the big problem is that you have to rewrite portions of yur CMakeFiles
using their macros --- but it does the job I need it to do.

It's good enough I'm not even going to bother with maintaing the CMake build.
If you want it, it's in the `cmake` branch, stripped of the conan functionality.

> This also means master no long builds Thomas' Pangolin-based GUI.   That's now
in its [own repo]() which is dependent on this repo.   Think of this repo as the "LSD SLAM Library",
with frontends elsewhere...

This fork started from [Thomas Whelan's fork](https://github.com/mp3guy/lsd_slam) which "relieves the user of the horrors of a ROS dependency and uses the much nicer lightweight [Pangolin](https://github.com/stevenlovegrove/Pangolin) framework instead."

Here is Jakob's original description:

> LSD-SLAM is a novel approach to real-time monocular SLAM. It is fully direct
> (i.e. does not use keypoints / features) and creates large-scale,
> semi-dense maps in real-time on a laptop. For more information see
> [http://vision.in.tum.de/lsdslam](http://vision.in.tum.de/lsdslam)
> where you can also find the corresponding publications and Youtube videos, as well as some
> example-input datasets, and the generated output as rosbag or .ply point cloud.

This repo contains my experiments with LSD-SLAM, for performance, functionality
and structure.   As of November 2016, it diverges significantly from either Jakob
or Thomas's branches in structure (I refactored as a way of learning the code),
but not significantly in terms of functionality (except for all the ways in which
I've broken it in the refactoring).

**master**  is my working / stable-ish branch.   **aaron_dev** is my **really unstable** branch.   **Please note: BOTH BRANCHES ARE MOVING TARGETS.**  it's just that **aaron_dev** is, uh, moving faster.

# 1. Quickstart

My targeted environments are Ubuntu 16.04, the [Jetson TX1](http://www.nvidia.com/object/jetson-tx1-module.html) using [NVidia Jetpack 2.3](https://developer.nvidia.com/embedded/jetpack) , and OS X 10.12 with [Homebrew](http://brew.sh/).

__If you want a GUI, go to [lsd-slam-pangolin-gui](https://github.com/amarburg/lsd-slam-pangolin-gui)

The most authoritative documentation is stored in the Ruby Rakefile (don't be scared, it's
pretty readable).   This includes tasks for installing dependencies (in Travis and Docker images for example), and for automating building and testing.

Assuming all of the "standard" (apt-gettable/Brew-able) deps have been installed,

    ./fips gen
    ./fips build

See also [doc/CommonProblems.md](doc/CommonProblems.md)

# 4. Running

Supports directories or sets of raw images. For example, you can download
any dataset from [here](http://vision.in.tum.de/lsdslam), and run:

    ./fips run LSD -- -c datasets/LSD_machine/cameraCalibration.cfg -f datasets/LSD_machine/images/

I've started to document my performance testing in [doc/Performance.md](doc/Performance.md)

# 5. Related Papers

* **LSD-SLAM: Large-Scale Direct Monocular SLAM**, *J. Engel, T. Schöps, D. Cremers*, ECCV '14

* **Semi-Dense Visual Odometry for a Monocular Camera**, *J. Engel, J. Sturm, D. Cremers*, ICCV '13

# 6. License

LSD-SLAM is licensed under the GNU General Public License Version 3 (GPLv3), see http://www.gnu.org/licenses/gpl.html.

# 7. TODOS
