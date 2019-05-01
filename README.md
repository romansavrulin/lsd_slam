# LSD-SLAM: Large-Scale Direct Monocular SLAM

Documentation at: [ReadTheDocs](https://lsd-slam.readthedocs.io/en/latest/) <br>
Drone: [![Build Status](https://github.drone.camhd.science/api/badges/apl-ocean-engineering/lsd-slam/status.svg)](https://github.drone.camhd.science/apl-ocean-engineering/lsd-slam)

See also my [Development Blog](http://staff.washington.edu/amarburg/site/) for current status.

Here is [Jakob Engel's](https://github.com/tum-vision/lsd_slam) original description:

> LSD-SLAM is a novel approach to real-time monocular SLAM. It is fully direct
> (i.e. does not use keypoints / features) and creates large-scale,
> semi-dense maps in real-time on a laptop. For more information see
> [http://vision.in.tum.de/lsdslam](http://vision.in.tum.de/lsdslam)
> where you can also find the corresponding publications and Youtube videos, as well as some
> example-input datasets, and the generated output as rosbag or .ply point cloud.

This repo contains my experiments with LSD-SLAM, for performance, functionality
and structure.   As of March 2019, it diverges significantly from Jakob's code
in structure (I refactored as a way of learning the code),
but not significantly in terms of functionality (except for all the ways in which
I've broken it in the refactoring).

**master**  is the working / stable-ish branch.   
**unstable** is my **really unstable** branch.   
**Please note: BOTH BRANCHES ARE MOVING TARGETS.**  it's just that **unstable** is, uh, moving faster.

=====
# Quickstart

My targeted environments are Ubuntu 18.04/16.04,
the [Jetson TX1](http://www.nvidia.com/object/jetson-tx1-module.html) using [NVidia Jetpack 2.3](https://developer.nvidia.com/embedded/jetpack), and OS X 10.12 with [Homebrew](http://brew.sh/).

__If you want a GUI, start with to [lsd-slam-pangolin-gui](https://github.com/amarburg/lsd-slam-pangolin-gui)__

I have adopted [fips](http://floooh.github.io/fips/index.html) as a
high-level "CMake package manager".  I think it does a good job and fits
the needs of LSD-SLAM reasonably well (organizing moderate collections of
related git repos).

Assuming all of the "standard" (apt-gettable/Brew-able) deps have been installed,

    ./fips gen
    ./fips build

Is all that's needed.   If the build fails, it might be necessary to run
`./fips gen` to force a CMake config.

CMake configuration (Release vs. Debug) and platform are controlled through
fips settings, so on a linux machine, either

    ./fips set config linux-make-release
    ./fips build

will build release binaries.  `linux-make-unittest` will build debug binaries and the unit tests.

======
# Running

This version of LSD builds as a library.  It does not contain an application
(it quickly adds lots of additional dependencies for loading config files
and images).

See [lsd-slam-pangolin-gui](https://github.com/amarburg/lsd-slam-pangolin-gui) for a
version which uses the [Pangolin](https://github.com/stevenlovegrove/Pangolin) graphics toolkit.

[lsd-slam-ros](https://gitlab.com/apl-ocean-engineering/lsd-slam/lsd-slam-ros) is
our ROS wrapper.

======
# Related Papers
* **LSD-SLAM: Large-Scale Direct Monocular SLAM**, *J. Engel, T. Schöps, D. Cremers*, ECCV '14

* **Semi-Dense Visual Odometry for a Monocular Camera**, *J. Engel, J. Sturm, D. Cremers*, ICCV '13

=====
# License

LSD-SLAM is licensed under the GNU General Public License Version 3 (GPLv3),
see http://www.gnu.org/licenses/gpl.html.
