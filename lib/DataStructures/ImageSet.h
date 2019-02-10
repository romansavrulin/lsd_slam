/**
* This file is part of LSD-SLAM.
*
* Copyright 2019 Aaron Marburg <amarburg at uw dot edu >
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once
#include <vector>
#include <opencv2/core.hpp>

#include <libvideoio/Camera.h>
#include "DataStructures/Frame.h"

// #include "util/SophusUtil.h"
// #include "util/settings.h"
// #include <boost/thread/recursive_mutex.hpp>
// #include <boost/thread/shared_mutex.hpp>
// #include "DataStructures/FramePoseStruct.h"
// #include "unordered_set"
// #include "util/settings.h"
// #include "util/Configuration.h"

namespace lsd_slam
{

  // Relies on the lazy-initialize nature of Frame to be efficient
  class ImageSet {
  public:

    ImageSet() = delete;
    ImageSet( const ImageSet & ) = delete;

    ImageSet( unsigned int id, const cv::Mat &img, const libvideoio::Camera &cam );
    ~ImageSet();

    Frame::SharedPtr &refFrame() { return _frames[_refFrame]; }

  private:

    unsigned int _refFrame;

    std::vector<Frame::SharedPtr> _frames;
    std::vector<Sophus::SE3d> _se3FromFirst;

  };

}
