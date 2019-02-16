#pragma once

#include <memory>

#include "libvideoio/ImageSource.h"
#include "libvideoio/Undistorter.h"

namespace lsd_slam {

struct ParseArgs {

  ParseArgs( int argc, char **argv );

  std::shared_ptr<libvideoio::ImageSource> dataSource;
  std::shared_ptr<libvideoio::Undistorter> undistorter;

};


}
