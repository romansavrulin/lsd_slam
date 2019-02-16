

#include "ImageSet.h"

namespace lsd_slam {

  ImageSet::ImageSet( unsigned int id, const cv::Mat &img,
                      const libvideoio::Camera &cam )
    : _refFrame(0)
  {
    _frames.push_back( std::make_shared<Frame>(id, cam, img.size(), 0.0, img.data ) );
    _se3FromFirst.push_back( Sophus::SE3() );
  }

  ImageSet::~ImageSet()
  {
  }

}
