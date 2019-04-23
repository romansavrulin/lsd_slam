

#include "ImageSet.h"

namespace lsd_slam {

ImageSet::ImageSet(unsigned int id, const cv::Mat &img,
                   const libvideoio::Camera &cam)
    : _refFrame(0) {
  _frames.push_back(std::make_shared<Frame>(id, cam, img.size(), 0.0, img.data));
  _se3FromRef.push_back(Sophus::SE3d());
  _frameId = id;
}

// ImageSet::ImageSet(unsigned int id, const std::vector<cv::Mat> &imgs,
//                    const std::vector<libvideoio::Camera> &cams,
//                    const unsigned int ref) {
//
//   _frames.push_back(std::make_shared<Frame>(id, cams.at(0), imgs.at(0).size(),
//                                             0.0, imgs.at(0).data));
//   _frames.push_back(std::make_shared<Frame>(id, cams.at(1), imgs.at(1).size(),
//                                             0.0, imgs.at(1).data));
//   _se3FromRef.push_back(Sophus::SE3());
//   _frameId = id;
//   _refFrame = ref;
// }

ImageSet::~ImageSet() {}

void ImageSet::addFrame(const cv::Mat &img,
                             const libvideoio::Camera &cam,
                             const Sophus::SE3d &frameToRef ) {
  _frames.push_back( std::make_shared<Frame>(_frameId, cam, img.size(), 0.0, img.data) );
  _se3FromRef.push_back( frameToRef );
}

void ImageSet::setDisparityMap(float *_iDepth, uint8_t *_iDepthValid,
                               int _size) {
  // iDepth = _iDepth;
  // iDepthValid = _iDepthValid;
  // iDepthsize = _size;
  disparity.iDepth = _iDepth;
  disparity.iDepthValid = _iDepthValid;
  disparity.iDepthSize = _size;
  // std::cout << "Size: " << _size << std::endl;
}


void ImageSet::propagatePoseFromRefFrame()
{
    boost::lock_guard<boost::shared_mutex> guard(setMutex);

    Sim3 refToParent = refFrame()->pose->thisToParent_raw;

    const size_t sz = size();
    for( int i = 0; i < sz; ++i ) {
      if( i == _refFrame ) continue;

      Frame::SharedPtr frame( _frames[i] );

      LOG(DEBUG) << "Propagating pose from ref frame to sub-image " << i;

      frame->pose->thisToParent_raw = sim3FromSE3(_se3FromRef[i])*refToParent;
    	frame->setTrackingParent( refFrame()->trackingParent() );

    }
}



} // namespace lsd_slam
