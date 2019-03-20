

#include "ImageSet.h"

namespace lsd_slam {

ImageSet::ImageSet(unsigned int id, const cv::Mat &img,
                   const libvideoio::Camera &cam)
    : _refFrame(0) {
  _frames.push_back(
      std::make_shared<Frame>(id, cam, img.size(), 0.0, img.data));
  _se3FromFirst.push_back(SE3());
  _frameId = id;
}

ImageSet::ImageSet(unsigned int id, const std::vector<cv::Mat> &imgs,
                   const std::vector<libvideoio::Camera> &cams,
                   const unsigned int ref) {

  _frames.push_back(std::make_shared<Frame>(id, cams.at(0), imgs.at(0).size(),
                                            0.0, imgs.at(0).data));
  _frames.push_back(std::make_shared<Frame>(id, cams.at(1), imgs.at(1).size(),
                                            0.0, imgs.at(1).data));
  _se3FromFirst.push_back(Sophus::SE3());
  _frameId = id;
  _refFrame = ref;
}

ImageSet::~ImageSet() {}

void ImageSet::pushbackFrame(const cv::Mat &img,
                             const libvideoio::Camera &cam) {
  _frames.push_back(
      std::make_shared<Frame>(_frameId, cam, img.size(), 0.0, img.data));
}
void ImageSet::setDisparityMap(unsigned char *data, float f, float T) {
  /*
  float *pt = _disparityMap;
  for (unsigned int i = 0; i < disparityMapSize; ++i, ++pt) {
    *pt = disparityMap[i];
  }
  */
  //_disparityMap disparityMap(data, f, T);
  disparityMap *_disparityMap = new disparityMap(data, f, T);
}

} // namespace lsd_slam
