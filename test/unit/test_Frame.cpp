
#include <gtest/gtest.h>

#include "libvideoio/Camera.h"
#include "libvideoio/ImageSize.h"

#include "DataStructures/Frame.h"

TEST( Frame, constructor )
{
  const libvideoio::Camera cam(1000,1000,320,240);
  const libvideoio::ImageSize sz(640,480);
  const int id = 123;
  const double timestamp = 1.5;

  const unsigned char *img = new unsigned char[ sz.width * sz.height ];

  {
    lsd_slam::Frame frame( id, cam, sz, timestamp, img );


  }
  delete[] img;
}
