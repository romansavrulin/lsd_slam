
#include <gtest/gtest.h>

#include "libvideoio/Camera.h"
#include "libvideoio/ImageSize.h"

#include "DataStructures/FrameData.h"

TEST( FrameData, constructor )
{
  const libvideoio::Camera cam(1000,1000,320,240);
  const libvideoio::ImageSize sz(640,480);

  lsd_slam::FrameData data( cam, sz );


  for( int i = 0; i < PYRAMID_LEVELS; ++i ) {
    float scalar=1.0/pow(2,i);

    ASSERT_FLOAT_EQ( data.camera[i].fx,  cam.fx*scalar );
    ASSERT_FLOAT_EQ( data.camera[i].fy,  cam.fy*scalar );
    ASSERT_FLOAT_EQ( data.camera[i].cx,  (cam.cx+0.5)*scalar-0.5 ); /// TODO.  This is kindof wierd math
    ASSERT_FLOAT_EQ( data.camera[i].cy,  (cam.cy+0.5)*scalar-0.5 );

    ASSERT_EQ( data.width[i], sz.width>>i );
    ASSERT_EQ( data.height[i], sz.height>>i );

  }

}

TEST( FrameDataDeathTest, NotDivisibleBy16 ) {
  const libvideoio::Camera cam(1000,1000,320,240);

  ASSERT_DEATH({
    const int width = 248;
    const int height = 256;
    lsd_slam::FrameData data( cam, libvideoio::ImageSize( width, height) );
  }, "std::exception");  //"Image width \\d* isn't divisible by \\d*" );

  ASSERT_DEATH({
    const int width = 256;
    const int height = 2248;
    lsd_slam::FrameData data( cam, libvideoio::ImageSize( width, height) );
  }, "std::exception" ); //"Image height \\d* isn't divisible by \\d*" );

}
