
#include "FrameData.h"

#include "DataStructures/FrameMemory.h"

namespace lsd_slam {

FrameData::FrameData( int i, double ts, const Camera &cam, const ImageSize &slamImageSize )
	: id( i ), timestamp( ts )
{

	camera[0] = cam;


	for (int level = 0; level < PYRAMID_LEVELS; ++ level)
	{
		width[level] = slamImageSize.width >> level;
		height[level] = slamImageSize.height >> level;

		imageValid[level] = false;
		gradientsValid[level] = false;
		maxGradientsValid[level] = false;
		idepthValid[level] = false;
		idepthVarValid[level] = false;

		image[level] = 0;
		gradients[level] = 0;
		maxGradients[level] = 0;
		idepth[level] = 0;
		idepthVar[level] = 0;
		reActivationDataValid = false;

// 		refIDValid[level] = false;

		if (level > 0)
		{
			camera[level] = camera[0].scale( 1.0f / (int)(1<<level) );
		}
	}

	validity_reAct = 0;
	idepthVar_reAct = 0;
	idepth_reAct = 0;

	refPixelWasGood = 0;

	hasIDepthBeenSet = false;

  CHECK( (width[0] & (PYRAMID_DIVISOR-1)) == 0 ) << "Image width " << width[0] << " isn't divisible by " << PYRAMID_DIVISOR;
  CHECK( (height[0] & (PYRAMID_DIVISOR-1)) == 0 ) << "Image height " << height[0] << " isn't divisible by " << PYRAMID_DIVISOR;
}

FrameData::~FrameData() {

  for (int level = 0; level < PYRAMID_LEVELS; ++ level)
  {
    FrameMemory::getInstance().returnBuffer(image[level]);
    FrameMemory::getInstance().returnBuffer(reinterpret_cast<float*>(gradients[level]));
    FrameMemory::getInstance().returnBuffer(maxGradients[level]);
    FrameMemory::getInstance().returnBuffer(idepth[level]);
    FrameMemory::getInstance().returnBuffer(idepthVar[level]);
  }

  FrameMemory::getInstance().returnBuffer((float*)validity_reAct);
  FrameMemory::getInstance().returnBuffer(idepth_reAct);
  FrameMemory::getInstance().returnBuffer(idepthVar_reAct);
}


void FrameData::setImage( const unsigned char *img ) {
  image[0] = FrameMemory::getInstance().getFloatBuffer(width[0]*height[0]);

	float *pt = image[0];
	for(unsigned int i = 0; i < width[0]*height[0]; ++i, ++pt ) {
	       *pt = img[i];
	}

	imageValid[0] = true;
}


void FrameData::setImage( const float *img ) {
  image[0] = FrameMemory::getInstance().getFloatBuffer(width[0]*height[0]);
	memcpy(image[0], img, width[0]*height[0] * sizeof(float));
	imageValid[0] = true;
}

}
