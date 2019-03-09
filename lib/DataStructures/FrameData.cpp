
#include "FrameData.h"

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

  	}



}
