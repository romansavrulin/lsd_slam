

#pragma once

#include "libvideoio/Camera.h"
#include "libvideoio/ImageSize.h"

#include "util/settings.h"
#include "util/Configuration.h"


namespace lsd_slam {

  using libvideoio::Camera;
  using libvideoio::ImageSize;

	struct FrameData
	{
		// Explicitly delete default and copy constructors
		FrameData() = delete;
		FrameData( const FrameData & ) = delete;

		FrameData( int id, double timestamp, const Camera &camera, const ImageSize &slamImageSize );
    ~FrameData();

    void setImage( const unsigned char *img );
    void setImage( const float *img );


		int id;

		int width[PYRAMID_LEVELS], height[PYRAMID_LEVELS];

		Camera camera[PYRAMID_LEVELS];

		// Eigen::Matrix3f K[PYRAMID_LEVELS], KInv[PYRAMID_LEVELS];
		// float fx[PYRAMID_LEVELS], fy[PYRAMID_LEVELS], cx[PYRAMID_LEVELS], cy[PYRAMID_LEVELS];
		// float fxInv[PYRAMID_LEVELS], fyInv[PYRAMID_LEVELS], cxInv[PYRAMID_LEVELS], cyInv[PYRAMID_LEVELS];

		double timestamp;

		float* image[PYRAMID_LEVELS];
		bool imageValid[PYRAMID_LEVELS];

		Eigen::Vector4f* gradients[PYRAMID_LEVELS];
		bool gradientsValid[PYRAMID_LEVELS];

		float* maxGradients[PYRAMID_LEVELS];
		bool maxGradientsValid[PYRAMID_LEVELS];


		bool hasIDepthBeenSet;

		// negative depthvalues are actually allowed, so setting this to -1 does NOT invalidate the pixel's depth.
		// a pixel is valid iff idepthVar[i] > 0.
		float* idepth[PYRAMID_LEVELS];
		bool idepthValid[PYRAMID_LEVELS];

		// MUST contain -1 for invalid pixel (that dont have depth)!!
		float* idepthVar[PYRAMID_LEVELS];
		bool idepthVarValid[PYRAMID_LEVELS];

		// data needed for re-activating the frame. theoretically, this is all data the
		// frame contains.
		unsigned char* validity_reAct;
		float* idepth_reAct;
		float* idepthVar_reAct;
		bool reActivationDataValid;


		// data from initial tracking, indicating which pixels in the reference frame ware good or not.
		// deleted as soon as frame is used for mapping.
		bool* refPixelWasGood;
	};

}
