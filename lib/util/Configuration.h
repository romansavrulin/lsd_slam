
#include <opencv2/core/core.hpp>

#include <g3log/g3log.hpp>            // Provides CHECK() macros

#ifdef USE_ZED
#include <zed/Camera.hpp>
#endif

#include "SophusUtil.h"

#include <libvideoio/ImageSize.h>
#include <libvideoio/Camera.h>

#pragma once

namespace lsd_slam {

  using libvideoio::Camera;
  using libvideoio::ImageSize;

// Slow migration from the global settings.[h,cpp] model to a Configuration
// object.
class Configuration {
public:

  Configuration();

  // Does additional validation on sz
  const ImageSize &setSlamImageSize( const ImageSize &sz );
  ImageSize slamImageSize;
  Camera camera;

  enum { NO_STEREO = 0, STEREO_ZED } doDepth;

  bool runRealTime;

  bool stopOnFailedRead;
  bool SLAMEnabled;
  bool doKFReActivation;
  bool doMapping;
  bool continuousPCOutput;

  // settings variables
  // controlled via keystrokes
 bool autoRun;
 bool autoRunWithinFrame;
 int  debugDisplay;
 bool displayDepthMap;
 bool onSceenInfoDisplay;
 bool dumpMap;
 bool doFullReConstraintTrack;


protected:

};

}
