
#include <opencv2/core/core.hpp>

#include <g3log/g3log.hpp>            // Provides CHECK() macros

#ifdef USE_ZED
#include <zed/Camera.hpp>
#endif

#include "SophusUtil.h"

#include "ImageSize.h"
#include "Camera.h"

#pragma once

namespace lsd_slam {

// Slow migration from the global settings.[h,cpp] model to a Configuration
// object.
class Configuration {
public:

  Configuration();

  ImageSize inputImage;
  SlamImageSize slamImage;
  Camera camera;

  enum { NO_STEREO = 0, STEREO_ZED } doDepth;

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
