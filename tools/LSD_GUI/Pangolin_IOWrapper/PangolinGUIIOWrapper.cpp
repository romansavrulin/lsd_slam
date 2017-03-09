/*
 * PangolinGUIIOWrapper.cpp
 *
 *  Created on: 17 Oct 2014
 *      Author: thomas
 */

#include "PangolinGUIIOWrapper.h"

// #include "util/SophusUtil.h"
// #include "util/settings.h"
// #include "DataStructures/Frame.h"
// #include "GlobalMapping/KeyFrameGraph.h"
// #include "sophus/sim3.hpp"
// #include "GlobalMapping/g2oTypeSim3Sophus.h"

namespace lsd_slam
{

PangolinGUIIOWrapper::PangolinGUIIOWrapper( const Configuration &conf, GUI & gui)
 : _conf( conf ),
   _gui(gui)
{

}

PangolinGUIIOWrapper::~PangolinGUIIOWrapper()
{

}

void PangolinGUIIOWrapper::updatePose( const Sophus::Sim3f &pose )
{
_gui.pose.assignValue( pose );
}

void PangolinGUIIOWrapper::updateFrameNumber( int runningIdx )
{
  _gui.updateFrameNumber( runningIdx );
}

void PangolinGUIIOWrapper::updateLiveImage( const cv::Mat &img )
{
  _gui.updateLiveImage( img.data );
}

}
