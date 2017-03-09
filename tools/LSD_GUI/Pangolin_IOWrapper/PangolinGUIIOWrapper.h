/*
 * PangolinOutput3DWrapper.h
 *
 *  Created on: 17 Oct 2014
 *      Author: thomas
 */

#pragma once

#include "IOWrapper/GUIIOWrapper.h"
#include "LSD_GUI/GUI.h"
#include "util/Configuration.h"

namespace lsd_slam
{


class PangolinGUIIOWrapper : public GUIIOWrapper
{
    public:
        PangolinGUIIOWrapper( const Configuration &conf, GUI & gui);
        virtual ~PangolinGUIIOWrapper();

        virtual void updatePose( const Sophus::Sim3f &pose );
      	virtual void updateFrameNumber( int );
      	virtual void updateLiveImage( const cv::Mat &img );

    private:
        const Configuration &_conf;

        GUI & _gui;
};
}
