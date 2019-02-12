#pragma once

#include "IOWrapper/OutputIOWrapper.h"

#include "util/ThreadMutexObject.h"

#include "SlamSystem.h"

#include "libvideoio/ImageSource.h"
#include "libvideoio/Undistorter.h"

#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

namespace lsd_slam {

  class InputThread {
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  cv::Mat callbackImage;
  ros::Subscriber sub =nh_.subscribe("/image_raw", 1,
    &InputThread::imageCallback, this);

  public:

    InputThread(  std::shared_ptr<lsd_slam::SlamSystem> &system,
                   std::shared_ptr<libvideoio::ImageSource> &dataSource,
                   std::shared_ptr<libvideoio::Undistorter> &undistorter );
    ~InputThread();

    void setIOOutputWrapper( const std::shared_ptr<lsd_slam::OutputIOWrapper> &out );

    // Entry point for boost::thread
    void operator()();

    std::shared_ptr<lsd_slam::SlamSystem> &system;
    std::shared_ptr<libvideoio::ImageSource> dataSource;
    std::shared_ptr<libvideoio::Undistorter> undistorter;

    ThreadMutexObject<bool> inputDone;
    ThreadSynchronizer inputReady;

    //ROS callback
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);


  protected:
    std::shared_ptr<lsd_slam::OutputIOWrapper> output;

  };
}
