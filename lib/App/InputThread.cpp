

#include "App/InputThread.h"
#include "App.h"

static const std::string OPENCV_WINDOW = "Image window";

namespace lsd_slam {


  InputThread::InputThread(  std::shared_ptr<lsd_slam::SlamSystem> &sys,
                              std::shared_ptr<libvideoio::ImageSource> &src,
                              std::shared_ptr<libvideoio::Undistorter> &und )
    : system( sys ), dataSource( src ), undistorter( und ),
      inputDone( false ),
      inputReady(),
      output( nullptr ),
      it_(nh_)
    {
      image_sub_ = it_.subscribe("/image_raw", 1,
        &InputThread::imageCallback, this);

      cv::namedWindow(OPENCV_WINDOW);

      LOG(INFO) << "InputThread constructor";
    }

  InputThread::~InputThread(){
    cv::destroyWindow(OPENCV_WINDOW);
  }

    //ROS callback
    void InputThread::imageCallback(const sensor_msgs::ImageConstPtr& msg){
      try
      {
        cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
        cv::waitKey(30);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
      }
    }

    void InputThread::setIOOutputWrapper( const std::shared_ptr<lsd_slam::OutputIOWrapper> &out )
    {
      output = out;
    }

    void InputThread::operator()() {
      // get HZ

      LOG(WARNING) << " !!! Running InputThread !!!!";

      bool runRealTime = system->conf().runRealTime;

      float fps = dataSource->fps();
      long int dt_us = (fps > 0) ? (1e6/fps) : 0;
      long int dt_fudge = 0;

      if( runRealTime && fps == 0 ) {
        LOG(INFO) << "Cannot run realtime, as input FPS is not known.";
        runRealTime = false;
      }

      inputReady.notify();

      startAll.wait();

      int numFrames = dataSource->numFrames();
      LOG_IF( INFO, numFrames > 0 ) << "Running for " << numFrames << " frames at " << fps << " fps";

      cv::Mat image = cv::Mat(system->conf().slamImage.cvSize(), CV_8U);
      int runningIdx=0;
      float fakeTimeStamp = 0;

      for(unsigned int i = 0; (numFrames < 0) || (i < (unsigned int)numFrames); ++i)
      {

        if(inputDone.getValue()) break;

        std::chrono::time_point<std::chrono::steady_clock> start(std::chrono::steady_clock::now());

        if( dataSource->grab() ) {
          if( dataSource->getImage( image ) >= 0 ) {
            CHECK(image.type() == CV_8UC1);

            cv::Mat imageUndist;
            undistorter->undistort(image, imageUndist);

            CHECK(imageUndist.data != nullptr) << "Undistorted image data is nullptr";
            CHECK(imageUndist.type() == CV_8UC1);
            LOG(WARNING) << "Image size: " << imageUndist.cols << " x " << imageUndist.rows;

            Frame *f = new Frame( runningIdx, system->conf(), fakeTimeStamp, imageUndist.data );
            system->trackFrame( f, runRealTime );

            runningIdx++;
            fakeTimeStamp += (fps > 0) ? (1.0/fps) : 0.03;

            if( output ) {
              output->updateFrameNumber( runningIdx );
              output->updateLiveImage( imageUndist );
            }

          }

          if(fullResetRequested)
          {
            LOG(WARNING) << "FULL RESET!";
            system.reset( system->fullReset() );

            fullResetRequested = false;
            runningIdx = 0;
          }

        } else {
          LOG(INFO) << "Bad read, still running...";
          if( system->conf().stopOnFailedRead ) break;
        }

        if( dt_us > 0 && runRealTime ) std::this_thread::sleep_until( start + std::chrono::microseconds( dt_us + dt_fudge ) );
      }

      LOG(INFO) << "Have processed all input frames.";
      inputDone.assignValue(true);
    }

  }
