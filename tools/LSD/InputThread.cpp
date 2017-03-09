

#include "InputThread.h"
#include "LSD.h"

namespace lsd_slam {


  InputThread::InputThread(  std::shared_ptr<lsd_slam::SlamSystem> &sys,
    std::shared_ptr<lsd_slam::DataSource> &src,
    std::shared_ptr<lsd_slam::Undistorter> &und,
    const std::shared_ptr< lsd_slam::GUIIOWrapper > &out )

    : system( sys ), dataSource( src ), undistorter( und ), output( out ),
    inputDone( false ),
    inputReady()
    {
    }

    void InputThread::operator()() {
      // get HZ
      float fps = dataSource->fps();
      long int dt_us = (fps > 0) ? (1e6/fps) : 0;
      long int dt_wiggle = 0;

      inputReady.notify();

      startAll.wait();

      int numFrames = dataSource->numFrames();
      LOG_IF( INFO, numFrames > 0 ) << "Running for " << numFrames << " frames at " << fps << " fps";

      cv::Mat image = cv::Mat(system->conf().slamImage.cvSize(), CV_8U);
      int runningIdx=0;
      float fakeTimeStamp = 0;

      for(unsigned int i = 0; (numFrames < 0) || (i < numFrames); ++i)
      {
        if(inputDone.getValue()) break;

        std::chrono::time_point<std::chrono::steady_clock> start(std::chrono::steady_clock::now());

        cv::Mat imageDist = cv::Mat( system->conf().inputImage.cvSize(), CV_8U);

        if( dataSource->grab() ) {

          dataSource->getImage( imageDist );
          undistorter->undistort(imageDist, image);

          CHECK(image.type() == CV_8U);

          std::shared_ptr<Frame> frame( new Frame( runningIdx, system->conf(), fakeTimeStamp, image.data ));
          if(runningIdx == 0)
          {
            system->randomInit( frame );
          }
          else
          {
            system->trackFrame( frame, fps == 0 );
          }

          runningIdx++;
          fakeTimeStamp += (fps > 0) ? (1.0/fps) : 0.03;

          // Fundamentally, much of this could be done in the SlamSystem
          if( output ) {
            output->updatePose(system->getCurrentPoseEstimateScale());
            output->updateFrameNumber( runningIdx );
            output->updateLiveImage( image );
          }

          if(fullResetRequested)
          {
            LOG(WARNING) << "FULL RESET!";
            system.reset( system->fullReset() );

            fullResetRequested = false;
            runningIdx = 0;
          }

        } else {
          if( system->conf().stopOnFailedRead ) break;
        }

        if( dt_us > 0 ) std::this_thread::sleep_until( start + std::chrono::microseconds( dt_us + dt_wiggle ) );
      }

      LOG(INFO) << "Have processed all input frames.";
      inputDone.assignValue(true);
    }

  }
