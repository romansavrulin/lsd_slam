

#include "App/InputThread.h"
#include "App.h"

static const std::string OPENCV_WINDOW = "Image window";

namespace lsd_slam {


  InputThread::InputThread(   std::shared_ptr<lsd_slam::SlamSystem> &sys,
                              std::shared_ptr<libvideoio::ImageSource> &src,
                              std::shared_ptr<libvideoio::Undistorter> &und )
    : system( sys ), dataSource( src ), undistorter( und ),
      inputDone( false ),
      inputReady(),
      output( nullptr ),
      it_(nh_)
    {

      cv::namedWindow(OPENCV_WINDOW);
      LOG(INFO) << "InputThread constructor";
    }
  //Add constructor to handle ROS output
  InputThread::InputThread(   std::shared_ptr<lsd_slam::SlamSystem> &sys,
                              std::shared_ptr<lsd_slam::ROSOutput3DWrapper> &owrap,
                              std::shared_ptr<libvideoio::ImageSource> &src,
                              std::shared_ptr<libvideoio::Undistorter> &und )
      : system( sys ), outputWrapper( owrap), dataSource( src ), undistorter( und ),
        inputDone( false ),
        inputReady(),
        output( nullptr ),
        it_(nh_)
      {
        this->outputWrapper = outputWrapper;
        system->set3DOutputWrapper(outputWrapper);
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
        callbackImage = cv_bridge::toCvShare(msg, "bgr8")->image;
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
      }
    }
    /*
    void InputThread::setIOOutputWrapper( const std::shared_ptr<lsd_slam::OutputIOWrapper> &out )
    {
      output = out;
    }
    */

    void InputThread::operator()() {
      // get HZ
      LOG(INFO) << " !!! Starting InputThread !!!!";

      float fps = dataSource->fps();

      if( Conf().runRealTime && fps == 0 ) {
        LOG(WARNING) << "Input FPS not provided, using 30fps.";
        fps = 30;
      }

      long int dt_us = (fps > 0) ? (1e6/fps) : 0;
      long int dt_fudge = 0;

      //Setup
      long int dt_us = (fps > 0) ? (1e6/fps) : 0;
      long int dt_fudge = 0;
      inputReady.notify();
      startAll.wait();

      cv::Mat image = cv::Mat(Conf().slamImageSize.cvSize(), CV_8U);

      int runningIdx=0;
      float fakeTimeStamp = 0;
      ros::Rate loop_rate(10);
      while (ros::ok()){
        std::chrono::time_point<std::chrono::steady_clock> start(std::chrono::steady_clock::now());
        if (callbackImage.size() == system->conf().slamImage.cvSize()){
          //Uncomment to show images...
          cv::imshow(OPENCV_WINDOW, callbackImage);
          cv::waitKey(30);
          cvtColor(callbackImage,image,CV_RGB2GRAY);
          CHECK(image.type() == CV_8UC1);

          cv::Mat imageUndist;
          undistorter->undistort(image, imageUndist);

          CHECK(imageUndist.data != nullptr) << "Undistorted image data is nullptr";
          CHECK(imageUndist.type() == CV_8UC1);
          LOG(DEBUG) << "Image size: " << imageUndist.cols << " x " << imageUndist.rows;

            // This will block if system->conf().runRealTime is false
            system->nextImage( runningIdx, imageUndist, Conf().camera );

          // This will block if system->conf().runRealTime is false
          system->trackFrame( f );

          runningIdx++;
          fakeTimeStamp += (fps > 0) ? (1.0/fps) : 0.03;

          if( output ) {
            output->updateFrameNumber( runningIdx );
            output->updateLiveImage( imageUndist );
          }

        if(fullResetRequested)
        {
          LOG(WARNING) << "FULL RESET!";
          system.reset( system->fullReset() );

          fullResetRequested = false;
          runningIdx = 0;
        }

        if( Conf().runRealTime && dt_us > 0 ) std::this_thread::sleep_until( start + std::chrono::microseconds( dt_us + dt_fudge ) );
        ros::spinOnce();
      }
      //Pretty sure these are worthless now...
      //LOG(INFO) << "Have processed all input frames.";
      //inputDone.assignValue(true);
    }

  }
