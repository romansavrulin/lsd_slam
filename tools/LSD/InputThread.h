#pragma once

#include "IOWrapper/GUIIOWrapper.h"

#include "util/ThreadMutexObject.h"

#include "SlamSystem.h"
#include "util/DataSource.h"
#include "util/Undistorter.h"

namespace lsd_slam {

  class InputThread {
  public:

    InputThread(  std::shared_ptr<lsd_slam::SlamSystem> &system,
                   std::shared_ptr<lsd_slam::DataSource> &dataSource,
                   std::shared_ptr<lsd_slam::Undistorter> &undistorter,
                  const  std::shared_ptr<lsd_slam::GUIIOWrapper> &output = std::shared_ptr<lsd_slam::GUIIOWrapper>(nullptr) );

      // Entry point for boost::thread
      void operator()();

      std::shared_ptr<lsd_slam::SlamSystem> &system;
      std::shared_ptr<lsd_slam::DataSource> dataSource;
      std::shared_ptr<lsd_slam::Undistorter> undistorter;

      std::shared_ptr< lsd_slam::GUIIOWrapper> output;

      ThreadMutexObject<bool> inputDone;
      ThreadSynchronizer inputReady;
    };
  }
