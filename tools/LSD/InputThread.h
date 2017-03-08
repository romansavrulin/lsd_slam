#pragma once

#include "util/ThreadMutexObject.h"

#include "SlamSystem.h"
#include "util/DataSource.h"
#include "util/Undistorter.h"

extern ThreadMutexObject<bool> inputDone;
extern ThreadSynchronizer inputReady;

extern void runInputThread( std::shared_ptr<lsd_slam::SlamSystem> &system,
                            const std::shared_ptr<lsd_slam::DataSource> &dataSource,
                            const std::shared_ptr<lsd_slam::Undistorter> &undistorter );
