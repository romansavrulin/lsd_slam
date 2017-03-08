#pragma once

#include "util/ThreadMutexObject.h"

#include "SlamSystem.h"
#include "util/DataSource.h"
#include "util/Undistorter.h"

extern ThreadMutexObject<bool> inputDone;
extern ThreadSynchronizer inputReady;

extern void runInputThread(lsd_slam::SlamSystem * system, lsd_slam::DataSource *dataSource, lsd_slam::Undistorter* undistorter );
