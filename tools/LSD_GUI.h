#pragma once

#include "LSD.h"
#include "LSD_GUI/GUI.h"

extern ThreadMutexObject<bool> guiDone;
extern ThreadSynchronizer guiReady;

extern void runGuiThread(const std::shared_ptr<GUI> &gui );
