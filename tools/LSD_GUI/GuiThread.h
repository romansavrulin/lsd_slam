#pragma once

#include <memory>

#include "GUI.h"

extern ThreadMutexObject<bool> guiDone;
extern ThreadSynchronizer guiReady;

extern void runGuiThread(const std::shared_ptr<GUI> &gui );
