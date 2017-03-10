

#pragma once

#include <mutex>

#include "DataStructures/Frame.h"
#include "util/ThreadMutexObject.h"

namespace lsd_slam {

  typedef MutexObject< Frame::SharedPtr > CurrentKeyFrame;

}
