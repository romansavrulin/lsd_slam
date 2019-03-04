#pragma once

#include "Frame.h"

namespace lsd_slam {

  class KeyFrame {
  public:

    typedef std::shared_ptr<KeyFrame> SharedPtr;

    KeyFrame() = delete;
    KeyFrame( const KeyFrame & ) = delete;

    KeyFrame( Frame::SharedPtr &frame );

  private:

    Frame::SharedPtr &_frame;

  };


}
