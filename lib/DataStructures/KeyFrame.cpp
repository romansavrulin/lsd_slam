

#include "KeyFrame.h"
#include "Frame.h"


namespace lsd_slam {

  KeyFrame::KeyFrame( Frame::SharedPtr &frame )
    : _frame( frame )
    {;}

}
