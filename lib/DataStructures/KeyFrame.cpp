

#include "KeyFrame.h"
#include "Frame.h"


namespace lsd_slam {

  //== Static functions for creation
  KeyFrame::SharedPtr KeyFrame::Create( const Frame::SharedPtr &frame ) {
    KeyFrame::SharedPtr kf( new KeyFrame( frame ) );

    kf->depthMap()->initializeFromFrame();
    kf->syncDepthMapToFrame();

    return kf;
  }

  KeyFrame::SharedPtr KeyFrame::PropagateAndCreate( const KeyFrame::SharedPtr &other, const Frame::SharedPtr &frame ) {
    KeyFrame::SharedPtr kf( new KeyFrame( frame ) );

    kf->depthMap()->propagateFrom( other->depthMap() );
    kf->syncDepthMapToFrame();

// TODO.  Need to get rescaleFactor from depthMap()->propagateFrom()
//    kf->frame()->pose->thisToParent_raw = sim3FromSE3( se3FromSim3( frame()->pose->thisToParent_raw ), rescaleFactor);
  	kf->frame()->pose->invalidateCache();

    return kf;
  }


  //=== Class functions

  KeyFrame::KeyFrame( const Frame::SharedPtr &frame )
    : numFramesTrackedOnThis(0),
      numMappedOnThis(0),
      numMappedOnThisTotal(0),
      _frame( frame ),
      _depthMap( new DepthMap( frame ) ),
      _trackingReference( new TrackingReference( frame ) )
    {
      _frame->setDepth( _depthMap );
    }


  void KeyFrame::updateDepthFrom( const Frame::SharedPtr &frame ) {

    assert(frame->hasTrackingParent());

  	if(  frame->trackingParent()->id() != id() ) {
  		LOGF(WARNING, "updating keyframe %d with frame %d, which was tracked on a different keyframe (%d).  While this should work, it is not recommended.",
  				id(),
          frame->id(),
  				frame->trackingParent()->id());
  	}

    if( !_depthMap->updateDepthFrom( frame ) ) {
      // Handle error

      return;
    }

    syncDepthMapToFrame();

  	numMappedOnThis++;
  	numMappedOnThisTotal++;

  }

  void KeyFrame::syncDepthMapToFrame() {
    _frame->setDepth(_depthMap);

  }

  void KeyFrame::finalize() {
    depthMap()->finalize();

    syncDepthMapToFrame();
    frame()->calculateMeanInformation();
    //frame()->takeReActivationData(currentDepthMap);

  }



}
