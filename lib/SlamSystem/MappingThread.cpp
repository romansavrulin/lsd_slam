
#include "MappingThread.h"

#include <g3log/g3log.hpp>

#include <boost/thread/shared_lock_guard.hpp>

#include "GlobalMapping/KeyFrameGraph.h"
#include "DataStructures/Frame.h"
#include "Tracking/TrackingReference.h"

#include "SlamSystem/TrackingThread.h"
#include "SlamSystem/ConstraintSearchThread.h"

#include "SlamSystem.h"
#include "util/settings.h"

namespace lsd_slam {

using active_object::ActiveIdle;

// static const bool depthMapScreenshotFlag = true;


MappingThread::MappingThread( SlamSystem &system )
	: unmappedTrackedFrames(),
		unmappedTrackedFramesMutex(),
		trackedFramesMapped(),
		relocalizer(),
		mappingTrackingReference( new TrackingReference() ),
		_system(system ),
		_newKeyFramePending( false ),
		_thread( ActiveIdle::createActiveIdle( std::bind( &MappingThread::doProcessTrackedFrames, this ), std::chrono::milliseconds(200)) )
{
	LOG(INFO) << "Started Mapping thread";
}

MappingThread::~MappingThread()
{
	if( _thread ) delete _thread.release();
	unmappedTrackedFrames.clear();
}

//==== Callbacks ======

void MappingThread::doProcessTrackedFrames( void )
{
	bool nMapped = false;
	size_t sz = 0;
	{
		std::lock_guard<std::mutex> lock( unmappedTrackedFramesMutex );
		sz = unmappedTrackedFrames.size();

		if( sz > 0 )
			nMapped = unmappedTrackedFrames.back()->trackingParent()->numMappedOnThisTotal < 10;
	}

	LOG(INFO) << "In callback with " << sz << " tracked frames ready to map";

	if(sz < 50 ||
	  (sz < 100 && nMapped) ) {

		bool didSomething = true;
		while( didSomething ) {

			// If there's no keyframe, then give up
			if( !(bool)_system.currentKeyFrame() ) {
				LOG(INFO) << "Nothing to map: no keyframe";
				break;
			}

			if( !_system.trackingThread->trackingIsGood() ) {
				LOG(DEBUG) << "Supposed to do good tracking iteration .. but tracking has become bad in the meantime";
			}

			// Process all of unmappedTrackedFrames
			didSomething = updateKeyframe();

			_system.updateDisplayDepthMap();

			LOG(DEBUG) << "Tracking is good, updating key frame, " << (didSomething ? "DID" : "DIDN'T") << " do something";
		}

		// TODO:  Was originally in the while() loop above.  However, there are
		// circumstances (if there's one untracked thread in the queue referenced
		// to an older keyframe) where doMappingIteration will return false, and this
		// notify never happens.   If that happens, TrackingThread will stop if it's
		// waiting on the signal.
		//
		// This should be called once per callback otherwise TrackingThread might get hung up?
		trackedFramesMapped.notify();
		//}
	}

	LOG(INFO) << "Done mapping.";
}

void MappingThread::doMergeOptimizationOffset()
{
	//TODO... This function is never called. Publishing graph data with keyframes
	LOG(DEBUG) << "Merging optimization offset";

	// lets us put the publishKeyframeGraph outside the mutex lock
	bool didUpdate = false;

	// if(_optThread->haveUnmergedOptimizationOffset())
	{
		boost::shared_lock_guard< boost::shared_mutex > pose_lock(_system.poseConsistencyMutex);
		boost::shared_lock_guard< boost::shared_mutex > kfLock( _system.keyFrameGraph()->keyframesAllMutex);

		for(unsigned int i=0;i<_system.keyFrameGraph()->keyframesAll.size(); i++)
			_system.keyFrameGraph()->keyframesAll[i]->pose->applyPoseGraphOptResult();

		// _optThread->clearUnmergedOptimizationOffset();

		didUpdate = true;
	}

	if ( didUpdate ) {
		_system.publishKeyframeGraph();
	}

	optimizationUpdateMerged.notify();
}


//==== Actual meat ====

void MappingThread::doNewKeyFrame( const Frame::SharedPtr &newFrame )
{
	LOG(INFO) << "Set " << newFrame->id() << " as new key frame";
	finishCurrentKeyframe();
	_system.changeKeyframe(newFrame, false, true, 1.0f);

	_newKeyFramePending = false;
}

void MappingThread::doBadTrackingIteration( const Frame::SharedPtr &newFrame )
{
	// If there's no keyframe, then give up
	if( !(bool)_system.currentKeyFrame() ) {
		LOG(INFO) << "Nothing to map: no keyframe";
		return;
	}

	if( _system.trackingThread->trackingIsGood() ) {
		LOG(DEBUG) << "Tracking became good while waiting for callback, dropping doBadTracking request";
		return;
	}

	relocalizer.updateCurrentFrame(newFrame);

	// invalidate map if it was valid.
	if(_system.depthMap()->isValid())
	{
		if( _system.currentKeyFrame()->numMappedOnThisTotal >= MIN_NUM_MAPPED)
			finishCurrentKeyframe();
		else
			discardCurrentKeyframe();

		_system.depthMap()->invalidateKeyFrame();
	}

	// start relocalizer if it isnt running already
	if(!relocalizer.isRunning)
		relocalizer.start(_system.keyFrameGraph()->keyframesAll);

	// did we find a frame to relocalize with?
	if(relocalizer.waitResult(50)) {

					// Frame* keyframe;
					// int succFrameID;
					// SE3 succFrameToKF_init;
					// std::shared_ptr<Frame> succFrame;
					//
					// relocalizer.stop();
					// relocalizer.getResult(keyframe, succFrame, succFrameID, succFrameToKF_init);

		relocalizer.stop();
		RelocalizerResult result( relocalizer.getResult() );

		_system.loadNewCurrentKeyframe(result.keyframe);

		// Is this the only way to get tracking working again?
		_system.trackingThread->takeRelocalizeResult( result );
	}
}




bool MappingThread::updateKeyframe()
{
	std::shared_ptr<Frame> reference = nullptr;
	std::deque< std::shared_ptr<Frame> > references;

	unmappedTrackedFramesMutex.lock();

	// Drops frames that have a different tracking parent.
	while(unmappedTrackedFrames.size() > 0 &&
			  (!unmappedTrackedFrames.front()->hasTrackingParent() ||
			   !unmappedTrackedFrames.front()->isTrackingParent( _system.currentKeyFrame() ) ) ) {
					 if( unmappedTrackedFrames.front()->hasTrackingParent() ) {
					 LOG(INFO) << "Dropping frame " << unmappedTrackedFrames.front()->id()
					  				<< " its has tracking parent " << unmappedTrackedFrames.front()->trackingParent()->id()
										<< " current keyframe is " << _system.currentKeyFrame()->id();
						} else {
							LOG(INFO) << "Dropping frame " << unmappedTrackedFrames.front()->id() << " which doesn't have a tracking parent";
						}
		unmappedTrackedFrames.front()->clear_refPixelWasGood();
		unmappedTrackedFrames.pop_front();
	}

	// clone list
	if(unmappedTrackedFrames.size() > 0) {
		// Copies all but only pops one?
		 for(unsigned int i=0;i<unmappedTrackedFrames.size(); i++)
		 	references.push_back(unmappedTrackedFrames[i]);
		//unmappedTrackedFrames().swap( references );

		std::shared_ptr<Frame> popped = unmappedTrackedFrames.front();
		unmappedTrackedFrames.pop_front();
		unmappedTrackedFramesMutex.unlock();

		LOGF_IF( INFO, Conf().print.threadingInfo,
			"MAPPING frames %d to %d (%d frames) onto keyframe %d", references.front()->id(), references.back()->id(), (int)references.size(),  _system.currentKeyFrame()->id());

		_system.depthMap()->updateKeyframe(references);

		popped->clear_refPixelWasGood();
		references.clear();
	}
	else
	{
		unmappedTrackedFramesMutex.unlock();
		return false;
	}

	_system.publishCurrentKeyframe();

	return true;
}




void MappingThread::finishCurrentKeyframe()
{
	LOG_IF(DEBUG, Conf().print.threadingInfo) << "FINALIZING KF " << _system.currentKeyFrame()->id();

	_system.depthMap()->finalizeKeyFrame();

	if(Conf().SLAMEnabled)
	{
		mappingTrackingReference->importFrame(_system.currentKeyFrame());
		_system.currentKeyFrame()->setPermaRef(mappingTrackingReference);
		mappingTrackingReference->invalidate();

		if(_system.currentKeyFrame()->idxInKeyframes < 0)
		{
			_system.keyFrameGraph()->keyframesAllMutex.lock();
			_system.currentKeyFrame()->idxInKeyframes = _system.keyFrameGraph()->keyframesAll.size();
			_system.keyFrameGraph()->keyframesAll.push_back(_system.currentKeyFrame() );
			_system.keyFrameGraph()->totalPoints += _system.currentKeyFrame()->numPoints;
			_system.keyFrameGraph()->totalVertices ++;
			_system.keyFrameGraph()->keyframesAllMutex.unlock();

			_system.constraintThread->newKeyFrame( _system.currentKeyFrame() );
		}
	}

	LOG(DEBUG) << "Finishing current keyframe, publishing keyframe " << _system.currentKeyFrame()->id();

	_system.publishCurrentKeyframe();
	//Publish graph and pointcloud at same frequency as Key Frame
	_system.publishKeyframeGraph();
	_system.publishPointCloud();
}

void MappingThread::discardCurrentKeyframe()
{
	LOG_IF(DEBUG, Conf().print.threadingInfo) << "DISCARDING KF " << _system.currentKeyFrame()->id();

	if(_system.currentKeyFrame()->idxInKeyframes >= 0)
	{
		LOG(WARNING) << "WARNING: trying to discard a KF that has already been added to the graph... finalizing instead.";
		finishCurrentKeyframe();
		return;
	}

	_system.depthMap()->invalidateKeyFrame();
	_system.keyFrameGraph()->dropKeyFrame( _system.currentKeyFrame() );
}


}
