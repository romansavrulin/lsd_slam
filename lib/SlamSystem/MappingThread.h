

#pragma once

#include <mutex>
#include <memory>

#include <boost/thread/shared_mutex.hpp>

#include "active_object/active.h"

#include "util/MovingAverage.h"
#include "util/ThreadMutexObject.h"

#include "DepthEstimation/DepthMap.h"
#include "Tracking/TrackingReference.h"

#include "Tracking/Relocalizer.h"


namespace lsd_slam {

	class KeyFrameGraph;
	class SlamSystem;

class MappingThread {
public:

	MappingThread( SlamSystem &system );
	~MappingThread();

	//=== Callbacks into the thread ===
	// void pushGoodTrackingIteration()
	// {
	// 	if( _thread ) _thread->send( std::bind( &MappingThread::doGoodTrackingIteration, this ));
	// }

	void pushBadTrackingIteration( Frame::SharedPtr frame )
	{
		if( _thread ) _thread->send( std::bind( &MappingThread::doBadTrackingIteration, this, frame ));
	}

	void pushTrackedFrameToMapping( const Frame::SharedPtr &frame )
	{
		{
			std::lock_guard<std::mutex> lock(unmappedTrackedFramesMutex );
			unmappedTrackedFrames.push_back( frame );
		}

		//if( _thread ) _thread->send( std::bind( &MappingThread::doTrackedFrameToMap, this ));
	}

	void mergeOptimizationUpdate( void )
	{
		optimizationUpdateMerged.reset();
		if( _thread ) _thread->send( std::bind( &MappingThread::doMergeOptimizationOffset, this ));
	}

	void pushNewKeyFrame( const Frame::SharedPtr frame )
	{
		if( newKeyFramePending() ) LOG(WARNING) << "Asked to make " << frame->id() << " a keyframe when a frame is already pending";
		_newKeyFramePending = true;
		if( _thread ) _thread->send( std::bind( &MappingThread::doNewKeyFrame, this, frame ));
	}

	bool newKeyFramePending( void )
	{
			return _newKeyFramePending;
	}

	void gtDepthInit( const Frame::SharedPtr &frame );
	void randomInit( const Frame::SharedPtr &frame );


	// SET & READ EVERYWHERE
	// std::mutex currentKeyFrameMutex;

	std::deque< Frame::SharedPtr > unmappedTrackedFrames;
	std::mutex unmappedTrackedFramesMutex;
	ThreadSynchronizer trackedFramesMapped;

	// during re-localization used
	Relocalizer relocalizer;

	std::unique_ptr<TrackingReference> mappingTrackingReference;

	ThreadSynchronizer optimizationUpdateMerged;

private:

	SlamSystem &_system;

	bool _newKeyFramePending;

	// == Thread callbacks ==
	void doProcessTrackedFrames( void );
	void doBadTrackingIteration( const Frame::SharedPtr &frame );
	void doNewKeyFrame( const Frame::SharedPtr &frame );
	void doMergeOptimizationOffset();

	// == Local functions ==
	
	bool updateKeyframe();

	void addTimingSamples();

	void finishCurrentKeyframe();
	void discardCurrentKeyframe();


	void debugDisplayDepthMap();


	// std::vector<Frame*> KFForReloc;
	// //int nextRelocIdx;
	// std::shared_ptr<Frame> latestFrameTriedForReloc;

	std::unique_ptr<active_object::ActiveIdle> _thread;

};


}
