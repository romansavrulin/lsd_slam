

#pragma once

#include <mutex>
#include <memory>

#include <boost/thread/shared_mutex.hpp>

#include "active_object/active.h"

#include "util/MovingAverage.h"
#include "util/ThreadMutexObject.h"

#include "DataStructures/KeyFrame.h"

#include "DepthEstimation/DepthMap.h"
#include "Tracking/TrackingReference.h"

#include "Tracking/Relocalizer.h"


namespace lsd_slam {

	class KeyFrameGraph;
	class SlamSystem;

class MappingThread {
public:

	MappingThread( SlamSystem &system, bool threaded );
	~MappingThread();

	//=== Callbacks into the thread ===
	// void pushDoIteration()
	// {
	//   //if( _thread ) _thread->send( std::bind( &MappingThread::doMappingIteration, this ));
	//   if( _thread ) {
	// 		_thread->send( std::bind( &MappingThread::doMappingIterationSet, this ));
	// 	} else {
	// 		doMappingIterationSet();
	// 	}
	// }
        /* REDUNDANT
	void pushUnmappedTrackedFrame( const Frame::SharedPtr &frame )
	{
		{
			std::lock_guard<std::mutex> lock(unmappedTrackedFramesMutex );
			unmappedTrackedFrames.push_back( frame );
		}

		if( _thread ) _thread->send( std::bind( &MappingThread::callbackUnmappedTrackedFrames, this ));
	}
        */

	void doMapSet( const KeyFrame::SharedPtr &kf, const ImageSet::SharedPtr &set)
	{
	    if( _thread )
				_thread->send( std::bind( &MappingThread::mapSetImpl, this, kf, set ));
			else
				mapSetImpl(kf, set);
	}

	void doMergeOptimizationUpdate( void )
	{
		optimizationUpdateMerged.reset();
		if( _thread ) _thread->send( std::bind( &MappingThread::mergeOptimizationOffsetImpl, this ));
	}

	void doCreateNewKeyFrame( const KeyFrame::SharedPtr &keyframe, const Frame::SharedPtr &frame )
	{
		if( _thread )
			_thread->send( std::bind( &MappingThread::createNewKeyFrameImpl, this, keyframe, frame ));
		else
			createNewKeyFrameImpl( keyframe, frame );

	}

        /* REDUNDANT
	void createNewKeyFrame( const Frame::SharedPtr &frame )
	{
                if( newKeyFramePending() )
                {

                    LOG(WARNING) << "Asked to make " << frame->id() << " a keyframe when " << _newKeyFrame()->id() << " is already pending";
                }
                _newKeyFrame = frame;
	}

	bool newKeyFramePending( void )
	{
			return (bool)(_newKeyFrame.get());
	}
        */

	// void createNewImageSet( const ImageSet::SharedPtr &set )
	// {
	//         if( newImageSetPending() )
	//         {
	//             LOG(WARNING) << "Asked to make " << set->id() << " a keyframe when " << _newImageSet()->id() << " is already pending";
	//         }
	//         _newImageSet = set;
	//
	// }

	// bool newImageSetPending( void )
	// {
	//                 return (bool)(_newImageSet.get());
	// }

	// SET & READ EVERYWHERE
	// std::mutex currentKeyFrameMutex;

	// std::deque< Frame::SharedPtr > unmappedTrackedFrames;
  // std::deque< ImageSet::SharedPtr > unmappedTrackedSets;

	// std::mutex unmappedTrackedFramesMutex;
	// ThreadSynchronizer trackedFramesMapped;

	// Used during re-localization
	Relocalizer relocalizer;

	//std::unique_ptr<TrackingReference> mappingTrackingReference;

	ThreadSynchronizer optimizationUpdateMerged;


private:

	SlamSystem &_system;

  // MutexObject< ImageSet::SharedPtr > _newImageSet;
	// MutexObject< Frame::SharedPtr > _newKeyFrame;

	// == Thread callbacks ==
  //REDUNDANT void callbackUnmappedTrackedFrames( void );
  void mapSetImpl( const KeyFrame::SharedPtr &kf, const ImageSet::SharedPtr &set );


	void createNewKeyFrameImpl( const KeyFrame::SharedPtr &keyframe, const Frame::SharedPtr &frame );

	//void callbackCreateNewKeyFrame( std::shared_ptr<Frame> frame );

	// == Local functions ==

  //REDUNDANT bool doMappingIteration();
  //bool doMappingIterationSet();

	void mergeOptimizationOffsetImpl();


	// == Local functions ==

	//REDUNDANT bool updateKeyframe();
	//bool updateImageSet();

	void addTimingSamples();

	void finishCurrentKeyframe();
	void discardCurrentKeyframe();


	void debugDisplayDepthMap();


	// std::vector<Frame*> KFForReloc;
	// //int nextRelocIdx;
	// std::shared_ptr<Frame> latestFrameTriedForReloc;

	std::unique_ptr<active_object::Active> _thread;

};


}
