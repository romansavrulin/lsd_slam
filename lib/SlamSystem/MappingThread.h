

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
	void pushDoIteration()
	{
                //if( _thread ) _thread->send( std::bind( &MappingThread::doMappingIteration, this ));
                if( _thread ) _thread->send( std::bind( &MappingThread::doMappingIterationSet, this ));
	}
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

        void pushUnmappedTrackedSet (const ImageSet::SharedPtr &set)
        {
            {
                std::lock_guard<std::mutex> lock(unmappedTrackedFramesMutex );
                unmappedTrackedSets.push_back( set );
            }

            if( _thread ) _thread->send( std::bind( &MappingThread::callbackUnmappedTrackedSet, this ));


        }

	void mergeOptimizationUpdate( void )
	{
		optimizationUpdateMerged.reset();
		if( _thread ) _thread->send( std::bind( &MappingThread::callbackMergeOptimizationOffset, this ));
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

        void createNewImageSet( const ImageSet::SharedPtr &set )
        {
                if( newImageSetPending() )
                {
                    LOG(WARNING) << "Asked to make " << set->id() << " a keyframe when " << _newImageSet()->id() << " is already pending";
                }
                _newImageSet = set;

        }

        bool newImageSetPending( void )
        {
                        return (bool)(_newImageSet.get());
        }

	// SET & READ EVERYWHERE
	// std::mutex currentKeyFrameMutex;

	std::deque< Frame::SharedPtr > unmappedTrackedFrames;
        std::deque< ImageSet::SharedPtr > unmappedTrackedSets;

	std::mutex unmappedTrackedFramesMutex;
	ThreadSynchronizer trackedFramesMapped;

	// during re-localization used
	Relocalizer relocalizer;

	std::unique_ptr<TrackingReference> mappingTrackingReference;

	ThreadSynchronizer optimizationUpdateMerged;

private:

	SlamSystem &_system;

        MutexObject< ImageSet::SharedPtr > _newImageSet;
	MutexObject< Frame::SharedPtr > _newKeyFrame;

	// == Thread callbacks ==
        //REDUNDANT void callbackUnmappedTrackedFrames( void );
        void callbackUnmappedTrackedSet ( void );
	//void callbackCreateNewKeyFrame( std::shared_ptr<Frame> frame );

	// == Local functions ==

        //REDUNDANT bool doMappingIteration();
        bool doMappingIterationSet();

	void callbackMergeOptimizationOffset();

	// == Local functions ==

        //REDUNDANT bool updateKeyframe();
        bool updateImageSet();

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
