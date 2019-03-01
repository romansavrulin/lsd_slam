
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

using active_object::Active;

// static const bool depthMapScreenshotFlag = true;


MappingThread::MappingThread( SlamSystem &system )
	: unmappedTrackedFrames(),
		unmappedTrackedFramesMutex(),
		trackedFramesMapped(),
		relocalizer(),
		mappingTrackingReference( new TrackingReference() ),
		_system(system ),
		_newKeyFrame( nullptr ),
                _newImageSet( nullptr ),
		_thread( Active::createActive() )
{
	LOG(INFO) << "Started Mapping thread";
}

MappingThread::~MappingThread()
{
	if( _thread ) delete _thread.release();
	unmappedTrackedFrames.clear();
}

//==== Callbacks ======

/* REDUNDANT
void MappingThread::callbackUnmappedTrackedFrames( void )
{
	bool nMapped = false;
	size_t sz = 0;
	{
		std::lock_guard<std::mutex> lock( unmappedTrackedFramesMutex );
		sz = unmappedTrackedFrames.size();

		if( sz > 0 )
			nMapped = unmappedTrackedFrames.back()->trackingParent()->numMappedOnThisTotal < 10;
	}

	LOG(INFO) << "In unmapped tracked frames callback with " << sz << " frames";

	if(sz < 50 ||
	  (sz < 100 && nMapped) ) {

		while( doMappingIteration() ) { ; }

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
*/
void MappingThread::callbackUnmappedTrackedSet( void )
{
        bool nMapped = false;
        size_t sz = 0;
        {
                std::lock_guard<std::mutex> lock( unmappedTrackedFramesMutex );
                sz = unmappedTrackedSets.size();

                if( sz > 0 )
                        nMapped = unmappedTrackedSets.back()->refFrame()->trackingParent()->numMappedOnThisTotal < 10;
        }

        LOG(INFO) << "In unmapped tracked frames callback with " << sz << " frames";

        if(sz < 50 ||
          (sz < 100 && nMapped) ) {

                while( doMappingIterationSet() ) { ; }

                // TODO:  Was originally in the while() loop above.  However, there are
                // circumstances (if there's one untracked thread in the queue referenced
                // to an older keyframe) where doMappingIteration will return false, and this
                // notify never happens.   If that happens, TrackingThread will stop if it's
                // waiting on the signal.
                //
                // This should be called once per callback otherwise TrackingThread might get hung up?

                //TODO not sure what this is doing...
                trackedFramesMapped.notify();
                //}
        }

        LOG(INFO) << "Done mapping.";
}

void MappingThread::callbackMergeOptimizationOffset()
{
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

// void MappingThread::callbackCreateNewKeyFrame( std::shared_ptr<Frame> frame )
// {
// 	LOG(INFO) << "Set " << frame->id() << " as new key frame";
// 	finishCurrentKeyframe();
// 	_system.changeKeyframe(frame, false, true, 1.0f);
//
// 	_system.updateDisplayDepthMap();
// }

//==== Actual meat ====

/* REDUNDANT
bool MappingThread::doMappingIteration()
{

	// If there's no keyframe, then give up
	if( !(bool)_system.currentKeyFrame() ) {
		LOG(INFO) << "Nothing to map: no keyframe";
		return false;
	}

		// TODO:  Don't know what circumstances cause this to happens
	// if(!doMapping && currentKeyFrame()->idxInKeyframes < 0)
	// {
	// 	if(currentKeyFrame()->numMappedOnThisTotal >= MIN_NUM_MAPPED)
	// 		finishCurrentKeyframe();
	// 	else
	// 		discardCurrentKeyframe();
	//
	// 	map->invalidate();
	// 	LOGF(INFO, "Finished KF %d as Mapping got disabled!\n",currentKeyFrame()->id());
	//
	// 	changeKeyframe(true, true, 1.0f);
	// }

	//callbackMergeOptimizationOffset();
	//addTimingSamples();

	// if(dumpMap)
	// {
	// 	keyFrameGraph()->dumpMap(packagePath+"/save");
	// 	dumpMap = false;
	// }

        bool didSomething = true;

	// set mappingFrame
	if( _system.trackingThread->trackingIsGood() )
	{
		// TODO:  Don't know under what circumstances doMapping = false
		// if(!doMapping)
		// {
		// 	//printf("tryToChange refframe, lastScore %f!\n", lastTrackingClosenessScore);
		// 	if(_system.trackingThread->lastTrackingClosenessScore > 1)
		// 		changeKeyframe(true, false, _system.trackingThread->lastTrackingClosenessScore * 0.75);
		//
		// 	if (displayDepthMap || depthMapScreenshotFlag)
		// 		debugDisplayDepthMap();
		//
		// 	return false;
		// }

		std::shared_ptr< Frame > frame( _newKeyFrame.const_ref() );
		if( frame ) {
			LOG(INFO) << "Set " << frame->id() << " as new key frame";
			finishCurrentKeyframe();
			_system.changeKeyframe(frame, false, true, 1.0f);

			_newKeyFrame().reset();
		} else {
			 didSomething = updateKeyframe();
		}

		_system.updateDisplayDepthMap();

		LOG(DEBUG) << "Tracking is good, updating key frame, " << (didSomething ? "DID" : "DIDN'T") << " do something";
	}

        //TODO have not seen this entered before?
	else
	{
		LOG(INFO) << "Tracking is bad";

		// invalidate map if it was valid.
		if(_system.depthMap()->isValid())
		{
			if( _system.currentKeyFrame()->numMappedOnThisTotal >= MIN_NUM_MAPPED)
				finishCurrentKeyframe();
			else
				discardCurrentKeyframe();

			_system.depthMap()->invalidate();
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

			_system.trackingThread->takeRelocalizeResult( result );
		}
	}

	return didSomething;
}
*/

bool MappingThread::doMappingIterationSet()
{

        // If there's no keyframe, then give up
        if( !(bool)_system.currentKeyFrame() ) {
                LOG(INFO) << "Nothing to map: no keyframe";
                return false;
        }

                // TODO:  Don't know what circumstances cause this to happens
        // if(!doMapping && currentKeyFrame()->idxInKeyframes < 0)
        // {
        // 	if(currentKeyFrame()->numMappedOnThisTotal >= MIN_NUM_MAPPED)
        // 		finishCurrentKeyframe();
        // 	else
        // 		discardCurrentKeyframe();
        //
        // 	map->invalidate();
        // 	LOGF(INFO, "Finished KF %d as Mapping got disabled!\n",currentKeyFrame()->id());
        //
        // 	changeKeyframe(true, true, 1.0f);
        // }

        //callbackMergeOptimizationOffset();
        //addTimingSamples();

        // if(dumpMap)
        // {
        // 	keyFrameGraph()->dumpMap(packagePath+"/save");
        // 	dumpMap = false;
        // }

        bool didSomething = true;

        // set mappingFrame
        if( _system.trackingThread->trackingIsGood() )
        {
                // TODO:  Don't know under what circumstances doMapping = false
                // if(!doMapping)
                // {
                // 	//printf("tryToChange refframe, lastScore %f!\n", lastTrackingClosenessScore);
                // 	if(_system.trackingThread->lastTrackingClosenessScore > 1)
                // 		changeKeyframe(true, false, _system.trackingThread->lastTrackingClosenessScore * 0.75);
                //
                // 	if (displayDepthMap || depthMapScreenshotFlag)
                // 		debugDisplayDepthMap();
                //
                // 	return false;
                // }

                std::shared_ptr< ImageSet > set( _newImageSet.const_ref() );
                if( set ) {
                        LOG(INFO) << "Set " << set->id() << " as new key frame";
                        finishCurrentKeyframe();
                        _system.changeKeyframe(set->refFrame(), false, true, 1.0f);

                        _newImageSet().reset();
                } else {
                         didSomething = updateImageSet();
                }

                _system.updateDisplayDepthMap();

                LOG(DEBUG) << "Tracking is good, updating image set frame, " << (didSomething ? "DID" : "DIDN'T") << " do something";
        }

        //TODO have not seen this entered before?

        else
        {
                LOG(INFO) << "Tracking is bad";

                // invalidate map if it was valid.
                if(_system.depthMap()->isValid())
                {
                        if( _system.currentKeyFrame()->numMappedOnThisTotal >= MIN_NUM_MAPPED)
                                finishCurrentKeyframe();
                        else
                                discardCurrentKeyframe();

                        _system.depthMap()->invalidate();
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

                        _system.trackingThread->takeRelocalizeResult( result, _newImageSet.const_ref() );
                }
        }

        return didSomething;
}




/* REDUNDANT
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
	//TODO when should we update the pointcloud?
        //_system.publishPointCloud();

	return true;
}
*/

bool MappingThread::updateImageSet()
{

        //TODO this function deals with ImageSets, but the vector 'refrences
        // is still of type Frame. Depth map needs to be configured to take a
        //vector of ImageSets instead of Frames.

        //TODO Feed in ImageSets as apposed to Frames
        std::shared_ptr<Frame> reference = nullptr;
        std::deque< std::shared_ptr<Frame> > references;

        unmappedTrackedFramesMutex.lock();

        // Drops frames that have a different tracking parent.
        while(unmappedTrackedSets.size() > 0 &&
                          (!unmappedTrackedSets.front()->refFrame()->hasTrackingParent() ||
                           !unmappedTrackedSets.front()->refFrame()->isTrackingParent( _system.currentKeyFrame() ) ) ) {
                                         if( unmappedTrackedSets.front()->refFrame()->hasTrackingParent() ) {
                                         LOG(INFO) << "Dropping frame " << unmappedTrackedSets.front()->refFrame()->id()
                                                                        << " its has tracking parent " << unmappedTrackedSets.front()->refFrame()->trackingParent()->id()
                                                                                << " current keyframe is " << _system.currentKeyFrame()->id();
                                                } else {
                                                        LOG(INFO) << "Dropping frame " << unmappedTrackedSets.front()->id() << " which doesn't have a tracking parent";
                                                }
                unmappedTrackedSets.front()->refFrame()->clear_refPixelWasGood();
                unmappedTrackedSets.pop_front();
        }

        // clone list
        if(unmappedTrackedSets.size() > 0) {
                // Copies all but only pops one?
                 for(unsigned int i=0;i<unmappedTrackedSets.size(); i++)
                        references.push_back(unmappedTrackedSets[i]->refFrame());
                //unmappedTrackedFrames().swap( references );

                std::shared_ptr<Frame> popped = unmappedTrackedSets.front()->refFrame();
                unmappedTrackedSets.pop_front();
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
        //TODO when should we update the pointcloud?
        //_system.publishPointCloud();

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
	_system.publishKeyframeGraph();
        //_system.publishPointCloud();
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
	_system.keyFrameGraph()->dropKeyFrame( _system.currentKeyFrame() );

	_system.depthMap()->invalidate();
}


}
