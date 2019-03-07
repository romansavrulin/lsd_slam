/**
* This file is part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/lsdslam>
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/


#include <memory>

#include "SlamSystem.h"

#include <boost/thread/shared_lock_guard.hpp>

#include <g3log/g3log.hpp>

#ifdef ANDROID
#include <android/log.h>
#endif

#include <opencv2/opencv.hpp>

#include "DataStructures/Frame.h"
#include "util/globalFuncs.h"
#include "GlobalMapping/KeyFrameGraph.h"
#include "GlobalMapping/TrackableKeyFrameSearch.h"
#include "DataStructures/FrameMemory.h"

#include "SlamSystem/MappingThread.h"
#include "SlamSystem/ConstraintSearchThread.h"
#include "SlamSystem/OptimizationThread.h"
#include "SlamSystem/TrackingThread.h"

using namespace lsd_slam;



SlamSystem::SlamSystem( )
: _perf(),
	_outputWrapper( new NullOutput3DWrapper() ),
	_finalized(),
	// _initialized( false ),
	_keyFrames(),
	_keyFrameGraph( new KeyFrameGraph ),
	_trackableKeyFrameSearch( new TrackableKeyFrameSearch( _keyFrameGraph ) )
{

	// Because some of these rely on Conf(), need to explicitly call after
 	// static initialization.  Is this true?
	const bool threaded = Conf().SLAMEnabled && Conf().runRealTime;
	_optThread.reset( new OptimizationThread( *this, threaded ) );
	_mapThread.reset( new MappingThread( *this, threaded ) );
	_constraintThread.reset( new ConstraintSearchThread( *this, threaded ) );
	_trackingThread.reset( new TrackingThread( *this, threaded ) );

	timeLastUpdate.start();
}


SlamSystem::~SlamSystem()
{
	// make sure no-one is waiting for something.
	LOG(INFO) << "... waiting for all threads to exit";

	_mapThread.reset();
	_constraintThread.reset();
	_optThread.reset();
	_trackingThread.reset();
	LOG(INFO) << "DONE waiting for all threads to exit";

	FrameMemory::getInstance().releaseBuffers();
}

KeyFrame::SharedPtr &SlamSystem::currentKeyFrame()
{ return _trackingThread->currentKeyFrame(); }



SlamSystem *SlamSystem::fullReset( void )
{
	SlamSystem *newSystem = new SlamSystem( );
	newSystem->set3DOutputWrapper( _outputWrapper );
	return newSystem;
}

void SlamSystem::finalize()
{
	LOG(INFO) << "Finalizing Graph... adding final constraints!!";

	// Run this in the foreground
	_constraintThread->doFullReConstraintTrack();
	_constraintThread->fullReConstraintTrackComplete.wait();

	LOG(INFO) << "Finalizing Graph... optimizing!!";
	// This happens in the foreground
	// This will kick off a final map publication with the newly optimized offsets (also in foreground)
	_optThread->doFinalOptimization();

	_optThread->finalOptimizationComplete.wait();
	_mapThread->optimizationUpdateMerged.wait();

	LOG(INFO) << "Done Finalizing Graph.!!";
	_finalized.notify();

}

// void SlamSystem::initialize( const std::shared_ptr<ImageSet> &set )
// {
// 	LOG_IF(FATAL, !Conf().doMapping ) << "WARNING: mapping is disabled, but we just initialized... THIS WILL NOT WORK! Set doMapping to true.";
//
// 	//depthMap()->setCurrentKeyFrame( set->refFrame() );
//
// 	// // Todo.  If multiple images are available in the set,
// 	// // use stereo disparity to initialize?
// 	// if( set->refFrame()->hasIDepthBeenSet() ) {
// 	// 	LOG(INFO) << "Using initial Depth estimate in first frame.";
// 	// 	depthMap()->initializeFromGTDepth( set->refFrame() );
// 	// } else {
// 	// 	LOG(INFO) << "Doing Stereo initialization!";
// 	// 	//depthMap()->initializeRandomly( set->refFrame() );
// 	// 	depthMap()->initializefromStereo(set);
// 	// }
// 	//updateDisplayDepthMap();
//
// 	// if( Conf().continuousPCOutput) {
// 	// 	LOG(DEBUG) << "Publishing keyframe " << currentKeyFrame()->id();
// 	// 	publishCurrentKeyframe();
// 	// }
//
// 	_initialized = true;
// }


void SlamSystem::nextImage( unsigned int id, const cv::Mat &img, const libvideoio::Camera &cam )
{
	nextImageSet( std::make_shared<ImageSet>(id, img, cam) );
}

void SlamSystem::nextImageSet( const std::shared_ptr<ImageSet> &set )
{
	// if( !_initialized ) {
	// 	initialize( set );
	// 	return;
	// }

	//LOG(INFO) << "Tracking frame; " << ( blockUntilMapped ? "WILL" : "won't") << " block";
	_trackingThread->doTrackSet( set );

	//TODO: At present only happens at frame rate.  Push to a thread?
	logPerformanceData();
}



//=== Keyframe maintenance functions ====

void SlamSystem::addKeyFrame( const KeyFrame::SharedPtr &keyframe )
{
	_keyFrames.push_back( keyframe );

//	keyFrameGraph()->addKeyFrame( currentKeyFrame() );

	keyFrameGraph()->idToKeyFrame.insert(std::make_pair(keyframe->id(), keyframe));
}

// void SlamSystem::changeKeyframe( const Frame::SharedPtr &candidate, bool noCreate, bool force, float maxScore)
// {
// 	Frame::SharedPtr newReferenceKF(nullptr);
//
// 	if( Conf().doKFReActivation && Conf().SLAMEnabled )
// 	{
// 		Timer timer;
// 		newReferenceKF = trackableKeyFrameSearch()->findRePositionCandidate( candidate, maxScore );
// 		_perf.findReferences.update( timer );
// 	}
//
// 	if(newReferenceKF != 0) {
// 		LOG(INFO) << "Reloading existing key frame " << newReferenceKF->id();
// 		loadNewCurrentKeyframe(newReferenceKF);
// 	} else {
// 		if(force)
// 		{
// 			if(noCreate)
// 			{
// 				LOG(INFO) << "mapping is disabled & moved outside of known map. Starting Relocalizer!";
// 				trackingThread->setTrackingIsBad();
// 				//nextRelocIdx = -1; /// What does this do?
// 			}
// 			else
// 			{
// 				createNewCurrentKeyframe( candidate );
// 			}
// 		}
// 	}
// 	// createNewKeyFrame = false;
// }
//
// void SlamSystem::loadNewCurrentKeyframe( const Frame::SharedPtr &keyframeToLoad)
// {
// 	depthMap()->activateExistingKF(keyframeToLoad);
//
// 	LOG_IF(DEBUG, Conf().print.regularizeStatistics ) << "re-activate frame " << keyframeToLoad->id() << "!";
//
// 	// Not entirely sure why they're doing this lookup...
// 	//_currentKeyFrame = keyFrameGraph()->idToKeyFrame.find(keyframeToLoad->id())->second;
// 	//currentKeyFrame()->depthHasBeenUpdatedFlag = false;
// }
//
//
// void SlamSystem::createNewCurrentKeyframe( const Frame::SharedPtr &newKeyframeCandidate)
// {
// 	LOG_IF(INFO, Conf().print.threadingInfo) << "CREATE NEW KF " << newKeyframeCandidate->id() << ", replacing " << currentKeyFrame()->id();
//
// 	if( Conf().SLAMEnabled)
// 	{
// 		boost::shared_lock_guard< boost::shared_mutex > lock( keyFrameGraph()->idToKeyFrameMutex );
// 		keyFrameGraph()->idToKeyFrame.insert(std::make_pair(newKeyframeCandidate->id(), newKeyframeCandidate));
// 	}
//
// 	// propagate & make new.
// 	depthMap()->createKeyFrame(newKeyframeCandidate);
// }




//===== Debugging output functions =====


void SlamSystem::logPerformanceData()
{
	float sPassed = timeLastUpdate.reset();
	if(sPassed > 1.0f)
	{

		// LOGF(DEBUG, "Mapping: %3.1fms (%.1fHz); Track: %3.1fms (%.1fHz); Create: %3.1fms (%.1fHz); FindRef: %3.1fms (%.1fHz); PermaTrk: %3.1fms (%.1fHz); Opt: %3.1fms (%.1fHz); FindConst: %3.1fms (%.1fHz);\n",
		// 			depthMap()->perf().update.ms(), depthMap()->perf().update.rate(),
		// 			_trackingThread->perf().track.ms(),  _trackingThread->perf().track.rate(),
		// 			depthMap()->perf().create.ms()+depthMap()->perf().finalize.ms(), depthMap()->perf().create.rate(),
		// 			_perf.findReferences.ms(), _perf.findReferences.rate(),
		// 			0.0, 0.0,
		// 			//trackableKeyFrameSearch != 0 ? trackableKeyFrameSearch->trackPermaRef.ms() : 0, trackableKeyFrameSearch != 0 ? trackableKeyFrameSearch->trackPermaRef.rate() : 0,
		// 			_optThread->perf.ms(), _optThread->perf.rate(),
		// 			_constraintThread->perf().findConstraint.ms(), _constraintThread->perf().findConstraint.rate() );
		//
		// depthMap()->logPerformanceData();

	}

}

void SlamSystem::updateDisplayDepthMap()
{
	if( !Conf().displayDepthMap ) return;

	const double scale = (bool)currentKeyFrame() ? currentKeyFrame()->frame()->getCamToWorld().scale() : 1.0;

	// debug plot depthmap
	char buf1[200] = "";
	char buf2[200] = "";

	if( Conf().onSceenInfoDisplay ){
		// snprintf(buf1,200,"Map: Upd %3.0fms (%2.0fHz); Trk %3.0fms (%2.0fHz); %d / %d",
		// 		depthMap()->perf().update.ms(), depthMap()->perf().update.rate(),
		// 		_trackingThread->perf().track.ms(), _trackingThread->perf().track.rate(),
		// 		currentKeyFrame()->numFramesTrackedOnThis, currentKeyFrame()->numMappedOnThis ); //, (int)unmappedTrackedFrames().size());

	// snprintf(buf2,200,"dens %2.0f%%; good %2.0f%%; scale %2.2f; res %2.1f/; usg %2.0f%%; Map: %d F, %d KF, %d E, %.1fm Pts",
	// 		100*currentKeyFrame->numPoints/(float)(Conf().slamImage.area()),
	// 		100*tracking_lastGoodPerBad,
	// 		scale,
	// 		tracking_lastResidual,
	// 		100*tracking_lastUsage,
	// 		(int)keyFrameGraph()->allFramePoses.size(),
	// 		keyFrameGraph()->totalVertices,
	// 		(int)keyFrameGraph()->edgesAll.size(),
	// 		1e-6 * (float)keyFrameGraph()->totalPoints);

	}

	currentKeyFrame()->depthMap()->plotDepthMap( buf1, buf2 );

	CHECK( currentKeyFrame()->depthMap()->debugImages().depthImage().data != NULL );
	publishDepthImage( currentKeyFrame()->depthMap()->debugImages().depthImage().data );
}



Sophus::SE3d SlamSystem::getCurrentPoseEstimate()
{
	boost::shared_lock_guard< boost::shared_mutex > lock( keyFrameGraph()->allFramePosesMutex );
	if( keyFrameGraph()->allFramePoses.size() > 0)
		return se3FromSim3(keyFrameGraph()->allFramePoses.back()->getCamToWorld());

	return SE3();
}

Sophus::Sim3d SlamSystem::getCurrentPoseEstimateScale()
{
	boost::shared_lock_guard< boost::shared_mutex > lock( keyFrameGraph()->allFramePosesMutex );
	if(keyFrameGraph()->allFramePoses.size() > 0)
		return keyFrameGraph()->allFramePoses.back()->getCamToWorld().cast<double>();

	return Sim3();
}

std::vector<FramePoseStruct::SharedPtr> SlamSystem::getAllPoses()
{
	return keyFrameGraph()->allFramePoses;
}


//=== 3DOutputWrapper functions ==

void SlamSystem::publishKeyframe( const Frame::SharedPtr &frame )
{
	if( _outputWrapper ) _outputWrapper->publishKeyframe( frame );
}

void SlamSystem::publishCurrentKeyframe( )
{
	LOG(DEBUG) << "Publishing current keyframe " << currentKeyFrame()->id();
	if( _outputWrapper  && currentKeyFrame() ) {
		_outputWrapper->publishKeyframe( currentKeyFrame()->frame() );
	} else {
		LOG(DEBUG) << "No currentKeyframe, unable to publish";
	}
}

void SlamSystem::publishPointCloud( ){
	if( _outputWrapper ) _outputWrapper->publishPointCloud( currentKeyFrame()->frame() );
}
