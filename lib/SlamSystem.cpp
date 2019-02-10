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
// #include "Tracking/SE3Tracker.h"
// #include "Tracking/Sim3Tracker.h"
// #include "DepthEstimation/DepthMap.h"
// #include "Tracking/TrackingReference.h"
#include "util/globalFuncs.h"
#include "GlobalMapping/KeyFrameGraph.h"
#include "GlobalMapping/TrackableKeyFrameSearch.h"
// #include "GlobalMapping/g2oTypeSim3Sophus.h"
// #include "IOWrapper/ImageDisplay.h"
// #include "IOWrapper/Output3DWrapper.h"
// #include <g2o/core/robust_kernel_impl.h>

#include "DataStructures/FrameMemory.h"
// #include "deque"

// for mkdir
// #include <sys/types.h>
// #include <sys/stat.h>

#include "SlamSystem/MappingThread.h"
#include "SlamSystem/ConstraintSearchThread.h"
#include "SlamSystem/OptimizationThread.h"
#include "SlamSystem/TrackingThread.h"

using namespace lsd_slam;



SlamSystem::SlamSystem( const Configuration &conf )
: _conf( conf ),
	_perf(),
	_outputWrapper( new NullOutput3DWrapper() ),
	_finalized(),
	_initialized( false ),
	_keyFrameGraph( new KeyFrameGraph ),
	_currentKeyFrame( nullptr ),
	_trackableKeyFrameSearch( new TrackableKeyFrameSearch( _keyFrameGraph, conf ) ),
	_depthMap( new DepthMap( _conf ) )
{

	// Because some of these rely on conf(), need to explicitly call after
 	// static initialization.  Is this true?
	optThread.reset( new OptimizationThread( *this, conf.SLAMEnabled ) );
	mapThread.reset( new MappingThread( *this ) );
	constraintThread.reset( new ConstraintSearchThread( *this, conf.SLAMEnabled ) );
	trackingThread.reset( new TrackingThread( *this ) );

	timeLastUpdate.start();
}


SlamSystem::~SlamSystem()
{
	// keepRunning = false;

	// make sure no-one is waiting for something.
	LOG(INFO) << "... waiting for all threads to exit";
	// newFrameMapped.notify();
	// unmappedTrackedFrames.notifyAll();
	// // unmappedTrackedFramesSignal.notify_all();
	//
	// newKeyFrames.notifyAll();

	// newConstraintCreatedSignal.notify_all();

	mapThread.reset();
	constraintThread.reset();
	optThread.reset();
	trackingThread.reset();
	LOG(INFO) << "DONE waiting for all threads to exit";

	FrameMemory::getInstance().releaseBuffes();

	// Util::closeAllWindows();
}

SlamSystem *SlamSystem::fullReset( void )
{
	SlamSystem *newSystem = new SlamSystem( conf() );
	newSystem->set3DOutputWrapper( _outputWrapper );
	return newSystem;
}



void SlamSystem::finalize()
{
	LOG(INFO) << "Finalizing Graph... adding final constraints!!";

	// This happens in the foreground
	constraintThread->doFullReConstraintTrack();
	constraintThread->fullReConstraintTrackComplete.wait();

	LOG(INFO) << "Finalizing Graph... optimizing!!";
	// This happens in the foreground
	// This will kick off a final map publication with the newly optimized offsets (also in foreground)
	optThread->doFinalOptimization();

	optThread->finalOptimizationComplete.wait();
	mapThread->optimizationUpdateMerged.wait();


	// doFinalOptimization = true;
	// newConstraintMutex.lock();
	// newConstraintAdded = true;
	// newConstraintCreatedSignal.notify_all();
	// newConstraintMutex.unlock();

	// while(doFinalOptimization)
	// {
	// 	usleep(200000);
	// }

	//printf("Finalizing Graph... publishing!!\n");
	//unmappedTrackedFrames.notifyAll();
	// unmappedTrackedFramesMutex.lock();
	// unmappedTrackedFramesSignal.notify_one();
	// unmappedTrackedFramesMutex.unlock();

	// while(doFinalOptimization)
	// {
	// 	usleep(200000);
	// }

	// newFrameMapped.wait();
	// newFrameMapped.wait();

	// usleep(200000);
	LOG(INFO) << "Done Finalizing Graph.!!";
	_finalized.notify();

}


// void SlamSystem::requestDepthMapScreenshot(const std::string& filename)
// {
// 	depthMapScreenshotFilename = filename;
// 	depthMapScreenshotFlag = true;
// }

void SlamSystem::initialize( const std::shared_ptr<ImageSet> &set )
{
	LOG_IF(FATAL, !conf().doMapping ) << "WARNING: mapping is disabled, but we just initialized... THIS WILL NOT WORK! Set doMapping to true.";

	_currentKeyFrame = set->refFrame();

	// Todo.  If multiple images are available in the set,
	// use stereo disparity to initialize?
	if( currentKeyFrame()->hasIDepthBeenSet() ) {
		LOG(INFO) << "Using initial Depth estimate in first frame.";
		depthMap()->initializeFromGTDepth( currentKeyFrame() );
	} else {
		LOG(INFO) << "Doing Random initialization!";
		depthMap()->initializeRandomly( currentKeyFrame() );
	}
	updateDisplayDepthMap();

	keyFrameGraph()->addFrame( currentKeyFrame() );

	if( conf().continuousPCOutput) {
		LOG(DEBUG) << "Publishing keyframe " << currentKeyFrame()->id();
		publishCurrentKeyframe();
	}

	_initialized = true;
}


void SlamSystem::nextImage( unsigned int id, const cv::Mat &img, const libvideoio::Camera &cam )
{
	nextImageSet( std::make_shared<ImageSet>(id, img, cam) );
}

void SlamSystem::nextImageSet( const std::shared_ptr<ImageSet> &set )
{
	if( !_initialized ) {
		initialize( set );
		return;
	}

	//LOG(INFO) << "Tracking frame; " << ( blockUntilMapped ? "WILL" : "won't") << " block";
	trackingThread->trackFrame( set->refFrame() );

	//TODO: At present only happens at frame rate.  Push to a thread?
	logPerformanceData();
}



//=== Keyframe maintenance functions ====

void SlamSystem::changeKeyframe( const Frame::SharedPtr &candidate, bool noCreate, bool force, float maxScore)
{
	Frame::SharedPtr newReferenceKF(nullptr);

	if( conf().doKFReActivation && conf().SLAMEnabled )
	{
		Timer timer;
		newReferenceKF = trackableKeyFrameSearch()->findRePositionCandidate( candidate, maxScore );
		_perf.findReferences.update( timer );
	}

	if(newReferenceKF != 0) {
		LOG(INFO) << "Reloading existing key frame " << newReferenceKF->id();
		loadNewCurrentKeyframe(newReferenceKF);
	} else {
		if(force)
		{
			if(noCreate)
			{
				LOG(INFO) << "mapping is disabled & moved outside of known map. Starting Relocalizer!";
				trackingThread->setTrackingIsBad();
				//nextRelocIdx = -1; /// What does this do?
			}
			else
			{
				createNewCurrentKeyframe( candidate );
			}
		}
	}
	// createNewKeyFrame = false;
}

void SlamSystem::loadNewCurrentKeyframe( const Frame::SharedPtr &keyframeToLoad)
{
	//std::lock_guard< std::mutex > lock( currentKeyFrame.mutex() );

	// LOG_IF(DEBUG, enablePrintDebugInfo && printThreadingInfo ) << "RE-ACTIVATE KF " << keyframeToLoad->id();

	depthMap()->setFromExistingKF(keyframeToLoad);

	LOG_IF(DEBUG, enablePrintDebugInfo && printRegularizeStatistics ) << "re-activate frame " << keyframeToLoad->id() << "!";

	_currentKeyFrame = keyFrameGraph()->idToKeyFrame.find(keyframeToLoad->id())->second;
	currentKeyFrame()->depthHasBeenUpdatedFlag = false;
}


void SlamSystem::createNewCurrentKeyframe( const Frame::SharedPtr &newKeyframeCandidate)
{
	LOG_IF(INFO, printThreadingInfo) << "CREATE NEW KF " << newKeyframeCandidate->id() << ", replacing " << currentKeyFrame()->id();

	if( conf().SLAMEnabled)
	{
		boost::shared_lock_guard< boost::shared_mutex > lock( keyFrameGraph()->idToKeyFrameMutex );
		keyFrameGraph()->idToKeyFrame.insert(std::make_pair(newKeyframeCandidate->id(), newKeyframeCandidate));
	}

	// propagate & make new.
	depthMap()->createKeyFrame(newKeyframeCandidate);
	_currentKeyFrame = newKeyframeCandidate;								// Locking



	// if(outputWrapper && printPropagationStatistics)
	// {
	//
	// 	Eigen::Matrix<float, 20, 1> data;
	// 	data.setZero();
	// 	data[0] = runningStats.num_prop_attempts / ((float)_conf.slamImage.area());
	// 	data[1] = (runningStats.num_prop_created + runningStats.num_prop_merged) / (float)runningStats.num_prop_attempts;
	// 	data[2] = runningStats.num_prop_removed_colorDiff / (float)runningStats.num_prop_attempts;
	//
	// 	outputWrapper->publishDebugInfo(data);
	// }

	// currentKeyFrameMutex.lock();
	// currentKeyFrameMutex.unlock();
}




//===== Debugging output functions =====


void SlamSystem::logPerformanceData()
{
	float sPassed = timeLastUpdate.reset();
	if(sPassed > 1.0f)
	{

		LOGF(DEBUG, "Mapping: %3.1fms (%.1fHz); Track: %3.1fms (%.1fHz); Create: %3.1fms (%.1fHz); FindRef: %3.1fms (%.1fHz); PermaTrk: %3.1fms (%.1fHz); Opt: %3.1fms (%.1fHz); FindConst: %3.1fms (%.1fHz);\n",
					depthMap()->perf().update.ms(), depthMap()->perf().update.rate(),
					trackingThread->perf().track.ms(),  trackingThread->perf().track.rate(),
					depthMap()->perf().create.ms()+depthMap()->perf().finalize.ms(), depthMap()->perf().create.rate(),
					_perf.findReferences.ms(), _perf.findReferences.rate(),
					0.0, 0.0,
					//trackableKeyFrameSearch != 0 ? trackableKeyFrameSearch->trackPermaRef.ms() : 0, trackableKeyFrameSearch != 0 ? trackableKeyFrameSearch->trackPermaRef.rate() : 0,
					optThread->perf.ms(), optThread->perf.rate(),
					constraintThread->perf().findConstraint.ms(), constraintThread->perf().findConstraint.rate() );

		depthMap()->logPerformanceData();

	}

}

void SlamSystem::updateDisplayDepthMap()
{
	if( !conf().displayDepthMap ) return;  //&& !depthMapScreenshotFlag)

	depthMap()->debugPlotDepthMap();
	double scale = 1;

	if( (bool)currentKeyFrame() ) scale = currentKeyFrame()->getCamToWorld().scale();

	// debug plot depthmap
	char buf1[200];
	char buf2[200];

	snprintf(buf1,200,"Map: Upd %3.0fms (%2.0fHz); Trk %3.0fms (%2.0fHz); %d / %d",
			depthMap()->perf().update.ms(), depthMap()->perf().update.rate(),
			trackingThread->perf().track.ms(), trackingThread->perf().track.rate(),
			currentKeyFrame()->numFramesTrackedOnThis, currentKeyFrame()->numMappedOnThis ); //, (int)unmappedTrackedFrames().size());

	// snprintf(buf2,200,"dens %2.0f%%; good %2.0f%%; scale %2.2f; res %2.1f/; usg %2.0f%%; Map: %d F, %d KF, %d E, %.1fm Pts",
	// 		100*currentKeyFrame->numPoints/(float)(conf().slamImage.area()),
	// 		100*tracking_lastGoodPerBad,
	// 		scale,
	// 		tracking_lastResidual,
	// 		100*tracking_lastUsage,
	// 		(int)keyFrameGraph()->allFramePoses.size(),
	// 		keyFrameGraph()->totalVertices,
	// 		(int)keyFrameGraph()->edgesAll.size(),
	// 		1e-6 * (float)keyFrameGraph()->totalPoints);


	if( conf().onSceenInfoDisplay )
		printMessageOnCVImage(depthMap()->debugImageDepth, buf1, buf2);

	CHECK( depthMap()->debugImageDepth.data != NULL );
	publishDepthImage( depthMap()->debugImageDepth.data );
}



Sophus::SE3d SlamSystem::getCurrentPoseEstimate()
{
	boost::shared_lock_guard< boost::shared_mutex > lock( keyFrameGraph()->allFramePosesMutex );
	if( keyFrameGraph()->allFramePoses.size() > 0)
		return se3FromSim3(keyFrameGraph()->allFramePoses.back()->getCamToWorld());

	return Sophus::SE3d();
}

Sophus::Sim3d SlamSystem::getCurrentPoseEstimateScale()
{
	boost::shared_lock_guard< boost::shared_mutex > lock( keyFrameGraph()->allFramePosesMutex );
	if(keyFrameGraph()->allFramePoses.size() > 0)
		return keyFrameGraph()->allFramePoses.back()->getCamToWorld().cast<double>();

	return Sophus::Sim3d();
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
	if( _outputWrapper  && _currentKeyFrame ) {
		_outputWrapper->publishKeyframe( _currentKeyFrame );
	} else {
		LOG(DEBUG) << "No currentKeyframe, unable to publish";
	}
}
