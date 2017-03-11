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

#pragma once
#include <vector>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <boost/thread/shared_mutex.hpp>
#include <memory>
#include <chrono>

#include "util/settings.h"
#include "IOWrapper/Timestamp.h"
#include "opencv2/core/core.hpp"

#include "IOWrapper/Output3DWrapper.h"

#include "DataStructures/Frame.h"

#include "util/SophusUtil.h"
#include "util/MovingAverage.h"
#include "util/Configuration.h"
#include "util/Timer.h"
#include "util/ThreadMutexObject.h"

#include "Tracking/Relocalizer.h"

namespace lsd_slam
{

	// class TrackingReference;
	class KeyFrameGraph;
	// class SE3Tracker;
	// class Sim3Tracker;
	// class DepthMap;
	// class Frame;
	// class DataSet;
	// class LiveSLAMWrapper;
	class Output3DWrapper;
	class FramePoseStruct;
	class TrackableKeyFrameSearch;
	// struct KFConstraintStruct;

	class TrackingThread;
	class OptimizationThread;
	class MappingThread;
	class ConstraintSearchThread;

	using std::unique_ptr;
	using std::shared_ptr;

class SlamSystem {

friend class IntegrationTest;
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	SlamSystem( const Configuration &conf );

	SlamSystem( const SlamSystem&) = delete;
	SlamSystem& operator=(const SlamSystem&) = delete;

	~SlamSystem();

	// Creates a new SlamSystem, and passes over relevant configuration info
	SlamSystem *fullReset();

	// tracks a frame.
	// first frame will return Identity = camToWord.
	// returns camToWord transformation of the tracked frame.
	// frameID needs to be monotonically increasing.
	void trackFrame(const Frame::SharedPtr &newFrame, bool blockUntilMapped );
	void trackFrame( Frame *newFrame, bool blockUntilMapped );


	// finalizes the system, i.e. blocks and does all remaining loop-closures etc.
	void finalize();
	ThreadSynchronizer &finalized() { return _finalized; }

	/** Returns the current pose estimate. */
	SE3 getCurrentPoseEstimate();

	Sophus::Sim3f getCurrentPoseEstimateScale();

	//==== KeyFrame maintenance functions ====
	MutexObject< Frame::SharedPtr >  &currentKeyFrame() { return _currentKeyFrame; };

	void changeKeyframe( const Frame::SharedPtr &frame, bool noCreate, bool force, float maxScore);
	void loadNewCurrentKeyframe( const Frame::SharedPtr &keyframeToLoad );
	void createNewCurrentKeyframe( const Frame::SharedPtr &newKeyframeCandidate );

	// void requestDepthMapScreenshot(const std::string& filename);

	// int findConstraintsForNewKeyFrames(Frame* newKeyFrame, bool forceParent=true, bool useFABMAP=true, float closeCandidatesTH=1.0);

	std::vector<FramePoseStruct::SharedPtr> getAllPoses();

	struct PerformanceData {
		PerformanceData( void ) {;}

		MsRateAverage findConstraint, findReferences;
	} perf;

	Timer timeLastUpdate;

	const Configuration &conf( void ) const     { return _conf; }

	//=== Debugging output functions =====
	shared_ptr<Output3DWrapper> outputWrapper( void )      { return _outputWrapper; }
	void set3DOutputWrapper( Output3DWrapper* outputWrapper ) {	_outputWrapper.reset(outputWrapper); }
	void set3DOutputWrapper( const shared_ptr<Output3DWrapper> &outputWrapper) {	_outputWrapper = outputWrapper; }

	void publishPose(const Sophus::Sim3f &pose ) 	                 { if( _outputWrapper ) _outputWrapper->publishPose(pose);}
	void publishTrackedFrame( const Frame::SharedPtr &frame )      { if( _outputWrapper ) _outputWrapper->publishTrackedFrame( frame ); }
	void publishKeyframeGraph( void )                              { if( _outputWrapper ) _outputWrapper->publishKeyframeGraph( keyFrameGraph() ); }
	void publishKeyframe(  const Frame::SharedPtr &frame )         { if( _outputWrapper ) _outputWrapper->publishKeyframe( frame ); }
	void publishDepthImage( unsigned char* data  )                 { if( _outputWrapper ) _outputWrapper->updateDepthImage( data ); }


	void updateDisplayDepthMap();

	unique_ptr<TrackingThread> trackingThread;
	unique_ptr<OptimizationThread> optThread;
	unique_ptr<MappingThread> mapThread;
	unique_ptr<ConstraintSearchThread> constraintThread;

	// mutex to lock frame pose consistency. within a shared lock of this, *->getScaledCamToWorld() is
	// GUARANTEED to give the same result each call, and to be compatible to each other.
	// locked exclusively during the pose-update by Mapping.
	boost::shared_mutex poseConsistencyMutex;


	const shared_ptr<KeyFrameGraph> &keyFrameGraph() { return _keyFrameGraph; };	  // has own locks
	shared_ptr<TrackableKeyFrameSearch> &trackableKeyFrameSearch() { return _trackableKeyFrameSearch; }


private:

	const Configuration &_conf;

	// Individual / no locking
	shared_ptr<Output3DWrapper> _outputWrapper;	// no lock required

	ThreadSynchronizer _finalized;

	bool _initialized;
	bool initialized( void ) const { return _initialized; }
	bool setInitialized( bool i ) { _initialized = i; return _initialized; }

	void initialize( const Frame::SharedPtr &frame );

	// ======= Functions =====

	void addTimingSamples();

	// == Shared data

	std::shared_ptr<KeyFrameGraph> _keyFrameGraph;	  // has own locks
	MutexObject< Frame::SharedPtr >  _currentKeyFrame;


	std::shared_ptr<TrackableKeyFrameSearch> _trackableKeyFrameSearch;



	// ============= EXCLUSIVELY TRACKING THREAD (+ init) ===============
	// TrackingReference* trackingReference; // tracking reference for current keyframe. only used by tracking.
	// SE3Tracker* tracker;



	// ============= EXCLUSIVELY MAPPING THREAD (+ init) =============



	// ============= EXCLUSIVELY FIND-CONSTRAINT THREAD (+ init) =============


	//
	//
	// // ============= SHARED ENTITIES =============
	// float tracking_lastResidual;
	// float tracking_lastUsage;
	// float tracking_lastGoodPerBad;
	// float tracking_lastGoodPerTotal;
	//
	// int lastNumConstraintsAddedOnFullRetrack;
	// bool doFinalOptimization;
	// float lastTrackingClosenessScore;
	//
	// // for sequential operation. Set in Mapping, read in Tracking.
	// // std::condition_variable  newFrameMappedSignal;
	// // std::mutex newFrameMappedMutex;
	//
	//
	//
	// // USED DURING RE-LOCALIZATION ONLY
	// Relocalizer relocalizer;
	//
	//
	//
	//
	//
	// // Tracking: if (!create) set candidate, set create.
	// // Mapping: if (create) use candidate, reset create.
	// // => no locking required.
	// std::shared_ptr<Frame> latestTrackedFrame;
	// bool createNewKeyFrame;
	//
	//
	//
	// // PUSHED in tracking, READ & CLEARED in mapping
	// // std::deque< std::shared_ptr<Frame> > unmappedTrackedFrames;
	// // ThreadSynchronizer unmappedTrackedFramesSynchro;
	// // std::mutex unmappedTrackedFramesMutex;
	// // std::condition_variable  unmappedTrackedFramesSignal;
	//
	//
	// // PUSHED by Mapping, READ & CLEARED by constraintFinder
	// ThreadMutexObject< std::deque< Frame* > > newKeyFrames;
	// // std::deque< Frame* > newKeyFrames;
	// // std::mutex newKeyFrameMutex;
	// // std::condition_variable newKeyFrameCreatedSignal;
	//
	//
	//
	//
	// // threads
	// // std::thread thread_mapping;
	// // std::thread thread_constraint_search;
	// //std::thread thread_optimization;
	//
	// // bool keepRunning; // used only on destruction to signal threads to finish.
	//
	//
	//
	// // optimization thread
	// // bool newConstraintAdded;
	// // std::mutex newConstraintMutex;
	// // std::condition_variable newConstraintCreatedSignal;
	//
	//
	//
	//
	//
	//
	//
	//
	// bool depthMapScreenshotFlag;
	// std::string depthMapScreenshotFilename;
	//
	// const Configuration &_conf;
	//
	//
	// /** Merges the current keyframe optimization offset to all working entities. */
	// void mergeOptimizationOffset();
	//
	//
	// // void mappingThreadLoop();
	//
	// void finishCurrentKeyframe();
	// void discardCurrentKeyframe();
	//
	// void changeKeyframe(bool noCreate, bool force, float maxScore);
	// void createNewCurrentKeyframe(std::shared_ptr<Frame> newKeyframeCandidate);
	// void loadNewCurrentKeyframe(Frame* keyframeToLoad);
	//
	//
	// bool updateKeyframe();
	//
	//
	// void debugDisplayDepthMap();
	//
	// void takeRelocalizeResult();
	//
	// void constraintSearchThreadLoop();
	// /** Calculates a scale independent error norm for reciprocal tracking results a and b with associated information matrices. */
	// float tryTrackSim3(
	// 		TrackingReference* A, TrackingReference* B,
	// 		int lvlStart, int lvlEnd,
	// 		bool useSSE,
	// 		Sim3 &AtoB, Sim3 &BtoA,
	// 		KFConstraintStruct* e1=0, KFConstraintStruct* e2=0);
	//
	// void testConstraint(
	// 		Frame* candidate,
	// 		KFConstraintStruct* &e1_out, KFConstraintStruct* &e2_out,
	// 		Sim3 candidateToFrame_initialEstimate,
	// 		float strictness);
	//
	// //void optimizationThreadLoop();



};

}
