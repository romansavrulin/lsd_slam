

#pragma once

#include "active_object/active.h"

#include "util/Configuration.h"
#include "util/ThreadMutexObject.h"
#include "GlobalMapping/TrackableKeyFrameSearch.h"
#include "Tracking/SE3Tracker.h"
#include "Tracking/Sim3Tracker.h"
#include "Tracking/TrackingReference.h"

namespace lsd_slam {

	class SlamSystem;

	struct KFConstraintStruct;

class ConstraintSearchThread {
public:
	ConstraintSearchThread( SlamSystem &system, bool threaded );
	~ConstraintSearchThread();

	void doFullReConstraintTrack( void )
	{ fullReConstraintTrackComplete.reset();
		if( _thread ) _thread->send( std::bind( &ConstraintSearchThread::callbackDoFullReConstraintTrack, this )); }

	void newKeyFrame( const Frame::SharedPtr &frame )
	{ if( _thread ) _thread->send( std::bind( &ConstraintSearchThread::callbackNewKeyFrame, this, frame )); }

	ThreadSynchronizer fullReConstraintTrackComplete;

	struct PerformanceData {
		MsRateAverage findConstraint;
	};

	PerformanceData perf() const { return _perf; }


private:

	SlamSystem &_system;

 	PerformanceData _perf;

	std::shared_ptr<Sim3Tracker>       constraintTracker;
	std::shared_ptr<SE3Tracker>        constraintSE3Tracker;
	std::shared_ptr<TrackingReference> newKFTrackingReference;
	std::shared_ptr<TrackingReference> candidateTrackingReference;

	int _failedToRetrack;
	int lastNumConstraintsAddedOnFullRetrack;

	//=== Callbacks ===
	void callbackIdle( void );
	int  callbackDoFullReConstraintTrack( void );
	void callbackNewKeyFrame( const Frame::SharedPtr &frame );

	//=== Internal functions ====
	int findConstraintsForNewKeyFrames(const Frame::SharedPtr &newKeyFrame, bool forceParent, bool useFABMAP, float closeCandidatesTH);

	std::unique_ptr<active_object::ActiveIdle> _thread;

	float tryTrackSim3(
			const std::shared_ptr<TrackingReference> &A,
			const std::shared_ptr<TrackingReference> &B,
			int lvlStart, int lvlEnd,
			bool useSSE,
			Sim3 &AtoB,
			Sim3 &BtoA,
			KFConstraintStruct* e1=nullptr,
			KFConstraintStruct* e2=nullptr);

	void testConstraint(
			const Frame::SharedPtr &candidate,
			KFConstraintStruct* e1_out, KFConstraintStruct* e2_out,
			Sim3 candidateToFrame_initialEstimate,
			float strictness);

};


}
