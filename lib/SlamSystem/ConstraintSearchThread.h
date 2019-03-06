

#pragma once

#include "active_object/active.h"

#include "util/Configuration.h"
#include "util/ThreadMutexObject.h"
#include "DataStructures/KeyFrame.h"
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
		if( _thread ) _thread->send( std::bind( &ConstraintSearchThread::fullReconstraintTrackImpl, this )); }

	void doNewKeyFrame( const KeyFrame::SharedPtr &keyframe )
	{ if( _thread ) _thread->send( std::bind( &ConstraintSearchThread::newKeyFrameImpl, this, keyframe )); }

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
	//std::shared_ptr<KeyFrame> 					newKF; //TrackingReference;
	//std::shared_ptr<TrackingReference> candidateTrackingReference;

	int _failedToRetrack;
	int lastNumConstraintsAddedOnFullRetrack;

	//=== Callbacks ===
	void idleImpl( void );
	int  fullReconstraintTrackImpl( void );
	void newKeyFrameImpl( const KeyFrame::SharedPtr &keyframe );

	//=== Internal functions ====
	int findConstraintsForNewKeyFrames(const KeyFrame::SharedPtr &newKeyFrame, bool forceParent, bool useFABMAP, float closeCandidatesTH);

	std::unique_ptr<active_object::ActiveIdle> _thread;

	float tryTrackSim3(
			const KeyFrame::SharedPtr &kfA,
			const KeyFrame::SharedPtr &kfB,
			int lvlStart, int lvlEnd,
			bool useSSE,
			Sim3 &AtoB,
			Sim3 &BtoA,
			KFConstraintStruct* e1=nullptr,
			KFConstraintStruct* e2=nullptr);

	void testConstraint(
			const KeyFrame::SharedPtr &keyframe,
			const KeyFrame::SharedPtr &candidate,
			KFConstraintStruct* e1_out, KFConstraintStruct* e2_out,
			Sim3 candidateToFrame_initialEstimate,
			float strictness);

};


}
