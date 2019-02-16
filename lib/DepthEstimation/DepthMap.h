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
#include "util/EigenCoreInclude.h"
#include "opencv2/core/core.hpp"
#include "util/settings.h"
#include "util/IndexThreadReduce.h"
#include "util/SophusUtil.h"
#include "util/Configuration.h"
#include "util/MovingAverage.h"
#include "util/Timer.h"

#include "DataStructures/Frame.h"

#include "DepthMapDebugImages.h"



namespace lsd_slam
{

class DepthMapPixelHypothesis;
class KeyFrameGraph;


/**
 * Keeps a detailed depth map (consisting of DepthMapPixelHypothesis) and does
 * stereo comparisons and regularization to update it.
 */
class DepthMap
{
public:

	// Delete default constructors
	DepthMap(const DepthMap&) = delete;
	DepthMap& operator=(const DepthMap&) = delete;

	DepthMap();
	~DepthMap();

	/** Resets everything. */
	void reset();

	//== The "public API" functions for Depth Map
	/**
	 * does obervation and regularization only.
	 **/
	void updateKeyframe(std::deque< Frame::SharedPtr > referenceFrames);

	/**
	 * does propagation and whole-filling-regularization (no observation, for that need to call updateKeyframe()!)
	 **/
	void createKeyFrame( const Frame::SharedPtr &new_keyframe );

	/**
	 * does one fill holes iteration
	 */
	void finalizeKeyFrame();

	void invalidateKeyFrame();

	inline bool isValid() {return (bool)activeKeyFrame;};

	void initializeFromGTDepth( const std::shared_ptr<Frame> &new_frame);
	void initializeRandomly( const std::shared_ptr<Frame> &new_frame);

	void activateExistingKF(const Frame::SharedPtr &kf);

	Frame::SharedPtr &currentKeyFrame() { return activeKeyFrame; }

	const DepthMapDebugImages &debugImages() const { return _debugImages; }

	struct PerformanceData {
		PerformanceData( void ) {;}
		void log();

		MsRateAverage update, create, finalize, observe, regularize, propagate, fillHoles, setDepth;
	};

	PerformanceData &performanceData() { return _perf; }

	void debugPlotDepthMap( const char *buf1, const char *buf2 );

private:

	const ImageSize _imageSize;

	DepthMapDebugImages _debugImages;

 	PerformanceData _perf;


	// ============= parameter copies for convenience ===========================
	Frame::SharedPtr activeKeyFrame;
	boost::shared_lock<boost::shared_mutex> activeKeyFramelock;

	const float* activeKeyFrameImageData() { return activeKeyFrame->image(0); };
	bool activeKeyFrameIsReactivated;


	Frame::SharedPtr oldest_referenceFrame;
	Frame::SharedPtr newest_referenceFrame;
	std::vector< Frame::SharedPtr > referenceFrameByID;
	int referenceFrameByID_offset;

	// ============= internally used buffers for intermediate calculations etc. =============
	// for internal depth tracking, their memory is managed (created & deleted) by this object.
	DepthMapPixelHypothesisVector currentDepthMap, scratchDepthMap;
	std::vector<int> validityIntegralBuffer;



	// ============ internal functions ==================================================

	// Reset currentDepthMap by re-projecting is from activeKeyFrame to new_keyframe
	void propagateDepthAndMakeActiveKeyFrame( const Frame::SharedPtr &new_keyframe);

	// The "do depth update" functions
	void observeDepth();
	void observeDepthRow(int yMin, int yMax, RunningStats* stats);
	bool observeDepthCreate(const int &x, const int &y, const int &idx, RunningStats* const &stats);
	bool observeDepthUpdate(const int &x, const int &y, const int &idx, const float* keyFrameMaxGradBuf, RunningStats* const &stats);
	bool makeAndCheckEPL(const int x, const int y, const Frame* const ref, float* pepx, float* pepy, RunningStats* const stats);

	// does the line-stereo seeking.
	// takes a lot of parameters, because they all have been pre-computed before.
	float doLineStereo(
			const float u, const float v, const float epxn, const float epyn,
			const float min_idepth, const float prior_idepth, float max_idepth,
			const Frame* const referenceFrame, const float* referenceFrameImage,
			float &result_idepth, float &result_var, float &result_eplLength,
			RunningStats* const stats);


	void regularizeDepthMap(bool removeOcclusion, int validityTH);
	template<bool removeOcclusions> void regularizeDepthMapRow(int validityTH, int yMin, int yMax, RunningStats* stats);

	void buildRegIntegralBuffer();
	void buildRegIntegralBufferRow1(int yMin, int yMax, RunningStats* stats);

	void regularizeDepthMapFillHoles();
	void regularizeDepthMapFillHolesRow(int yMin, int yMax, RunningStats* stats);

	void resetCounters();

	IndexThreadReduce threadReducer;

};

}
