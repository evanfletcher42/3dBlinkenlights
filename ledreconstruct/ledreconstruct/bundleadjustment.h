#pragma once

#include <ceres/ceres.h>
#include <random>
#include <string>

#include "ledobservationresidual.h"
#include "pointtracker.h"
#include "pointsvisualizer.h"

class BundleAdjuster
{
public:
	BundleAdjuster( std::vector< std::vector< PointObservation_t > > *pObservations );

	~BundleAdjuster();

	// Load a camera calibration from a JSON file.
	bool LoadCameraCalibrationFromJson(std::string& jsonPath);

	// Initialize the bundle adjustment problem.
	// This scans for a pair of frames where point motion cannot be explained by a homography,
	// then initializes poses & map via Nister's five-point algorithm for essential matrix extraction.  
	bool InitializeBA();

	// Attempt to reconstruct everything.  Runs the entire process.
	bool Reconstruct();

	// Find correspondences between the two specified frames.
	bool FindFrameCorrespondences(int nFrame1, int nFrame2,
		std::vector< const PointObservation_t* >& frame1Obs, std::vector<cv::Point2f>& frame1Points,
		std::vector< const PointObservation_t* >& frame2Obs, std::vector<cv::Point2f>& frame2Points);

	// Attempt to find a homography that explains image point correspondences between two frames.
	bool TwoFrameHomography( int nFrame1, int nFrame2, cv::Mat& homography );

	// Estimate the relative poses of two frames via Nister's five-point method + a RANSAC / cheirality check. 
	// Also initializes locations of LEDs seen by this pair.  
	bool TwoFrameEssentialMatInit(int nFrame1, int nFrame2, cv::Mat &cameraMat);
	
	// Initialize the specified camera's cameraFromWorld transform via a PnP solve on the current map.
	bool InitCameraPosePnP(int nFrameIdx);

	// Attempt to initialize poses of new cameras that can see points in the current map.  
	// Returns the number of new cameras initialized.  
	int InitializeNewCameras();

	// Attempt to initialize new points that are visible in 2+ frames in the current map.
	int TriangulateNewPoints();

	// Test whether an LED is in front of all cameras that observe it.
	// Useful for resetting bad points or poor triangulations.
	bool TestLedInFrontOfAllObservingCameras( int nLedIdx );

	// Mark as unknown points that are in invalid locations (e.g. behind a camera).
	int CullInvalidPoints();

	// Set up a bundle adjustment problem over the entire map, as specified.
	bool SetupBundleAdjustmentProblem( ceres::Problem &problem, bool bSolveIntrinsics, bool bSolvePoses, bool bSolvePoints );

	bool SolveProblem(ceres::Problem& problem);

	// Find the transform that aligns the reconstruction along the Z axis.  
	bool FindAlignmentTransform();

	// Output a file containing all recovered LED locations and camera poses.
	bool WriteSolutionToJson(std::string& outPath);

	// Output a file containing LED locations, after alignment.
	bool WriteAlignedReconstructionToJson(const std::string& outPath);

	bool DrawReconstructionOnVideo(std::string videoPath);

	static constexpr int k_nMaxLeds = 500;

	// Minimum number of map point observations required to initialize the pose of a new camera.
	// Should be >= 4.  
	static constexpr int k_nMinObservationsForPnPInit = 6;

	// Attempt up to this many times to triangulate a point.
	static constexpr int k_nMaxTriangulationRetries = 10;

	// In TwoFrameEssentialMatInit(...), require that at least this fraction of points are inliers.
	static constexpr float k_flMinEssentialMatInitInlierFrac = 0.75;

	const std::vector< std::vector< PointObservation_t > >* m_pObservations;

	// Map of camera poses, indexed by frame id.
	// Each pose describes the change of basis from world coordinates to camera coodrinates.
	std::unordered_map< int, std::array<double, 6> > m_cameraFromWorldPoses;

	// LED points in 3D space ("the map")
	// TBD: Heap?  std::vector?  (this is big)
	double m_ledPoints[k_nMaxLeds][3];

	// Marks whether a given LED has been initialized.
	bool m_pointsInitialized[k_nMaxLeds];

private:
	static void composeRT(double* pose, cv::Mat& poseRT);

	// The frame ID which defines the origin.
	int m_nOriginFrameId;

	// Alignment transform.
	double m_alignedFromWorld[6];

	// Camera calibration.
	cv::Mat m_cameraMat;
	cv::Mat m_distCoefs;
	double m_intrinsics[LedObservationResidual::nIntrinsicsTraits];

	// RNG
	std::random_device m_rd;
	std::mt19937 m_gen;

	// Visualizations
	CPointsVisualizer m_visualizer;
};

class VisualizerCallback : public ceres::IterationCallback
{
public:
	explicit VisualizerCallback(BundleAdjuster* pBundleAdjuster, CPointsVisualizer* pVisualizer);
	~VisualizerCallback();

	ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary);

	BundleAdjuster* m_pBundleAdjuster = NULL;
	CPointsVisualizer* m_pVisualizer = NULL;
};
