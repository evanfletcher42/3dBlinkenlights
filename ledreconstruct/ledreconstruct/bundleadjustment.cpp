#include <algorithm>
#include <fstream>

#include <Eigen/Core>
#include <ceres/rotation.h>
#include <json/json.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "bundleadjustment.h"

BundleAdjuster::BundleAdjuster(std::vector<std::vector<PointObservation_t>>* pObservations)
	: m_pObservations(pObservations)
{
	// No points are initialized (yet)
	memset(m_pointsInitialized, 0, sizeof(m_pointsInitialized));

	// Initial alignment transform -> identity
	memset(m_alignedFromWorld, 0, sizeof(m_alignedFromWorld));

	// Origin frame ID is unknownd
	m_nOriginFrameId = -1;

	// Initialize camera calibration to something that would never work.
	// User should load camera calibration from JSON.  
	m_cameraMat = cv::Mat::zeros(3, 3, CV_32FC1);
	m_distCoefs = cv::Mat::zeros(1, 5, CV_32FC1);

	// Seed the RNG.
	m_gen = std::mt19937(m_rd());
}

BundleAdjuster::~BundleAdjuster()
{
}

bool BundleAdjuster::LoadCameraCalibrationFromJson(std::string& jsonPath)
{
	Json::Value calJson;
	std::ifstream ifs(jsonPath);

	if (!ifs)
		return false;

	ifs >> calJson;

	if (!calJson.isMember("fx") ||
		!calJson.isMember("fy") ||
		!calJson.isMember("cx") ||
		!calJson.isMember("cy") ||
		!calJson.isMember("dist"))
		return false;

	m_cameraMat = cv::Mat::eye(cv::Size(3, 3), CV_32FC1);
	m_cameraMat.at<float>(0, 0) = calJson["fx"].asFloat();
	m_cameraMat.at<float>(1, 1) = calJson["fy"].asFloat();
	m_cameraMat.at<float>(0, 2) = calJson["cx"].asFloat();
	m_cameraMat.at<float>(1, 2) = calJson["cy"].asFloat();

	m_distCoefs = cv::Mat::zeros(1, 5, CV_32FC1);
	for (int i = 0; i < 5; i++)
	{
		m_distCoefs.at<float>(i) = calJson["dist"][i].asFloat();
		printf("Dist: %f\n", m_distCoefs.at<float>(i));
	}

	m_intrinsics[LedObservationResidual::eFx] = calJson["fx"].asDouble();
	m_intrinsics[LedObservationResidual::eFy] = calJson["fy"].asDouble();
	m_intrinsics[LedObservationResidual::eCx] = calJson["cx"].asDouble();
	m_intrinsics[LedObservationResidual::eCy] = calJson["cy"].asDouble();
	m_intrinsics[LedObservationResidual::eK1] = calJson["dist"][0].asDouble();
	m_intrinsics[LedObservationResidual::eK2] = calJson["dist"][1].asDouble();
	m_intrinsics[LedObservationResidual::eP1] = calJson["dist"][2].asDouble();
	m_intrinsics[LedObservationResidual::eP2] = calJson["dist"][3].asDouble();
	m_intrinsics[LedObservationResidual::eK3] = calJson["dist"][4].asDouble();


	return true;
}

bool BundleAdjuster::InitializeBA()
{
	// Find a pair of frames where the transformation cannot be explained by a homography.
	// This enforces parallax between the two frames, making triangulation possible.

	int nFrame1 = 0;

	// Scan ahead until we find a frame with labelled LEDs.
	while (true)
	{
		int nLabeledLeds = 0;

		for (const PointObservation& obs : (*m_pObservations)[nFrame1])
		{
			if (obs.status == PointObservation::ObservationStatus::eLabeled)
				nLabeledLeds++;
		}

		if (nLabeledLeds > 0)
			break;

		nFrame1++;
	}
	printf("Scanned ahead to frame %d\n", nFrame1);

	printf("Looking for perspective differences...\n");
	int nFrame2;
	for (nFrame2 = nFrame1+5; nFrame2 < m_pObservations->size(); nFrame2++)
	{
		cv::Mat homography;
		bool bHomographySuccess = TwoFrameHomography(nFrame1, nFrame2, homography);

		printf("Frame %d -> %d: %d\n", nFrame1, nFrame2, bHomographySuccess);

		if (!bHomographySuccess)
			break;
	}
	
	// Given that we have parallax, initialize camera poses and triangulate points via an essential-matrix decomposition.
	cv::Mat cameraMat = cv::Mat::eye(cv::Size(3, 3), CV_32FC1);
	TwoFrameEssentialMatInit(nFrame1, nFrame2, cameraMat);

	// Refine with a two-frame bundle adjustment.  
	ceres::Problem problem;

	SetupBundleAdjustmentProblem(problem, false, true, true);

	SolveProblem(problem);

	return true;
}

bool BundleAdjuster::Reconstruct()
{
	if (!InitializeBA())
	{
		printf("Reconstruct: Initialization failed\n");
		return false;
	}

	m_visualizer.OnDataUpdate(m_cameraFromWorldPoses, (double*)m_ledPoints, m_pointsInitialized);

	int nInitPointsRemoved = CullInvalidPoints();
	printf("Reconstruct: Removed %d invalid points from initial BA solve\n", nInitPointsRemoved);

	int nIterations = 0;
	while (true)
	{
		printf("\nReconstruct: Iteration %d\n", nIterations);
		nIterations++;

		printf("\nReconstruct: Initializing new viewpoints...\n");
		int nCamerasAdded = InitializeNewCameras();

		m_visualizer.OnDataUpdate(m_cameraFromWorldPoses, (double*)m_ledPoints, m_pointsInitialized);

		if (nCamerasAdded <= 0)
		{
			printf("Reconstruct: No new viewpoints\n");
			break;
		}

		printf("Solving poses for new viewpoints...\n");
		ceres::Problem problem1;
		SetupBundleAdjustmentProblem(problem1, false, true, false);
		SolveProblem(problem1);

		m_visualizer.OnDataUpdate(m_cameraFromWorldPoses, (double*)m_ledPoints, m_pointsInitialized);

		printf("Solving poses + points for new viewpoints...\n");
		ceres::Problem problem2;
		SetupBundleAdjustmentProblem(problem2, false, true, true);
		SolveProblem(problem2);

		m_visualizer.OnDataUpdate(m_cameraFromWorldPoses, (double*)m_ledPoints, m_pointsInitialized);

		printf("Triangulating new points...\n");
		int nPointsAdded = TriangulateNewPoints();
		printf("Triangulated %d points\n", nPointsAdded);

		printf("Culling invalid points...\n");
		int nPointsRemoved = CullInvalidPoints();
		printf("Culled %d invalid points\n", nPointsRemoved);

		m_visualizer.OnDataUpdate(m_cameraFromWorldPoses, (double*)m_ledPoints, m_pointsInitialized);

		// Skip 2nd BA solve if we didn't add new points
		if (nPointsAdded <= 0)
			continue;

		printf("Solving poses + points for new points...\n");
		ceres::Problem problem3;
		SetupBundleAdjustmentProblem(problem3, false, true, true);
		SolveProblem(problem3);

		m_visualizer.OnDataUpdate(m_cameraFromWorldPoses, (double*)m_ledPoints, m_pointsInitialized);
	}

	// Do one final BA solve where the intrinsics of the camera are refined.
	// (Typically BA problems would do this sooner.  However, LED coverage can be somewhat sparse compared to natural features.) 

	printf("\nReconstruct: Solving final BA...\n");

	ceres::Problem problem;
	SetupBundleAdjustmentProblem(problem, true, true, true);
	SolveProblem(problem);

	// With final BA, try one last round of triangulation.  
	int nPointsAdded = TriangulateNewPoints();
	printf("Reconstruct: Triangulated %d points\n", nPointsAdded);

	int nPointsRemoved = CullInvalidPoints();
	printf("Reconstruct: Culled %d invalid points\n", nPointsRemoved);

	ceres::Problem problem4;
	SetupBundleAdjustmentProblem(problem4, false, true, true);
	SolveProblem(problem4);

	printf("Reconstruct complete\n");
	printf("Solution used %zu of %zu frames\n", m_cameraFromWorldPoses.size(), m_pObservations->size());

	return true;
}

bool BundleAdjuster::FindFrameCorrespondences(int nFrame1, int nFrame2, 
	std::vector< const PointObservation_t* > &frame1Obs, std::vector<cv::Point2f>& frame1Points, 
	std::vector< const PointObservation_t* > &frame2Obs, std::vector<cv::Point2f>& frame2Points)
{
	for (const PointObservation_t& obs1 : (*m_pObservations)[nFrame1])
	{
		if (obs1.status != PointObservation::ObservationStatus::eLabeled)
			continue;

		for (const PointObservation_t& obs2 : (*m_pObservations)[nFrame2])
		{
			if (obs2.status != PointObservation::ObservationStatus::eLabeled)
				continue;

			if (obs1.nLedIdx == obs2.nLedIdx)
			{
				frame1Points.push_back(obs1.p);
				frame2Points.push_back(obs2.p);

				frame1Obs.push_back(&obs1);
				frame2Obs.push_back(&obs2);

				break;
			}
		}
	}

	return true;
}

bool BundleAdjuster::TwoFrameHomography(int nFrame1, int nFrame2, cv::Mat &homography)
{
	std::vector< cv::Point2f > frame1Pts;
	std::vector< cv::Point2f > frame2Pts;

	std::vector< const PointObservation_t* > frame1Obs;
	std::vector< const PointObservation_t* > frame2Obs;

	if (!FindFrameCorrespondences(nFrame1, nFrame2, frame1Obs, frame1Pts, frame2Obs, frame2Pts))
	{
		printf("TwoFrameHomography: Failed to find correspondences between frames %d and %d\n", nFrame1, nFrame2);
		return false;
	}
		

	if (frame1Pts.size() < 4)
	{
		// While we could find *a* homography, it wouldn't be particularly meaningful
		printf("TwoFrameHomography: Too few points (%zu)\n", frame1Pts.size());
		return false;
	}

	// Undistort points

	// A reasonable camera matrix is expected for findHomography, which expects coordinates in units of pixels.
	// Outlier rejection seems to misbehave if provided normalized camera coordinates.
	cv::Mat optimalCameraMat = cv::getOptimalNewCameraMatrix(m_cameraMat, m_distCoefs, cv::Size(1920, 1080), 0);

	std::vector< cv::Point2f > frame1PtsUndistorted;
	std::vector< cv::Point2f > frame2PtsUndistorted;
	cv::undistortPoints(frame1Pts, frame1PtsUndistorted, m_cameraMat, m_distCoefs, cv::noArray(), optimalCameraMat);
	cv::undistortPoints(frame2Pts, frame2PtsUndistorted, m_cameraMat, m_distCoefs, cv::noArray(), optimalCameraMat);

	std::vector<uint8_t> inlierMask(frame1Pts.size(), 0);

	const double k_flRansacReprojThresholdPx = 10.0;
	const int k_nMaxIters = 2000;
	const double k_flConfidence = 0.995;

	homography = cv::findHomography(frame1PtsUndistorted, frame2PtsUndistorted, cv::RANSAC, k_flRansacReprojThresholdPx, inlierMask, k_nMaxIters, k_flConfidence);

	if (homography.empty())
	{
		printf("TwoFrameHomography: returned empty matrix\n");
		return false;
	}

	// Judge whether that was acceptable
	std::vector< cv::Point2f > transformedPoints;
	try
	{
		cv::perspectiveTransform(frame1PtsUndistorted, transformedPoints, homography);
	}
	catch (cv::Exception& e)
	{
		printf("Exception in cv::perspectiveTransform: %s\n", e.what());
		return false;
	}

	int nInliers = 0;
	for (int i = 0; i < frame1PtsUndistorted.size(); i++)
	{
		if (!inlierMask[i])
			continue;

		nInliers++;
	}
	printf("\n");

	const double k_flInlierThreshold = 0.50;
	double flInlierRatio = ((double)(nInliers)) / frame1Pts.size();

	if (flInlierRatio < k_flInlierThreshold)
	{
		printf("TwoFrameHomography: Not enough inliers (%d / %zu)\n", nInliers, frame1Pts.size());
		return false;
	}
		
	printf("TwoFrameHomography: Homography works (%d / %zu)\n", nInliers, frame1Pts.size());
	return true;
}

bool BundleAdjuster::TwoFrameEssentialMatInit(int nFrame1, int nFrame2, cv::Mat &cameraMat)
{
	std::vector< cv::Point2f > frame1Pts;
	std::vector< cv::Point2f > frame2Pts;

	std::vector< const PointObservation_t* > frame1Obs;
	std::vector< const PointObservation_t* > frame2Obs;

	if (!FindFrameCorrespondences(nFrame1, nFrame2, frame1Obs, frame1Pts, frame2Obs, frame2Pts))
		return false;

	// Undistort points to normalized camera coordinates
	cv::Mat normalizedCameraMat = cv::Mat::eye(3, 3, CV_32FC1);
	std::vector< cv::Point2f > frame1PtsUndistorted;
	std::vector< cv::Point2f > frame2PtsUndistorted;
	cv::undistortPoints(frame1Pts, frame1PtsUndistorted, m_cameraMat, m_distCoefs);
	cv::undistortPoints(frame2Pts, frame2PtsUndistorted, m_cameraMat, m_distCoefs);

	// Nister's 5-point method requires... five points... but subsequent attempts to decompose the essential matrix can yield 
	// multiple solutions.  A RANSAC approach over more-than-5 points will be used to find a unique solution.
	if (frame1Pts.size() < 8)
		return false;

	std::vector<uint8_t> inlierMask(frame1Pts.size(), 1);

	double k_flDesiredConfidence = 0.995;
	double k_flRansacReprojThresholdPx = 2.5;

	cv::Mat E;
	try
	{
		E = cv::findEssentialMat(frame1PtsUndistorted, frame2PtsUndistorted, normalizedCameraMat, cv::RANSAC, k_flDesiredConfidence, k_flRansacReprojThresholdPx, inlierMask);
	}
	catch (cv::Exception& e)
	{
		printf("Exception in cv::findEssentialMat: %s\n", e.what());
		return false;
	}
	// Maximum distance threshold for dropping infinite-distance points.
	// Note: The essential matrix decomposition under the hood in recoverPose cannot estimate the scale of a translation; recoverPose
	//		 will always return a unit vector.  As a consequence, all triangulated points have units of whatever-this-translation-was, 
	//		 which may be a large-ish number in case of small-ish real translations.  
	double k_flMaxDistance = 100.0;

	cv::Mat frame2FromFrame1R;
	cv::Mat frame2FromFrame1T;

	// Triangulated points as a 4xN array (homogeneous coordinates).
	cv::Mat pInFrame1;

	int nRecoverPoseInliers = 0;
	try
	{
		// The return value from cv::recoverPose() is undocumented, but a variable name in OpenCV's test_cameracalibration.cpp 
		// suggests it is the number of inliers.  This seems accurate.
		nRecoverPoseInliers = cv::recoverPose(E, frame1PtsUndistorted, frame2PtsUndistorted, normalizedCameraMat, frame2FromFrame1R, frame2FromFrame1T, k_flMaxDistance, inlierMask, pInFrame1);
	}
	catch (cv::Exception& e)
	{
		printf("Exception in cv::recoverPose: %s\n", e.what());
		return false;
	}

	// Since we are using this to initialize the entire solve, we should succeed only if there are a large number of inliers
	if (((float)nRecoverPoseInliers) / frame1Pts.size() < k_flMinEssentialMatInitInlierFrac)
		return false;

	// ============== Initialize parts of the problem as best we can ================
	
	// Set frame 1 camera pose (origin) 
	m_cameraFromWorldPoses[nFrame1][0] = 0.0;
	m_cameraFromWorldPoses[nFrame1][1] = 0.0;
	m_cameraFromWorldPoses[nFrame1][2] = 0.0;
	m_cameraFromWorldPoses[nFrame1][3] = 0.0;
	m_cameraFromWorldPoses[nFrame1][4] = 0.0;
	m_cameraFromWorldPoses[nFrame1][5] = 0.0;

	// Mark this as the origin frame
	m_nOriginFrameId = nFrame1;

	/// Set frame 2 camera pose - use found transform
	Eigen::Matrix<double, 3, 3> frame2FromFrame1REigen;
	cv::cv2eigen(frame2FromFrame1R, frame2FromFrame1REigen);

	ceres::RotationMatrixToAngleAxis(frame2FromFrame1REigen.data(), m_cameraFromWorldPoses[nFrame2].data());
	m_cameraFromWorldPoses[nFrame2][3] = frame2FromFrame1T.at<double>(0);
	m_cameraFromWorldPoses[nFrame2][4] = frame2FromFrame1T.at<double>(1);
	m_cameraFromWorldPoses[nFrame2][5] = frame2FromFrame1T.at<double>(2);

	// Set point locations for inliers
	for (int i = 0; i < frame1Obs.size(); i++)
	{
		// Ignore outliers
		if (!inlierMask[i])
			continue;

		int nLedIdx = frame1Obs[i]->nLedIdx;

		m_ledPoints[nLedIdx][0] = pInFrame1.at<double>(0, i) / pInFrame1.at<double>(3, i);
		m_ledPoints[nLedIdx][1] = pInFrame1.at<double>(1, i) / pInFrame1.at<double>(3, i);
		m_ledPoints[nLedIdx][2] = pInFrame1.at<double>(2, i) / pInFrame1.at<double>(3, i);

		m_pointsInitialized[nLedIdx] = true;
	}

	// Normalize map and camera scale.
	// At this point, points and poses have been reconstructed up to a scale parameter; the transform between the two views is of
	// unit length, and the point cloud is arbitrarily sized.
	// For analytical convenience, rescale everything so that the largest LED coordinate is in the range [-1...1].  
	// This serves no mathematical purpose other than to make it easier to plot things.  

	double flMaxAbsX = 0.0;
	double flMaxAbsY = 0.0;
	double flMaxAbsZ = 0.0;

	for (int i = 0; i < frame1Obs.size(); i++)
	{
		// Ignore outliers
		if (!inlierMask[i])
			continue;

		flMaxAbsX = std::max(flMaxAbsX, std::abs(m_ledPoints[frame1Obs[i]->nLedIdx][0]));
		flMaxAbsY = std::max(flMaxAbsY, std::abs(m_ledPoints[frame1Obs[i]->nLedIdx][1]));
		flMaxAbsZ = std::max(flMaxAbsZ, std::abs(m_ledPoints[frame1Obs[i]->nLedIdx][2]));
	}

	double flMaxScale = std::max(flMaxAbsX, std::max(flMaxAbsY, flMaxAbsZ));

	if (flMaxScale < 10*std::numeric_limits<double>::epsilon())
	{
		printf("TwoFrameEssentialMatInit: Error: Tiny max abs LED coordinate: %f.  This is probably not correct.\n", flMaxScale);
		return false;
	}

	// Scale frame 2 translation (frame 1 is already at the origin)
	m_cameraFromWorldPoses[nFrame2][3] /= flMaxScale;
	m_cameraFromWorldPoses[nFrame2][4] /= flMaxScale;
	m_cameraFromWorldPoses[nFrame2][5] /= flMaxScale;

	// Scale LED points
	for (int i = 0; i < frame1Obs.size(); i++)
	{
		// Ignore outliers
		if (!inlierMask[i])
			continue;

		m_ledPoints[frame1Obs[i]->nLedIdx][0] /= flMaxScale;
		m_ledPoints[frame1Obs[i]->nLedIdx][1] /= flMaxScale;
		m_ledPoints[frame1Obs[i]->nLedIdx][2] /= flMaxScale;

		printf("LED %d: %f %f %f\n", frame1Obs[i]->nLedIdx, m_ledPoints[frame1Obs[i]->nLedIdx][0], m_ledPoints[frame1Obs[i]->nLedIdx][1], m_ledPoints[frame1Obs[i]->nLedIdx][2]);
	}

	return true;
}

bool BundleAdjuster::InitCameraPosePnP(int nFrameIdx)
{
	std::vector< cv::Point3d > mapPoints;
	std::vector< cv::Point2d > imgPoints;

	for (const PointObservation_t& obs : (*m_pObservations)[nFrameIdx])
	{
		// Does this point have a label?
		if (obs.nLedIdx < 0)
			continue;

		// Is this a point in the current map?
		if (!m_pointsInitialized[obs.nLedIdx])
			continue;

		cv::Point3d mapPt(m_ledPoints[obs.nLedIdx][0], m_ledPoints[obs.nLedIdx][1], m_ledPoints[obs.nLedIdx][2]);
		mapPoints.push_back(mapPt);
		imgPoints.push_back(obs.p);
	}

	if (mapPoints.size() < k_nMinObservationsForPnPInit)
	{
		//printf("InitCameraPosePnP: Not enough map points observed (%zu, required %d)\n", mapPoints.size(), k_nMinObservationsForPnPInit);
		return false;
	}

	cv::Mat rCameraFromWorld, tCameraFromWorld;
	std::vector<int> inliers;
	bool bSuccess = cv::solvePnPRansac(mapPoints, imgPoints, m_cameraMat, m_distCoefs, rCameraFromWorld, tCameraFromWorld, false, 100, 8.0, 0.99, inliers, cv::SOLVEPNP_EPNP);

	if (!bSuccess)
	{
		//printf("InitCameraPosePnP: PnP solve for frame %d failed\n", nFrameIdx);
		return false;
	}

	// Sanity check: Most input points need to be inliners
	//if (inliers.size() < mapPoints.size() / 2)
	if(inliers.size() < mapPoints.size() * 0.95)
	{
		return false;
	}

	// Sanity check: At least k_nMinObservationsForPnPInit need to be inliers
	if (inliers.size() < k_nMinObservationsForPnPInit)
	{
		return false;
	}

	// Initialize camera
	m_cameraFromWorldPoses[nFrameIdx][0] = rCameraFromWorld.at<double>(0);
	m_cameraFromWorldPoses[nFrameIdx][1] = rCameraFromWorld.at<double>(1);
	m_cameraFromWorldPoses[nFrameIdx][2] = rCameraFromWorld.at<double>(2);

	m_cameraFromWorldPoses[nFrameIdx][3] = tCameraFromWorld.at<double>(0);
	m_cameraFromWorldPoses[nFrameIdx][4] = tCameraFromWorld.at<double>(1);
	m_cameraFromWorldPoses[nFrameIdx][5] = tCameraFromWorld.at<double>(2);

	// Sanity check: All points in the map that this camera claims to observe need to be in front of the camera
	double pInCamera[3];
	for (const PointObservation_t& obs : (*m_pObservations)[nFrameIdx])
	{
		// Does this point have a label?
		if (obs.status != PointObservation::ObservationStatus::eLabeled)
			continue;

		// Is this a point in the current map?
		if (!m_pointsInitialized[obs.nLedIdx])
			continue;

		const double* const pInWorld = m_ledPoints[obs.nLedIdx];

		ceres::AngleAxisRotatePoint(m_cameraFromWorldPoses[nFrameIdx].data(), pInWorld, pInCamera);
		pInCamera[0] += m_cameraFromWorldPoses[nFrameIdx][3];
		pInCamera[1] += m_cameraFromWorldPoses[nFrameIdx][4];
		pInCamera[2] += m_cameraFromWorldPoses[nFrameIdx][5];

		if (pInCamera[2] <= 0)
		{
			m_cameraFromWorldPoses.erase(nFrameIdx);
			return false;
		}
	}

	return true;
}

int BundleAdjuster::InitializeNewCameras()
{
	int nFramesAdded = 0;

	// Attempt to initialize every camera frame that is not already in the map.
	for (int nFrameIdx = 0; nFrameIdx < m_pObservations->size(); nFrameIdx++)
	{
		if (m_cameraFromWorldPoses.find(nFrameIdx) != m_cameraFromWorldPoses.end())
			continue;

		bool bSuccess = InitCameraPosePnP(nFrameIdx);

		if (bSuccess)
		{
			nFramesAdded++;
		}
	}

	printf("InitializeNewCameras: Added %d viewpoints\n", nFramesAdded);

	return nFramesAdded;
}

int BundleAdjuster::TriangulateNewPoints()
{
	// TBD: This initializes new points using 2 frames.  New points are probably visible in more than 2 frames. 
	//		Using all of them (i.e. DLT over all viewpoints) would probably yield a better initialization.
	//		That said, it may not be worth the expense - the BA solve right after triangulation does much the same
	//		thing as a more-optimal initialization would.
	
	// All tracked frames where a given untracked LED is observed.
	// Addressing: ledTrackedFrames[nLedIdx] -> vector of frames where LED is tracked
	std::unordered_map<int, std::vector< int > > ledTrackedFrames;

	// Locations of each LED in each tracked frame.
	std::unordered_map<int, std::vector< cv::Point2f> > ledPoints;

	for (auto& kv : m_cameraFromWorldPoses)
	{
		int nFrameIdx = kv.first;

		for (const PointObservation_t& obs : (*m_pObservations)[nFrameIdx])
		{
			// Got a label?
			if (obs.status != PointObservation::ObservationStatus::eLabeled)
				continue;

			// LED already initialized?
			if (m_pointsInitialized[obs.nLedIdx])
				continue;

			ledTrackedFrames[obs.nLedIdx].push_back(nFrameIdx);
			ledPoints[obs.nLedIdx].push_back(obs.p);
		}
	}

	int nPointsTriangulated = 0;

	for (const auto& kv : ledTrackedFrames)
	{
		const int nLedIdx = kv.first;
		const std::vector<int>& frames = kv.second;

		// Two observations required
		if (frames.size() < 2)
			continue;

		std::uniform_int_distribution<int> d1(0, frames.size() - 1);
		std::uniform_int_distribution<int> d2(0, frames.size() - 2);

		// Choose random pairs & attempt triangulation up to a few times.
		for (int nRetries = 0; nRetries < k_nMaxTriangulationRetries; nRetries++)
		{
			// Choose a random pair of frames
			int i1 = d1(m_gen);
			int i2 = d2(m_gen);
			if (i2 >= i1)
				i2++;

			int nFrame1 = frames[i1];
			int nFrame2 = frames[i2];

			cv::Mat camFromWorld1;
			composeRT(m_cameraFromWorldPoses[nFrame1].data(), camFromWorld1);
			cv::Mat proj1 = m_cameraMat * camFromWorld1;

			cv::Mat camFromWorld2;
			composeRT(m_cameraFromWorldPoses[nFrame2].data(), camFromWorld2);
			cv::Mat proj2 = m_cameraMat * camFromWorld2;

			// Undistort both observations of this LED.
			std::vector<cv::Point2f> pDistorted1;
			std::vector<cv::Point2f> pDistorted2;

			pDistorted1.push_back(ledPoints[nLedIdx][i1]);
			pDistorted2.push_back(ledPoints[nLedIdx][i2]);

			std::vector<cv::Point2f> pUndistorted1;
			std::vector<cv::Point2f> pUndistorted2;

			cv::undistortPoints(pDistorted1, pUndistorted1, m_cameraMat, m_distCoefs);
			cv::undistortPoints(pDistorted2, pUndistorted2, m_cameraMat, m_distCoefs);

			cv::Mat pInWorldHomogenous;

			cv::triangulatePoints(proj1, proj2, pUndistorted1, pUndistorted2, pInWorldHomogenous);

			// Initialize point
			m_ledPoints[nLedIdx][0] = pInWorldHomogenous.at<float>(0) / pInWorldHomogenous.at<float>(3);
			m_ledPoints[nLedIdx][1] = pInWorldHomogenous.at<float>(1) / pInWorldHomogenous.at<float>(3);
			m_ledPoints[nLedIdx][2] = pInWorldHomogenous.at<float>(2) / pInWorldHomogenous.at<float>(3);

			//printf("Triangulated LED %d: %f, %f, %f\n", nLedIdx, m_ledPoints[nLedIdx][0], m_ledPoints[nLedIdx][1], m_ledPoints[nLedIdx][2]);

			m_pointsInitialized[nLedIdx] = true;

			// Confirm point is reasonable.  If it's not, roll a new pair of frames and try again.
			if (!TestLedInFrontOfAllObservingCameras(nLedIdx))
			{
				m_pointsInitialized[nLedIdx] = false;
				continue;
			}
			nPointsTriangulated++;

			// stop retrying pairs of frames, it worked
			break;
		}
	}

	return nPointsTriangulated;
}

bool BundleAdjuster::TestLedInFrontOfAllObservingCameras(int nLedIdx)
{
	if (!m_pointsInitialized[nLedIdx])
		return false;

	const double* const pInWorld = m_ledPoints[nLedIdx];

	bool bLedObserved = false;

	double pInCamera[3];
	for (auto& kv : m_cameraFromWorldPoses)
	{
		int nFrameIdx = kv.first;
		std::array<double, 6>& cameraFromWorld = kv.second;

		for (const PointObservation_t& obs : (*m_pObservations)[nFrameIdx])
		{
			if (obs.status != PointObservation::ObservationStatus::eLabeled)
				continue;

			if (obs.nLedIdx != nLedIdx)
				continue;

			// Found the LED
			bLedObserved = true;

			ceres::AngleAxisRotatePoint(cameraFromWorld.data(), pInWorld, pInCamera);
			pInCamera[0] += cameraFromWorld[3];
			pInCamera[1] += cameraFromWorld[4];
			pInCamera[2] += cameraFromWorld[5];

			if (pInCamera[2] <= 0)
			{
				return false;
			}

			break;
		}
	}

	// Return false if nothing saw the LED at all
	if (!bLedObserved)
		return false;

	return true;
}

int BundleAdjuster::CullInvalidPoints()
{
	int nLedsRemoved = 0;

	double pInCamera[3];
	for (auto& kv : m_cameraFromWorldPoses)
	{
		int nFrameIdx = kv.first;
		std::array<double, 6>& cameraFromWorld = kv.second;

		for (const PointObservation_t& obs : (*m_pObservations)[nFrameIdx] )
		{
			if (obs.status != PointObservation::ObservationStatus::eLabeled)
				continue;

			if (!m_pointsInitialized[obs.nLedIdx])
				continue;

			const double* const pInWorld = m_ledPoints[obs.nLedIdx];

			ceres::AngleAxisRotatePoint(cameraFromWorld.data(), pInWorld, pInCamera);
			pInCamera[0] += cameraFromWorld[3];
			pInCamera[1] += cameraFromWorld[4];
			pInCamera[2] += cameraFromWorld[5];

			if (pInCamera[2] <= 0)
			{
				m_pointsInitialized[obs.nLedIdx] = false;
				nLedsRemoved++;
			}
		}
	}

	return nLedsRemoved;
}

bool BundleAdjuster::SetupBundleAdjustmentProblem(ceres::Problem& problem, bool bSolveIntrinsics, bool bSolvePoses, bool bSolvePoints)
{
	if (m_nOriginFrameId < 0)
	{
		printf("SetupBundleAdjustmentProblem: Problem must be initialized first\n");
		return false;
	}

	for (auto& poseKV : m_cameraFromWorldPoses)
	{
		int nFrameIdx = poseKV.first;
		std::array<double, 6>& pose = poseKV.second;

		const std::vector<PointObservation_t>& obsThisFrame = (*m_pObservations)[nFrameIdx];

		for (const PointObservation_t &obs : obsThisFrame)
		{
			// Labelled LEDs only
			if (obs.status != PointObservation::ObservationStatus::eLabeled)
				continue;

			// Points with initializations only
			if (!m_pointsInitialized[obs.nLedIdx])
				continue;

			ceres::CostFunction* cf = LedObservationResidual::Create(obs);
			problem.AddResidualBlock(cf, new ceres::CauchyLoss(2.0), pose.data(), m_intrinsics, m_ledPoints[obs.nLedIdx]);

			if (!bSolvePoints)
			{
				problem.SetParameterBlockConstant(m_ledPoints[obs.nLedIdx]);
			}
		}

		if (!bSolvePoses)
		{
			problem.SetParameterBlockConstant(pose.data());
		}
	}

	if (problem.NumResiduals() <= 0)
	{
		printf("SetupBundleAdjustmentProblem: Error: Empty problem\n");
		return false;
	}

	// Confirm the origin frame is in the problem
	std::vector< ceres::ResidualBlockId > residualBlocksForOriginCamera;
	problem.GetResidualBlocksForParameterBlock(m_cameraFromWorldPoses[m_nOriginFrameId].data(), &residualBlocksForOriginCamera);
	if (residualBlocksForOriginCamera.size() <= 0)
	{
		printf("SetupBundleAdjustmentProblem: Error: Origin is not in the problem\n");
		return false;
	}

	// The camera at the origin defines the coordinate system, and is always locked
	problem.SetParameterBlockConstant(m_cameraFromWorldPoses[m_nOriginFrameId].data());

	if (!bSolveIntrinsics)
	{
		problem.SetParameterBlockConstant(m_intrinsics);
	}

	return true;
}

bool BundleAdjuster::SolveProblem(ceres::Problem& problem)
{
	ceres::Solver::Options options;

	// Parallel processing: Use a number of threads appropriate for the machine and the problem
	int k_nMinimumResidualsPerThread = 16;
	int nMachineThreads = std::thread::hardware_concurrency();
	nMachineThreads = MAX(nMachineThreads, 8);  // nMachineThreads may be 0 if detection fails.  8 threads seems sane for a ~2021 PC.

	int nSolveThreads = MIN(nMachineThreads, problem.NumResidualBlocks() / k_nMinimumResidualsPerThread);
	nSolveThreads = MAX(nSolveThreads, 1);

	options.num_threads = nSolveThreads;

	options.linear_solver_type = ceres::LinearSolverType::SPARSE_SCHUR;
	options.trust_region_strategy_type = ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT;
	options.use_nonmonotonic_steps = true;

	options.max_num_iterations = 200;

	options.minimizer_progress_to_stdout = true;

	// TBD: This is a bundle adjustment problem - we may be able to save a bit of preprocessing time by specifying
	//		an explicit linear_solver_ordering that splits all points and all cameras into separate groups.

	// TBD: Convergence tuning...

	// Visualizer callback
	options.callbacks.push_back(new VisualizerCallback(this, &m_visualizer));
	options.update_state_every_iteration = true;

	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	printf(summary.BriefReport().c_str());
	printf("\n");

	// Update the OpenCV float copy of the camera intrinsics, in case this solve changed them.  
	m_cameraMat.at<float>(0, 0) = (float)(m_intrinsics[LedObservationResidual::eFx]);
	m_cameraMat.at<float>(1, 1) = (float)(m_intrinsics[LedObservationResidual::eFy]);
	m_cameraMat.at<float>(0, 2) = (float)(m_intrinsics[LedObservationResidual::eCx]);
	m_cameraMat.at<float>(1, 2) = (float)(m_intrinsics[LedObservationResidual::eCy]);

	m_distCoefs.at<float>(0) = (float)(m_intrinsics[LedObservationResidual::eK1]);
	m_distCoefs.at<float>(1) = (float)(m_intrinsics[LedObservationResidual::eK2]);
	m_distCoefs.at<float>(2) = (float)(m_intrinsics[LedObservationResidual::eP1]);
	m_distCoefs.at<float>(3) = (float)(m_intrinsics[LedObservationResidual::eP2]);
	m_distCoefs.at<float>(4) = (float)(m_intrinsics[LedObservationResidual::eK3]);

	return summary.IsSolutionUsable();
}

bool BundleAdjuster::FindAlignmentTransform()
{
	ceres::Problem problem;

	for (int i = 0; i < k_nMaxLeds; i++)
	{
		if (!m_pointsInitialized[i])
			continue;

		ceres::CostFunction* cf = AlignmentResidual::Create(m_ledPoints[i]);

		// Use robust loss - there may be outlier points in the reconstruction
		problem.AddResidualBlock(cf, new ceres::CauchyLoss(0.5), m_alignedFromWorld);
	}

	ceres::Solver::Options options;
	options.num_threads = 1;
	options.linear_solver_type = ceres::LinearSolverType::SPARSE_NORMAL_CHOLESKY;
	options.trust_region_strategy_type = ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT;
	options.use_nonmonotonic_steps = true;
	options.max_num_iterations = 100;
	options.minimizer_progress_to_stdout = false;

	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	printf("Alignment solve result: %s\n", summary.BriefReport().c_str());
	printf("Alignment transform:\n\tR: %0.06f %0.06f %0.06f\n\tT: %0.06f %0.06f %0.06f\n",
		m_alignedFromWorld[0], m_alignedFromWorld[1], m_alignedFromWorld[2], 
		m_alignedFromWorld[3], m_alignedFromWorld[4], m_alignedFromWorld[5]);

	if (!summary.IsSolutionUsable())
		return false;

	// TBD: Tree might be upside down.
	//		Figure out which end of the tree is pointing up.

	return true;
}

bool BundleAdjuster::WriteSolutionToJson(std::string& outPath)
{
	Json::Value outJson;

	outJson["leds"] = Json::Value(Json::arrayValue);

	for (int i = 0; i < k_nMaxLeds; i++)
	{
		Json::Value ledEntry(Json::arrayValue);
		
		if (m_pointsInitialized[i])
		{
			ledEntry.append(m_ledPoints[i][0]);
			ledEntry.append(m_ledPoints[i][1]);
			ledEntry.append(m_ledPoints[i][2]);
		}
		else
		{
			ledEntry.append(NAN);
			ledEntry.append(NAN);
			ledEntry.append(NAN);
		}

		outJson["leds"].append(ledEntry);
	}
	
	outJson["poses"] = Json::Value();
	for (const auto& kv : m_cameraFromWorldPoses)
	{
		const int nFrameIdx = kv.first;
		const std::array<double, 6> &cameraFromWorld = kv.second;

		Json::Value cameraEntry(Json::arrayValue);
		for (int i = 0; i < 6; i++)
		{
			cameraEntry.append(cameraFromWorld[i]);
		}
		outJson["poses"][nFrameIdx] = cameraEntry;
	}

	Json::StreamWriterBuilder builder;
	builder["commentStyle"] = "None";
	builder["indentation"] = "  ";

	std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
	std::ofstream ofs(outPath);

	if (!ofs)
		return false;

	writer->write(outJson, &ofs);

	return true;
}

bool BundleAdjuster::WriteAlignedReconstructionToJson(const std::string& outPath)
{
	Json::Value outJson;

	outJson["leds"] = Json::Value(Json::arrayValue);

	double pAligned[3];

	for (int i = 0; i < k_nMaxLeds; i++)
	{
		// Apply alignment transform
		ceres::AngleAxisRotatePoint(m_alignedFromWorld, m_ledPoints[i], pAligned);
		pAligned[0] += m_alignedFromWorld[3];
		pAligned[1] += m_alignedFromWorld[4];
		pAligned[2] += m_alignedFromWorld[5];

		Json::Value ledEntry(Json::arrayValue);

		if (m_pointsInitialized[i])
		{
			ledEntry.append(pAligned[0]);
			ledEntry.append(pAligned[1]);
			ledEntry.append(pAligned[2]);
		}
		else
		{
			ledEntry.append(NAN);
			ledEntry.append(NAN);
			ledEntry.append(NAN);
		}

		outJson["leds"].append(ledEntry);
	}

	Json::StreamWriterBuilder builder;
	builder["commentStyle"] = "None";
	builder["indentation"] = "  ";

	std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
	std::ofstream ofs(outPath);

	if (!ofs)
		return false;

	writer->write(outJson, &ofs);

	return true;
}

bool BundleAdjuster::DrawReconstructionOnVideo(std::string videoPath)
{
	cv::VideoCapture cap(videoPath);

	if (!cap.isOpened())
	{
		return false;
	}

	cv::namedWindow("Video", cv::WindowFlags::WINDOW_AUTOSIZE);

	int nFrameIdx = 0;
	while (true)
	{
		cv::Mat img;
		cap >> img;
		if (img.empty())
		{
			break;
		}

		// Do we have a pose for this frame?
		if (m_cameraFromWorldPoses.find(nFrameIdx) != m_cameraFromWorldPoses.end())
		{
			// Project all reconstructed points onto video
			cv::Mat rvec = cv::Mat::zeros(cv::Size(1, 3), CV_64F);
			cv::Mat tvec = cv::Mat::zeros(cv::Size(1, 3), CV_64F);

			rvec.at<double>(0) = m_cameraFromWorldPoses[nFrameIdx][0];
			rvec.at<double>(1) = m_cameraFromWorldPoses[nFrameIdx][1];
			rvec.at<double>(2) = m_cameraFromWorldPoses[nFrameIdx][2];

			tvec.at<double>(0) = m_cameraFromWorldPoses[nFrameIdx][3];
			tvec.at<double>(1) = m_cameraFromWorldPoses[nFrameIdx][4];
			tvec.at<double>(2) = m_cameraFromWorldPoses[nFrameIdx][5];

			std::vector< cv::Point3d > points;
			std::vector< int > labels;
			for (int i = 0; i < k_nMaxLeds; i++)
			{
				if (!m_pointsInitialized[i])
					continue;

				cv::Point3d p;
				p.x = m_ledPoints[i][0];
				p.y = m_ledPoints[i][1];
				p.z = m_ledPoints[i][2];

				points.push_back(p);
				labels.push_back(i);
			}

			std::vector< cv::Point2d > imgPoints;
			cv::projectPoints(points, rvec, tvec, m_cameraMat, m_distCoefs, imgPoints);

			// Draw a labelled circle on each point
			for (int i = 0; i < points.size(); i++)
			{
				cv::circle(img, imgPoints[i], 25, cv::Scalar(0, 255, 0), 1, cv::LineTypes::LINE_AA);
				cv::putText(img, std::to_string(labels[i]), imgPoints[i], cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 255, 0), 1, cv::LineTypes::LINE_AA);
			}
		}

		cv::imshow("Video", img);

		if (cv::waitKey(30) >= 0)
			break;

		nFrameIdx++;
	}

	return true;
}

void BundleAdjuster::composeRT(double* pose, cv::Mat& poseRT)
{
	cv::Mat rvec(3, 1, CV_64F, pose);
	cv::Mat tvec(3, 1, CV_64F, pose + 3);

	cv::Mat R;
	cv::Rodrigues(rvec, R);

	poseRT = cv::Mat::zeros(3, 4, CV_64F);

	cv::Rect roiR = cv::Rect(0, 0, 3, 3);
	cv::Rect roiT = cv::Rect(3, 0, 1, 3);

	cv::Mat poseR = poseRT(roiR);
	cv::Mat poseT = poseRT(roiT);

	R.copyTo(poseR);
	tvec.copyTo(poseT);

	poseRT.convertTo(poseRT, CV_32F);
}

VisualizerCallback::VisualizerCallback(BundleAdjuster* pBundleAdjuster, CPointsVisualizer* pVisualizer) 
	: m_pBundleAdjuster(pBundleAdjuster)
	, m_pVisualizer(pVisualizer)
{
}

VisualizerCallback::~VisualizerCallback()
{
}

ceres::CallbackReturnType VisualizerCallback::operator()(const ceres::IterationSummary& summary)
{
	// Update visualization every frame
	m_pVisualizer->OnDataUpdate(m_pBundleAdjuster->m_cameraFromWorldPoses, (double*)(m_pBundleAdjuster->m_ledPoints), m_pBundleAdjuster->m_pointsInitialized);
	return ceres::SOLVER_CONTINUE;
}
