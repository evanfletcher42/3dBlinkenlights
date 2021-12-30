#include <opencv2/imgproc.hpp>

#include "pointsvisualizer.h"

CPointsVisualizer::CPointsVisualizer()
{
	m_frame = cv::Mat::zeros(k_nImageRows, k_nImageCols, CV_8UC3);

	// Create ROIs for all three views
	cv::Rect roiViewXY = cv::Rect(0, 0, k_nViewWidth, k_nViewHeight);
	cv::Rect roiViewYZ = cv::Rect(k_nViewWidth, k_nViewHeight, k_nViewWidth, k_nViewHeight);
	cv::Rect roiViewXZ = cv::Rect(0, k_nViewHeight, k_nViewWidth, k_nViewHeight);

	m_viewXY = m_frame(roiViewXY);
	m_viewYZ = m_frame(roiViewYZ);
	m_viewXZ = m_frame(roiViewXZ);

	m_bNewDataAvailable = false;
	m_bQuit = false;

	memset(m_ledPoints, 0, sizeof(m_ledPoints));
	memset(m_bLedsInitialized, 0, sizeof(m_bLedsInitialized));

	// Initialize bounds to something reasonable that will probably immediately get blown away
	m_flBoundsMinX = -1.0;
	m_flBoundsMaxX = 1.0;
	m_flBoundsMinY = -1.0;
	m_flBoundsMaxY = 1.0;
	m_flBoundsMinZ = -1.0;
	m_flBoundsMaxZ = 1.0;

	// Start the UI thread
	m_uiThread = std::thread(&CPointsVisualizer::UIThreadFunc, this);
}

CPointsVisualizer::~CPointsVisualizer()
{
	m_bQuit = true;
	if (m_uiThread.joinable())
	{
		m_uiThread.join();
	}
}

void CPointsVisualizer::OnDataUpdate(const std::unordered_map<int, std::array<double, 6>>& cameraFromWorldPoses, const double* const ledPoints, const bool* const pointsInitialized)
{
	std::unique_lock<std::mutex> lk(m_dataMutex);

	m_cameraFromWorldPoses = cameraFromWorldPoses;
	memcpy(m_ledPoints, ledPoints, sizeof(m_ledPoints));
	memcpy(m_bLedsInitialized, ledPoints, sizeof(m_bLedsInitialized));

	m_bNewDataAvailable = true;
	
	lk.unlock();
}

void CPointsVisualizer::UIThreadFunc()
{
	cv::namedWindow("Reconstruction", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO | cv::WINDOW_GUI_NORMAL);

	while (!m_bQuit)
	{
		// Poll for new data.
		if (m_bNewDataAvailable)
		{
			std::unique_lock<std::mutex> lk(m_dataMutex);
			Draw();
			lk.unlock();

			cv::imshow("Reconstruction", m_frame);
		}

		// Perform UI maintenance
		cv::waitKey(33);
	}
}

void CPointsVisualizer::Draw()
{
	// Start with a blank slate
	EraseImage();

	// Update bounding box used in subsequent functions
	UpdateBoundingBox();

	// Draw points & camera poses
	DrawTrackedPoints();
	DrawCameraPoses();
}

void CPointsVisualizer::UpdateBoundingBox()
{
	// Reset bounds to a reasonable minimum
	m_flBoundsMinX = -0.25;
	m_flBoundsMaxX = 0.25;

	m_flBoundsMinY = -0.25;
	m_flBoundsMaxY = 0.25;

	m_flBoundsMinZ = -0.25;
	m_flBoundsMaxZ = 0.25;

	// Scan through all camera poses
	for (const auto& kv : m_cameraFromWorldPoses)
	{
		const std::array<double, 6>& pose = kv.second;

		m_flBoundsMinX = MIN(pose[3], m_flBoundsMinX);
		m_flBoundsMaxX = MAX(pose[3], m_flBoundsMaxX);

		m_flBoundsMinY = MIN(pose[4], m_flBoundsMinY);
		m_flBoundsMaxY = MAX(pose[4], m_flBoundsMaxY);

		m_flBoundsMinZ = MIN(pose[5], m_flBoundsMinZ);
		m_flBoundsMaxZ = MAX(pose[5], m_flBoundsMaxZ);
	}

	// Scan through all points
	for (int i = 0; i < 500; i++)
	{
		if (!m_bLedsInitialized[i])
			continue;

		m_flBoundsMinX = MIN(m_ledPoints[i][0], m_flBoundsMinX);
		m_flBoundsMaxX = MAX(m_ledPoints[i][0], m_flBoundsMaxX);

		m_flBoundsMinY = MIN(m_ledPoints[i][1], m_flBoundsMinY);
		m_flBoundsMaxY = MAX(m_ledPoints[i][1], m_flBoundsMaxY);

		m_flBoundsMinZ = MIN(m_ledPoints[i][2], m_flBoundsMinZ);
		m_flBoundsMaxZ = MAX(m_ledPoints[i][2], m_flBoundsMaxZ);
	}

	// Equal aspect ratio: Range of all bounds should be equal to the largest range in any axis
	double flRangeX = m_flBoundsMaxX - m_flBoundsMinX;
	double flRangeY = m_flBoundsMaxY - m_flBoundsMinY;
	double flRangeZ = m_flBoundsMaxZ - m_flBoundsMinZ;

	double flMaxRange = MAX(flRangeX, MAX(flRangeY, flRangeZ));

	double flMidX = (m_flBoundsMinX + m_flBoundsMaxX) / 2;
	double flMidY = (m_flBoundsMinY + m_flBoundsMaxY) / 2;
	double flMidZ = (m_flBoundsMinZ + m_flBoundsMaxZ) / 2;

	m_flBoundsMinX = flMidX - (flMaxRange / 2 * (1.0 + k_flBorderFrac));
	m_flBoundsMaxX = flMidX + (flMaxRange / 2 * (1.0 + k_flBorderFrac));

	m_flBoundsMinY = flMidY - (flMaxRange / 2 * (1.0 + k_flBorderFrac));
	m_flBoundsMaxY = flMidY + (flMaxRange / 2 * (1.0 + k_flBorderFrac));

	m_flBoundsMinZ = flMidZ - (flMaxRange / 2 * (1.0 + k_flBorderFrac));
	m_flBoundsMaxZ = flMidZ + (flMaxRange / 2 * (1.0 + k_flBorderFrac));
}

void CPointsVisualizer::EraseImage()
{
	m_frame.setTo( cv::Scalar( 0, 0, 0) );
}

void CPointsVisualizer::DrawTrackedPoints()
{
	for (int i = 0; i < 500; i++)
	{
		if (!m_bLedsInitialized[i])
			continue;

		// Scale to bounding box
		double flDrawX = (m_ledPoints[i][0] - m_flBoundsMinX) / (m_flBoundsMaxX - m_flBoundsMinX);
		double flDrawY = (m_ledPoints[i][1] - m_flBoundsMinY) / (m_flBoundsMaxY - m_flBoundsMinY);
		double flDrawZ = (m_ledPoints[i][2] - m_flBoundsMinZ) / (m_flBoundsMaxZ - m_flBoundsMinZ);

		// draw XY
		cv::Point2d pXY(flDrawX * k_nViewWidth, flDrawY * k_nViewHeight);
		cv::circle(m_viewXY, pXY, 1.5, cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_AA);

		// draw YZ
		cv::Point2d pYZ(flDrawY * k_nViewWidth, flDrawZ * k_nViewHeight);
		cv::circle(m_viewYZ, pYZ, 1.5, cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_AA);

		// draw XZ
		cv::Point2d pXZ(flDrawX * k_nViewWidth, flDrawZ * k_nViewHeight);
		cv::circle(m_viewXZ, pXZ, 1.5, cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_AA);

		// TBD: Label IDs?  Might be too busy
	}
}

void CPointsVisualizer::DrawCameraPoses()
{
	for (const auto& kv : m_cameraFromWorldPoses)
	{
		const std::array<double, 6>& pose = kv.second;

		// Scale to bounding box
		double flDrawX = (pose[3] - m_flBoundsMinX) / (m_flBoundsMaxX - m_flBoundsMinX);
		double flDrawY = (pose[4] - m_flBoundsMinY) / (m_flBoundsMaxY - m_flBoundsMinY);
		double flDrawZ = (pose[5] - m_flBoundsMinZ) / (m_flBoundsMaxZ - m_flBoundsMinZ);

		// draw XY
		cv::Point2d pXY(flDrawX * k_nViewWidth, flDrawY * k_nViewHeight);
		cv::circle(m_viewXY, pXY, 1.5, cv::Scalar(255, 128, 128), cv::FILLED, cv::LINE_AA);

		// draw YZ
		cv::Point2d pYZ(flDrawY * k_nViewWidth, flDrawZ * k_nViewHeight);
		cv::circle(m_viewYZ, pYZ, 1.5, cv::Scalar(255, 128, 128), cv::FILLED, cv::LINE_AA);

		// draw XZ
		cv::Point2d pXZ(flDrawX * k_nViewWidth, flDrawZ * k_nViewHeight);
		cv::circle(m_viewXZ, pXZ, 1.5, cv::Scalar(255, 128, 128), cv::FILLED, cv::LINE_AA);
	}

	// TBD: Draw orientation
	//		Line? Axes? Frustum probably too busy.. there are tens of thousands of these
}
