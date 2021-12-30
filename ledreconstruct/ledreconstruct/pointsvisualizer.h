#pragma once

#include <array>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

#include <opencv2/highgui.hpp>


class CPointsVisualizer
{
public:
	CPointsVisualizer();
	~CPointsVisualizer();

	// New data callback
	void OnDataUpdate(const std::unordered_map< int, std::array<double, 6> >& cameraFromWorldPoses, const double* const ledPoints, const bool* const pointsInitialized);

	// UI thread
	void UIThreadFunc();

private:
	// Update m_frame
	void Draw();
	
	// Scan through dataset and update bounding box
	void UpdateBoundingBox();

	// Erase the drawn image
	void EraseImage();

	// Draw tracked LEDs in all views
	void DrawTrackedPoints();

	// Draw camera poses in all views
	void DrawCameraPoses();

	// UI image size
	static constexpr int k_nImageRows = 1080;
	static constexpr int k_nImageCols = 1080;

	static constexpr int k_nViewWidth = k_nImageCols / 2;
	static constexpr int k_nViewHeight = k_nImageRows / 2;

	// Fractional space to draw around outside of bounding box
	static constexpr double k_flBorderFrac = 0.05;

	// Image buffer for drawing
	cv::Mat m_frame;

	// third-angle-projection views
	cv::Mat m_viewXY;  // "top," in upper left
	cv::Mat m_viewYZ;  // "right," in lower right
	cv::Mat m_viewXZ;  // "front," in lower left

	// Mutex protecting LED and camera pose data
	std::mutex m_dataMutex;

	// Local copies of camera and point poses
	std::unordered_map< int, std::array<double, 6> > m_cameraFromWorldPoses;
	double m_ledPoints[500][3];
	bool m_bLedsInitialized[500];

	// new-data-available flag
	bool m_bNewDataAvailable;

	// XYZ bounding box
	double m_flBoundsMinX;
	double m_flBoundsMaxX;

	double m_flBoundsMinY;
	double m_flBoundsMaxY;

	double m_flBoundsMinZ;
	double m_flBoundsMaxZ;

	std::thread m_uiThread;

	// quit flag
	bool m_bQuit;
};

