//===========================================================================
/// Tracker.cpp
/// 
/// \date	18 Apr 2013
/// \author	S. Sundaram
//===========================================================================

#include <Tracker.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

//=============================================================================
/// Standard constructor
//=============================================================================
Tracker::Tracker(void)
{}

//=============================================================================
/// Standard destructor
//=============================================================================
Tracker::~Tracker(void)
{}

//=============================================================================
/// Update with new frame
//=============================================================================
void Tracker::Update(const cv::Mat &frame)
{
	// Check if this is the first frame
	if (pFrame.rows == 0 || pFrame.cols == 0)
	{
		// Initialise good features to track
		std::vector<cv::Point2f> points;
		cv::goodFeaturesToTrack(frame, points, NUM_FEATURES_PER_FRAME, 0.01, 5.0);

		// Add all features to new trajectories
		std::vector<cv::Point2f>::const_iterator p = points.begin();
		for (; p != points.end(); p++)
			trjs.push_back(Trajectory(*p));
	}
}
