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
#include <opencv2/video/tracking.hpp>

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
		cv::goodFeaturesToTrack(frame, keypoints, NUM_FEATURES_PER_FRAME, 0.01, 5.0);

		// Add all features to new trajectories
		std::vector<cv::Point2f>::const_iterator p = keypoints.begin();
		for (; p != keypoints.end(); p++)
			trjs.push_back(Trajectory(*p));

		// Update processed frame
		frame.copyTo(pFrame);

		return;
	}

	// Track feature points using optical flow
	std::vector<cv::Point2f> trackedPoints;
	std::vector<unsigned char> status;
	std::vector<float> err;
	cv::calcOpticalFlowPyrLK(pFrame, frame, keypoints, trackedPoints, status, err);

	// Mask out tracked points to calculate new features to track
	cv::Mat mask = cv::Mat::ones(frame.rows, frame.cols, CV_8UC1);

	// Update/kill trajectories
	std::vector<cv::Point2f>::const_iterator p = trackedPoints.begin();
	std::vector<unsigned char>::const_iterator s = status.begin();
	int numKilled = 0;
	for (int idx = 0; p != trackedPoints.end(); p++, s++, idx++)
	{
		if (*s)
		{
			trjs.at(idx).Update(*p);
			for (int y = p->y - 1; y <= p->y + 1; y++)
				for (int x = p->x - 1; x <= p->x + 1; x++)
					if (y > 0 && y < frame.rows && x > 0 && x < frame.cols)
						mask.at<unsigned char>(y, x) = 0;
		}
		else
		{
			trjs.at(idx).Kill();
			numKilled++;
		}
	}

	// Get new features to track
	std::vector<cv::Point2f> newPoints;
	cv::goodFeaturesToTrack(frame, newPoints, NUM_FEATURES_PER_FRAME - numKilled, 0.01, 5.0, mask);

	// Create new trajectories with the new feature points
	std::vector<Trajectory>::iterator t = trjs.begin();
	for (; t != trjs.end() && !newPoints.empty(); t++)
	{
		if (!t->IsAlive())
		{
			*t = Trajectory(newPoints.back());
			newPoints.pop_back();
		}
	}
}
