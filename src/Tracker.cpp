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
		cv::goodFeaturesToTrack(frame, keypoints, NUM_FEATURES_PER_FRAME, 0.01,
				5.0);

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
						mask.at<unsigned char> (y, x) = 0;
		}

		else
		{
			trjs.at(idx).Kill();
			numKilled++;
		}
	}

	// Update keypoints
	keypoints = trackedPoints;

	// Get new features to track
	std::vector<cv::Point2f> newPoints;
	cv::goodFeaturesToTrack(frame, newPoints, NUM_FEATURES_PER_FRAME - numKilled, 0.01, 5.0, mask);

	// Create new trajectories with the new feature points
	std::vector<Trajectory>::iterator t = trjs.begin();
	int count = 0;
	for (; t != trjs.end() && !newPoints.empty(); t++, count++)
	{
		if (!t->IsAlive())
		{
			*t = Trajectory(newPoints.back());
			keypoints.at(count) = newPoints.back();
			newPoints.pop_back();
		}
	}

	// Update processed frame
	frame.copyTo(pFrame);

//	VisualiseTracking(keypoints, trackedPoints);
//	Visualise();
//	sleep(2);
}

//=============================================================================
/// Get trajectory matrix for n frames
//=============================================================================
bool Tracker::GetTrjMatrix(int n, cv::Mat &t)
{
	// Get all possible trajectories
	std::vector<Trajectory>::const_iterator trj = trjs.begin();
	std::vector<int> tidx;
	for (int i = 0; trj != trjs.end(); trj++, i++)
		if (trj->IsAlive() && trj->Length() > n)
			tidx.push_back(i);

	// Make sure we have enough valid trajectories
	if ((int)tidx.size() < MIN_TRACKED_POINTS)
	{
		std::cout << "Only found " << tidx.size() << " trajectories of length "
				<< n << std::endl;
		return false;
	}

	std::vector<int>::const_iterator idx = tidx.begin();
	t = cv::Mat(2 * n, tidx.size(), CV_32F);
	for (int i = 0; idx != tidx.end(); idx++, i++)
	{
		// Get trajectory
		const std::vector<cv::Point2f> &pt = trjs.at(*idx).GetTrj();

		// Copy
		std::vector<cv::Point2f>::const_reverse_iterator p = pt.rbegin();
		for (int f = 2 * n - 1; f >= 0; f -= 2, p++)
		{
			t.at<float>(f, i) = p->y;
			t.at<float>(f - 1, i) = p->x;
		}
	}

	return true;
}

//=============================================================================
/// Visualise all point trajectories
//=============================================================================
void Tracker::Visualise()
{
	// Create a copy of the processed frame
	cv::Mat img;
	pFrame.copyTo(img);
	cvtColor(img, img, CV_GRAY2BGR);

	// Go through all trajectories and mark the points
	std::vector<Trajectory>::const_iterator t = trjs.begin();
	for (; t != trjs.end(); t++)
	{
		std::vector<cv::Point2f>::const_reverse_iterator p = t->trj.rbegin();
		cv::circle(img, *p, 2, cv::Scalar(CV_RGB(255, 0, 0)), 1);
		p++;
		for (; p != t->trj.rend(); p++)
			cv::line(img, *(p - 1), *p, cv::Scalar(CV_RGB(0, 255, 0)), 1);
	}

	cv::imwrite("traj.png", img);
}

//=============================================================================
/// Visualise point tracking
//=============================================================================
void Tracker::VisualiseTracking(const std::vector<cv::Point2f> & f1,
								const std::vector<cv::Point2f> & f2)
{
	// Create a copy of the processed frame
	cv::Mat img;
	pFrame.copyTo(img);
	cvtColor(img, img, CV_GRAY2BGR);

	// Go through all pairs of points and draw
	std::vector<cv::Point2f>::const_iterator f1p, f2p;
	for (f1p = f1.begin(), f2p = f2.begin();
		 f1p != f1.end() && f2p != f2.end();
		 f1p++, f2p++)
	{
		cv::circle(img, *f1p, 2, cv::Scalar(CV_RGB(255, 0, 0)), 1);
		cv::circle(img, *f2p, 2, cv::Scalar(CV_RGB(0, 255, 0)), 1);
		cv::line(img, *f1p, *f2p, cv::Scalar(CV_RGB(0, 255, 0)), 1);
	}

	cv::imwrite("tracking.png", img);
}
