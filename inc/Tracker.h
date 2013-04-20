//===========================================================================
/// Tracker.h
/// 
/// \date	14 Apr 2013
/// \author	S. Sundaram
//===========================================================================

#include <sysdef.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <exception>

#ifndef TRACKER_H_
#define TRACKER_H_

//=============================================================================
/// Class to track feature points in 2D
//=============================================================================
class Tracker
{

public:

	Tracker();
	~Tracker();

	// Track features in given frame using sparse optical flow
	void Update(const cv::Mat &frame);

	// Get trajectory matrix for a specified number of frames
	bool GetTrjMatrix(int n, cv::Mat &t);

	// Visualise trajectories
	void Visualise();

private:

	/// Processed frame
	cv::Mat pFrame;

	/// Recently detected feature points
	std::vector<cv::Point2f> keypoints;

	/// Structure that stores trajectories
	struct Trajectory
	{
		// Status (tracking / dead)
		bool alive;

		// Feature trajectory
		std::vector<cv::Point2f> trj;

		// Construct new trajectory
		Trajectory(const cv::Point2f &point)
		{
			alive = true;
			trj.push_back(point);
		}

		// Update trajectory with tracked feature point
		void Update(const cv::Point2f &point)
		{
			trj.push_back(point);
		}

		// Kill trajectory because tracking is lost
		void Kill()
		{
			alive = false;
		}

		// Accessors
		bool IsAlive() const
		{
			return alive;
		}

		int Length() const
		{
			return trj.size();
		}

		// Convert trajectory to Mat
		const std::vector<cv::Point2f> &GetTrj() const
		{
			return trj;
		}

	private:

		// Explicitly disallow an empty constructor
		Trajectory()
		{}
	};

	// Visualise tracked keypoints
	void VisualiseTracking(const std::vector<cv::Point2f> &f1,
						   const std::vector<cv::Point2f> &f2);

	/// Trajectories of feature points
	std::vector<Trajectory> trjs;

	/// Settings
	static const int NUM_FEATURES_PER_FRAME = 160;
	static const int MIN_TRACKED_POINTS = 40;
};

#endif /* TRACKER_H_ */
