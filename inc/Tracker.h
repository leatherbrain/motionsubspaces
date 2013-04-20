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
		void GetTrj(cv::Mat &t) const
		{
			// Check sizes
			if (t.cols != 1)
			{
				std::cerr << "Expected a column matrix" << std::endl;
				throw std::exception();
			}
			if (t.rows > (int)trj.size() * 2)
			{
				std::cerr << "Trajectory not long enough" << std::endl;
				throw std::exception();
			}

			// Copy coordinates
			std::vector<cv::Point2f>::const_reverse_iterator p = trj.rbegin();
			for (int i = t.rows - 1; i >= 0; i -= 2, p++)
			{
				t.at<double>(i, 0) = p->y;
				t.at<double>(i - 1, 0) = p->x;
			}
		}

	private:

		// Explicitly disallow an empty constructor
		Trajectory()
		{}
	};

	/// Trajectories of feature points
	std::vector<Trajectory> trjs;

	/// Settings
	static const int NUM_FEATURES_PER_FRAME = 160;
};

#endif /* TRACKER_H_ */
