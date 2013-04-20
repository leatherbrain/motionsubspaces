//===========================================================================
/// Undistort.h
/// 
/// \date	20 Apr 2013
/// \author	S. Sundaram
///
/// (copyright) Clueless Engineer
//===========================================================================

#include <opencv/cxcore.h>
#include <opencv/highgui.h>

#ifndef UNDISTORT_H_
#define UNDISTORT_H_

//=============================================================================
/// Undistortion
//=============================================================================
class Undistort
{
public:

	Undistort()
	{}

	~Undistort()
	{}

	Undistort(const cv::Mat &_cam, const cv::Mat &_k)
	{
		SetParams(_cam, _k);
	}

	/// Set intrinsic camera matrix and distortion parameters
	void SetParams(const cv::Mat &_cam, const cv::Mat &_k)
	{
		_cam.copyTo(cam);
		_k.copyTo(k);
	}

	/// Undistort
	void Run(const cv::Mat &in, cv::Mat &out) const
	{
		cv::undistort(in, out, cam, k);
	}

private:

	/// Intrinsic camera matrix
	cv::Mat cam;
	cv::Mat k;
};

#endif /* UNDISTORT_H_ */
