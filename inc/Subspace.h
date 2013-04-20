//===========================================================================
/// Subspace.h
/// 
/// \date	14 Apr 2013
/// \author	S. Sundaram
//===========================================================================

#include <sysdef.h>
#include <TooN/TooN.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>

#ifndef SUBSPACE_H_
#define SUBSPACE_H_

//=============================================================================
/// Class that defines a subspace based on a projection matrix
//=============================================================================
class Subspace
{

public:

	Subspace(void);
	~Subspace(void);

	// Extract motion subspace from a set of trajectories
	void Extract(const cv::Mat &trjs);

	// Accessor
	const TooN::Vector<-1, double> &GetCharVec() const
	{
		return *charvec;
	}

private:

	TooN::Matrix<-1, -1, double> *proj;
	TooN::Vector<-1, double> *charvec;

	static const int NUM_RANSAC_ITERATIONS = 16;
	static const double RANSAC_CONVERGENCE = 0.8;
};


#endif /* SUBSPACE_H_ */
