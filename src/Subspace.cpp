//===========================================================================
/// Subspace.cpp
/// 
/// \date	14 Apr 2013
/// \author	S. Sundaram
///
/// (copyright) Clueless Engineer
//===========================================================================

#include <Subspace.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include <TooN/TooN.h>
#include <TooN/LU.h>
#include <TooN/SymEigen.h>

//=============================================================================
/// Standard constructor
//=============================================================================
Subspace::Subspace(void)
{}

//=============================================================================
/// Standard destructor
//=============================================================================
Subspace::~Subspace(void)
{}

//=============================================================================
/// Extract projection matrix from a set of motion trajectories
//=============================================================================
void Subspace::Extract(const cv::Mat &trjs)
{
	// Run ransac
	int maxInliers = 0;
	const int P = trjs.cols;
	const int F = trjs.rows / 2;
	const int convergence = RANSAC_CONVERGENCE * P;
	for (int iter = 0; iter < NUM_RANSAC_ITERATIONS; iter++)
	{
		// Pick three basis trajectories at random
		std::set<int, bool> unique;
		while (unique.size() < 3)
			unique.insert(rand() % P);

		// Create basis trajectory matrix
		TooN::Matrix<-1, -1, double> W3(2 * F, 3);
		for (int c = 0; c < 3; c++)
		{
			const int tidx = unique[c].first;
			for (int r = 0; r < trjs.rows; r++)
				W3(r, c) = trjs.at<double>(r, tidx);
		}

		// Compute the projection matrix corresponding to these trajectories
		TooN::LU<> lu(W3->T() * W3);
		proj = W3 * lu.get_inverse() * W3->T();

		// Compute eigenvector of the projection matrix as the characteristic
		// trajectory
		TooN::SymEigen eigM(proj);
		charvec = eigM.get_evalues()[0];
	}
}
