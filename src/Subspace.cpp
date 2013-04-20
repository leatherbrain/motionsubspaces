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
#include <set>

//=============================================================================
/// Standard constructor
//=============================================================================
Subspace::Subspace(void) : proj(NULL), charvec(NULL)
{}

//=============================================================================
/// Standard destructor
//=============================================================================
Subspace::~Subspace(void)
{
	delete proj;
	delete charvec;
}

//=============================================================================
/// Extract projection matrix from a set of motion trajectories
//=============================================================================
void Subspace::Extract(const cv::Mat &trjs)
{
	// Parameter initialisation
	double errmin = std::numeric_limits<double>::max();
	const int P = trjs.cols;
	const int F = trjs.rows / 2;

	// Allocate projection matrix
	delete proj;
	proj = new TooN::Matrix<-1, -1, double>(2 * F, 2 * F);

	for (int iter = 0; iter < NUM_RANSAC_ITERATIONS; iter++)
	{
		// Pick three basis trajectories at random
		std::set<int> unique;
		while (unique.size() < 3)
			unique.insert(rand() % P);

		// Create basis trajectory matrix
		TooN::Matrix<-1, -1, double> W3(2 * F, 3);
		std::set<int>::const_iterator p = unique.begin();
		for (int c = 0; p != unique.end(); p++, c++)
			for (int r = 0; r < trjs.rows; r++)
				W3(r, c) = trjs.at<float>(r, *p);

		// Compute the projection matrix corresponding to these trajectories
		TooN::LU<> lu(W3.T() * W3);
		TooN::Matrix<-1, -1, double> proj_i(W3 * lu.get_inverse() * W3.T());

		// Compute the re-projection error
		double err = 0;
		for (int c = 0; c < trjs.cols; c++)
		{
			TooN::Vector<-1, double> t(trjs.rows);
			for (int r = 0; r < trjs.rows; r++)
				t[r] = trjs.at<float>(r, c);

			TooN::Vector<-1, double> t_err((proj_i * t) - t);
			double err_i = 0;
			for (int i = 0; i < trjs.rows; i++)
				err_i += t_err[i] * t_err[i];

			err += sqrt(err_i);
		}

		// Check for min error
		if (err < errmin)
		{
			errmin = err;
			*proj = proj_i;
		}
	}

	// Compute eigenvector of the projection matrix as the characteristic
	// trajectory
	TooN::SymEigen<TooN::Dynamic, double> eigM(*proj);
	delete charvec;
	charvec = new TooN::Vector<-1, double>(2 * F);
	*charvec = eigM.get_evectors()[0];
}
