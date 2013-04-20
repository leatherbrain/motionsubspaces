//===========================================================================
/// AnalyseSingleVideo.cpp
/// 
/// \date	20 Apr 2013
/// \author	S. Sundaram
//===========================================================================

#include <sysdef.h>
#include <Subspace.h>
#include <Tracker.h>
#include <dirent.h>
#include <string>
#include <set>
#include <Undistort.h>

int main(int argc, char **argv)
{
	// Check input arguments
	if (argc < 2)
	{
		std::cout << "Specify directory of action sequence as argument" << std::endl;
		return -1;
	}

	// Undistortion parameters for GoPro camera in Kitani's data
	cv::Mat intrinsics(3, 3, CV_64F);
    intrinsics.at<double>(0,0) = 423.67758179;
    intrinsics.at<double>(0,1) = 0.0;
    intrinsics.at<double>(0,2) = 412.36691284;
    intrinsics.at<double>(1,0) = 0.0;
    intrinsics.at<double>(1,1) = 429.06423950;
    intrinsics.at<double>(1,2) = 323.02185059;
    intrinsics.at<double>(2,0) = 0.0;
    intrinsics.at<double>(2,1) = 0.0;
    intrinsics.at<double>(2,2) = 1.0;

	cv::Mat distortion(5, 1, CV_64F);
    distortion.at<double>(0,0) =  -0.25010384;
    distortion.at<double>(0,1) =  0.05971173;
    distortion.at<double>(0,2) =  -7.23865523e-004;
    distortion.at<double>(0,3) =  -3.96828807e-004;
    distortion.at<double>(0,4) =  0.0;

	Undistort goPro(intrinsics, distortion);

	// Open directory and get list of files
	std::string path(argv[1]);
	DIR *dp;
	struct dirent *dirp;
	if((dp  = opendir(path.c_str())) == NULL)
	{
		std::cout << "Sequence directory not found" << std::endl;
		return -1;
	}

	// List files and store them in a set, so that they are sorted
	std::set<std::string> filenames;
	while ((dirp = readdir(dp)) != NULL)
	{
		std::string name(dirp->d_name);
		if (std::string::npos != name.find(".png"))
			filenames.insert(name);
	}

	// Process all images
	Tracker t;
	std::set<std::string>::const_iterator name = filenames.begin();
	for (; name != filenames.end(); name++)
	{
		std::string pathToFile = path + "/" + *name;
		std::cout << "Processing " << pathToFile << std::endl;

		// Read file
		cv::Mat raw = cv::imread(pathToFile.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
		cv::Mat frame(840, 480, raw.type());
		cv::resize(raw, frame, cv::Size(frame.rows, frame.cols));
//		raw(cv::Range(0, 480), cv::Range(4, 844)).copyTo(frame);
		cv::Mat uframe(frame.rows, frame.cols, frame.type());

		// Update tracker
		goPro.Run(frame, uframe);
		t.Update(uframe);

//		cv::imwrite("undistorted.png", uframe);
//		sleep(5);
	}

	// Extract motion subspace
	Subspace s;
	cv::Mat trajMat;
	t.GetTrjMatrix(30, trajMat);
	s.Extract(trajMat);
	std::cout << "Eigenvector: " << s.GetCharVec() << std::endl;

	closedir(dp);
}
