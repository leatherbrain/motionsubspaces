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

int main(int argc, char **argv)
{
	// Check input arguments
	if (argc < 2)
	{
		std::cout << "Specify directory of action sequence as argument" << std::endl;
		return -1;
	}

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
		cv::Mat frame = cv::imread(pathToFile.c_str(), CV_LOAD_IMAGE_GRAYSCALE);

		// Update tracker
		t.Update(frame);
	}

	// Extract motion subspace
	Subspace s;
	cv::Mat trajMat;
	t.GetTrjMatrix(30, trajMat);
	s.Extract(trajMat);
	std::cout << "Eigenvector: " << s.GetCharVec() << std::endl;

	closedir(dp);
}
