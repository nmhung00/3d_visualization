#include "read_directory.h"

using namespace std;

vector<string> read_directory(string directory) {
	// Reading the directory of images
	DIR *dp;
	struct dirent *dirp;
	
	dp = opendir(directory.c_str());
	
	string filePath;
	vector<string> result;

	struct stat filestat;
	
	while(dirp = readdir(dp)) {
		filePath = directory + "/" + dirp->d_name;
		
		// Skip if that file is invalid or a directory
		if (stat( filePath.c_str(), &filestat ))
			continue;
		if (S_ISDIR( filestat.st_mode ))
			continue;
		
		// Add into list of filePaths
		result.push_back(filePath);	
	}
	
	sort(result.begin(), result.end());
	
	closedir(dp);
	
	return result;
}
