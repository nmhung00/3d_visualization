#include "viewer.h"

#include <iostream>

int main(int argv, char **argc) {
	cout << "Usage: ./viewer [positionsDirectory] [pointsDirectory]\n";
	cout << "Default positionsDirectory is \"positions\" and Default pointsDirectory is \"points\"";
	
	string positionsDirectory;
	string pointsDirectory;
	if(argv != 1) {
		positionsDirectory = argc[1];
		pointsDirectory = argc[2];
	}
	else {
		positionsDirectory = "positions";
		pointsDirectory = "points";
	}
	//~ cout << "positionsDirectory: " << positionsDirectory << endl;
	//~ cout << "pointsDirectory: " << pointsDirectory << endl;
	
	Viewer testViewer("Test Window");
	
	testViewer.setPositionsDirectory(positionsDirectory);
	testViewer.setPointsDirectory(pointsDirectory);
	
	testViewer.run();
	return 0;
}
