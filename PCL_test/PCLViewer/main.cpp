#include "viewer.h"

#include <iostream>

extern bool stop;

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
	
	Viewer testViewer("Test Window");
	
	testViewer.setPositionsDirectory(positionsDirectory);
	testViewer.setPointsDirectory(pointsDirectory);

	testViewer.setBackgroundColor(0, 0, 0);
	
	testViewer.autoTracking = false;
	
	
	Eigen::Vector3d v1(0, 0, 0);
	Eigen::Vector3d v2(0, 2, 2);
	testViewer.addLine(v1, v2, 0, 0, 255);
	
	Eigen::Vector3d v3(1, 0, 0);
	Eigen::Vector3d v4(2, 0, 0);
	Eigen::Vector3d v5(3, 0, 0);
	list<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > p;
	for(double i = 0; i < 10; i+= 0.1) {
		Eigen::Vector3d v(i, 0, 0);
		p.push_front(v);
	}
	p.push_front(v3);
	p.push_front(v4);
	p.push_front(v5);
	testViewer.addPoints(p, -1);
	
	int counter = 0;
	while(!testViewer.wasStopped()) {
		if( (counter < testViewer.getNumberOfFiles()) && (!stop) ) {
			testViewer.updatePoints(testViewer.getViewer());
			testViewer.removePosition(counter - 100);
			counter++;
		}
		testViewer.refresh(50);
		if(counter == testViewer.getNumberOfFiles())
			testViewer.finish();
	}
	
	return 0;
}
