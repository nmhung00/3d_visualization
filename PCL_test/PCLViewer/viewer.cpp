#include "viewer.h"

using namespace pcl;
using namespace std;

Viewer::Viewer() {
	boost::shared_ptr<pcl::visualization::PCLVisualizer> temp_viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer = temp_viewer;
	init();
};

Viewer::Viewer(string windowName) {
	boost::shared_ptr<pcl::visualization::PCLVisualizer> temp_viewer (new pcl::visualization::PCLVisualizer (windowName));
	viewer = temp_viewer;
	init();
};

void Viewer::init() {
	counter = 0;
	pointsColor[0] = 255;
	pointsColor[1] = 0;
	pointsColor[2] = 0;
	positionsColor[0] = 0;
	positionsColor[1] = 255;
	positionsColor[2] = 0;
	backgroundColor[0] = 0;
	backgroundColor[1] = 0;
	backgroundColor[2] = 0;
	viewer->addCoordinateSystem (1.0);
	autoTracking = false;
	lastPosition[0] = 0;
	lastPosition[1] = 0;
	lastPosition[2] = 0;
}

Viewer::~Viewer() {

};

void Viewer::setPointsDirectory(string pointsDirectory) {
	this->pointsFiles = read_directory(pointsDirectory);
}

void Viewer::setPositionsDirectory(string positionsDirectory) {
	this->positionsFiles = read_directory(positionsDirectory);
}	

void Viewer::setBackgroundColor(int r, int g, int b) {
	backgroundColor[0] = r;
	backgroundColor[1] = g;
	backgroundColor[2] = b;
}

void Viewer::setPointsColor(int r, int g, int b) {
	pointsColor[0] = r;
	pointsColor[1] = g;
	pointsColor[2] = b;
}

void Viewer::setPositionsColor(int r, int g, int b) {
	positionsColor[0] = r;
	positionsColor[1] = g;
	positionsColor[2] = b;
}

void Viewer::setCameraPosition(double pos_x, double pos_y, double pos_z, double view_x, double view_y, double view_z) {
	viewer->setCameraPosition(pos_x, pos_y, pos_z, view_x, view_y, view_z);
}

void Viewer::updatePoints(boost::shared_ptr<visualization::PCLVisualizer> PCLViewer) {	
	//~ cout << "Counter: " << counter << endl;
	
	ifstream tempFile;
	string tempLine;
    char *temp;
    double v[3];
    double cameraDirection[3];
	
	PointCloud<PointXYZRGB>::Ptr pc_ptr (new PointCloud<PointXYZRGB>);
    
    // Setting points color
	uint8_t point_r = pointsColor[0], point_g = pointsColor[1], point_b = pointsColor[2];
	uint32_t point_rgb = (static_cast<uint32_t>(point_r) << 16 | static_cast<uint32_t>(point_g) << 8 | static_cast<uint32_t>(point_b));
	
	// Setting position color
	uint8_t position_r = positionsColor[0], position_g = positionsColor[1], position_b = positionsColor[2];
	uint32_t position_rgb = (static_cast<uint32_t>(position_r) << 16 | static_cast<uint32_t>(position_g) << 8 | static_cast<uint32_t>(position_b));

	// Reading a position file
	tempFile.open(positionsFiles[counter].c_str());
	while(getline(tempFile, tempLine)) {
		temp = (char *) malloc ( (tempLine.size() + 1) * sizeof(char) );
		strcpy(temp, tempLine.c_str());
		v[0] = atof(strtok(temp, " ,"));
		v[1] = atof(strtok(NULL, " ,"));
		v[2] = atof(strtok(NULL, " ,"));
		PointXYZRGB p;
		p.rgb = *reinterpret_cast<float*>(&position_rgb);
		p.x = v[0];
		p.y = v[1];
		p.z = v[2];
		pc_ptr->points.push_back(p);
		
		// Setting camera's info
		if(autoTracking == true) {
			cameraDirection[0] = (-1) * (lastPosition[0] - v[0]);
			cameraDirection[1] = (-1) * (lastPosition[1] - v[1]);
			cameraDirection[2] = (-1) * (lastPosition[2] - v[2]);
			setCameraPosition(v[0] - 2, v[1] -2, v[2] -2, cameraDirection[0], cameraDirection[1], cameraDirection[2]);
		}
		
		free(temp);
	}
	tempFile.close();
			
	// Reading a points file
	tempFile.open(pointsFiles[counter].c_str());
	while(getline(tempFile, tempLine)) {
		temp = (char *) malloc ( (tempLine.size() + 1) * sizeof(char) );
		strcpy(temp, tempLine.c_str());
		v[0] = atof(strtok(temp, " ,"));
		v[1] = atof(strtok(NULL, " ,"));
		v[2] = atof(strtok(NULL, " ,"));
		PointXYZRGB p(pointsColor[0], pointsColor[1], pointsColor[2]);
		p.rgb = *reinterpret_cast<float*>(&point_rgb);
		p.x = v[0];
		p.y = v[1];
		p.z = v[2];
		pc_ptr->points.push_back(p);
		free(temp);
	}
	tempFile.close();
	
	stringstream ss;
	ss << counter;
	
    visualization::PointCloudColorHandlerRGBField<PointXYZRGB> colorHandler(pc_ptr);
    
    PCLViewer->addPointCloud<pcl::PointXYZRGB>(pc_ptr, colorHandler, ss.str());
	
	counter++;
	// Stop when all the files have been drawn
	//~ if(counter >= pointsFiles.size())
		//~ PCLViewer->spin();
}

void Viewer::addLine(Eigen::Vector3d _p1, Eigen::Vector3d _p2, int r, int g, int b) {
	PointXYZ p1;
	PointXYZ p2;
}

void Viewer::run() {
	viewer->setBackgroundColor(backgroundColor[0], backgroundColor[1], backgroundColor[2]);
	while (!viewer->wasStopped ()) {
		if(counter < pointsFiles.size())
			updatePoints(viewer);
		viewer->spinOnce(10);
	}
}
