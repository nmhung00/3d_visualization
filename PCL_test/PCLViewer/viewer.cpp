#include "viewer.h"

//~ #include <boost/thread/thread.hpp>

bool stop = false;

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
	viewer->setBackgroundColor(backgroundColor[0], backgroundColor[1], backgroundColor[2]);
	viewer->registerKeyboardCallback (Viewer::keyboardEventOccurred, (void*)&viewer);
	viewer->registerMouseCallback (Viewer::mouseEventOccurred, (void*)&viewer);
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
	//~ if(stop)
		//~ return;
	
	ifstream tempFile;
	string tempLine;
    char *temp;
    double v[3];
    double cameraDirection[3];
	
	PointCloud<PointXYZRGB>::Ptr pc_ptr_position (new PointCloud<PointXYZRGB>);
	PointCloud<PointXYZRGB>::Ptr pc_ptr_points (new PointCloud<PointXYZRGB>);
    
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
		pc_ptr_position->points.push_back(p);
		
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
		pc_ptr_points->points.push_back(p);
		free(temp);
	}
	tempFile.close();
	
	stringstream ss;
	ss << counter;
	
    visualization::PointCloudColorHandlerRGBField<PointXYZRGB> colorHandlerPosition(pc_ptr_position);
    visualization::PointCloudColorHandlerRGBField<PointXYZRGB> colorHandlerPoints(pc_ptr_points);
    
    PCLViewer->addPointCloud<PointXYZRGB>(pc_ptr_position, colorHandlerPosition, "position" + ss.str());
    PCLViewer->addPointCloud<PointXYZRGB>(pc_ptr_points, colorHandlerPoints, "points" + ss.str());
	
	counter++;
	//~ PCLViewer->spinOnce(50);
	// Stop when all the files have been drawn
	//~ if(counter >= pointsFiles.size())
		//~ PCLViewer->spin();
}

bool Viewer::removePosition(int index) {
	stringstream ss;
	ss << index;
	string id = "position" + ss.str();
	return viewer->removePointCloud(id);
}

void Viewer::addLine(Eigen::Vector3d &_p1, Eigen::Vector3d &_p2, int r, int g, int b) {
	PointXYZ p1;
	PointXYZ p2;
	p1.x = _p1(0);
	p1.y = _p1(1);
	p1.z = _p1(2);
	p2.x = _p2(0);
	p2.y = _p2(1);
	p2.z = _p2(2);
	viewer->addLine<PointXYZ> (p1, p2, r, g, b, "line");
}

void Viewer::addPosition(Eigen::Vector3d position, int id) {
	PointCloud<PointXYZRGB>::Ptr pc_ptr_position (new PointCloud<PointXYZRGB>);
	
	// Setting position color
	uint8_t position_r = positionsColor[0], position_g = positionsColor[1], position_b = positionsColor[2];
	uint32_t position_rgb = (static_cast<uint32_t>(position_r) << 16 | static_cast<uint32_t>(position_g) << 8 | static_cast<uint32_t>(position_b));
	
	PointXYZRGB p;
	p.rgb = *reinterpret_cast<float*>(&position_rgb);
	p.x = position(0);
	p.y = position(1);
	p.z = position(2);
	pc_ptr_position->points.push_back(p);

	stringstream ss;
	ss << id;
	
	visualization::PointCloudColorHandlerRGBField<PointXYZRGB> colorHandlerPosition(pc_ptr_position);
    
    viewer->addPointCloud<PointXYZRGB>(pc_ptr_position, colorHandlerPosition, "position" + ss.str());
}

void Viewer::addPoints(list<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > points, int id) {
	PointCloud<PointXYZRGB>::Ptr pc_ptr_points (new PointCloud<PointXYZRGB>);
    
    // Setting points color
	uint8_t point_r = pointsColor[0], point_g = pointsColor[1], point_b = pointsColor[2];
	uint32_t point_rgb = (static_cast<uint32_t>(point_r) << 16 | static_cast<uint32_t>(point_g) << 8 | static_cast<uint32_t>(point_b));

	list<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >::iterator it;
	for( it = points.begin(); it != points.end(); it++) {
		PointXYZRGB p;
		p.rgb = *reinterpret_cast<float*>(&point_rgb);
		p.x = (*it)(0);
		p.y = (*it)(1);
		p.z = (*it)(2);
		pc_ptr_points->points.push_back(p);

	}
	
	stringstream ss;
	ss << id;
	
	visualization::PointCloudColorHandlerRGBField<PointXYZRGB> colorHandlerPoints(pc_ptr_points);

	viewer->addPointCloud<PointXYZRGB>(pc_ptr_points, colorHandlerPoints, "points" + ss.str());
}

bool Viewer::wasStopped() {
	return viewer->wasStopped();
}

void Viewer::finish() {
	viewer->spin();
}

void Viewer::refresh(int time) {
	viewer->spinOnce(time);
}

int Viewer::getNumberOfFiles() {
	return pointsFiles.size();
}

void Viewer::run() {
	
	//~ viewer->setBackgroundColor(backgroundColor[0], backgroundColor[1], backgroundColor[2]);
	//~ viewer->registerKeyboardCallback (Viewer::keyboardEventOccurred, (void*)&viewer);
	//~ viewer->registerMouseCallback (Viewer::mouseEventOccurred, (void*)&viewer);
	
	Eigen::Vector3d v1(0, 0, 0);
	Eigen::Vector3d v2(0, 2, 2);
	addLine(v1, v2, 0, 0, 255);
	
	//~ Eigen::Vector3d v3(1, 0, 1);
	//~ Eigen::Vector3d v4(0, 0, 1);
	//~ Eigen::Vector3d v5(0, 1, 0);
	//~ list<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > p;
	//~ p.push_front(v3);
	//~ p.push_front(v4);
	//~ p.push_front(v5);
	//~ addPoints(p, -1);
	
	
	
	while (!viewer->wasStopped ()) {
		if( (counter < pointsFiles.size()) &&  (!stop) ) {
			updatePoints(viewer);
			removePosition(counter - 100);
		}
		viewer->spinOnce(50);
	}
}

void Viewer::keyboardEventOccurred (const visualization::KeyboardEvent &event, void* viewer_void) {
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
	if (event.getKeySym() == "space" && event.keyDown()) {
		cout << "space was pressed. Testing testing" << endl;
		if (stop == false) {
			stop = true;
			cout << "stop true\n";
		}
		else {
			stop = false;
			cout << "stop false\n";
		}
	}
	
}

void Viewer::mouseEventOccurred (const visualization::MouseEvent &event, void* viewer_void) {
//~ void mouseEventOccurred (const visualization::MouseEvent &event, void* viewer_void) {
	boost::shared_ptr<visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<visualization::PCLVisualizer> *> (viewer_void);
	if (event.getButton() == visualization::MouseEvent::LeftButton &&
		event.getType() == visualization::MouseEvent::MouseButtonPress) {
		cout << "Left mouse pressed\n";
	}
}
