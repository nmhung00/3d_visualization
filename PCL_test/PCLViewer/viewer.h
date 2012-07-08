#ifndef VIEWER_H
#define VIEWER_H

#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <vector>
#include <list>
#include <Eigen/Dense>

#include "read_directory.h"

using namespace pcl;
using namespace std;

class Viewer {
public:

	//! Default constructor
	/**
	 * Create a viewer where default name of displaying window is Viewer
	 */
	Viewer();

	//! Constructor with window name
	/**
	 * @param windowName
	 * 	Name of the displaying window
	 */
	Viewer(string windowName);

	//! Destructor
	~Viewer();

	//! Initialize counter = 0, pointsColor = (255, 0, 0), positionsColor (0, 255, 0), background color = (0, 0, 0)
	void init();

	//! Set the path to points directory
	/**
	 * @param pointsDirectory
	 * 	Path to the directory of points
	 */
	void setPointsDirectory(string pointsDirectory);

	//! Set the path to positions directory
	/**
	 * @param positionsDirectory
	 * 	Path to directory of positions
	 */
	void setPositionsDirectory(string positionsDirectory);
	
	//! Updating points from files which are going to be drawn
	void updatePoints(boost::shared_ptr<visualization::PCLVisualizer> PCLViewer);
	
	//! Setting camera position
	/**
	 * @param pos_x
	 * 	x coordinate of the camera location
	 * @param pos_y
	 * 	y coordinate of the camera location	 
	 * @param pos_z
	 * 	z coordinate of the camera location
	 * @param view_x
	 * 	x component of viewing direction of the camera
	 * @param view_y
	 * 	y component of viewing direction of the camera
	 * @param view_z
	 * 	z component of viewing direction of the camera
	 */
	void setCameraPosition(double pos_x, double pos_y, double pos_z, double view_x, double view_y, double view_z);

	//! Setting background color of the viewer
	/**
	 * @param r
	 * 	red component
	 * @param g
	 * 	green component
	 * @param b
	 * 	blue component
	 */
	void setBackgroundColor(int r, int g, int b);

	//! Setting color for points
	/**
	 * @param r
	 * 	red component
	 * @param g
	 * 	green component
	 * @param b
	 * 	blue component
	 */
	void setPointsColor(int r, int g, int b);

	//! Setting color for position
	/**
	 * @param r
	 * 	red component
	 * @param g
	 * 	green component
	 * @param b
	 * 	blue component
	 */
	void setPositionsColor(int r, int g, int b);
	
	//! Displaying points on the screen
	void run();
	
	//! Remove the position of the robot by index
	/**
	 * @param index
	 * 	index of the position that is going to be removed
	 * @return
	 * 	Return true if the position is successfully removed and false if it fails to remove the position
	 */
	bool removePosition(int index);
	
	//! Add a line into the viewer
	/**
	 * @param p1
	 * 	One end of the line
	 * @param p2
	 * 	The other end of the line
	 * @param r
	 * 	red component of the line's color
	 * @param g
	 * 	green component of the line's color
	 * @param b
	 * 	blue component of the line's color
	 */
	void addLine(Eigen::Vector3d &p1, Eigen::Vector3d &p2, int r, int g, int b);
	
	//! Add points into the viewer
	/**
	 * @param points
	 * 	List of points
	 * @param id
	 * 	Id of the adding points
	 */
	void addPoints(list<Eigen::Vector3d> points, int id);
	
	//! Add a position into the viewer
	/**
	 * @param position
	 * 	The coordinate of the position
	 * @param id
	 * 	Id of the position
	 */
	void addPosition(Eigen::Vector3d position, int id);

	bool autoTracking; /*! Let the program change camera's positions and directions automatically */
private:
	boost::shared_ptr<visualization::PCLVisualizer> viewer; /*! The viewer which displays point clouds*/
	vector<string> pointsFiles; /*! List of names of files of points */
	vector<string> positionsFiles; /*! List of name of files of positions */
	int counter; /*! Count number of files have been read */
	int pointsColor[3]; /*! Color of points, stored in format [red, green, blue] */
	int positionsColor[3]; /*! Color of positions, stored in format [red, green, blue] */
	int backgroundColor[3]; /*! Color of background, stored in format [red, green, blue] */
	double lastPosition[3]; /*! Last position which has been drawn. It is used for camera to track automatically */
};
#endif
