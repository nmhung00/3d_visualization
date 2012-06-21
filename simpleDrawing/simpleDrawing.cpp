#include <cstdlib>
#include <iostream>
#include <fstream>
#include <string.h>
#include <vector>
#include <unistd.h>
#include <QKeyEvent>
#include "simpleDrawing.h"
#include "read_directory.h"

using namespace std;
using namespace qglviewer;

vector<vector<float> *> points; // points[i] includes position and points of a file

unsigned int counter = 0; // Count number of files which have been drawn

int refresh_rate = 5; // Number of mili second between each update (The screen is also updated when you use the mouse)

void Viewer::draw() {

	vector<float> *temp;
	glBegin(GL_POINTS);
	
	for(unsigned int i = 0; i <= counter; i++) {
		temp = points[i];
		// Draw the position
		glColor3f(0.0f, 0.0f, 1.0f); // Coloring position
		glVertex3f( (*temp)[0], (*temp)[1], (*temp)[2] );
		
		// Draw the points
		glColor3f(0.0f, 1.0f, 0.0f); // Coloring points
		for(unsigned int k = 1; k < temp->size()/3; k++)
			glVertex3f( (*temp)[3 * k], (*temp)[3 * k + 1], (*temp)[3 * k + 2] );
	}
	
	glEnd();
	
	setSceneRadius(50);
	setSceneCenter(Vec (0, 0, 0.6 + counter/100) );
	camera()->setViewDirection(Vec(-1, 0, 0));
	camera()->showEntireScene();
	//~ camera()->setPosition(Vec(1, 0, counter/100));
	//~ camera()->setViewDirection(Vec(0, 0, 1));
	
	if(counter < (points.size() - 1))
		counter++;
		
	if(counter == (points.size() - 1) )
		timer->stop(); // Stop timer when all the files have been drawn
	

}

void Viewer::init() {
    // Restore previous viewer state.
	restoreStateFromFile();
  
	// Opens help window
	help();
	
	//~ setSceneRadius(100);
	//~ setSceneCenter(qglviewer::Vec (0, 0, 0) );
	//~ camera()->showEntireScene();
	
	// Directories of points and positions
	string pointsDir = "points";
	string positionsDir = "positions";
	
	vector<string> pointsFiles = read_directory(pointsDir);
	vector<string> positionsFiles = read_directory(positionsDir);
	
	ifstream tempFile;
	int len;
	
	string tempLine;
    char *temp;
    float v[3];
	
	for(unsigned int i = 0; i < pointsFiles.size(); i++) {
		points.push_back(new vector<float>);
		len = points.size();
		
		tempFile.open(positionsFiles[i].c_str());
		
		// Reading a position file
		while(getline(tempFile, tempLine)) {
			temp = (char *) malloc ( (tempLine.size() + 1) * sizeof(char) );
			strcpy(temp, tempLine.c_str());
			v[0] = atof(strtok(temp, " "));
			v[1] = atof(strtok(NULL, " "));
			v[2] = atof(strtok(NULL, " "));
			points[len - 1]->push_back((v[0]));
			points[len - 1]->push_back((v[1]));
			points[len - 1]->push_back((v[2]));
			free(temp);
		}
		
		tempFile.close();
		
		tempFile.open(pointsFiles[i].c_str());
		
		// Reading a points file
		while(getline(tempFile, tempLine)) {
			temp = (char *) malloc ( (tempLine.size() + 1) * sizeof(char) );
			strcpy(temp, tempLine.c_str());
			v[0] = atof(strtok(temp, " "));
			v[1] = atof(strtok(NULL, " "));
			v[2] = atof(strtok(NULL, " "));
			points[len - 1]->push_back((v[0]));
			points[len - 1]->push_back((v[1]));
			points[len - 1]->push_back((v[2]));
			free(temp);
		}
		
		tempFile.close();
		
	}
	
	// Create a timer
	timer = new QTimer(this);
	// Everytime timeout() is called, updateGL() is also called. When updateGL() is called, the display will be redrawn.
	connect(timer, SIGNAL(timeout()), SLOT(updateGL()));
	// Start the timer with refresh rate as defined
	timer->start(refresh_rate);
}

QString Viewer::helpString() const {
	QString text("<h2>S i m p l e D r a w i n g</h2>");
	text += "Use the mouse to move the camera around the object. ";
	text += "You can respectively revolve around, zoom and translate with the three mouse buttons. ";
	text += "Left and middle buttons pressed together rotate around the camera view direction axis<br><br>";
	text += "Pressing <b>Alt</b> and one of the function keys (<b>F1</b>..<b>F12</b>) defines a camera keyFrame. ";
	text += "Simply press the function key again to restore it. Several keyFrames define a ";
	text += "camera path. Paths are saved when you quit the application and restored at next start.<br><br>";
	text += "Press <b>F</b> to display the frame rate, <b>A</b> for the world axis, ";
	text += "<b>Alt+Return</b> for full screen mode and <b>Control+S</b> to save a snapshot. ";
	text += "See the <b>Keyboard</b> tab in this window for a complete shortcut list.<br><br>";
	text += "Double clicks automates single click actions: A left button double click aligns the closer axis with the camera (if close enough). ";
	text += "A middle button double click fits the zoom of the camera and the right button re-centers the scene.<br><br>";
	text += "A left button double click while holding right button pressed defines the camera <i>Revolve Around Point</i>. ";
	text += "See the <b>Mouse</b> tab and the documentation web pages for details.<br><br>";
	text += "Press <b>Escape</b> to exit the viewer.";
	return text;
}
