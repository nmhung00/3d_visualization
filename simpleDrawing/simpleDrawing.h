#include <QGLViewer/qglviewer.h>
#include <QTimer>

class Viewer : public QGLViewer
{
protected :
  virtual void draw();
  virtual void init();
	
  virtual QString helpString() const;
public:
	QTimer *timer;
};
