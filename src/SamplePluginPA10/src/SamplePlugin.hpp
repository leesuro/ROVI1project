#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP

//Includes
#include "../release/ui_SamplePlugin.h"
#include <QPushButton>

#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"

#include <rw/kinematics/State.hpp>
#include <rw/math/Q.hpp>
#include <rw/loaders/ImageLoader.hpp>
#include <rw/loaders/WorkCellFactory.hpp>
#include <rw/kinematics/MovableFrame.hpp>
#include <rws/RobWorkStudio.hpp>
#include <rws/RobWorkStudioPlugin.hpp>
#include <rwlibs/opengl/RenderImage.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>

#include <fstream>
#include <time.h>
#include <stdio.h>
#include <iostream>
#include <sys/time.h>

//Namespaces
using namespace rw::common;
using namespace rw::math;
using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::sensor;
using namespace rwlibs::opengl;
using namespace rwlibs::simulation;
using namespace rws;

using namespace cv;

using namespace Eigen;

using namespace std;

class SamplePlugin: public RobWorkStudioPlugin, private Ui::SamplePlugin
{
Q_OBJECT
Q_INTERFACES( rws::RobWorkStudioPlugin )
public:
	//Constructors and destructors
	SamplePlugin();
	virtual ~SamplePlugin();

	//Plugin Methods
	virtual void initialize();
	virtual void open(WorkCell* workcell);
	virtual void close();

	//Methods
	Mat getImageAndShow();
	void writeData();
	void computeError();
	Q getdQ(Mat & image);

	//Detection
	Point2f corny(Mat img_input);
	Point2f color(Mat img_input);
	Point2f linesH(Mat img_input);
	float pointDistance(Point2f p, Point2f q);
	bool acceptLinePair(Vec2f line1, Vec2f line2, float minTheta);
	Point2f computeIntersect(Vec2f line1, Vec2f line2)
	vector<Point2f> pointExtractFromLine(Vec2f line);

private slots:
	void buttonPressed();
	void loop();

	void stateChangedListener(const rw::kinematics::State& state);

private:
	ifstream _motionFile;
	ofstream _cameraPoseFile;
	ofstream _errorPoseFile;
	ofstream _qRobotFile;

	vector<Transform3D<> > _markerPoseVec;
	vector<Transform3D<> > _cameraPoseVec;
	vector<Transform3D<> > _errorPoseVec;
	vector<Q> _qRobotVec;

	QTimer* _loop;

	float _previousPoints[3][2]; //[point][0] X coordinate, [point][1] Y coordinate
	bool _firstTime;

	WorkCell::Ptr _wc;
	Device::Ptr _device;
	State _state;
	RenderImage *_textureRender, *_bgRender;
	GLFrameGrabber* _framegrabber;
};

#endif /*RINGONHOOKPLUGIN_HPP_*/
