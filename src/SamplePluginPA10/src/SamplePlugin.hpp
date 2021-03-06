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
#include <iostream>

//Defines
#define NUMBER_OF_POINTS 1 //1, 2 or 3
#define TRACKING 4 //0=Marker's frame, 1=Color, 2=LinesA, 3=LinesB, 4=Corny
#define USER 0 //Jorge = 0, Lukash = 1
#define SPEED 2	 //0=Slow, 1=Medium, 2=Fast

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
	void calculateDeltaT();
	void checkVelocityLimits(Q & dqToCheck);
	void calculateSpeed(const Q & dQ);

	//Detection
	void calculateAverageTime();
	Point2f cornyDetection(Mat & img_input);
	Point2f colorDetection(Mat & img_input);
	Point2f linesHDetection(Mat & img_input);
	float pointDistance(Point2f & p, Point2f & q);
	bool isPossible(Vec2f & line_1, Vec2f & line_2, const float minAngle);
	Point2f calculateLineCrossing(Vec2f & line_1, Vec2f & line_2);
	vector<Point2f> pointExtractFromLine(Vec2f & line);

	//LaTeX
	void latexPlotQ(ofstream & file, const string & name);
	void latexPlotSpeed(ofstream & file, const string & name);
	void latexPlotCameraPose(ofstream & file, const string & name);
	void latexPlotError(ofstream & file, const string & name);

private slots:
	void buttonPressed();
	void loop();
	void stateChangedListener(const rw::kinematics::State& state);

private:
	ifstream _motionFile;

	vector<Transform3D<> > _markerPoseVec;
	vector<Transform3D<> > _cameraPoseVec;
	vector<Vector3D<> > _errorPoseVec;
	vector<Q> _qRobotVec;
	vector<Q> _speedVec;
	vector<float> _featureExtractionTime;
	vector<float> _deltaT;
	vector<float> _sumDeltaT;

	QTimer* _loop;

	float _previousPoints[3][2]; //[0]X coordinate, [1] Y coordinate
	float _previousdUdV[3][2];
	Q _previousdQ;
	bool _firstTime;

	WorkCell::Ptr _wc;
	Device::Ptr _device;
	State _state;
	RenderImage *_textureRender, *_bgRender;
	GLFrameGrabber* _framegrabber;

};

#endif /*RINGONHOOKPLUGIN_HPP_*/
