#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP

//Includes
#include "../release/ui_SamplePlugin.h"
#include <QPushButton>

#include <opencv2/opencv.hpp>

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
	Q getdQ(Mat image);

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

	int _previousPoints[3][2]; //[point][0] X coordinate, [point][1] Y coordinate

	WorkCell::Ptr _wc;
	Device::Ptr _device;
	State _state;
	RenderImage *_textureRender, *_bgRender;
	GLFrameGrabber* _framegrabber;
};

#endif /*RINGONHOOKPLUGIN_HPP_*/
