#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP

#include "../release/ui_SamplePlugin.h"

#include <fstream>

#include <opencv2/opencv.hpp>

#include <rws/RobWorkStudioPlugin.hpp>

#include <rw/kinematics/State.hpp>
#include <rw/math/Q.hpp>
#include <rwlibs/opengl/RenderImage.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>

class SamplePlugin: public rws::RobWorkStudioPlugin, private Ui::SamplePlugin
{
Q_OBJECT
Q_INTERFACES( rws::RobWorkStudioPlugin )
public:
	//Constructors and destructors
	SamplePlugin();
	virtual ~SamplePlugin();

	//Plugin Methods
	virtual void initialize();
	virtual void open(rw::models::WorkCell* workcell);
	virtual void close();

	//Methods
	cv::Mat getImageAndShow();
	rw::math::Q getdQ(cv::Mat image);

private slots:
	void buttonPressed();
	void loop();

	void stateChangedListener(const rw::kinematics::State& state);

private:
	std::ifstream _motionFile;

	QTimer* _loop;

	rw::models::WorkCell::Ptr _wc;
	rw::models::Device::Ptr _device;
	rw::kinematics::State _state;
	rwlibs::opengl::RenderImage *_textureRender, *_bgRender;
	rwlibs::simulation::GLFrameGrabber* _framegrabber;


};

#endif /*RINGONHOOKPLUGIN_HPP_*/
