//Includes
#include "SamplePlugin.hpp"

//#include <rw/rw.hpp>
#include <rws/RobWorkStudio.hpp>

#include <QPushButton>

#include <rw/loaders/ImageLoader.hpp>
#include <rw/loaders/WorkCellFactory.hpp>


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
/*
//Global variables
QString iconPath = "/mnt/Free/Dropbox/Programming/robWork/ROVI1project/src/SamplePluginPA10/src/pa_icon.png";
string workcellPath = "/mnt/Free/Dropbox/Programming/robWork/ROVI1project/res/PA10WorkCell/ScenePA10RoVi1.wc.xml";
string imagePath = "/mnt/Free/Dropbox/Programming/robWork/ROVI1project/src/SamplePluginPA10/src/lena.bmp";
string markerPath = "/mnt/Free/Dropbox/Programming/robWork/ROVI1project/src/SamplePluginPA10/markers/Marker1.ppm";
string backgroundPath = "/mnt/Free/Dropbox/Programming/robWork/ROVI1project/src/SamplePluginPA10/backgrounds/color1.ppm";
*/

QString iconPath = "/home/pyc/workspace/ROVI1project/src/SamplePluginPA10/src/pa_icon.png";
string workcellPath = "/home/pyc/workspace/ROVI1project/res/PA10WorkCell/ScenePA10RoVi1.wc.xml";
string imagePath = "/home/pyc/workspace/ROVI1project/src/SamplePluginPA10/src/lena.bmp";
string markerPath = "/home/pyc/workspace/ROVI1project/src/SamplePluginPA10/markers/Marker1.ppm";
string backgroundPath = "/home/pyc/workspace/ROVI1project/src/SamplePluginPA10/backgrounds/color1.ppm";



SamplePlugin::SamplePlugin():
    RobWorkStudioPlugin("SamplePluginUI", QIcon(iconPath))
{
	setupUi(this);

	_timer = new QTimer(this);
    connect(_timer, SIGNAL(timeout()), this, SLOT(timer()));

	// now connect stuff from the ui component
	connect(_btn0    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_btn1    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	connect(_spinBox  ,SIGNAL(valueChanged(int)), this, SLOT(btnPressed()) );

	Image textureImage(300,300,Image::GRAY,Image::Depth8U);
	_textureRender = new RenderImage(textureImage);
	Image bgImage(0,0,Image::GRAY,Image::Depth8U);
	_bgRender = new RenderImage(bgImage,2.5/1000.0);
	_framegrabber = NULL;
}

SamplePlugin::~SamplePlugin()
{
	delete _textureRender;
	delete _bgRender;
}

void SamplePlugin::initialize() {
	log().info() << "INITALIZE" << "\n";

	getRobWorkStudio()->stateChangedEvent().add(boost::bind(&SamplePlugin::stateChangedListener, this, _1), this);

	// Auto load workcell
	WorkCell::Ptr wc = WorkCellLoader::Factory::load(workcellPath);

	getRobWorkStudio()->setWorkCell(wc);

	_spinBox->setMaximum(5000);
	_spinBox->setMinimum(10);
	_spinBox->setValue(100);

	// Load Lena image
	Mat im, image;
	im = imread(imagePath, CV_LOAD_IMAGE_COLOR); // Read the file
	cvtColor(im, image, CV_BGR2RGB); // Switch the red and blue color channels
	if(! image.data ) {
		RW_THROW("Could not open or find the image: please modify the file path in the source code!");
	}
	QImage img(image.data, image.cols, image.rows, image.step, QImage::Format_RGB888); // Create QImage from the OpenCV image
	_label->setPixmap(QPixmap::fromImage(img)); // Show the image at the label in the plugin
}

void SamplePlugin::open(WorkCell* workcell)
{
	log().info() << "OPEN" << "\n";
	_wc = workcell;
	_state = _wc->getDefaultState();
	_device = _wc->findDevice("PA10");

	Q q(7,1,1,1,1,1,1,1);
	_device->setQ(q,_state);

	log().info() << workcell->getFilename() << "\n";

	if (_wc != NULL) {
		// Add the texture render to this workcell if there is a frame for texture
		Frame* textureFrame = _wc->findFrame("MarkerTexture");
		if (textureFrame != NULL) {
			getRobWorkStudio()->getWorkCellScene()->addRender("TextureImage",_textureRender,textureFrame);
		}
		// Add the background render to this workcell if there is a frame for texture
		Frame* bgFrame = _wc->findFrame("Background");
		if (bgFrame != NULL) {
			getRobWorkStudio()->getWorkCellScene()->addRender("BackgroundImage",_bgRender,bgFrame);
		}

		// Create a GLFrameGrabber if there is a camera frame with a Camera property set
		Frame* cameraFrame = _wc->findFrame("CameraSim");
		if (cameraFrame != NULL) {
			if (cameraFrame->getPropertyMap().has("Camera")) {
				// Read the dimensions and field of view
				double fovy;
				int width,height;
				std::string camParam = cameraFrame->getPropertyMap().get<std::string>("Camera");
				std::istringstream iss (camParam, std::istringstream::in);
				iss >> fovy >> width >> height;
				// Create a frame grabber
				_framegrabber = new GLFrameGrabber(width,height,fovy);
				SceneViewer::Ptr gldrawer = getRobWorkStudio()->getView()->getSceneViewer();
				_framegrabber->init(gldrawer);
			}
		}
	}
}

void SamplePlugin::close() {
	log().info() << "CLOSE" << "\n";

	// Stop the timer
	_timer->stop();
	// Remove the texture render
	Frame* textureFrame = _wc->findFrame("MarkerTexture");
	if (textureFrame != NULL) {
		getRobWorkStudio()->getWorkCellScene()->removeDrawable("TextureImage",textureFrame);
	}
	// Remove the background render
	Frame* bgFrame = _wc->findFrame("Background");
	if (bgFrame != NULL) {
		getRobWorkStudio()->getWorkCellScene()->removeDrawable("BackgroundImage",bgFrame);
	}
	// Delete the old framegrabber
	if (_framegrabber != NULL) {
		delete _framegrabber;
	}
	_framegrabber = NULL;
	_wc = NULL;
}

Mat SamplePlugin::toOpenCVImage(const Image& img) {
	Mat res(img.getHeight(),img.getWidth(), CV_8SC3);
	res.data = (uchar*)img.getImageData();
	return res;
}

void SamplePlugin::btnPressed() {
	QObject *obj = sender();
	if(obj==_btn0){
		log().info() << "Button 0\n";
		// Set a new texture (one pixel = 1 mm)
		Image::Ptr image;
		image = ImageLoader::Factory::load(markerPath);
		_textureRender->setImage(*image);
		image = ImageLoader::Factory::load(backgroundPath);
		_bgRender->setImage(*image);
		getRobWorkStudio()->updateAndRepaint();
	} else if(obj==_btn1){
		log().info() << "Button 1\n";
		// Toggle the timer on and off
		if (!_timer->isActive())
		    _timer->start(_spinBox->value()); // run 10 Hz
		else
			_timer->stop();
	} else if(obj==_spinBox){
		log().info() << "spin value:" << _spinBox->value() << "\n";
	}
}

void SamplePlugin::timer() {
	if (_framegrabber != NULL) {
		// Get the image as a RW image
		Frame* cameraFrame = _wc->findFrame("CameraSim");
		_framegrabber->grab(cameraFrame, _state);
		const Image& image = _framegrabber->getImage();

		// Convert to OpenCV image
		Mat im = toOpenCVImage(image);
		Mat imflip;
		cv::flip(im, imflip, 0);

		// Show in QLabel
		QImage img(imflip.data, imflip.cols, imflip.rows, imflip.step, QImage::Format_RGB888);
		QPixmap p = QPixmap::fromImage(img);
		unsigned int maxW = 400;
		unsigned int maxH = 800;
		_label->setPixmap(p.scaled(maxW,maxH,Qt::KeepAspectRatio));
	}

	Q q1(7,1,1,1,1,1,1,1);
	Q q2(7,2,2,2,2,2,2,2);

	if (_device->getQ(_state) == q2) _device->setQ(q1,_state);

	if (_device->getQ(_state) == q1) _device->setQ(q2,_state);

	getRobWorkStudio()->setState(_state);
}

void SamplePlugin::stateChangedListener(const State& state) {
	_state = state;
}

Q_EXPORT_PLUGIN(SamplePlugin);
