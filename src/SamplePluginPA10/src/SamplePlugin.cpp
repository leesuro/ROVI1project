//Includes
#include "SamplePlugin.hpp"

#include <rws/RobWorkStudio.hpp>

#include <QPushButton>

#include <rw/loaders/ImageLoader.hpp>
#include <rw/loaders/WorkCellFactory.hpp>
#include <rw/kinematics/MovableFrame.hpp>

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

using namespace std;

//Global variables
const QString iconPath = "/mnt/Free/Dropbox/Programming/robWork/ROVI1project/src/SamplePluginPA10/src/pa_icon.png";
const string workcellPath = "/mnt/Free/Dropbox/Programming/robWork/ROVI1project/res/PA10WorkCell/ScenePA10RoVi1.wc.xml";
const string imagePath = "/mnt/Free/Dropbox/Programming/robWork/ROVI1project/src/SamplePluginPA10/src/lena.bmp";
const string markerPath = "/mnt/Free/Dropbox/Programming/robWork/ROVI1project/src/SamplePluginPA10/markers/Marker1.ppm";
const string backgroundPath = "/mnt/Free/Dropbox/Programming/robWork/ROVI1project/src/SamplePluginPA10/backgrounds/color1.ppm";
const string motionFilePath = "/mnt/Free/Dropbox/Programming/robWork/ROVI1project/src/SamplePluginPA10/motions/MarkerMotionSlow.txt";

/*
const QString iconPath = "/home/pyc/workspace/ROVI1project/src/SamplePluginPA10/src/pa_icon.png";
const string workcellPath = "/home/pyc/workspace/ROVI1project/res/PA10WorkCell/ScenePA10RoVi1.wc.xml";
const string imagePath = "/home/pyc/workspace/ROVI1project/src/SamplePluginPA10/src/lena.bmp";
const string markerPath = "/home/pyc/workspace/ROVI1project/src/SamplePluginPA10/markers/Marker1.ppm";
const string backgroundPath = "/home/pyc/workspace/ROVI1project/src/SamplePluginPA10/backgrounds/color1.ppm";
const string motionFilePath = "/home/pyc/workspace/ROVI1project/src/SamplePluginPA10/motions/MarkerMotionSlow.txt";
*/

//--------------------------------------------------------
//					  Constructors
//--------------------------------------------------------
SamplePlugin::SamplePlugin():
    RobWorkStudioPlugin("SamplePluginUI", QIcon(iconPath))
{
	setupUi(this);

	_loop = new QTimer(this);
    connect(_loop, SIGNAL(timeout()), this, SLOT(loop()));

	// now connect stuff from the ui component
	connect(_btn0, SIGNAL(pressed()), this, SLOT(buttonPressed()) );
	connect(_btn1, SIGNAL(pressed()), this, SLOT(buttonPressed()) );
	connect(_spinBox, SIGNAL(valueChanged(int)), this, SLOT(buttonPressed()) );

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

//--------------------------------------------------------
//					 Plugin Methods
//--------------------------------------------------------
/**
 * Actions before being opened
 */
void SamplePlugin::initialize()
{
	log().info() << "INITALIZE" << "\n";

	getRobWorkStudio()->stateChangedEvent().add(boost::bind(&SamplePlugin::stateChangedListener, this, _1), this);

	// Auto load workcell
	WorkCell::Ptr wc = WorkCellLoader::Factory::load(workcellPath);

	getRobWorkStudio()->setWorkCell(wc);

	_spinBox->setMaximum(1100);
	_spinBox->setMinimum(0.05);
	_spinBox->setValue(10);

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
/**
 * Open the plugin and the workcell. Also loads the Marker and Background frame
 * along with the Camera.
 * @param workcell The workcell to be loaded
 */
void SamplePlugin::open(WorkCell* workcell)
{
	log().info() << "OPEN" << "\n";

	log().info() << workcell->getFilename() << "\n";
	_wc = workcell;
	_state = _wc->getDefaultState();
	_device = _wc->findDevice("PA10");
	_motionFile.open(motionFilePath.c_str());


	if (_wc != NULL) {
		// Add the texture render to this workcell if there is a frame for texture
		Frame* textureFrame = _wc->findFrame("MarkerTexture");
		if (textureFrame != NULL) getRobWorkStudio()->getWorkCellScene()->addRender("TextureImage",_textureRender,textureFrame);

		// Add the background render to this workcell if there is a frame for texture
		Frame* bgFrame = _wc->findFrame("Background");
		if (bgFrame != NULL) getRobWorkStudio()->getWorkCellScene()->addRender("BackgroundImage",_bgRender,bgFrame);

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

/**
 * Close the plugin
 */
void SamplePlugin::close()
{
	log().info() << "CLOSE" << "\n";
	// Stop the loop
	_loop->stop();
	// Remove the texture render
	Frame* textureFrame = _wc->findFrame("MarkerTexture");
	if (textureFrame != NULL) getRobWorkStudio()->getWorkCellScene()->removeDrawable("TextureImage",textureFrame);
	// Remove the background render
	Frame* bgFrame = _wc->findFrame("Background");
	if (bgFrame != NULL) getRobWorkStudio()->getWorkCellScene()->removeDrawable("BackgroundImage",bgFrame);
	// Delete the old framegrabber
	if (_framegrabber != NULL) delete _framegrabber;
	_framegrabber = NULL;
	_wc = NULL;
}



//--------------------------------------------------------
//					   Methods
//--------------------------------------------------------

/**
 * Gets the image of the virtual camera attached to the robot. This will be used
 * for see the movement. Also show the image in the _label plugin.
 * @return The image to check the movement
 */
Mat SamplePlugin::getImageAndShow()
{
	// Get the image as a RW image
	Frame* cameraFrame = _wc->findFrame("CameraSim");
	_framegrabber->grab(cameraFrame, _state);
	const Image& imageGrabbed = _framegrabber->getImage();

	// Convert to OpenCV image
	Mat im(imageGrabbed.getHeight(),imageGrabbed.getWidth(), CV_8SC3);
	im.data = (uchar*)imageGrabbed.getImageData();
	Mat image;
	flip(im, image, 0);

	// Show in QLabel
	QImage imageToShow(image.data, image.cols, image.rows, image.step, QImage::Format_RGB888);
	QPixmap p = QPixmap::fromImage(imageToShow);
	unsigned int maxW = 600;
	unsigned int maxH = 800;
	_label->setPixmap(p.scaled(maxW,maxH,Qt::KeepAspectRatio));

	return image;
}

Q SamplePlugin::getdQ(Mat image){
	Q q(7,0.01,0.01,0.01,0.01,0.01,0.01,0.01);
	return q;
}

//--------------------------------------------------------
//					 Private Slots
//--------------------------------------------------------

/**
 * The loop to be repeated with each frame
 */
void SamplePlugin::loop()
{
	if (_motionFile.is_open()){
		//Create a movable frame
		MovableFrame* markerFrame = static_cast<MovableFrame*>(_wc->findFrame("Marker"));
		//Read the marker's movement
		string line;
		double data[6];
		if(getline(_motionFile,line)){
			//Read a line and store its values
			istringstream istr(line);
			istr >> data[0] >> data[1] >> data[2] >> data[3] >> data[4] >> data[5];
			//And generates the transformation
			Transform3D<> markerTransformation=Transform3D<>(Vector3D<>(data[0], data[1], data[2]), RPY<>(data[3], data[4], data[5]).toRotation3D());
			//So now the position is updated
			markerFrame->setTransform(markerTransformation,_state);
			getRobWorkStudio()->getWorkCellScene()->addRender("TextureImage",_textureRender,markerFrame);
			//Get the image seen from the camera and show it in the plugin
			Mat image = getImageAndShow();
			//Calculates dq due to the image movement
			Q dq = getdQ(image);
			//Updates the position based on that dq
			_device->setQ(_device->getQ(_state) + dq,_state);
			//Updates the state in the RobWork Studio
			getRobWorkStudio()->setState(_state);
		} else {
			log().info() << "Motion file finished!" << "\n!";
			_motionFile.close();
		}
	}

}

/**
 * Determine what happens when a button is pressed
 */
void SamplePlugin::buttonPressed()
{
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
		// Toggle the loop on and off
		if (!_loop->isActive())
		    _loop->start(_spinBox->value()); // run 10 Hz
		else
			_loop->stop();
	} else if(obj==_spinBox){
		log().info() << "spin value:" << _spinBox->value() << "\n";
	}
}

/**
 * Returns the actual state of the workcell
 * @param state
 */
void SamplePlugin::stateChangedListener(const State& state) {
	_state = state;
}

Q_EXPORT_PLUGIN(SamplePlugin);
