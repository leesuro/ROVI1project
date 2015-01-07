//Includes
#include "SamplePlugin.hpp"

//Global variables
/*const QString iconPath = "/mnt/Free/Dropbox/Programming/robWork/ROVI1project/src/SamplePluginPA10/src/pa_icon.png";
const string workcellPath = "/mnt/Free/Dropbox/Programming/robWork/ROVI1project/res/PA10WorkCell/ScenePA10RoVi1.wc.xml";
const string imagePath = "/mnt/Free/Dropbox/Programming/robWork/ROVI1project/src/SamplePluginPA10/src/lena.bmp";
const string markerPath = "/mnt/Free/Dropbox/Programming/robWork/ROVI1project/src/SamplePluginPA10/markers/Marker3.ppm";
const string backgroundPath = "/mnt/Free/Dropbox/Programming/robWork/ROVI1project/src/SamplePluginPA10/backgrounds/color1.ppm";
const string motionFilePath = "/mnt/Free/Dropbox/Programming/robWork/ROVI1project/src/SamplePluginPA10/motions/MarkerMotionFast.txt";
const string cameraPosePath = "/mnt/Free/Dropbox/Programming/robWork/ROVI1project/data/cameraPose.txt";
const string errorPosePath = "/mnt/Free/Dropbox/Programming/robWork/ROVI1project/data/errorPose.txt";
const string qRobotPath = "/mnt/Free/Dropbox/Programming/robWork/ROVI1project/data/qRobot.txt";
*/


const QString iconPath = "/home/pyc/workspace/ROVI1project/src/SamplePluginPA10/src/pa_icon.png";
const string workcellPath = "/home/pyc/workspace/ROVI1project/res/PA10WorkCell/ScenePA10RoVi1.wc.xml";
const string imagePath = "/home/pyc/workspace/ROVI1project/src/SamplePluginPA10/src/lena.bmp";
const string markerPath = "/home/pyc/workspace/ROVI1project/src/SamplePluginPA10/markers/Marker3.ppm";
const string backgroundPath = "/home/pyc/workspace/ROVI1project/src/SamplePluginPA10/backgrounds/color1.ppm";
const string motionFilePath = "/home/pyc/workspace/ROVI1project/src/SamplePluginPA10/motions/MarkerMotionSlow.txt";
const string cameraPosePath = "/home/pyc/workspace/ROVI1project/data/cameraPose.txt";
const string errorPosePath = "/home/pyc/workspace/ROVI1project/data/errorPose.txt";
const string qRobotPath = "/home/pyc/workspace/ROVI1project/data/qRobot.txt";



//--------------------------------------------------------
//					  Constructors
//--------------------------------------------------------
SamplePlugin::SamplePlugin():
    RobWorkStudioPlugin("SamplePluginUI", QIcon(iconPath))
{
	setupUi(this);

	_loop = new QTimer(this);
    connect(_loop, SIGNAL(timeout()), this, SLOT(loop()));

	//Now connect stuff from the ui component
	connect(_btn0, SIGNAL(pressed()), this, SLOT(buttonPressed()) );
	connect(_btn1, SIGNAL(pressed()), this, SLOT(buttonPressed()) );
	connect(_spinBox, SIGNAL(valueChanged(int)), this, SLOT(buttonPressed()) );

	Image textureImage(300,300,Image::GRAY,Image::Depth8U);
	_textureRender = new RenderImage(textureImage);
	Image bgImage(0,0,Image::GRAY,Image::Depth8U);
	_bgRender = new RenderImage(bgImage,2.5/1000.0);
	_framegrabber = NULL;
	_firstTime = 0;
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

	//Auto load workcell
	WorkCell::Ptr wc = WorkCellLoader::Factory::load(workcellPath);
	getRobWorkStudio()->setWorkCell(wc);
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

	//Show the camera view in the plugin
	getImageAndShow();

	//Adjust the Spin Box Limits
	_spinBox->setMaximum(9999);
	_spinBox->setMinimum(0.05);
	_spinBox->setValue(10);

	Q qInit(7, 0, -0.65, 0, 1.80, 0, 0.42, 0);
	_device->setQ(qInit, _state);
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
	//Mat im(imageGrabbed.getHeight(),imageGrabbed.getWidth(), CV_8SC3);
	Mat im(imageGrabbed.getHeight(),imageGrabbed.getWidth(), CV_8UC3);
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

/**
 * Exports the data to files
 * @param _cameraPoseVec The Camera pose
 * @param _errorPoseVec The error pose
 * @param _qRobotVec The Robot Q
 */
void SamplePlugin::writeData()
{
	_cameraPoseFile.open(cameraPosePath.c_str());
	_errorPoseFile.open(errorPosePath.c_str());
	_qRobotFile.open(qRobotPath.c_str());

	_cameraPoseFile.precision(3);
	_cameraPoseFile.setf(std::ios::fixed);
	_errorPoseFile.precision(3);
	_errorPoseFile.setf(std::ios::fixed);
	_qRobotFile.precision(3);
	_qRobotFile.setf(std::ios::fixed);


	_cameraPoseFile << "\\newcommand{\\cameraPoseData}{\n";
	for (unsigned int i=0; i<_cameraPoseVec.size(); i++){
		_cameraPoseFile << "("
						<< _cameraPoseVec[i].P()(0) << ","
						<< _cameraPoseVec[i].P()(1) << ","
						<< _cameraPoseVec[i].P()(2) << ","
						<< RPY<>(_cameraPoseVec[i].R())(0) << ","
						<< RPY<>(_cameraPoseVec[i].R())(1) << ","
						<< RPY<>(_cameraPoseVec[i].R())(2) << ")\n";
	}
	_cameraPoseFile << "}";

	_errorPoseFile << "\\newcommand{\\errorPoseData}{\n";
	for (unsigned int i=0; i<_errorPoseVec.size(); i++){
		_errorPoseFile  << "("
						<< _errorPoseVec[i].P()(0) << ","
						<< _errorPoseVec[i].P()(1) << ","
						<< _errorPoseVec[i].P()(2) << ","
						<< RPY<>(_errorPoseVec[i].R())(0) << ","
						<< RPY<>(_errorPoseVec[i].R())(1) << ","
						<< RPY<>(_errorPoseVec[i].R())(2) << ")\n";
	}
	_errorPoseFile << "}";

	for (unsigned char joint=0; joint < 7; joint++){
		_qRobotFile << "\\newcommand{\\qRobotData}[" << (int)joint << "]{\n";
		for (unsigned int i=0; i<_qRobotVec.size(); i++){
			_qRobotFile << "("
						<< i << ", "
						<< _qRobotVec[i](joint) << ")\n";
		}
		_qRobotFile << "}\n\n";
	}

	_cameraPoseFile.close();
	_errorPoseFile.close();
	_qRobotFile.close();
}

/**
 * Computes the error between the marker and the camera
 */
void SamplePlugin::computeError(){
	for (unsigned int i=0; i<_cameraPoseVec.size(); i++){
		Vector3D<> P = _markerPoseVec[i].P()-_cameraPoseVec[i].P();
		RPY<> RPYMarker = RPY<>(_markerPoseVec[i].R());
		RPY<> RPYCamera = RPY<>(_cameraPoseVec[i].R());
		RPY<> RPY (RPYMarker(0)-RPYCamera(0), RPYMarker(1)-RPYCamera(1), RPYMarker(2)-RPYCamera(2) );
		Transform3D<> trans_aux(P, RPY.toRotation3D());
		_errorPoseVec.push_back(trans_aux);
	}
}

/**
 *
 * @param image
 * @return
 */
Q SamplePlugin::getdQ(Mat & image)
{
	//Get points from OpenCV algorithms
	//Vector3D<> P = (_wc->findFrame("Marker"))->getTransform(_state).P();
	//Vector3D<> Pt = inverse(_device->worldTbase(_state) * _device->baseTframe(_wc->findFrame("Camera"), _state)) * P * 1000;

	Point2f point = corny(image);

/*	float u = Pt(0);
	float v = Pt(1);*/

	float u = point.x + 1024/2;
	float v = point.y + 768/2;

	//log().info() << "x:" << point.x << ", y:" << point.y << "\n";

	//Continue with the device's jacobian
	Jacobian deviceJacobian_aux = _device->baseJframe(_wc->findFrame("Camera"), _state);
	MatrixXd deviceJacobian(6,7);
	for (unsigned char row=0; row<6; row++){
		for (unsigned char col=0; col<7; col++){
			deviceJacobian(row, col) = deviceJacobian_aux(row, col);
		}
	}

	//Now the image's jacobian. f and z are fixed values given in the description
	MatrixXd imageJacobian(2,6);
	double z = 0.5, f = 823;

	imageJacobian(0,0)=f/z; imageJacobian(0,1)= 0; imageJacobian(0,2)=-u/z;
	imageJacobian(0,3)=-u*v/f; imageJacobian(0,4)=(f*f+u*u)/f; imageJacobian(0,5)=-v;
	imageJacobian(1,0)=0; imageJacobian(1,1)=f/z; imageJacobian(1,2)=-v/z;
	imageJacobian(1,3)=-(f*f+u*u)/f; imageJacobian(1,4)=u*v/f; imageJacobian(1,5)=u;

	//As the d(u,v) is not referenced to the base, we need to adapt it
	MatrixXd Sq(6,6);
	Rotation3D<> R_device_T = inverse(_device->baseTframe(_wc->findFrame("Camera"), _state).R());
	for (unsigned char row=0; row<6; row++){
		for (unsigned char col=0; col<6; col++){
			if (row<3 && col<3) Sq(row, col) = R_device_T(row, col);
			else if (row>2 && col>2) Sq(row, col) = R_device_T(row-3, col-3);
			else Sq(row, col) =0;
		}
	}

	//Now we can calculate Zimage
	MatrixXd Zimage(2, 7);
	Zimage = imageJacobian*Sq*deviceJacobian;

	//Perhaps, now the du and dv
	MatrixXd dudv(2,1);

	if (_firstTime){
		dudv(0,0) = u - _previousPoints[0][0];
		dudv(1,0) = v - _previousPoints[0][1];
	}
	_firstTime = 1;

	//Show the differential
	//log().info() << "x:" << dudv(0,0) << ", y:" << dudv(1,0) << "\n";

	//Store the actual point
	_previousPoints[0][0] = u;
	_previousPoints[0][1] = v;

	//And calculate dq
	MatrixXd dq_aux (7,1);
	dq_aux = Zimage.transpose() * (Zimage*Zimage.transpose()).inverse() * dudv;

	//dq_aux*=2.0;

	Q dq = Q(7, dq_aux(0,0), dq_aux(1,0), dq_aux(2,0), dq_aux(3,0),	dq_aux(4,0), dq_aux(5,0), dq_aux(6,0));

	return dq;
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
			Transform3D<> markerTransformation = Transform3D<>(Vector3D<>(data[0], data[1], data[2]), RPY<>(data[3], data[4], data[5]).toRotation3D());
			//So now the position is updated
			markerFrame->setTransform(markerTransformation,_state);
			//Get the image seen from the camera and show it in the plugin
			Mat image = getImageAndShow();
			//Calculates dq due to the image movement
			Q dq = getdQ(image);
			//Updates the position based on that dq
			_device->setQ(_device->getQ(_state) + dq, _state);
			//Updates the state in the RobWork Studio
			getRobWorkStudio()->setState(_state);
			//Store the Q and the Camera's Pose
			_markerPoseVec.push_back(markerFrame->getTransform(_state));
			_cameraPoseVec.push_back(_device->worldTbase(_state)*_device->baseTframe(_wc->findFrame("Camera"), _state));
			_qRobotVec.push_back(_device->getQ(_state));

		} else {
			log().info() << "Motion file finished!" << "\n";
			computeError();
			writeData();
			log().info() << "Data exported!" << "\n!";

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
	//Button 0
	if(obj==_btn0){
		log().info() << "Button 0\n";
		// Set a new texture (one pixel = 1 mm)
		Image::Ptr image;
		image = ImageLoader::Factory::load(markerPath);
		_textureRender->setImage(*image);
		image = ImageLoader::Factory::load(backgroundPath);
		_bgRender->setImage(*image);

		getRobWorkStudio()->updateAndRepaint();
	}
	//Button 1
	else if(obj==_btn1) {
		log().info() << "Button 1\n";
		// Toggle the loop on and off
		if (!_loop->isActive()) _loop->start(_spinBox->value()); // run 10 Hz
		else _loop->stop();
	}
	//SpinBox
	else if(obj==_spinBox) log().info() << "spin value:" << _spinBox->value() << "\n";
}

/**
 * Returns the actual state of the workcell
 * @param state
 */
void SamplePlugin::stateChangedListener(const State& state) {
	_state = state;
}


//--------------------------------------------------------
//					   Detection
//--------------------------------------------------------
Point2f SamplePlugin::corny (Mat img_input)
{
	//Calculate the processing time
	Timer calculationTime;
	calculationTime.reset();

	//Loads the image and the marker
	Mat img_scene;
	cvtColor(img_input, img_scene, CV_RGB2GRAY);

	Mat img_object = imread(markerPath, CV_LOAD_IMAGE_GRAYSCALE);

	//Detect the keypoints using SURF Detector
	unsigned int minHessian = 400;

	SurfFeatureDetector detector(minHessian);
	vector<KeyPoint> keypoints_object, keypoints_scene;

	detector.detect(img_object, keypoints_object);
	detector.detect(img_scene, keypoints_scene);

	//Calculate descriptors (feature vectors)
	SurfDescriptorExtractor extractor;
	Mat descriptors_object, descriptors_scene;

	extractor.compute(img_object, keypoints_object, descriptors_object);
	extractor.compute(img_scene, keypoints_scene, descriptors_scene);

	// Matching descriptor vectors using FLANN matcher
	FlannBasedMatcher matcher;
	vector<DMatch> matches;
	matcher.match(descriptors_object, descriptors_scene, matches);

	double max_dist = 0;
	double min_dist = 100;

	//Quick calculation of max and min distances between keypoints
	for (int i = 0; i < descriptors_object.rows; i++) {
		double dist = matches[i].distance;
		if (dist < min_dist)
			min_dist = dist;
		if (dist > max_dist)
			max_dist = dist;
	}

	//Filter only the good matches
	vector<DMatch> good_matches;
	for (int i = 0; i < descriptors_object.rows; i++) {
		if (matches[i].distance < 3 * min_dist) good_matches.push_back(matches[i]);
	}

	//Localize the object
	vector<Point2f> obj;
	vector<Point2f> scene;

	for (unsigned int i = 0; i < good_matches.size(); i++) {
		//-- Get the key points from the good matches
		obj.push_back(keypoints_object[good_matches[i].queryIdx].pt);
		scene.push_back(keypoints_scene[good_matches[i].trainIdx].pt);
	}

	Mat H = findHomography(obj, scene, CV_RANSAC);

	//Get the corners from the image_1 (the object to be "detected")
	vector<Point2f> obj_corners(4);
	obj_corners[0] = cvPoint(0, 0);
	obj_corners[1] = cvPoint(img_object.cols, 0);
	obj_corners[2] = cvPoint(img_object.cols, img_object.rows);
	obj_corners[3] = cvPoint(0, img_object.rows);
	vector<Point2f> scene_corners(4);

	perspectiveTransform(obj_corners, scene_corners, H);

	//Draw lines between the corners and the calculated point and show the image
/*	Mat img_matches=img_scene;

	line(img_matches, scene_corners[0],	scene_corners[1] , Scalar(0, 255, 0), 4);
	line(img_matches, scene_corners[1],	scene_corners[2] , Scalar(0, 255, 0), 4);
	line(img_matches, scene_corners[2],	scene_corners[3] , Scalar(0, 255, 0), 4);
	line(img_matches, scene_corners[3],	scene_corners[0] , Scalar(0, 255, 0), 4);
	circle(img_matches,scene_corners[0],5, Scalar(0,255,0));
	imshow("Good Matches & Object detection", img_matches);*/

	//Show the processing time
	long totalTime = calculationTime.getTimeMs();
	log().info() << "Processing Time: " << totalTime << "[ms]\n";

	//log().info() << "-----x:" << scene_corners[0].x << ", y:" << scene_corners[0].y << "\n";

	return scene_corners[0];
}

Q_EXPORT_PLUGIN(SamplePlugin);
