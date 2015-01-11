//Includes
#include "SamplePlugin.hpp"

//Global variables
#if (USER == 0)
	const string userPath = "/mnt/Free/Dropbox/Programming/robWork/";
#elif (USER == 1)
	const string userPath = "/home/pyc/workspace/";
#endif

#if (SPEED==0)
	const string motionFilePath = userPath + "ROVI1project/src/SamplePluginPA10/motions/MarkerMotionSlow.txt";
#elif (SPEED==1)
	const string motionFilePath = userPath + "ROVI1project/src/SamplePluginPA10/motions/MarkerMotionMedium.txt";
#elif (SPEED==2)
	const string motionFilePath = userPath + "ROVI1project/src/SamplePluginPA10/motions/MarkerMotionFast.txt";
#endif

#if (TRACKING==0)
	const string markerPath = userPath + "ROVI1project/src/SamplePluginPA10/markers/Marker1.ppm";
#elif (TRACKING==1)
	const string markerPath = userPath + "ROVI1project/src/SamplePluginPA10/markers/Marker1.ppm";
#elif (TRACKING==2)
	const string markerPath = userPath + "ROVI1project/src/SamplePluginPA10/markers/Marker2a.ppm";
#elif (TRACKING==3)
	const string markerPath = userPath + "ROVI1project/src/SamplePluginPA10/markers/Marker2b.ppm";
#elif (TRACKING==4)
	const string markerPath = userPath + "ROVI1project/src/SamplePluginPA10/markers/Marker3.ppm";
#endif

const QString iconPath = QString::fromStdString(userPath) + "ROVI1project/src/SamplePluginPA10/src/pa_icon.png";
const string workcellPath = userPath + "ROVI1project/res/PA10WorkCell/ScenePA10RoVi1.wc.xml";
const string backgroundPath = userPath + "ROVI1project/src/SamplePluginPA10/backgrounds/color1.ppm";
const string cameraPosePath = userPath + "ROVI1project/data/cameraPose";
const string errorPath = userPath + "ROVI1project/data/error";
const string qRobotPath = userPath + "ROVI1project/data/qRobot";
const string speedPath = userPath + "ROVI1project/data/speed";

//--------------------------------------------------------
//					  Constructors
//--------------------------------------------------------
SamplePlugin::SamplePlugin():
    RobWorkStudioPlugin("SamplePluginUI", QIcon(iconPath))
{
	setupUi(this);

	_loop = new QTimer(this);
    connect(_loop, SIGNAL(timeout()), this, SLOT(loop()));

	//Now connect stuff from the UI
	connect(_btn0, SIGNAL(pressed()), this, SLOT(buttonPressed()) );
	connect(_spinBox, SIGNAL(valueChanged(int)), this, SLOT(buttonPressed()) );

	Image textureImage(300,300,Image::GRAY,Image::Depth8U);
	_textureRender = new RenderImage(textureImage);
	Image bgImage(0,0,Image::GRAY,Image::Depth8U);
	_bgRender = new RenderImage(bgImage,2.5/1000.0);
	_framegrabber = NULL;

	//First time? Yes, it is!
	_firstTime = 1;
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

	_motionFile.open(motionFilePath.c_str());

	//Set a new texture (one pixel = 1 mm)
	Image::Ptr image;
	image = ImageLoader::Factory::load(markerPath);
	_textureRender->setImage(*image);
	image = ImageLoader::Factory::load(backgroundPath);
	if (TRACKING != 2 && TRACKING != 4) _bgRender->setImage(*image);

	getRobWorkStudio()->updateAndRepaint();

	//Show the camera view in the plugin
	getImageAndShow();

	//Adjust the Spin Box Limits
	_spinBox->setMaximum(9999);
	_spinBox->setMinimum(10);
	_spinBox->setValue(50);
	if (TRACKING == 4)_spinBox->setValue(1200);
	if (TRACKING == 2)_spinBox->setValue(900);

	Q qInit(7, 0, -0.65, 0, 1.80, 0, 0.42, 0);
	_device->setQ(qInit, _state);
	getRobWorkStudio()->setState(_state);
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
	Mat im(imageGrabbed.getHeight(),imageGrabbed.getWidth(), CV_8UC3);
	im.data = (uchar*)imageGrabbed.getImageData();
	Mat image;
	flip(im, image, 0);

	// Show in QLabel
	QImage imageToShow(image.data, image.cols, image.rows, image.step, QImage::Format_RGB888);
	_label->setPixmap(QPixmap::fromImage(imageToShow).scaled(600,800,Qt::KeepAspectRatio));

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
	string tracking;
	string experiment;
	if (NUMBER_OF_POINTS==3) tracking = "Three";
	if (TRACKING == 0) tracking = "Tracking";
	if (TRACKING == 1) tracking = "Color";
	if (TRACKING == 2) tracking = "Lines";
	if (TRACKING == 4) tracking = "Corny";
	if (SPEED == 0) experiment = tracking + "Slow";
	if (SPEED == 1) experiment = tracking + "Medium";
	if (SPEED == 2) experiment = tracking + "Fast";

	//Q Robot
	ofstream qRobotFile; qRobotFile.open((qRobotPath + experiment + ".tex").c_str()); qRobotFile.precision(3); qRobotFile.setf(std::ios::fixed);
	latexPlotQ(qRobotFile, ("qRobot" + experiment + "Plot"));
	qRobotFile.close();

	//Speed
	ofstream speedFile; speedFile.open((speedPath + experiment + ".tex").c_str()); speedFile.precision(3); speedFile.setf(std::ios::fixed);
	latexPlotSpeed(speedFile, ("speed" + experiment + "Plot"));
	speedFile.close();

	//Camera Pose
	ofstream cameraPoseFile; cameraPoseFile.open((cameraPosePath + experiment + ".tex").c_str()); cameraPoseFile.precision(3); cameraPoseFile.setf(std::ios::fixed);
	latexPlotCameraPose(cameraPoseFile, ("cameraPose" + experiment + "Plot"));
	cameraPoseFile.close();

	//Error
	ofstream errorFile; errorFile.open((errorPath + tracking + ".tex").c_str()); errorFile.precision(3); errorFile.setf(std::ios::fixed);
	latexPlotError(errorFile, ("error" + tracking + "Plot"));
	errorFile.close();

	ofstream errorXFile; errorXFile.open((errorPath + experiment + "X.txt").c_str()); errorXFile.precision(3); errorXFile.setf(std::ios::fixed);
	for (unsigned int i=0; i<_errorPoseVec.size()-1; i++) errorXFile   << "				(" << _sumDeltaT[i]/1000 << "," << _errorPoseVec[i+1](0) << ")\n";
	errorXFile.close();

	ofstream errorYFile; errorYFile.open((errorPath + experiment + "Y.txt").c_str()); errorYFile.precision(3); errorYFile.setf(std::ios::fixed);
	for (unsigned int i=0; i<_errorPoseVec.size()-1; i++) errorYFile   << "				(" << _sumDeltaT[i]/1000 << "," << _errorPoseVec[i+1](1) << ")\n";
	errorYFile.close();
}

/**
 * Computes the error between the marker and the camera
 */
void SamplePlugin::computeError(){
	Vector3D<> Pdiff = ( (_markerPoseVec.end()-2)->P() - (_markerPoseVec.end()-1)->P()) * 823 * 2;
	Vector3D<> dudv(_previousdUdV[0][0], _previousdUdV[0][1], 0);
	_errorPoseVec.push_back(Pdiff - dudv);
}

/**
 * Calculates the feature extraction average time
 */
void SamplePlugin::calculateAverageTime(){
	double total = 0;
	for(unsigned int i=0; i<_featureExtractionTime.size(); i++) total += _featureExtractionTime[i];
	total = total /_featureExtractionTime.size();
	log().info() << "Average time: " << total << "\n";
}

/**
 * Calculates deltaT
 */
void SamplePlugin::calculateDeltaT(){
	//Calculate deltaT
	float deltaT;
	if (TRACKING != 0) deltaT = (_spinBox->value() - *(_featureExtractionTime.end()-1));
	else deltaT = _spinBox->value();

	//Adapt and save deltaT
	if (deltaT < 0) deltaT = 0;

	_deltaT.push_back((long)deltaT);

	if (_firstTime) _sumDeltaT.push_back((long)deltaT);
	else _sumDeltaT.push_back((long)deltaT + *(_sumDeltaT.end()-1));
}

/**
 * Check if the speed is over the joint's limits and, if so, limit it to the limit
 * @param qToCheck
 */
void SamplePlugin::checkVelocityLimits(Q & dqToCheck){
	for(unsigned char joint=0; joint<7; joint++){
		if ( *(_deltaT.end()-1) == 0 || abs(dqToCheck(joint)) * 1000 / *(_deltaT.end()-1) > abs(_device->getVelocityLimits()(joint)) ){
			if (abs(dqToCheck(joint)) * 1000 / *(_deltaT.end()-1) > abs(_device->getVelocityLimits()(joint))) log().info() << abs(_device->getVelocityLimits()(0)) << "\n";
			if (dqToCheck(joint) > 0) dqToCheck(joint) = abs(_device->getVelocityLimits()(joint)) * *(_deltaT.end()-1) / 1000;
			if (dqToCheck(joint) < 0) dqToCheck(joint) = - abs(_device->getVelocityLimits()(joint)) * *(_deltaT.end()-1) / 1000;
			log().info() << "Too fast - ";
		}
	}
}

/**
 * Calculates the speed reached for each link in an specific dQ
 * @param dQ
 */
void SamplePlugin::calculateSpeed(const Q & dQ){
	_speedVec.push_back(dQ*1000*180/Pi / *(_deltaT.end()-1) ); //In deg/s
}

/**
 *
 * @param image
 * @return
 */
Q SamplePlugin::getdQ(Mat & image)
{
	float u[3];
	float v[3];
	//Get the point directly from the marker frame. From Camera to base, from base to world, from world to Marker
	if (TRACKING == 0){
		Vector3D<> Pt0 = (inverse(_device->worldTbase(_state) * _device->baseTframe(_wc->findFrame("Camera"), _state)) * _wc->findFrame("Marker")->getTransform(_state)) * Vector3D<>(0,0,0);
		Vector3D<> Pt1 = (inverse(_device->worldTbase(_state) * _device->baseTframe(_wc->findFrame("Camera"), _state)) * _wc->findFrame("Marker")->getTransform(_state)) * Vector3D<>(0.1,0,0);
		Vector3D<> Pt2 = (inverse(_device->worldTbase(_state) * _device->baseTframe(_wc->findFrame("Camera"), _state)) * _wc->findFrame("Marker")->getTransform(_state)) * Vector3D<>(0,0.1,0);
		u[0] = Pt0(0)*823*2;
		v[0] = Pt0(1)*823*2;
		if (NUMBER_OF_POINTS==2 || NUMBER_OF_POINTS==3) u[1] = Pt1(0)*823*2;
		if (NUMBER_OF_POINTS==2 || NUMBER_OF_POINTS==3) v[1] = Pt1(1)*823*2;
		if (NUMBER_OF_POINTS==3) u[2] = Pt2(0)*823*2;
		if (NUMBER_OF_POINTS==3) v[2] = Pt2(1)*823*2;
	}

	//Get points from OpenCV algorithms
	else {
		Point2f point;

		Timer featureExtractionTimer;
		featureExtractionTimer.reset();
		if (TRACKING == 1) point = colorDetection(image);
		if (TRACKING == 2) point = linesHDetection(image);
		if (TRACKING == 4) point = cornyDetection(image);
		_featureExtractionTime.push_back(featureExtractionTimer.getTimeMs());

		u[0] = point.x - 1024/2;
		v[0] = point.y - 768/2;
	}

	//Continue with the device's jacobian
	MatrixXd deviceJacobian(6,7);
	deviceJacobian = _device->baseJframe(_wc->findFrame("Camera"), _state).e();

	//Now the image's jacobian. f and z are fixed values given in the description
	MatrixXd imageJacobian(NUMBER_OF_POINTS*2,6);
	float z = 0.5, f = 823;

	for(unsigned char i=0; i<NUMBER_OF_POINTS; i++){
		imageJacobian(i*2,0)=f/z; imageJacobian(i*2,1)= 0; imageJacobian(i*2,2)=-u[i]/z;
		imageJacobian(i*2,3)=-u[i]*v[i]/f; imageJacobian(i*2,4)=(f*f+u[i]*u[i])/f; imageJacobian(i*2,5)=-v[i];
		imageJacobian(i*2+1,0)=0; imageJacobian(i*2+1,1)=f/z; imageJacobian(i*2+1,2)=-v[i]/z;
		imageJacobian(i*2+1,3)=-(f*f+u[i]*u[i])/f; imageJacobian(i*2+1,4)=u[i]*v[i]/f; imageJacobian(i*2+1,5)=u[i];
	}

	//As the d(u,v) is not referenced to the base, we need to adapt it with Sq
	Matrix3d R_device_T = (_device->baseTframe(_wc->findFrame("Camera"), _state).R().e()).transpose();

	MatrixXd Sq(6,6);
	for (unsigned char row=0; row<6; row++){
		for (unsigned char col=0; col<6; col++){
			if (row<3 && col<3) Sq(row, col) = R_device_T(row, col);
			else if (row>2 && col>2) Sq(row, col) = R_device_T(row-3, col-3);
			else Sq(row, col) =0;
		}
	}

	//Now we can calculate Zimage
	MatrixXd Zimage(NUMBER_OF_POINTS*2, 7);
	Zimage = imageJacobian*Sq*deviceJacobian;

	//Perhaps, now the du and dv
	MatrixXd dudv(NUMBER_OF_POINTS*2,1);
	dudv(0,0) = u[0] - _previousPoints[0][0];
	dudv(1,0) = v[0] - _previousPoints[0][1];
	if (NUMBER_OF_POINTS==2 || NUMBER_OF_POINTS==3) dudv(2,0) = u[1] - _previousPoints[1][0];
	if (NUMBER_OF_POINTS==2 || NUMBER_OF_POINTS==3) dudv(3,0) = v[1] - _previousPoints[1][1];
	if (NUMBER_OF_POINTS==3) dudv(4,0) = u[2] - _previousPoints[2][0];
	if (NUMBER_OF_POINTS==3) dudv(5,0) = v[2] - _previousPoints[2][1];

	//Store the actual point
	_previousPoints[0][0] = u[0];
	_previousPoints[0][1] = v[0];
	if (NUMBER_OF_POINTS==2 || NUMBER_OF_POINTS==3) _previousPoints[1][0] = u[1];
	if (NUMBER_OF_POINTS==2 || NUMBER_OF_POINTS==3) _previousPoints[1][1] = v[1];
	if (NUMBER_OF_POINTS==3) _previousPoints[2][0] = u[2];
	if (NUMBER_OF_POINTS==3) _previousPoints[2][1] = v[2];

	//And calculate dq
	MatrixXd dq_aux (7,1);
	dq_aux = Zimage.transpose() * (Zimage*Zimage.transpose()).inverse() * dudv * 0.96; //0.97 is the max

	//If the detected point is strange, return the previous dQ
	unsigned char limitdudv = 255; //Fast
	if (SPEED==0) limitdudv = 10; //Slow
	if (SPEED==1) limitdudv = 50; //Medium
	if (SPEED==2) limitdudv = 100; //Fast

	for (unsigned char i=0; i<3; i++){
		if(abs(dudv(0,0)) > limitdudv || abs(dudv(1,0)) > limitdudv ||
		   abs(dudv(2,0)) > limitdudv || abs(dudv(3,0)) > limitdudv ||
		   abs(dudv(4,0)) > limitdudv || abs(dudv(5,0)) > limitdudv){
			//log().info() << "Meeehh!  -  ";
			calculateDeltaT();
			return _previousdQ;
		}
	}

	_previousdUdV[0][0] = dudv(0,0);
	_previousdUdV[0][1] = dudv(1,0);
	if (NUMBER_OF_POINTS==2 || NUMBER_OF_POINTS==3) _previousdUdV[1][0] = dudv(2,0);
	if (NUMBER_OF_POINTS==2 || NUMBER_OF_POINTS==3) _previousdUdV[1][1] = dudv(3,0);
	if (NUMBER_OF_POINTS==3) _previousdUdV[2][0] = dudv(4,0);
	if (NUMBER_OF_POINTS==3) _previousdUdV[2][1] = dudv(5,0);

	Q dq = Q(7, dq_aux(0,0), dq_aux(1,0), dq_aux(2,0), dq_aux(3,0),	dq_aux(4,0), dq_aux(5,0), dq_aux(6,0));
	calculateDeltaT();
	checkVelocityLimits(dq);
	calculateSpeed(dq);
	_previousdQ = dq;
	_firstTime = 0;

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
			//Updates the state in the RobWork Studio
			getRobWorkStudio()->setState(_state);
			//Get the image seen from the camera and show it in the plugin
			Mat image = getImageAndShow();
			//Take the values of the first frame
			if(_firstTime){
				//Get the image seen from the camera and show it in the plugin
				if (TRACKING == 0){
					Vector3D<> Pt0 = (inverse(_device->worldTbase(_state) * _device->baseTframe(_wc->findFrame("Camera"), _state)) * _wc->findFrame("Marker")->getTransform(_state)) * Vector3D<>(0,0,0);
					Vector3D<> Pt1 = (inverse(_device->worldTbase(_state) * _device->baseTframe(_wc->findFrame("Camera"), _state)) * _wc->findFrame("Marker")->getTransform(_state)) * Vector3D<>(0.1,0,0);
					Vector3D<> Pt2 = (inverse(_device->worldTbase(_state) * _device->baseTframe(_wc->findFrame("Camera"), _state)) * _wc->findFrame("Marker")->getTransform(_state)) * Vector3D<>(0,0.1,0);
					_previousPoints[0][0] = Pt0(0)*823*2;
					_previousPoints[0][1] = Pt0(1)*823*2;
					if (NUMBER_OF_POINTS==2 || NUMBER_OF_POINTS==3) _previousPoints[1][0] = Pt1(0)*823*2;
					if (NUMBER_OF_POINTS==2 || NUMBER_OF_POINTS==3) _previousPoints[1][1] = Pt1(1)*823*2;
					if (NUMBER_OF_POINTS==3) _previousPoints[2][0] = Pt2(0)*823*2;
					if (NUMBER_OF_POINTS==3) _previousPoints[2][1] = Pt2(1)*823*2;
				}

				else {
				Point2f point;
					if (TRACKING == 1) point = colorDetection(image);
					if (TRACKING == 2) point = linesHDetection(image);
					if (TRACKING == 4) point = cornyDetection(image);
					_previousPoints[0][0] = point.x - 1024/2;
					_previousPoints[0][1] = point.y - 768/2;
				}
			}
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
			computeError();
		} else {
			log().info() << "Motion file finished!" << "\n";
			calculateAverageTime();
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
	//Button Start/Stop
	if(obj==_btn0){
		log().info() << "Button 1\n";
		// Toggle the loop on and off
		if (!_loop->isActive()) _loop->start(_spinBox->value());
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
Point2f SamplePlugin::cornyDetection (Mat & img_input)
{
	//Loads the image and the marker
	Mat img_scene;
	cvtColor(img_input, img_scene, CV_RGB2GRAY);

	Mat img_object = imread(markerPath, CV_LOAD_IMAGE_GRAYSCALE);

	//Detect the key points using SURF Detector
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

	//log().info() << "-----x:" << scene_corners[0].x << ", y:" << scene_corners[0].y << "\n";

	return scene_corners[1];
}

Point2f SamplePlugin::colorDetection(Mat & img_input){
	Mat im_thresh, im_cont, imHSV;
	Point2f centerMass, centerMasstemp, centerMassRed;
	vector<vector<Point> > contours, contoursTotal;
	vector<Vec4i> hierarchy;
	const unsigned char radiusMin = 37,radiusMax=100;

	cvtColor(img_input, imHSV, CV_BGR2HSV);
	Mat im_contFin = Mat::zeros(img_input.size(), CV_8UC3);

/*		namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

	int iLowH = 108;
	int iHighH = 120;

	int iLowS = 95;
	int iHighS = 168;

	int iLowV = 39;
	int iHighV = 143;//110

	 cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
	 cvCreateTrackbar("HighH", "Control", &iHighH, 179);

	 cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
	 cvCreateTrackbar("HighS", "Control", &iHighS, 255);

	 cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
	 cvCreateTrackbar("HighV", "Control", &iHighV, 255);*/

	//while (true) {
	for (unsigned char icounter = 0; icounter < 2; icounter++) {
		//Thresholding
		//inRange(imHSV, Scalar(iLowH, iLowS, iLowV),Scalar(iHighH, iHighS, iHighV), im_thresh); //Threshold the image
		if (icounter == 0) inRange(imHSV, Scalar(108, 253, 240), Scalar(120, 255, 255), im_thresh); //Threshold the image red circle
		else inRange(imHSV, Scalar(0, 253, 240), Scalar(12, 255, 255), im_thresh); //Threshold the image blue circles

		erode(im_thresh, im_thresh, getStructuringElement(MORPH_ELLIPSE, Size(6, 6)));
		dilate(im_thresh, im_thresh, getStructuringElement(MORPH_ELLIPSE, Size(6, 6)));

		dilate(im_thresh, im_thresh, getStructuringElement(MORPH_ELLIPSE, Size(10, 10)));
		erode(im_thresh, im_thresh, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

		//Contours
		RNG rng(12345);
		im_cont = im_thresh.clone();
		findContours(im_cont, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

		//Approximate contours and enclosing circle calculation
		vector<vector<Point> > contours_poly(contours.size());
		vector<Point2f> center(contours.size());
		vector<float> radius(contours.size());

		for (unsigned int i = 0; i < contours.size(); i++) {
			approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
			minEnclosingCircle((Mat) contours_poly[i], center[i], radius[i]);
		}



		unsigned int counter = 0;
		centerMass.x = 0;
		centerMass.y = 0;
		if (radius[0] > radiusMin && radius[0] < radiusMax) {
			centerMass = center[0];
			counter++;
		}

		for (unsigned int i = 1; i < center.size(); i++) {
			centerMasstemp = center[i];
			if (radius[i] > radiusMin && radius[i] < radiusMax) {
				centerMass.x = centerMass.x + centerMasstemp.x;
				centerMass.y = centerMass.y + centerMasstemp.y;
				counter++;
			}
		}

		//Estimating Center of the marker
		if (icounter == 0) {
			centerMassRed.x = centerMass.x / counter;
			centerMassRed.y = centerMass.y / counter;

		} else {
			centerMass.x = (centerMass.x + centerMassRed.x) / (counter+1);
			centerMass.y = (centerMass.y + centerMassRed.y) / (counter+1);
		}

		//Draw contours and circle
/*			for (unsigned int i = 0; i < contours.size(); i++) {
			if ((radius[i] > radiusMin)&&(radius[i] < radiusMax)) {
				Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255),
						rng.uniform(0, 255));
				drawContours(im_contFin, contours, i, color, 2, 8, hierarchy, 0,Point());
				drawContours(img_input, contours, i, color, 2, 8, hierarchy, 0,Point());
				circle(im_contFin, center[i], (int) radius[i], color, 2, 8, 0);
				//circle(im_contFin, mc[i], 5, 255);
			}
		circle(im_contFin, centerMass, 5, Scalar(0, 255, 0));
		}*/
	}

/*	imshow("Thresholded Image", im_thresh); //show the thresholded image
	//imshow("Original", img_input); //show the original image*
	imshow("Contoured", im_contFin); //show the original image*


	if (waitKey(30) == 27) break; //Wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
	}*/


	return centerMass;
}

Point2f SamplePlugin::linesHDetection(Mat & img_input)
{
	Mat img_gray, img_edges, thinHough, thinHoughEdit;
	vector<Vec2f> s_lines, s_lines_new;
	Point2f mc, centerMass;
	const unsigned char extra_thresh = 150;

	//Create Trackbars for Thresholds
/*	char thresh_label[50];
	sprintf(thresh_label, "Thres: %d + input", 50);
	namedWindow("LinesH", WINDOW_AUTOSIZE);
	createTrackbar(thresh_label, "LinesH", &s_trackbar, max_trackbar, thinHough);*/

	//Edge detection - Canny
	cvtColor(img_input, img_gray, COLOR_RGB2GRAY);
	Canny(img_gray, img_edges, 150, 250, 3);
	cvtColor(img_edges, thinHough, COLOR_GRAY2BGR);
	thinHoughEdit = thinHough.clone();

	//Use Standard Hough Transform
	HoughLines(img_edges, s_lines_new, 2, CV_PI / 180, 50 + extra_thresh, 0, 0);

	//Show the result
	float lactual, ldiff;

	Point pt1, pt2;

	vector<Point2f> pt;
	vector<size_t> lignored;

	//Filter ortogonal lines
	for (size_t i = 0; i < s_lines_new.size(); i++) {
		lactual = s_lines_new[i][1];

		for (size_t j = 0; j < s_lines_new.size(); j++) {
			ldiff = fabs((s_lines_new[j][1]) - lactual);
			cout << ldiff << endl;
			if (((ldiff > 1.565) && (ldiff < 1.575)) || (ldiff < 0.0005)) {
				lignored.push_back(j);
				s_lines.push_back(s_lines_new[j]);
				s_lines_new.erase(s_lines_new.begin() + j);
			}
		}
	}

	for (size_t i = 0; i < s_lines.size(); i++) {
		pt = pointExtractFromLine(s_lines[i]);

		//calculate the points from hough lines r and theta
		line(thinHoughEdit, pt[0], pt[1], Scalar(255, 0, 0), 3, CV_AA);
	}

	//Compute the intersection from the lines detected...
	vector<Point2f> intersections;
	for (size_t i = 0; i < s_lines.size(); i++) {
		for (size_t j = 0; j < s_lines.size(); j++) {
			Vec2f line1 = s_lines[i];
			Vec2f line2 = s_lines[j];
			if (isPossible(line1, line2, CV_PI / 3)) {
				Point2f intersection = calculateLineCrossing(line1, line2);
				intersections.push_back(intersection);
			}
		}

	}

	if (intersections.size() > 0) {
		vector<Point2f> intersections_new;
		vector<size_t> ignored;
		Point2f actual;
		float distance;
		for (size_t i = 0; i < intersections.size(); i++) {
			actual = intersections[i];
			ignored.push_back(i);
			for (size_t j = 1; j < intersections.size(); j++) {
				distance = pointDistance(actual, intersections[j]);
				if (distance <= 1.0) ignored.push_back(j);
			}
			//Detect clusters and filter the lonely points
			if (ignored.size() > 3) intersections_new.push_back(actual);
			ignored.clear();
		}
		centerMass = intersections[0];

		for (size_t i = 1; i < intersections_new.size(); i++) {
			mc = intersections_new[i];
			centerMass.x = centerMass.x + mc.x;
			centerMass.y = centerMass.y + mc.y;
			circle(thinHoughEdit, mc, 1, Scalar(0, 0, 255), 3);
		}
		//Estimated Centre of Marker
		centerMass.x = centerMass.x / intersections_new.size();
		centerMass.y = centerMass.y / intersections_new.size();
		circle(thinHoughEdit, centerMass, 5, 255);
	}


	imshow("LinesH", thinHough);
	imshow("LinesH processed", thinHoughEdit);

	return centerMass;
}

float SamplePlugin::pointDistance(Point2f & p, Point2f & q) {
	return cv::sqrt((p - q).x * (p - q).x + (p - q).y * (p - q).y);
}

bool SamplePlugin::isPossible(Vec2f & line_1, Vec2f & line_2, const float minAngle) {
	float theta_1 = line_1[1], theta_2 = line_2[1];
	//Check if not parallel
	if (theta_1 < minAngle) theta_1 += CV_PI;
	if (theta_2 < minAngle) theta_2 += CV_PI;

	return abs(theta_1 - theta_2) > minAngle;
}

/**
 * Calculate intersection
 */
Point2f SamplePlugin::calculateLineCrossing(Vec2f & line_1, Vec2f & line_2) {
	vector<Point2f> pt1 = pointExtractFromLine(line_1);
	vector<Point2f> pt2 = pointExtractFromLine(line_2);
	Point2f intersect;

	Point2f line1_pt1 = pt1[0], line_1_pt2 = pt1[1];
	Point2f line2_pt1 = pt2[0], line2_pt2 = pt2[1];

	float denumerator = (line1_pt1.x - line_1_pt2.x) * (line2_pt1.y - line2_pt2.y)
			- (line1_pt1.y - line_1_pt2.y) * (line2_pt1.x - line2_pt2.x);
	intersect.x =
			((line1_pt1.x * line_1_pt2.y - line1_pt1.y * line_1_pt2.x)
					* (line2_pt1.x - line2_pt2.x)
					- (line1_pt1.x - line_1_pt2.x)
							* (line2_pt1.x * line2_pt2.y
									- line2_pt1.y * line2_pt2.x)) / denumerator;
			intersect.y=((line1_pt1.x * line_1_pt2.y - line1_pt1.y * line_1_pt2.x)
					* (line2_pt1.y - line2_pt2.y)
					- (line1_pt1.y - line_1_pt2.y)
							* (line2_pt1.x * line2_pt2.y
									- line2_pt1.y * line2_pt2.x)) / denumerator;

	return intersect;
}

vector<Point2f> SamplePlugin::pointExtractFromLine(Vec2f & line) {
	vector<Point2f> ret_pt;
	float r = line[0], t = line[1];
	double cos_t, sin_t, x0, y0, alpha = 1000;
	Point2f pt1, pt2;
	//Calculate the points from hough lines r and theta
	cos_t = cos(t);
	sin_t = sin(t);
	x0 = r * cos_t;
	y0 = r * sin_t;

	pt1.x = cvRound(x0 + alpha * (-sin_t));
	pt1.y = cvRound(y0 + alpha * cos_t);
	pt2.x = cvRound(x0 - alpha * (-sin_t));
	pt2.y = cvRound(y0 - alpha * cos_t);
	ret_pt.push_back(pt1);
	ret_pt.push_back(pt2);

	return ret_pt;
}

void SamplePlugin::latexPlotQ(ofstream & file, const string & name){
	file << "\\providecommand{\\" << name << "}{\n"
		 << "\\begin{figure}[!ht]\n"
		 << "	\\centering\n"
		 //<< "	\\tikzset{every mark/.append style={scale=0.5}}\n"
		 << "	\\begin{tikzpicture}\n"
		 << "		\\begin{axis}[height=9cm, width=\\textwidth, grid=major,\n"
		 << "		xlabel={Step},ylabel={Rad}\n"
		 << "		]\n"
		 << "			\\addplot [color=Turquoise, mark=o] coordinates {\n";
		 for (unsigned int i=0; i<_qRobotVec.size(); i++) file << "				(" << i << ", " << _qRobotVec[i](0) << ")\n";
		 file << "			};\n"
		 << "			\\addlegendentry{q1}\n"
		 << "\n"
		 << "			\\addplot [color=Orchid, mark=o] coordinates {\n";
		 for (unsigned int i=0; i<_qRobotVec.size(); i++) file << "				(" << i << ", " << _qRobotVec[i](1) << ")\n";
		 file << "			};\n"
		 << "			\\addlegendentry{q2}\n"
		 << "\n"
		 << "			\\addplot [color=YellowGreen, mark=o] coordinates {\n";
		 for (unsigned int i=0; i<_qRobotVec.size(); i++) file << "				(" << i << ", " << _qRobotVec[i](2) << ")\n";
		 file << "			};\n"
		 << "			\\addlegendentry{q3}\n"
		 << "\n"
		 << "			\\addplot [color=WildStrawberry, mark=o] coordinates {\n";
		 for (unsigned int i=0; i<_qRobotVec.size(); i++) file << "				(" << i << ", " << _qRobotVec[i](3) << ")\n";
		 file << "			};\n"
		 << "			\\addlegendentry{q4}\n"
		 << "\n"
		 << "			\\addplot [color=Goldenrod, mark=o] coordinates {\n";
		 for (unsigned int i=0; i<_qRobotVec.size(); i++) file << "				(" << i << ", " << _qRobotVec[i](4) << ")\n";
		 file << "			};\n"
		 << "			\\addlegendentry{q5}\n"
		 << "\n"
		 << "			\\addplot [color=BlueGreen, mark=o] coordinates {\n";
		 for (unsigned int i=0; i<_qRobotVec.size(); i++) file << "				(" << i << ", " << _qRobotVec[i](5) << ")\n";
		 file << "			};\n"
		 << "			\\addlegendentry{q6}\n"
		 << "\n"
		 << "			\\addplot [color=CadetBlue, mark=o] coordinates {\n";
		 for (unsigned int i=0; i<_qRobotVec.size(); i++) file << "				(" << i << ", " << _qRobotVec[i](6) << ")\n";
		 file << "			};\n"
		 << "			\\addlegendentry{q7}\n"
		 << "\n"
		 << "			\\addplot[Turquoise,sharp plot,update limits=false] coordinates {"
		 << "				(0,3.14)\n"
		 << "				(" << (_speedVec.size()-1) << ",3.14)\n"
		 << "			};\n"
		 << "			\\addplot[Turquoise,sharp plot,update limits=false] coordinates {"
		 << "				(0,-3.14)\n"
		 << "				(" << (_speedVec.size()-1) << ",-3.14)\n"
		 << "			};\n"
		 << "\n"
		 << "			\\addplot[Orchid,sharp plot,update limits=false] coordinates {"
		 << "				(0,1.64)\n"
		 << "				(" << (_speedVec.size()-1) << ",1.64)\n"
		 << "			};\n"
		 << "			\\addplot[Orchid,sharp plot,update limits=false] coordinates {"
		 << "				(0,-1.64)\n"
		 << "				(" << (_speedVec.size()-1) << ",-1.64)\n"
		 << "			};\n"
		 << "\n"
		 << "			\\addplot[YellowGreen,sharp plot,update limits=false] coordinates {"
		 << "				(0,3.14)\n"
		 << "				(" << (_speedVec.size()-1) << ",3.14)\n"
		 << "			};\n"
		 << "			\\addplot[YellowGreen,sharp plot,update limits=false] coordinates {"
		 << "				(0,-3.14)\n"
		 << "				(" << (_speedVec.size()-1) << ",-3.14)\n"
		 << "			};\n"
		 << "\n"
		 << "			\\addplot[WildStrawberry,sharp plot,update limits=false] coordinates {"
		 << "				(0,2.5)\n"
		 << "				(" << (_speedVec.size()-1) << ",2.5)\n"
		 << "			};\n"
		 << "			\\addplot[WildStrawberry,sharp plot,update limits=false] coordinates {"
		 << "				(0,-2.5)\n"
		 << "				(" << (_speedVec.size()-1) << ",-2.5)\n"
		 << "			};\n"
		 << "\n"
		 << "			\\addplot[Goldenrod,sharp plot,update limits=false] coordinates {"
		 << "				(0,4.71)\n"
		 << "				(" << (_speedVec.size()-1) << ",4.71)\n"
		 << "			};\n"
		 << "			\\addplot[Goldenrod,sharp plot,update limits=false] coordinates {"
		 << "				(0,-4.71)\n"
		 << "				(" << (_speedVec.size()-1) << ",-4.71)\n"
		 << "			};\n"
		 << "\n"
		 << "			\\addplot[BlueGreen,sharp plot,update limits=false] coordinates {"
		 << "				(0,2.09)\n"
		 << "				(" << (_speedVec.size()-1) << ",2.09)\n"
		 << "			};\n"
		 << "			\\addplot[BlueGreen,sharp plot,update limits=false] coordinates {"
		 << "				(0,-2.09)\n"
		 << "				(" << (_speedVec.size()-1) << ",-2.09)\n"
		 << "			};\n"
		 << "\n"
		 << "			\\addplot[CadetBlue,sharp plot,update limits=false] coordinates {"
		 << "				(0,6.28)\n"
		 << "				(" << (_speedVec.size()-1) << ",6.28)\n"
		 << "			};\n"
		 << "			\\addplot[CadetBlue,sharp plot,update limits=false] coordinates {\n"
		 << "				(0,-6.28)\n"
		 << "				(" << (_speedVec.size()-1) << ",-6.28)\n"
		 << "			};\n"
		 << "		\\end{axis}\n"
		 << "	\\end{tikzpicture}\n"
		 << "	\\caption{Robot's state while tracking the marker at medium speed.}\n"
		 << "	\\label{fig:" << name <<  "}\n"
		 << "\\end{figure}\n"
		 << "}\n";
}

void SamplePlugin::latexPlotSpeed(ofstream & file, const string & name){
	file << "\\providecommand{\\" << name << "}{\n"
		 << "\\begin{figure}[!ht]\n"
		 << "	\\centering\n"
		 //<< "	\\tikzset{every mark/.append style={scale=0.5}}\n"
		 << "	\\begin{tikzpicture}\n"
		 << "		\\begin{axis}[height=9cm, width=\\textwidth, grid=major,\n"
		 << "		xlabel={Step},ylabel={Deg/s}\n"
		 << "		]\n"
		 << "			\\addplot [color=Turquoise, mark=o] coordinates {\n";
		 for (unsigned int i=0; i<_speedVec.size(); i++) file << "				(" << i << ", " << _speedVec[i](0) << ")\n";
		 file << "			};\n"
		 << "			\\addlegendentry{$\\dot q$1}\n"
		 << "\n"
		 << "			\\addplot [color=Orchid, mark=o] coordinates {\n";
		 for (unsigned int i=0; i<_speedVec.size(); i++) file << "				(" << i << ", " << _speedVec[i](1) << ")\n";
		 file << "			};\n"
		 << "			\\addlegendentry{$\\dot q$2}\n"
		 << "\n"
		 << "			\\addplot [color=YellowGreen, mark=o] coordinates {\n";
		 for (unsigned int i=0; i<_speedVec.size(); i++) file << "				(" << i << ", " << _speedVec[i](2) << ")\n";
		 file << "			};\n"
		 << "			\\addlegendentry{$\\dot q$3}\n"
		 << "\n"
		 << "			\\addplot [color=WildStrawberry, mark=o] coordinates {\n";
		 for (unsigned int i=0; i<_speedVec.size(); i++) file << "				(" << i << ", " << _speedVec[i](3) << ")\n";
		 file << "			};\n"
		 << "			\\addlegendentry{$\\dot q$4}\n"
		 << "\n"
		 << "			\\addplot [color=Goldenrod, mark=o] coordinates {\n";
		 for (unsigned int i=0; i<_speedVec.size(); i++) file << "				(" << i << ", " << _speedVec[i](4) << ")\n";
		 file << "			};\n"
		 << "			\\addlegendentry{$\\dot q$5}\n"
		 << "\n"
		 << "			\\addplot [color=BlueGreen, mark=o] coordinates {\n";
		 for (unsigned int i=0; i<_speedVec.size(); i++) file << "				(" << i << ", " << _speedVec[i](5) << ")\n";
		 file << "			};\n"
		 << "			\\addlegendentry{$\\dot q$6}\n"
		 << "\n"
		 << "			\\addplot [color=CadetBlue, mark=o] coordinates {\n";
		 for (unsigned int i=0; i<_speedVec.size(); i++) file << "				(" << i << ", " << _speedVec[i](6) << ")\n";
		 file << "			};\n"
		 << "			\\addlegendentry{$\\dot q$7}\n"
		 << "\n"
		 << "			\\addplot[Turquoise,sharp plot,update limits=false] coordinates {\n"
		 << "				(0,90)\n"
		 << "				(" << (_speedVec.size()-1) << ",90)\n"
		 << "			};\n"
		 << "			\\addplot[Turquoise,sharp plot,update limits=false] coordinates {\n"
		 << "				(0,-90)\n"
		 << "				(" << (_speedVec.size()-1) << ",-90)\n"
		 << "			};\n"
		 << "\n"
		 << "			\\addplot[Orchid,sharp plot,update limits=false] coordinates {\n"
		 << "				(0,94)\n"
		 << "				(" << (_speedVec.size()-1) << ",94)\n"
		 << "			};\n"
		 << "			\\addplot[Orchid,sharp plot,update limits=false] coordinates {\n"
		 << "				(0,-94)\n"
		 << "				(" << (_speedVec.size()-1) << ",-94)\n"
		 << "			};\n"
		 << "\n"
		 << "			\\addplot[YellowGreen,sharp plot,update limits=false] coordinates {\n"
		 << "				(0,180)\n"
		 << "				(" << (_speedVec.size()-1) << ",180)\n"
		 << "			};\n"
		 << "			\\addplot[YellowGreen,sharp plot,update limits=false] coordinates {\n"
		 << "				(0,-180)\n"
		 << "				(" << (_speedVec.size()-1) << ",-180)\n"
		 << "			};\n"
		 << "\n"
		 << "			\\addplot[WildStrawberry,sharp plot,update limits=false] coordinates {\n"
		 << "				(0,143)\n"
		 << "				(" << (_speedVec.size()-1) << ",143)\n"
		 << "			};\n"
		 << "			\\addplot[WildStrawberry,sharp plot,update limits=false] coordinates {\n"
		 << "				(0,-143)\n"
		 << "				(" << (_speedVec.size()-1) << ",-143)\n"
		 << "			};\n"
		 << "\n"
		 << "			\\addplot[Goldenrod,sharp plot,update limits=false] coordinates {\n"
		 << "				(0,270)\n"
		 << "				(" << (_speedVec.size()-1) << ",270)\n"
		 << "			};\n"
		 << "			\\addplot[Goldenrod,sharp plot,update limits=false] coordinates {\n"
		 << "				(0,-270)\n"
		 << "				(" << (_speedVec.size()-1) << ",-270)\n"
		 << "			};\n"
		 << "\n"
		 << "			\\addplot[BlueGreen,sharp plot,update limits=false] coordinates {\n"
		 << "				(0,120)\n"
		 << "				(" << (_speedVec.size()-1) << ",120)\n"
		 << "			};\n"
		 << "			\\addplot[BlueGreen,sharp plot,update limits=false] coordinates {\n"
		 << "				(0,-120)\n"
		 << "				(" << (_speedVec.size()-1) << ",-120)\n"
		 << "			};\n"
		 << "\n"
		 << "			\\addplot[CadetBlue,sharp plot,update limits=false] coordinates {\n"
		 << "				(0,360)\n"
		 << "				(" << (_speedVec.size()-1) << ",360)\n"
		 << "			};\n"
		 << "			\\addplot[CadetBlue,sharp plot,update limits=false] coordinates {\n"
		 << "				(0,-360)\n"
		 << "				(" << (_speedVec.size()-1) << ",-360)\n"
		 << "			};\n"
		 << "		\\end{axis}\n"
		 << "	\\end{tikzpicture}\n"
		 << "	\\caption{Robot joint's speed while tracking the marker at medium speed.}\n"
		 << "	\\label{fig:" << name <<  "}\n"
		 << "\\end{figure}\n"
		 << "}\n";
}

void SamplePlugin::latexPlotCameraPose(ofstream & file, const string & name){
	file << "\\providecommand{\\" << name << "}{\n"
		 << "\\begin{figure}[!ht]\n"
		 << "	\\centering\n"
		 //<< "	\\tikzset{every mark/.append style={scale=0.5}}\n"
		 << "	\\begin{tikzpicture}\n"
		 << "		\\begin{axis}[height=9cm, width=\\textwidth, grid=major,\n"
		 << "		xlabel={Step},ylabel={x,y,z [m] | R,P,Y [rad]}\n"
		 << "		]\n"
		 << "			\\addplot [color=Turquoise, mark=o] coordinates {\n";
		 for (unsigned int i=0; i<_qRobotVec.size(); i++) file << "				(" << i << ", " << _cameraPoseVec[i].P()(0) << ")\n";
		 file << "			};\n"
		 << "			\\addlegendentry{x}\n"
		 << "\n"
		 << "			\\addplot [color=Orchid, mark=o] coordinates {\n";
		 for (unsigned int i=0; i<_qRobotVec.size(); i++) file << "				(" << i << ", " << _cameraPoseVec[i].P()(1) << ")\n";
		 file << "			};\n"
		 << "			\\addlegendentry{y}\n"
		 << "\n"
		 << "			\\addplot [color=YellowGreen, mark=o] coordinates {\n";
		 for (unsigned int i=0; i<_qRobotVec.size(); i++) file << "				(" << i << ", " << _cameraPoseVec[i].P()(2) << ")\n";
		 file << "			};\n"
		 << "			\\addlegendentry{z}\n"
		 << "\n"
		 << "			\\addplot [color=WildStrawberry, mark=o] coordinates {\n";
		 for (unsigned int i=0; i<_qRobotVec.size(); i++) file << "				(" << i << ", " << RPY<>(_cameraPoseVec[i].R())(0) << ")\n";
		 file << "			};\n"
		 << "			\\addlegendentry{R}\n"
		 << "\n"
		 << "			\\addplot [color=Goldenrod, mark=o] coordinates {\n";
		 for (unsigned int i=0; i<_qRobotVec.size(); i++) file << "				(" << i << ", " << RPY<>(_cameraPoseVec[i].R())(1) << ")\n";
		 file << "			};\n"
		 << "			\\addlegendentry{P}\n"
		 << "\n"
		 << "			\\addplot [color=BlueGreen, mark=o] coordinates {\n";
		 for (unsigned int i=0; i<_qRobotVec.size(); i++) file << "				(" << i << ", " << RPY<>(_cameraPoseVec[i].R())(2) << ")\n";
		 file << "			};\n"
		 << "			\\addlegendentry{Y}\n"
		 << "\n"
		 << "		\\end{axis}\n"
		 << "	\\end{tikzpicture}\n"
		 << "	\\caption{Robot camera's frame while tracking the marker at medium speed.}\n"
		 << "	\\label{fig:" << name <<  "}\n"
		 << "\\end{figure}\n"
		 << "}\n";
}

void SamplePlugin::latexPlotError(ofstream & file, const string & name){
	file << "\\providecommand{\\" << name << "}{\n"
		 << "\\begin{figure}[!ht]\n"
		 << "	\\centering\n"
		 //<< "	\\tikzset{every mark/.append style={scale=0.5}}\n"
		 << "	\\begin{tikzpicture}\n"
		 << "		\\begin{axis}[height=9cm, width=\\textwidth, grid=major,\n"
		 << "		xlabel={T[s]},ylabel={Pixels}\n"
		 << "		]\n"
		 << "			\\addplot [color=Turquoise, mark=o] coordinates {\n"
		 << "				%Insert data here\n"
		 << "			};\n"
		 << "			\\addlegendentry{Error X Slow}\n"
		 << "\n"
		 << "			\\addplot [color=Orchid, mark=o] coordinates {\n"
		 << "				%Insert data here\n"
		 << "			};\n"
		 << "			\\addlegendentry{Error Y Slow}\n"
		 << "\n"
		 << "			\\addplot [color=YellowGreen, mark=o] coordinates {\n"
		 << "				%Insert data here\n"
		 << "			};\n"
		 << "			\\addlegendentry{Error X Medium}\n"
		 << "\n"
		 << "			\\addplot [color=WildStrawberry, mark=o] coordinates {\n"
		 << "				%Insert data here\n"
		 << "			};\n"
		 << "			\\addlegendentry{Error Y Medium}\n"
		 << "\n"
		 << "			\\addplot [color=Goldenrod, mark=o] coordinates {\n"
		 << "				%Insert data here\n"
		 << "			};\n"
		 << "			\\addlegendentry{Error X Fast}\n"
		 << "\n"
		 << "			\\addplot [color=BlueGreen, mark=o] coordinates {\n"
		 << "				%Insert data here\n"
		 << "			};\n"
		 << "			\\addlegendentry{Error Y Fast}\n"
		 << "\n"
		 << "		\\end{axis}\n"
		 << "	\\end{tikzpicture}\n"
		 << "	\\caption{Error in the X and Y coordinates. Tracking marker at different speeds.}\n"
		 << "	\\label{fig:" << name <<  "}\n"
		 << "\\end{figure}\n"
		 << "}\n";
}


Q_EXPORT_PLUGIN(SamplePlugin);


