/*
 * MarkerColor.cpp
 *
 *  Created on: Jan 2, 2015
 *      Author: pyc
 */

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <time.h>
#include <sys/time.h>
struct timespec t2, t3;
double dt1;

using namespace cv;
using namespace std;

Point2f colorDetection(Mat img_input);

Mat image;

void separateChannels(Mat img) {
	Mat img_out[] = { Mat::zeros(img.size(), CV_8UC1), Mat::zeros(img.size(),
	CV_8UC1), Mat::zeros(img.size(), CV_8UC1) };

	int ch[] = { 0, 0, 1, 1, 2, 2 };
	mixChannels(&img, 1, img_out, 3, ch, 3);
	imshow("Hue", img_out[0]);
	imshow("Saturation", img_out[1]);
	imshow("Value", img_out[2]);
	cvWaitKey(0);
}

void mark() {

	const Rect region(76, 150, 70, 30);
	rectangle(image, Point(region.x, region.y),
			Point(region.x + region.width, region.y + region.height),
			Scalar(1.0, 1.0, 1.0));
	//imshow("rectangle",image);
	//Mat rect = image(region);
	//Scalar meanScalar, stdScalar;
	//meanStdDev(rect, meanScalar,stdScalar);
	//Mat_<float>mean(3,1);
	//for (int i=0;i<3;i++){
	//	mean(i,0)=meanScalar[i];
	//}
}

int main(void) {
	int cnti = 1;
	stringstream sstm;
	string scene_addr;
	Point2f markerCenter;
	for (cnti = 1; cnti < 51; cnti++) {
		clock_gettime(CLOCK_MONOTONIC, &t2);

		sstm.str("");

		//HARD MARKER
		if (cnti < 10)		sstm<< "/home/pyc/workspace/ROVI1project/res/markers/marker_color_hard/marker_color_hard_0"<< cnti << ".png";
		else
		sstm << "/home/pyc/workspace/ROVI1project/res/markers/marker_color_hard/marker_color_hard_"	<< cnti << ".png";

		//EASY MARKER
		/*if (cnti < 10)
			sstm
					<< "/home/pyc/workspace/ROVI1project/res/markers/marker_color/marker_color_0"
					<< cnti << ".png";
		else
			sstm
					<< "/home/pyc/workspace/ROVI1project/res/markers/marker_color/marker_color_"
					<< cnti << ".png";*/
		scene_addr = sstm.str();
		image = imread(scene_addr);
		markerCenter = colorDetection(image);
		clock_gettime(CLOCK_MONOTONIC, &t3);
		dt1 = (t3.tv_sec - t2.tv_sec)
				+ (double) (t3.tv_nsec - t2.tv_nsec) * 1e-9;
		cout << "elapsed time: " << dt1 << " s  " << endl;
		//waitKey(0);
//mark();
	}
	waitKey(0);

	return 0;
}

Point2f colorDetection(Mat img_input) {

	Mat im_thresh, im_cont, imHSV;
	Point2f centerMass, centerMasstemp;
	vector<vector<Point> > contours,contoursTotal;
	vector<Vec4i> hierarchy;
	float radiusMin=37;

	cvtColor(img_input, imHSV, CV_BGR2HSV);


	 namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

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
	 cvCreateTrackbar("HighV", "Control", &iHighV, 255);


	while (true) {
	//for (icounter=0;icounter<2;icounter++){

	inRange(imHSV, Scalar(iLowH, iLowS, iLowV),Scalar(iHighH, iHighS, iHighV), im_thresh); //Threshold the image
	//EASY MARKER
	//inRange(imHSV, Scalar(61, 51, 0),Scalar(90, 255, 255), im_thresh); //Threshold the image green plate
	//inRange(imHSV, Scalar(0, 128, 88), Scalar(13, 219, 200), im_thresh); //Threshold the image red circle
	//inRange(imHSV, Scalar(111,123, 60),Scalar(132, 195, 255), im_thresh); //Threshold the image blue circles
	//HARD MARKER
	//inRange(imHSV, Scalar(0, 150, 85), Scalar(8, 214, 213), im_thresh); //Threshold the image red circle
	//inRange(imHSV, Scalar(100,50, 0),Scalar(150, 180, 255), im_thresh); //Threshold the image blue circles

	//inRange(imHSV, Scalar(52,237,168),Scalar(73, 255, 255), im_thresh); //Threshold the image red circle

	//morphological opening (remove small objects from the foreground)
	erode(im_thresh, im_thresh,
			getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	dilate(im_thresh, im_thresh,
			getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

	//morphological closing (fill small holes in the foreground)
	dilate(im_thresh, im_thresh,
			getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	erode(im_thresh, im_thresh,
			getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

	//contours
	RNG rng(12345);
	im_cont = im_thresh.clone();
	findContours(im_cont, contours, hierarchy, CV_RETR_TREE,
			CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	// Approximate contours to polygons + get bounding rects and circles
	vector<vector<Point> > contours_poly(contours.size());
	vector<Rect> boundRect(contours.size());
	vector<Point2f> center(contours.size());
	vector<float> radius(contours.size());

	for (unsigned int i = 0; i < contours.size(); i++) {
		approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
		boundRect[i] = boundingRect(Mat(contours_poly[i]));
		minEnclosingCircle((Mat) contours_poly[i], center[i], radius[i]);
	}
	cout << "number of circles: " << center.size() << endl;
	vector<Moments> mu(contours.size());
	for (unsigned int i = 0; i < contours.size(); i++) {
		if (radius[i] > 50)
			mu[i] = moments(contours[i], false);
	}

	//Mass center
	vector<Point2f> mc(contours.size());
	for (unsigned int i = 0; i < contours.size(); i++) {
		if (radius[i] > radiusMin)
			mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
	}

	// Draw contours
	Mat im_contFin = Mat::zeros(im_cont.size(), CV_8UC3);
	for (unsigned int i = 0; i < contours.size(); i++) {
		if (radius[i] > radiusMin) {
			Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255),
					rng.uniform(0, 255));
			drawContours(im_contFin, contours, i, color, 2, 8, hierarchy, 0,
					Point());
			rectangle(im_contFin, boundRect[i].tl(), boundRect[i].br(), color,
					2, 8, 0);
			circle(im_contFin, center[i], (int) radius[i], color, 2, 8, 0);
			circle(im_contFin, mc[i], 5, 255);
		}
	}

	int counter = 0;
	centerMass.x=0;
	centerMass.y=0;
	if (radius[0] > radiusMin) {
		centerMass = center[0];
		counter++;
	}

	for (size_t i = 1; i < center.size(); i++) {
		//for (i = intersections.begin(); i != intersections.end(); ++i) {
		centerMasstemp = center[i];
		if (radius[i] > 30) {
			centerMass.x = centerMass.x + centerMasstemp.x;
			centerMass.y = centerMass.y + centerMasstemp.y;
			circle(im_contFin, centerMasstemp, 1, Scalar(0, 0, 255), 3);
			counter++;
		}
		cout << "Center is " << centerMasstemp.x << ", " << centerMasstemp.y
				<< "  " << "  " << endl;

	}

	//Estimated Center of the marker
	centerMass.x = centerMass.x / counter;
	centerMass.y = centerMass.y / counter;
	cout << "mass center = " << centerMass << endl;
	circle(im_contFin, centerMass, 5, Scalar(0, 255, 0));

	imshow("Thresholded Image", im_thresh); //show the thresholded image
	//imshow("Original", imHSV); //show the original image*
	imshow("Contoured", im_contFin); //show the original image*


	 if (waitKey(30) == 27) { //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
	 cout << "esc key is pressed by user" << endl;
	 break;
	 }
	}
	return centerMass;
}

