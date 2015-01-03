/*
 * MarkerColor.cpp
 *
 *  Created on: Jan 2, 2015
 *      Author: pyc
 */

/*
 * main.cpp
 *
 *  Created on: Jan 2, 2015
 *      Author: pyc
 */

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>



using namespace cv;
using namespace std;

Mat image, imageHSV;

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

void detection(Mat imHSV) {
	Mat im_thresh, im_cont;
	namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

	int iLowH = 0;
	int iHighH = 179;

	int iLowS = 0;
	int iHighS = 255;

	int iLowV = 0;
	int iHighV = 255;
	cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
	cvCreateTrackbar("HighH", "Control", &iHighH, 179);

	cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
	cvCreateTrackbar("HighS", "Control", &iHighS, 255);

	cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
	cvCreateTrackbar("HighV", "Control", &iHighV, 255);

	while (true) {

		//inRange(imHSV, Scalar(iLowH, iLowS, iLowV),Scalar(iHighH, iHighS, iHighV), im_thresh); //Threshold the image
		//EASY MARKER
		inRange(imHSV, Scalar(61, 51, 0),Scalar(90, 255, 255), im_thresh); //Threshold the image green plate
		//inRange(imHSV, Scalar(0, 71, 0),Scalar(12, 255, 255), im_thresh); //Threshold the image red circle
		//inRange(imHSV, Scalar(111,123, 60),Scalar(132, 195, 255), im_thresh); //Threshold the image blue circles

		//HARD MARKER
		//inRange(imHSV, Scalar(0, 128, 88),Scalar(13, 219, 200), im_thresh); //Threshold the image red circle
		//inRange(imHSV, Scalar(111,135, 75),Scalar(135, 174, 137), im_thresh); //Threshold the image blue circles

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
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		RNG rng(12345);
		im_cont = im_thresh.clone();
		findContours(im_cont, contours, hierarchy, CV_RETR_TREE,
				CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

		/// Approximate contours to polygons + get bounding rects and circles
		vector<vector<Point> > contours_poly(contours.size());
		vector<Rect> boundRect(contours.size());
		vector<Point2f> center(contours.size());
		vector<float> radius(contours.size());

		for (int i = 0; i < contours.size(); i++) {
			approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
			boundRect[i] = boundingRect(Mat(contours_poly[i]));
			minEnclosingCircle((Mat) contours_poly[i], center[i], radius[i]);

		}

		vector<Moments> mu(contours.size() );
		  for( int i = 0; i < contours.size(); i++ ){
			  if (radius[i]>50)
		      mu[i] = moments( contours[i], false );
		  }

		//Mass center
		vector<Point2f> mc( contours.size() );
		  for( int i = 0; i < contours.size(); i++ ){
			  if (radius[i]>50)
		    mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
		  }


		/// Draw contours
		Mat im_contFin = Mat::zeros(im_cont.size(), CV_8UC3);
		for (int i = 0; i < contours.size(); i++) {
			if (radius[i]>50){
			Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255),
					rng.uniform(0, 255));
			drawContours(im_contFin, contours, i, color, 2, 8, hierarchy, 0,
					Point());
			rectangle(im_contFin, boundRect[i].tl(), boundRect[i].br(), color,
					2, 8, 0);
			circle(im_contFin, center[i], (int) radius[i], color, 2, 8, 0);
			circle(im_contFin,mc[i],5,255);
			}
		}

		imshow("Thresholded Image", im_thresh); //show the thresholded image
		imshow("Original", imHSV); //show the original image*
		imshow("Contoured", im_contFin); //show the original image*

		if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
				{
			cout << "esc key is pressed by user" << endl;
			break;
		}
	}
	return;

}
int main(void) {
	int cnti = 1;
	stringstream sstm;
	string scene_addr;
	for (cnti = 1; cnti < 51; cnti++) {
		sstm.str("");

		//HARD MARKER
		if (cnti < 10)		sstm<< "/home/pyc/workspace/ROVI1project/res/markers/marker_color_hard/marker_color_hard_0"<< cnti << ".png";
		else
			sstm << "/home/pyc/workspace/ROVI1project/res/markers/marker_color_hard/marker_color_hard_"	<< cnti << ".png";

		//EASY MARKER
		//if (cnti < 10)
		//	sstm << "/home/pyc/workspace/ROVI1project/res/markers/marker_color/marker_color_0"<< cnti << ".png";
		//else sstm<< "/home/pyc/workspace/ROVI1project/res/markers/marker_color/marker_color_"<< cnti << ".png";
		scene_addr = sstm.str();
		image = imread(scene_addr);
		cvtColor(image, imageHSV, CV_BGR2HSV);

//separateChannels(image);
//separateChannels(imageHSV);
//namedWindow( "Display Image", CV_WINDOW_AUTOSIZE );
//segmentation();
		detection(imageHSV);
//mark();
	}
	waitKey(0);

	return 0;
}
