/*
 * MarkerThin.cpp
 *
 *  Created on: Jan 3, 2015
 *      Author: pyc
 */

/**
 * @file HoughLines_Demo.cpp
 * @brief Demo code for Hough Transform
 * @author OpenCV team
 */
//#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <math.h>
using namespace cv;
using namespace std;

Point2f computeIntersect(Vec2f line1, Vec2f line2);
vector<Point2f> pointExtractFromLine(Vec2f line);
bool acceptLinePair(Vec2f line1, Vec2f line2, float minTheta);

/// Global variables

Mat src, edges;
Mat src_gray;
Mat standard_hough, probabilistic_hough, standard_hough_edit;
int min_threshold = 50;
int max_trackbar = 150;

const char* standard_name = "Standard Hough Lines Demo";
const char* standard_name_edit = "Edit Standard Hough Lines Demo";
const char* probabilistic_name = "Probabilistic Hough Lines Demo";
int s_trackbar = max_trackbar;
int p_trackbar = max_trackbar;
/// Function Headers
void help();
void Standard_Hough(int, void*);
void Probabilistic_Hough(int, void*);

int main() {
/// Read the image
	int cnti = 1;
	stringstream sstm;
	string scene_addr;
	for (cnti = 1; cnti < 51; cnti++) {
		sstm.str("");

		//HARD MARKER
		//if (cnti < 10)
		//	sstm << "/home/pyc/workspace/ROVI1project/res/markers/marker_thinline_hard/marker_thinline_hard_0" << cnti << ".png";
		//else sstm << "/home/pyc/workspace/ROVI1project/res/markers/marker_thinline_hard/marker_thinline_hard_" << cnti << ".png";

		//EASY MARKER
		if (cnti < 10)
			sstm
					<< "/home/pyc/workspace/ROVI1project/res/markers/marker_thinline/marker_thinline_0"
					<< cnti << ".png";
		else
			sstm
					<< "/home/pyc/workspace/ROVI1project/res/markers/marker_thinline/marker_thinline_"
					<< cnti << ".png";
		scene_addr = sstm.str();
		src = imread(scene_addr);
/// Pass the image to gray
		cvtColor(src, src_gray, COLOR_RGB2GRAY);
/// Apply Canny edge detector
		Canny(src_gray, edges, 60, 180, 3);
/// Create Trackbars for Thresholds
		char thresh_label[50];
		sprintf(thresh_label, "Thres: %d + input", min_threshold);
		namedWindow(standard_name, WINDOW_AUTOSIZE);
		createTrackbar(thresh_label, standard_name, &s_trackbar, max_trackbar,
				Standard_Hough);
		//namedWindow(probabilistic_name, WINDOW_AUTOSIZE);
		//createTrackbar(thresh_label, probabilistic_name, &p_trackbar, max_trackbar,
		//		Probabilistic_Hough);
/// Initialize
		Standard_Hough(0, 0);
		Probabilistic_Hough(0, 0);
		waitKey(0);
	}
	return 0;
}
void Standard_Hough(int, void*) {
	vector<Vec2f> s_lines;

	cvtColor(edges, standard_hough, COLOR_GRAY2BGR);
	standard_hough_edit = standard_hough.clone();
/// 1. Use Standard Hough Transform
	HoughLines(edges, s_lines, 2, CV_PI / 180, min_threshold + s_trackbar, 0,
			0);

/// Show the result
	float r, t, r2, t2;
	double cos_t, sin_t, x0, y0, alpha = 1000;
	Point pt1, pt2;

	vector<Point2f> pt;
	for (size_t i = 0; i < s_lines.size(); i++) {
		r = s_lines[i][0];
		t = s_lines[i][1];
		cout << "r = " << r << " pix \t theta = t" << t << " rad" << endl;
		pt = pointExtractFromLine(s_lines[i]);

		//calculate the points from hough lines r and theta

		line(standard_hough, pt[0], pt[1], Scalar(255, 0, 0), 3, CV_AA);
	}
	cout << "starting averaging" << endl;
	//Average multiple lines at the same place
	float eps = 5.0, t_eps = (CV_PI / 180.0) * 5.0;
	vector<Vec2f> s_lines_edit = s_lines;

	size_t cnt = 0;

	/*for (size_t i = 0; i < s_lines.size(); i++) {
	 r=s_lines[i][0];
	 t=s_lines[i][1];

	 for (size_t j = i+1 ; j < s_lines.size()-1; j++) {
	 r2=s_lines[j][0];
	 t2=s_lines[j][1];
	 cout<< i<<"  "<< j<<" r: "<< r<<" r2 "<<r2<<"  "<<"r-r2 = "<<r-r2<<" ";


	 if ((((((r-r2))<eps)&&(((r-r2))>0))||((((r-r2))>-eps)&&(((r-r2))<0)))){
	 cout<<"same line found "<< (s_lines[i][0]-s_lines[j][0])<<endl;
	 }
	 else{cout<<"different line found"<<endl;

	 s_lines_edit[cnt][0]=r;
	 s_lines_edit[cnt][1]=t;
	 cnt++;
	 }
	 }
	 }*/
	cout << "finish averaging" << endl;
	//Draw edited lines at the same place
	/*for (size_t i = 0; i < s_lines_edit.size(); i++) {
	 //for (size_t j = i + 1; j < (s_lines.size() - 1); j++) {
	 r = s_lines_edit[i][0]; t = s_lines_edit[i][1];
	 //cout << "r = " << r << " pix \t theta = t" << t << " rad" << endl;
	 cos_t = cos(t); sin_t = sin(t);
	 x0 = r * cos_t; y0 = r * sin_t;

	 pt1.x = cvRound(x0 + alpha * (-sin_t));
	 pt1.y = cvRound(y0 + alpha * cos_t);
	 pt2.x = cvRound(x0 - alpha * (-sin_t));
	 pt2.y = cvRound(y0 - alpha * cos_t);

	 line(standard_hough_edit, pt1, pt2, Scalar(255, 0, 0), 3, CV_AA);
	 //}
	 }*//*	 vector<Point2f> p1 = pointExtractFromLine(line1);
	 vector<Point2f> p2 = pointExtractFromLine(line2);

	 float denom = (p1[0].x - p1[1].x)*(p2[0].y - p2[1].y) - (p1[0].y - p1[1].y)*(p2[0].x - p2[1].x);
	 Point2f intersect(((p1[0].x*p1[1].y - p1[0].y*p1[1].x)*(p2[0].x - p2[1].x) -
	 (p1[0].x - p1[1].x)*(p2[0].x*p2[1].y - p2[0].y*p2[1].x)) / denom,
	 ((p1[0].x*p1[1].y - p1[0].y*p1[1].x)*(p2[0].y - p2[1].y) -
	 (p1[0].y - p1[1].y)*(p2[0].x*p2[1].y - p2[0].y*p2[1].x)) / denom);
	 */
	cout << "finish drawing" << endl;

	// compute the intersection from the lines detected...

	vector<Point2f> intersections;
	for (size_t i = 0; i < s_lines.size(); i++) {
		for (size_t j = 0; j < s_lines.size(); j++) {
			Vec2f line1 = s_lines[i];
			Vec2f line2 = s_lines[j];
			if (acceptLinePair(line1, line2, CV_PI / 3)) {
				Point2f intersection = computeIntersect(line1, line2);
				intersections.push_back(intersection);
			}
		}

	}

	if (intersections.size() > 0) {
		vector<Point2f>::iterator i;
		double res;
		for (i = intersections.begin(); i != intersections.end(); ++i) {

			res = cv::norm(*i - *(i + 1));
			cout << "Intersection is " << i->x << ", " << i->y << " distance  "
					<< res << "  " << endl;
			circle(standard_hough_edit, *i, 1, Scalar(0, 0, 255), 3);
		}
	}
	for (size_t i = 0; i < s_lines.size(); i++) {
		for (size_t j = 0; j < s_lines.size(); j++) {

		}
	}
	imshow(standard_name, standard_hough);
	imshow(standard_name_edit, standard_hough_edit);
}

void Probabilistic_Hough(int, void*) {

	/*vector<Vec4i> p_lines;*
	 cvtColor(edges, probabilistic_hough, COLOR_GRAY2BGR);
	 /// 2. Use Probabilistic Hough Transform
	 HoughLinesP(edges, p_lines, 1, CV_PI / 180, min_threshold + p_trackbar, 30,
	 10);
	 /// Show the result
	 for (size_t i = 0; i < p_lines.size(); i++) {
	 Vec4i l = p_lines[i];
	 line(probabilistic_hough, Point(l[0], l[1]), Point(l[2], l[3]),
	 Scalar(255, 0, 0), 3, CV_AA);
	 }
	 imshow(probabilistic_name, probabilistic_hough);*/
}

bool acceptLinePair(Vec2f line1, Vec2f line2, float minTheta) {
	float theta1 = line1[1], theta2 = line2[1];

	if (theta1 < minTheta) {
		theta1 += CV_PI; // dealing with 0 and 180 ambiguities...
	}

	if (theta2 < minTheta) {
		theta2 += CV_PI; // dealing with 0 and 180 ambiguities...
	}

	return abs(theta1 - theta2) > minTheta;
}

// calculate intersection
Point2f computeIntersect(Vec2f line1, Vec2f line2) {
	vector<Point2f> pt1 = pointExtractFromLine(line1);
	vector<Point2f> pt2 = pointExtractFromLine(line2);

	Point2f line1_pt1 = pt1[0], line_1_pt2 = pt1[1];
	Point2f line2_pt1 = pt2[0], line2_pt2 = pt2[1];

	float denom = (line1_pt1.x - line_1_pt2.x) * (line2_pt1.y - line2_pt2.y)
			- (line1_pt1.y - line_1_pt2.y) * (line2_pt1.x - line2_pt2.x);
	Point2f intersect(
			((line1_pt1.x * line_1_pt2.y - line1_pt1.y * line_1_pt2.x)
					* (line2_pt1.x - line2_pt2.x)
					- (line1_pt1.x - line_1_pt2.x)
							* (line2_pt1.x * line2_pt2.y
									- line2_pt1.y * line2_pt2.x)) / denom,
			((line1_pt1.x * line_1_pt2.y - line1_pt1.y * line_1_pt2.x)
					* (line2_pt1.y - line2_pt2.y)
					- (line1_pt1.y - line_1_pt2.y)
							* (line2_pt1.x * line2_pt2.y
									- line2_pt1.y * line2_pt2.x)) / denom);

	/*	 vector<Point2f> p1 = pointExtractFromLine(line1);
	 vector<Point2f> p2 = pointExtractFromLine(line2);

	 float denom = (p1[0].x - p1[1].x)*(p2[0].y - p2[1].y) - (p1[0].y - p1[1].y)*(p2[0].x - p2[1].x);
	 Point2f intersect(((p1[0].x*p1[1].y - p1[0].y*p1[1].x)*(p2[0].x - p2[1].x) -
	 (p1[0].x - p1[1].x)*(p2[0].x*p2[1].y - p2[0].y*p2[1].x)) / denom,
	 ((p1[0].x*p1[1].y - p1[0].y*p1[1].x)*(p2[0].y - p2[1].y) -
	 (p1[0].y - p1[1].y)*(p2[0].x*p2[1].y - p2[0].y*p2[1].x)) / denom);
	 */
	return intersect;
}

vector<Point2f> pointExtractFromLine(Vec2f line) {
	vector<Point2f> ret_pt;
	float r = line[0], t = line[1];
	double cos_t, sin_t, x0, y0, alpha = 1000;
	Point2f pt1, pt2;
	//calculate the points from hough lines r and theta
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
	cout << "The pt1 " << pt1.x << " " << pt1.y << " Pt2" << pt2.x << " "
			<< pt2.y << endl;

	return ret_pt;
}
