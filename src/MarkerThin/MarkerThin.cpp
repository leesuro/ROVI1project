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
#include <time.h>
#include <sys/time.h>
using namespace cv;
using namespace std;
struct timespec t2, t3;
double dt1;

Point2f computeIntersect(Vec2f line1, Vec2f line2);
vector<Point2f> pointExtractFromLine(Vec2f line);
bool acceptLinePair(Vec2f line1, Vec2f line2, float minTheta);
float pointDistance(Point2f p, Point2f q);

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

void Standard_Hough(int, void*);

int main() {
/// Read the image
	int cnti = 1;
	stringstream sstm;
	string scene_addr;
	for (cnti = 1; cnti < 51; cnti++) {
		clock_gettime(CLOCK_MONOTONIC, &t2);
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
		Canny(src_gray, edges, 150, 250, 3);
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
		clock_gettime(CLOCK_MONOTONIC, &t3);
		dt1 = (t3.tv_sec - t2.tv_sec)
				+ (double) (t3.tv_nsec - t2.tv_nsec) * 1e-9;
		cout << "elapsed time: " << dt1 << " s  " << endl;
		waitKey(0);

		//waitKey(0);
	}
	return 0;
}
void Standard_Hough(int, void*) {
	vector<Vec2f> s_lines,s_lines_new;

	cvtColor(edges, standard_hough, COLOR_GRAY2BGR);
	standard_hough_edit = standard_hough.clone();
/// 1. Use Standard Hough Transform
	HoughLines(edges, s_lines_new, 2, CV_PI / 180, min_threshold + s_trackbar, 0,
			0);


/// Show the result
	float r, t, r2, t2,lactual,ldiff;
	double cos_t, sin_t, x0, y0, alpha = 1000;
	Point pt1, pt2;

	vector<Point2f> pt;
	vector<size_t> lignored;


	//filter ortogonal lines
	for (size_t i = 0; i < s_lines_new.size(); i++) {
		lactual=s_lines_new[i][1];

			for (size_t j = 0; j < s_lines_new.size(); j++) {
				ldiff=fabs((s_lines_new[j][1])-lactual);
				cout<<ldiff<<endl;
				if(((ldiff>1.565)&&(ldiff<1.575))||(ldiff<0.001)){
					lignored.push_back(j);
					s_lines.push_back(s_lines_new[j]);
					s_lines_new.erase(s_lines_new.begin()+j);
				}
			}
	}

	//for (size_t i = 0; i < lignored.size(); i++) {
		//s_lines.push_back(s_lines_new[lignored[i]]);

	//}

	for (size_t i = 0; i < s_lines.size(); i++) {
		r = s_lines[i][0];
		t = s_lines[i][1];
		cout << "r = " << r << " pix \t theta =  " << t << " rad" << endl;
		pt = pointExtractFromLine(s_lines[i]);

		//calculate the points from hough lines r and theta

		line(standard_hough, pt[0], pt[1], Scalar(255, 0, 0), 3, CV_AA);
	}
	cout << "starting averaging" << endl;

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

		vector<Point2f> intersections_new;
		vector<size_t> ignored;
		Point2f actual;
		float distance;
		/*for (size_t i = 0; i < intersections.size(); i++) {
		 actual=intersections[i];
		 intersections_new.push_back(actual);
		 for (size_t j = i; j < intersections.size(); j++) {
		 distance = pointDistance(actual,intersections[j]);

		 //cout<<"Distance: "<<distance<<endl;
		 if (distance<18.0){
		 cout<<"Distance is smaller"<<endl;
		 }
		 else{intersections_new.push_back(intersections[j]);

		 }
		 }
		 intersections.erase();
		 }*/
		for (size_t i = 0; i < intersections.size(); i++) {

			actual = intersections[i];
			ignored.push_back(i);
			for (size_t j = 1; j < intersections.size(); j++) {
				distance = pointDistance(actual, intersections[j]);
				//cout << distance << ", ";
				if (distance <= 1.0) {
					//cout << "is the same point --";
					ignored.push_back(j);
				} else {

				}

			}
			//Detect clusters and filter the lonely points
			//cout << "\n ignored: ";
			//for (size_t k = 0; k < ignored.size(); k++) {
				//cout << ignored[k] << " ";
				if (ignored.size() > 3)
					//intersections.erase(intersections.begin() + ignored[k]);
					intersections_new.push_back(actual);
			//}
			ignored.clear();
			cout << endl;
		}
		Point2f mc, centerMass = intersections[0];

		for (size_t i = 1; i < intersections_new.size(); i++) {
			//for (i = intersections.begin(); i != intersections.end(); ++i) {
			mc = intersections_new[i];
			centerMass.x = centerMass.x + mc.x;
			centerMass.y = centerMass.y + mc.y;
			//cout << "Intersection is " << mc.x << ", " << mc.y << "  " << "  "<< endl;
			circle(standard_hough_edit, mc, 1, Scalar(0, 0, 255), 3);
		}
		//Estimated Centre of Marker
		centerMass.x = centerMass.x / intersections_new.size();
		centerMass.y = centerMass.y / intersections_new.size();
		circle(standard_hough_edit, centerMass, 5, 255);
	}

	imshow(standard_name, standard_hough);
	imshow(standard_name_edit, standard_hough_edit);
}

float pointDistance(Point2f p, Point2f q) {
	Point diff = p - q;
	return cv::sqrt(diff.x * diff.x + diff.y * diff.y);
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
	//cout << "The pt1 " << pt1.x << " " << pt1.y << " Pt2" << pt2.x << " "	<< pt2.y << endl;

	return ret_pt;
}
