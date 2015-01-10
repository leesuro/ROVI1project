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

Point2f calculateLineCrossing(Vec2f line_1, Vec2f line_2);
vector<Point2f> pointExtractFromLine(Vec2f line);
bool isPossible(Vec2f line_1, Vec2f line_2, float minAngle);
float pointDistance(Point2f p, Point2f q);

Point2f linesH(Mat img_input);

int main() {
/// Read the image
	Mat img_in;
	int cnti = 1;
	stringstream sstm;
	string scene_addr;
	Point2f centerOfMass;
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
		img_in = imread(scene_addr);

/// Initialize
		centerOfMass = linesH(img_in);
		clock_gettime(CLOCK_MONOTONIC, &t3);
		dt1 = (t3.tv_sec - t2.tv_sec)
				+ (double) (t3.tv_nsec - t2.tv_nsec) * 1e-9;
		cout << "elapsed time: " << dt1 << " s  " << endl;
		waitKey(0);

		//waitKey(0);
	}
	return 0;
}
Point2f linesH(Mat img_input) {
	Mat img_gray,img_edges, thinHough, thinHoughEdit;
	vector<Vec2f> s_lines, s_lines_new;
	Point2f mc, centerMass;
	const int extra_thresh=150;

	/// Create Trackbars for Thresholds
			char thresh_label[50];
			sprintf(thresh_label, "Thres: %d + input", 50);
			namedWindow("LinesH", WINDOW_AUTOSIZE);
			//createTrackbar(thresh_label, "LinesH", &s_trackbar, max_trackbar,
			//	thinHough);



	/// Edge detection - Canny
	cvtColor(img_input, img_gray, COLOR_RGB2GRAY);
	Canny(img_gray, img_edges, 150, 250, 3);
	cvtColor(img_edges, thinHough, COLOR_GRAY2BGR);
	thinHoughEdit = thinHough.clone();

/// 1. Use Standard Hough Transform
	HoughLines(img_edges, s_lines_new, 2, CV_PI / 180, 50 + extra_thresh,
			0, 0);

/// Show the result
	float r, t, lactual, ldiff;

	Point pt1, pt2;

	vector<Point2f> pt;
	vector<size_t> lignored;

	//filter ortogonal lines
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

	//for (size_t i = 0; i < lignored.size(); i++) {
	//s_lines.push_back(s_lines_new[lignored[i]]);

	//}

	for (size_t i = 0; i < s_lines.size(); i++) {
		r = s_lines[i][0];
		t = s_lines[i][1];
		cout << "r = " << r << " pix \t theta =  " << t << " rad" << endl;
		pt = pointExtractFromLine(s_lines[i]);

		//calculate the points from hough lines r and theta

		line(thinHough, pt[0], pt[1], Scalar(255, 0, 0), 3, CV_AA);
	}
	cout << "starting averaging" << endl;

	cout << "finish drawing" << endl;

	// compute the intersection from the lines detected...

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
		centerMass = intersections[0];

		for (size_t i = 1; i < intersections_new.size(); i++) {
			//for (i = intersections.begin(); i != intersections.end(); ++i) {
			mc = intersections_new[i];
			centerMass.x = centerMass.x + mc.x;
			centerMass.y = centerMass.y + mc.y;
			//cout << "Intersection is " << mc.x << ", " << mc.y << "  " << "  "<< endl;
			circle(thinHoughEdit, mc, 1, Scalar(0, 0, 255), 3);
		}
		//Estimated Centre of Marker
		centerMass.x = centerMass.x / intersections_new.size();
		centerMass.y = centerMass.y / intersections_new.size();
		circle(thinHoughEdit, centerMass, 5, 255);
	}

	imshow("LinesH", thinHough);
	imshow("LinesH processed", thinHoughEdit);
	//cout<< centerMass<<endl;
	//waitKey();
	return centerMass;
}

float pointDistance(Point2f p, Point2f q) {
	Point diff = p - q;
	return cv::sqrt(diff.x * diff.x + diff.y * diff.y);
}

bool isPossible(Vec2f line_1, Vec2f line_2, float minAngle) {
	float theta_1 = line_1[1], theta_2 = line_2[1];
	//check if not parallel
	if (theta_1 < minAngle) {
		theta_1 += CV_PI;
	}
	if (theta_2 < minAngle) {
		theta_2 += CV_PI;
	}

	return abs(theta_1 - theta_2) > minAngle;
}

// calculate intersection
Point2f calculateLineCrossing(Vec2f line_1, Vec2f line_2) {
	vector<Point2f> pt1 = pointExtractFromLine(line_1);
	vector<Point2f> pt2 = pointExtractFromLine(line_2);
	Point2f intersect;

	Point2f line1_pt1 = pt1[0], line_1_pt2 = pt1[1];
	Point2f line2_pt1 = pt2[0], line2_pt2 = pt2[1];

	float denumerator = (line1_pt1.x - line_1_pt2.x) * (line2_pt1.y - line2_pt2.y)
			- (line1_pt1.y - line_1_pt2.y) * (line2_pt1.x - line2_pt2.x);
	intersect.x=
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
