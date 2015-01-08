/*
 * MarkerCorny.cpp
 *
 *  Created on: Jan 1, 2015
 *      Author: pyc
 */

#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include <time.h>
#include <sys/time.h>

using namespace cv;
using namespace std;


void readme();

/** @function main */
int main() {
	std::string scene_addr;
	std::stringstream sstm;
	 struct timespec t2, t3;
	    double dt1;
	int cnti;
	Mat img_object =
			imread(
					"/home/pyc/workspace/ROVI1project/src/SamplePluginPA10/markers/Marker3.ppm",
					CV_LOAD_IMAGE_GRAYSCALE);
	Mat img_scene;

	for (cnti = 1; cnti < 50; cnti++) {
		  clock_gettime(CLOCK_MONOTONIC,  &t2);
		sstm.str("");
		std::cout << cnti << std::endl;
		if (cnti < 10)
			sstm
					<< "/home/pyc/workspace/ROVI1project/res/markers/marker_corny_hard/marker_corny_hard_0"
					<< cnti << ".png";
		else
			sstm
					<< "/home/pyc/workspace/ROVI1project/res/markers/marker_corny_hard/marker_corny_hard_"
					<< cnti << ".png";

		//if (cnti<10) sstm << "/home/pyc/workspace/ROVI1project/res/markers/marker_corny/marker_corny_0"<<cnti<<".png";
		//	else sstm << "/home/pyc/workspace/ROVI1project/res/markers/marker_corny/marker_corny_"<<cnti<<".png";

		scene_addr = sstm.str();
		cout << scene_addr << endl;

		img_scene = imread(scene_addr, CV_LOAD_IMAGE_GRAYSCALE);


		if (!img_object.data || !img_scene.data) {
			std::cout << " --(!) Error reading images " << std::endl;
			return -1;
		}

		//Detect the keypoints using SURF Detector
		int minHessian = 1000;

		SurfFeatureDetector detector(minHessian);

		std::vector<KeyPoint> keypoints_object, keypoints_scene;

		detector.detect(img_object, keypoints_object);
		detector.detect(img_scene, keypoints_scene);

		//Calculate descriptors (feature vectors)
		SurfDescriptorExtractor extractor;

		Mat descriptors_object, descriptors_scene;

		extractor.compute(img_object, keypoints_object, descriptors_object);
		extractor.compute(img_scene, keypoints_scene, descriptors_scene);

		// Matching descriptor vectors using FLANN matcher
		FlannBasedMatcher matcher;
		std::vector<DMatch> matches;
		matcher.match(descriptors_object, descriptors_scene, matches);

		double max_dist = 0;
		double min_dist = 100;

		//-- Quick calculation of max and min distances between keypoints
		for (int i = 0; i < descriptors_object.rows; i++) {
			double dist = matches[i].distance;
			if (dist < min_dist)
				min_dist = dist;
			if (dist > max_dist)
				max_dist = dist;
		}

		printf("-- Max dist : %f \n", max_dist);
		printf("-- Min dist : %f \n", min_dist);

		//-- Draw only good matches 
		std::vector<DMatch> good_matches;

		for (int i = 0; i < descriptors_object.rows; i++) {
			if (matches[i].distance < 3 * min_dist) {
				good_matches.push_back(matches[i]);
			}
		}

		Mat img_matches=img_scene;
		//drawMatches(img_object, keypoints_object, img_scene, keypoints_scene,
			//	good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
				//vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

		//-- Localize the object
		std::vector<Point2f> obj;
		std::vector<Point2f> scene;

		for (unsigned int i = 0; i < good_matches.size(); i++) {
			//-- Get the keypoints from the good matches
			obj.push_back(keypoints_object[good_matches[i].queryIdx].pt);
			scene.push_back(keypoints_scene[good_matches[i].trainIdx].pt);
		}

		Mat H = findHomography(obj, scene, CV_RANSAC);

		//-- Get the corners from the image_1 ( the object to be "detected" )
		std::vector<Point2f> obj_corners(4);
		obj_corners[0] = cvPoint(0, 0);
		obj_corners[1] = cvPoint(img_object.cols, 0);
		obj_corners[2] = cvPoint(img_object.cols, img_object.rows);
		obj_corners[3] = cvPoint(0, img_object.rows);
		std::vector<Point2f> scene_corners(4);

		perspectiveTransform(obj_corners, scene_corners, H);

		//-- Draw lines between the corners (the mapped object in the scene - image_2 )
		//line(img_matches, scene_corners[0],	scene_corners[1] , Scalar(0, 255, 0), 4);
		//line(img_matches, scene_corners[1],	scene_corners[2] , Scalar(0, 255, 0), 4);
		//line(img_matches, scene_corners[2],	scene_corners[3] , Scalar(0, 255, 0), 4);
		//line(img_matches, scene_corners[3],	scene_corners[0] , Scalar(0, 255, 0), 4);

		circle(img_matches,scene_corners[0],5, Scalar(0,255,0));
		//-- Show detected matches
		imshow("Good Matches & Object detection", img_matches);
		clock_gettime(CLOCK_MONOTONIC,  &t3);
		 dt1 = (t3.tv_sec - t2.tv_sec) + (double) (t3.tv_nsec - t2.tv_nsec) * 1e-9;
		cout << "elapsed time: " << dt1		<< " s  "  << endl;
		waitKey(0);

	}
	return 0;
}

/** @function readme */
void readme() {
	std::cout << " Usage: ./SURF_descriptor <img1> <img2>" << std::endl;
}
