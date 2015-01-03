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
/*#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
using namespace cv;
using namespace std;
/// Global variables

Mat src, edges;
Mat src_gray;
Mat standard_hough, probabilistic_hough;
int min_threshold = 50;
int max_trackbar = 150;

const char* standard_name = "Standard Hough Lines Demo";
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
		//for (cnti = 1; cnti < 51; cnti++) {
			sstm.str("");

			//HARD MARKER
			//if (cnti < 10)
			//	sstm << "/home/pyc/workspace/ROVI1project/res/markers/marker_thinline_hard/marker_thinline_hard_0" << cnti << ".png";
			//else sstm << "/home/pyc/workspace/ROVI1project/res/markers/marker_thinline_hard/marker_thinline_hard_" << cnti << ".png";

			//EASY MARKER
			if (cnti < 10)
				sstm << "/home/pyc/workspace/ROVI1project/res/markers/marker_thinline/marker_thinline_0"<< cnti << ".png";
			else sstm<< "/home/pyc/workspace/ROVI1project/res/markers/marker_thinline/marker_thinline_"<< cnti << ".png";
			scene_addr = sstm.str();
			src = imread(scene_addr);
/// Pass the image to gray
	cvtColor(src, src_gray, COLOR_RGB2GRAY);
/// Apply Canny edge detector
	Canny(src_gray, edges, 50, 200, 3);
/// Create Trackbars for Thresholds
	char thresh_label[50];
	sprintf(thresh_label, "Thres: %d + input", min_threshold);
	namedWindow(standard_name, WINDOW_AUTOSIZE);
	createTrackbar(thresh_label, standard_name, &s_trackbar, max_trackbar,
			Standard_Hough);
	namedWindow(probabilistic_name, WINDOW_AUTOSIZE);
	createTrackbar(thresh_label, probabilistic_name, &p_trackbar, max_trackbar,
			Probabilistic_Hough);
/// Initialize
	Standard_Hough(0, 0);
	Probabilistic_Hough(0, 0);
	waitKey(0);
	return 0;
}
void Standard_Hough(int, void*) {
	vector<Vec2f> s_lines;
	cvtColor(edges, standard_hough, COLOR_GRAY2BGR);
/// 1. Use Standard Hough Transform
	HoughLines(edges, s_lines, 1, CV_PI / 180, min_threshold + s_trackbar, 0,
			0);
/// Show the result
	for (size_t i = 0; i < s_lines.size(); i++) {
		float r = s_lines[i][0], t = s_lines[i][1];
		double cos_t = cos(t), sin_t = sin(t);
		double x0 = r * cos_t, y0 = r * sin_t;
		double alpha = 1000;
		Point pt1(cvRound(x0 + alpha * (-sin_t)), cvRound(y0 + alpha * cos_t));
		Point pt2(cvRound(x0 - alpha * (-sin_t)), cvRound(y0 - alpha * cos_t));
		line(standard_hough, pt1, pt2, Scalar(255, 0, 0), 3, CV_AA);
	}
	imshow(standard_name, standard_hough);
}

void Probabilistic_Hough(int, void*) {
	vector<Vec4i> p_lines;
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
	imshow(probabilistic_name, probabilistic_hough);
}
*/

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;
using namespace std;

/// Global variables
Mat src, src_gray;
int thresh = 200;
int max_thresh = 255;

char* source_window = "Source image";
char* corners_window = "Corners detected";

/// Function header
void cornerHarris_demo( int, void* );

/** @function main */
int main( int argc, char** argv )
{
  /// Load source image and convert it to gray
  src = imread("/home/pyc/workspace/ROVI1project/res/markers/marker_thinline/marker_thinline_01.png");
  cvtColor( src, src_gray, CV_BGR2GRAY );

  /// Create a window and a trackbar
  namedWindow( source_window, CV_WINDOW_AUTOSIZE );
  createTrackbar( "Threshold: ", source_window, &thresh, max_thresh, cornerHarris_demo );
  imshow( source_window, src );

  cornerHarris_demo( 0, 0 );

  waitKey(0);
  return(0);
}

/** @function cornerHarris_demo */
void cornerHarris_demo( int, void* )
{

  Mat dst, dst_norm, dst_norm_scaled;
  dst = Mat::zeros( src.size(), CV_32FC1 );

  /// Detector parameters
  int blockSize = 2;
  int apertureSize = 3;
  double k = 0.04;

  /// Detecting corners
  cornerHarris( src_gray, dst, blockSize, apertureSize, k, BORDER_DEFAULT );

  /// Normalizing
  normalize( dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
  convertScaleAbs( dst_norm, dst_norm_scaled );

  /// Drawing a circle around corners
  for( int j = 0; j < dst_norm.rows ; j++ )
     { for( int i = 0; i < dst_norm.cols; i++ )
          {
            if( (int) dst_norm.at<float>(j,i) > thresh )
              {
               circle( dst_norm_scaled, Point( i, j ), 5,  Scalar(0), 2, 8, 0 );
              }
          }
     }
  /// Showing the result
  namedWindow( corners_window, CV_WINDOW_AUTOSIZE );
  imshow( corners_window, dst_norm_scaled );
}
