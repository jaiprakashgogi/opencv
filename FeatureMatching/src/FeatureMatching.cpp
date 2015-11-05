//============================================================================
// Name        : FeatureMatching.cpp
// Author      : Jai Prakash
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

#if 0
Mat detectSurf(Mat& img) {
	Mat img_keypoints;
	drawKeypoints(img, keypoints, img_keypoints, Scalar::all(-1),
			DrawMatchesFlags::DEFAULT);
	return img_keypoints;
}
#endif

int main() {
	Mat img1 = imread("template.jpg");
	int minHessian = 400;
	Ptr<SURF> detector = SURF::create();
	detector->setHessianThreshold(minHessian);
	std::vector<KeyPoint> keypoints_1, keypoints_2;
	Mat descriptors_1, descriptors_2;
	detector->detectAndCompute(img1, Mat(), keypoints_1, descriptors_1);

	VideoCapture cap;
	char key = 'a';
	if (!cap.open(0))
		return 0;
	while (key != 'q') {
		Mat frame;
		cap >> frame;
		if (frame.empty()) {
			cout << "Frame is empty" << endl;
			break;
		}
		detector->detectAndCompute(frame, Mat(), keypoints_2, descriptors_2);
		  FlannBasedMatcher matcher;
		  std::vector< DMatch > matches;
		  matcher.match( descriptors_1, descriptors_2, matches );
		  double max_dist = 0; double min_dist = 100;
		  //-- Quick calculation of max and min distances between keypoints
		  for( int i = 0; i < descriptors_1.rows; i++ )
		  { double dist = matches[i].distance;
		    if( dist < min_dist ) min_dist = dist;
		    if( dist > max_dist ) max_dist = dist;
		  }
		  printf("-- Max dist : %f \n", max_dist );
		  printf("-- Min dist : %f \n", min_dist );
		  //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
		  //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
		  //-- small)
		  //-- PS.- radiusMatch can also be used here.
		  std::vector< DMatch > good_matches;
		  for( int i = 0; i < descriptors_1.rows; i++ )
		  { if( matches[i].distance <= max(2*min_dist, 0.02) )
		    { good_matches.push_back( matches[i]); }
		  }
		  //-- Draw only "good" matches
		  Mat img_matches;
		  drawMatches( img1, keypoints_1, frame, keypoints_2,
		               good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
		               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
		  //-- Show detected matches
		  imshow( "Good Matches", img_matches );
		  for( int i = 0; i < (int)good_matches.size(); i++ )
		  { printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx ); }
		key = waitKey(1);
		/*		if (key == 's'){
		 imwrite("template_jai.jpg", frame );
		 cout << "Image saved to Desktop" << endl;
		 }
		 cout << key << endl;*/
	}
	return 0;
}
