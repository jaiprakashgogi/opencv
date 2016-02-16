//============================================================================
// Name        : FeatureLKT.cpp
// Author      : Jai Prakash
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include "opencv2/opencv.hpp"
//#include <stdio.h>
//#include <stdlib.h>
#include <vector>

using namespace cv;
using namespace std;

RNG rng(12345);

int main(int, char**) {
	VideoCapture cap(0); // open the default camera
	if (!cap.isOpened())  // check if we succeeded
		return -1;

	vector<Point> corners;
	double qualityLevel = 0.01;
	double minDistance = 10;
	int blockSize = 3;
	bool useHarrisDetector = false;
	int maxCorners = 100;
	double k = 0.04;
	Mat matGray;
	Mat matCopy;
	for (;;) {
		Mat frame;
		cap >> frame; // get a new frame from camera
		cvtColor(frame, matGray, COLOR_BGR2GRAY);
		  goodFeaturesToTrack( matGray,
		               corners,
		               maxCorners,
		               qualityLevel,
		               minDistance,
		               Mat(),
		               blockSize,
		               useHarrisDetector,
		               k );
		  matCopy = frame.clone();
		//GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
		//Canny(edges, edges, 0, 30, 3);
		  cout<<"** Number of corners detected: "<<corners.size()<<endl;
		  int r = 4;
		  for( int i = 0; i < corners.size(); i++ )
		     { circle( matCopy, corners[i], r, Scalar(rng.uniform(0,255), rng.uniform(0,255),
		              rng.uniform(0,255)), -1, 8, 0 ); }

		  /// Show what you got
		imshow( "tracked", matCopy );
		//imshow("edges", matGray);
		if (waitKey(30) >= 0)
			break;
	}
	// the camera will be deinitialized automatically in VideoCapture destructor
	return 0;
}
