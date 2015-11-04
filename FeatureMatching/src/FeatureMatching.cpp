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

Mat detectSurf(Mat& img) {
	int minHessian = 400;
	 Ptr<SURF> detector = SURF::create( minHessian );
	std::vector<KeyPoint> keypoints;

	detector->detect(img, keypoints);
	Mat img_keypoints;
	drawKeypoints(img, keypoints, img_keypoints, Scalar::all(-1),
			DrawMatchesFlags::DEFAULT);
	return img_keypoints;
}

int main() {
	VideoCapture cap;
	char key = 'a';
	if (!cap.open(0))
		return 0;
	while (key != 'q') {
		Mat frame;
		cap >> frame;
		if (frame.empty()){
			cout << "Frame is empty" << endl;
			break;
		}
		imshow("window", frame);
		Mat surf = detectSurf(frame);
		imshow("surf", surf);
		key = waitKey(1);
		cout << key << endl;
	}
	return 0;
}
