//============================================================================
// Name        : FeatureLKT.cpp
// Author      : Jai Prakash
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/videoio/videoio.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <ctype.h>

using namespace cv;
using namespace std;

Point2f point;
bool addRemovePt = false;

RNG rng(12345);

int main(int argc, char** argv) {
	VideoCapture cap;
	TermCriteria termcrit(TermCriteria::COUNT | TermCriteria::EPS, 20, 0.03);
	Size subPixWinSize(10, 10), winSize(31, 31);
	const int MAX_COUNT = 500;
	bool needToInit = true;
	vector<Scalar> color;

	cap.open(0);

	if (!cap.isOpened()) {
		cout << "Could not initialize capturing...\n";
		return 0;
	}

	namedWindow("LK Demo", 1);

	Mat gray, prevGray, image, frame;
	vector<Point2f> points[2];

	for(int i= 0 ; i< MAX_COUNT ; i++){
		color.push_back(Scalar(rng.uniform(0,255), rng.uniform(0,255),rng.uniform(0,255)));
	}

	for (;;) {
		cap >> frame;
		if (frame.empty())
			break;

		frame.copyTo(image);
		cvtColor(image, gray, COLOR_BGR2GRAY);

		if (needToInit) {
			// automatic initialization
			goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.01, 10, Mat(), 3,
					0, 0.04);
			cornerSubPix(gray, points[1], subPixWinSize, Size(-1, -1),
					termcrit);
			addRemovePt = false;
		} else if (!points[0].empty()) {
			vector<uchar> status;
			vector<float> err;
			if (prevGray.empty())
				gray.copyTo(prevGray);
			calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status,
					err, winSize, 3, termcrit, 0, 0.001);
			size_t i, k;
			for (i = k = 0; i < points[1].size(); i++) {
				if (addRemovePt) {
					if (norm(point - points[1][i]) <= 5) {
						addRemovePt = false;
						continue;
					}
				}

				if (!status[i])
					continue;

				points[1][k++] = points[1][i];
				circle(image, points[1][i], 3, color.at(i), -1, 8);
			}
			points[1].resize(k);
		}

		if (addRemovePt && points[1].size() < (size_t) MAX_COUNT) {
			vector<Point2f> tmp;
			tmp.push_back(point);
			cornerSubPix(gray, tmp, winSize, Size(-1, -1), termcrit);
			points[1].push_back(tmp[0]);
			addRemovePt = false;
		}

		needToInit = false;
		imshow("LK Demo", image);

		char c = (char) waitKey(10);
		if (c == 27)
			break;
		switch (c) {
		case 'r':
			needToInit = true;
			break;
		case 'c':
			points[0].clear();
			points[1].clear();
			break;
		}

		std::swap(points[1], points[0]);
		cv::swap(prevGray, gray);
	}

	return 0;
}
