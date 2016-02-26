/*
 * cmusfm.cpp
 *
 *  Created on: Feb 24, 2016
 *      Author: jaiprakashgogi
 */

#include "cmusfm.h"

cmusfm::cmusfm() :
		no_images(0), MAX_COUNT(500) {
	RNG rng(12345);
	for (int i = 0; i < MAX_COUNT; i++) {
		color.push_back(
				Scalar(rng.uniform(0, 255), rng.uniform(0, 255),
						rng.uniform(0, 255)));
	}

	cout << __func__ << endl;
}

cmusfm::~cmusfm() {
	// TODO Auto-generated destructor stub
}

void cmusfm::readfiles( string prefix) {
	int count = 0;
	string image_name;
	ifstream myfile(prefix + "dataset.txt");
	while (getline(myfile, image_name)) {
		filenames.push_back(prefix + image_name);
		count++;
	}
	no_images = count;
	myfile.close();
}

Mat cmusfm::showKLT(int idx){
	if(idx<0 || idx>no_images -1) {
		cout << __func__ << ": File size exceeded" << endl;
	}

	TermCriteria termcrit(TermCriteria::COUNT | TermCriteria::EPS, 20, 0.03);
	Size subPixWinSize(10, 10), winSize(31, 31);

	Mat gray1, gray2;
	Mat im1 = imread(filenames.at(idx));
	Mat im2 = imread(filenames.at(idx+1));
#if RESIZE
	Mat frame1, frame2;
	resize(im1, frame1, Size(), 0.25, 0.25);
	resize(im2, frame2, Size(), 0.25, 0.25);
	cvtColor(frame1, gray1, COLOR_BGR2GRAY);
	cvtColor(frame2, gray2, COLOR_BGR2GRAY);
#else
	cvtColor(im1, gray1, COLOR_BGR2GRAY);
	cvtColor(im2, gray2, COLOR_BGR2GRAY);
#endif
	vector<uchar> status;
	vector<float> err;

	goodFeaturesToTrack(gray1, points[0], MAX_COUNT, 0.01, 10, Mat(), 3,
			0, 0.04);
	cornerSubPix(gray1, points[0], subPixWinSize, Size(-1, -1),
			termcrit);
	calcOpticalFlowPyrLK(gray1, gray2, points[0], points[1], status,
			err, winSize, 3, termcrit, 0, 0.001);
	size_t i, k;
	for (i = k = 0; i < points[1].size(); i++) {
		//if (!status[i])
		//	continue;

		points[1][k++] = points[1][i];
#if RESIZE
		line(frame2, points[0][i], points[1][i], color.at(i), 1, 1);
		circle(frame2, points[1][i], 3, color.at(i), 1, 4);
#else
		line(im2, points[0][i], points[1][i], color.at(i), 1, 1);
		//circle(im2, points[1][i], 3, color.at(i), 1, 4);
#endif
	}
	points[1].resize(k);
#if RESIZE
	return frame2;
#else
	return im2;
#endif

}
