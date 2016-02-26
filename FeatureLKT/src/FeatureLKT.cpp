//============================================================================
// Name        : FeatureLKT.cpp
// Author      : Jai Prakash
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================
#undef CAM
#include "cmusfm.h"

#include <opencv2/viz/vizcore.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>

using namespace cv;
using namespace std;

/**
 * @function main
 */
int main1() {
	/// Create a window
	viz::Viz3d myWindow("Coordinate Frame");

	/// Add coordinate axes
	myWindow.showWidget("Coordinate Widget", viz::WCoordinateSystem());

	/// Add line to represent (1,1,1) axis
	viz::WLine axis(Point3f(-1.0f, -1.0f, -1.0f), Point3f(1.0f, 1.0f, 1.0f));
	axis.setRenderingProperty(viz::LINE_WIDTH, 4.0);
	myWindow.showWidget("Line Widget", axis);

	while (!myWindow.wasStopped()) {
		/* Rotation using rodrigues */
		/// Rotate around (1,1,1)

		myWindow.spinOnce(1, true);
	}

	return 0;
}

int main(int argc, char** argv) {
	namedWindow("LK", 1);
	/// Create a window
	viz::Viz3d myWindow("Coordinate Frame");

	/// Add coordinate axes
	myWindow.showWidget("Coordinate Widget", viz::WCoordinateSystem());

	/// Add line to represent (1,1,1) axis
	viz::WLine axis(Point3f(-1.0f, -1.0f, -1.0f), Point3f(1.0f, 1.0f, 1.0f));
	axis.setRenderingProperty(viz::LINE_WIDTH, 4.0);
	myWindow.showWidget("Line Widget", axis);


	cmusfm *mSfm = new cmusfm();
	string prefix = "/Users/jaiprakashgogi/workspace/mscv-nea/data/temple/";
	// Allegheny_River_Crossing
	// FLT442_H_September_Woodland_Wires
	// Flying_Circus_Unit_Test_run09_A_Winter
	mSfm->readfiles(prefix);
	cout << __func__ << ": # of images = " << mSfm->no_images << endl;
	float val[] = { 1520.400000, 0.000000, 302.320000, 0.000000, 1525.900000,
			246.870000, 0.000000, 0.000000, 1.000000 };
	//float val[] = { 3636.24649 * 0.25, 0.0, 1378.989775 * 0.25, 0.0, 3621.076208
	//	* 0.25, 1135.851108 * 0.25, 0.0, 0.0, 1.0 };
	Mat K(3, 3, CV_32F, val);
	Mat K1;
	K.convertTo(K1, CV_64F);

	//Mat K(3636.24649, 0.0, 1378.989775, 0.0, 3621.076208, 1135.851108, 0.0, 0.0, 1.0);

	cout << K1 << endl;
	Mat img;

	for (int i = 0; i < mSfm->no_images - 1; i++) {
		//while (!myWindow.wasStopped()) {
		img = mSfm->showKLT(257);
		imshow("LK", img);
		cout << __func__ << i << endl;
		//cout << __func__ << mSfm->points[0].size() << " " << mSfm->points[1].size() << endl;
#if 1
		Mat F = findFundamentalMat(Mat(mSfm->points[0]), Mat(mSfm->points[1]),
		CV_FM_RANSAC, 3.f, 0.99f);
		Mat E = K1.t() * F * K1; //according to HZ (9.12)

		SVD svd(E);
		Matx33d W(0, -1, 0,   //HZ 9.13
				1, 0, 0, 0, 0, 1);
		Matx33d Winv(0, 1, 0, -1, 0, 0, 0, 0, 1);
		Mat_<double> R = svd.u * Mat(Winv) * svd.vt; //HZ 9.19
		Mat_<double> t = svd.u.col(2); //u3
		// Ref: http://stackoverflow.com/questions/12098363/testing-a-fundamental-matrix

		Mat P = K1 * Mat(Matx34d(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0));

		Mat P1 = K1
				* Mat(
						Matx34d(R(0, 0), R(0, 1), R(0, 2), t(0), R(1, 0),
								R(1, 1), R(1, 2), t(1), R(2, 0), R(2, 1),
								R(2, 2), t(2)));

		cout << P << endl;
		cout << P1 << endl;

		vector<Mat> point2DT;
		point2DT.push_back(Mat(mSfm->points[0]));
		point2DT.push_back(Mat(mSfm->points[1]));

		vector<Mat> cameraT;
		cameraT.push_back(P);
		cameraT.push_back(P1);

		Mat point3DTH, point3DT;

		triangulatePoints(P, P1, point2DT[0], point2DT[1], point3DTH);

		cout << point3DTH.rows << "x" << point3DTH.cols << endl;
		cout << point2DT[0].rows << "x" << point2DT[0].cols << endl;

		transpose(point3DTH, point3DTH);
		convertPointsFromHomogeneous(point3DTH, point3DT);
		Mat dst;
		point3DT.convertTo(dst, CV_32FC3);
		dst = dst.t();
		cout << dst.rows << dst.cols << dst.type() << endl;
		// res = res.reshape(1, 1);

#endif
		if ((char) waitKey(0) == 27) {
			break;
		}
	    /// Create a cloud widget.
	    viz::WCloud cloud_widget(dst, viz::Color::green());


	    /// Visualize widget
	    myWindow.showWidget("bunny", cloud_widget);


		myWindow.spinOnce(1, true);
		//}
	}
	cout << img.cols << "x" << img.rows << endl;

	return 0;
}
