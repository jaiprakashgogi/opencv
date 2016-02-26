//============================================================================
// Name        : FeatureLKT.cpp
// Author      : Jai Prakash
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================
#undef CAM
#include "cmusfm.h"

int main(int argc, char** argv) {
	namedWindow("LK", 1);
	string prefix = "/Users/jaiprakashgogi/workspace/mscv-nea/data/FLT442_H_September_Woodland_Wires/";
	// Allegheny_River_Crossing
	// FLT442_H_September_Woodland_Wires
	// Flying_Circus_Unit_Test_run09_A_Winter
	//float val[] = { 1520.400000, 0.000000, 302.320000, 0.000000, 1525.900000,
	//		246.870000, 0.000000, 0.000000, 1.000000 };
	float val[] = { 3636.24649 * 0.25, 0.0, 1378.989775 * 0.25, 0.0, 3621.076208
		* 0.25, 1135.851108 * 0.25, 0.0, 0.0, 1.0 };

	// Initialize Viz
	viz::Viz3d myWindow("Coordinate Frame");
	myWindow.showWidget("Coordinate Widget", viz::WCoordinateSystem());
	viz::WLine axis(Point3f(-1.0f, -1.0f, -1.0f), Point3f(1.0f, 1.0f, 1.0f));
	axis.setRenderingProperty(viz::LINE_WIDTH, 1.0);
	myWindow.showWidget("Line Widget", axis);

	cmusfm *mSfm = new cmusfm();
	mSfm->readfiles(prefix);
	cout << __func__ << ": # of images = " << mSfm->no_images << endl;

	Mat K(3, 3, CV_32F, val);
	K.convertTo(K, CV_64F);
	mSfm->setIntrinsic(K);

	Mat img;

	for (int i = 0; i < mSfm->no_images - 1; i++) {
		img = mSfm->showKLT(i);
		imshow("LK", img);
		Mat point3DT = mSfm->find3D();

	    viz::WCloud cloud_widget(point3DT, viz::Color::green());
	    myWindow.showWidget("3D view", cloud_widget);
		myWindow.spinOnce(1, true);
		if ((char) waitKey(0) == 27) {
			break;
		}
	}
	return 0;
}
