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
	cmusfm *mSfm = new cmusfm();
	string prefix =
			"/Users/jaiprakashgogi/workspace/mscv-nea/data/FLT442_H_September_Woodland_Wires/";
	// Allegheny_River_Crossing
	// FLT442_H_September_Woodland_Wires
	// Flying_Circus_Unit_Test_run09_A_Winter
	mSfm->readfiles(prefix);
	cout << __func__ << ": # if images = " << mSfm->no_images << endl;

	for (int i = 0; i < mSfm->no_images - 1; i++) {
		imshow("LK", mSfm->showKLT(i));
		if ((char) waitKey(10) == 27){
			break;
		}
	}

	return 0;
}
