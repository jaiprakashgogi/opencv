#include <iostream>
#include <fstream>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace std;
using namespace cv;

int main_readfile() {
	string image_name;
	Mat image;
	namedWindow("image");
	string prefix =
			"/Users/jaiprakashgogi/workspace/mscv-nea/data/Allegheny_River_Crossing/";
	ifstream myfile(prefix + "dataset.txt");
	if (myfile.is_open()) {
		while (getline(myfile, image_name)) {
			cout << prefix + image_name << '\n';
			image = imread(prefix + image_name);
			Mat dest;
			resize(image, dest, Size(), 0.25, 0.25);
			imshow("image", dest);
			char c = (char) waitKey(10);
			if (c == 27)
				break;
		}
		myfile.close();
	} else {
		cout << "Unable to open file";
	}
	return 0;
}
