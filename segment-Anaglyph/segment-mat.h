/*
Copyright (C) 2006 Pedro Felzenszwalb

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
*/

#ifndef SEGMENT_IMAGE
#define SEGMENT_IMAGE

#include <cstdlib>
#include "image.h"
#include "misc.h"
#include "filter.h"
#include "segment-graph.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include <iostream>
#include <stdio.h>

#define X_START 0
#define Y_START 0
#define MAX_WIDTH 640
#define MAX_HEIGHT 480

using namespace cv;
using namespace std;

// random color
rgb random_rgb(){ 
  rgb c;
  double r;
  
  c.r = (uchar)random();
  c.g = (uchar)random();
  c.b = (uchar)random();

  return c;
}

// dissimilarity measure between pixels
static inline float diff(image<float> *r, image<float> *g, image<float> *b,
			 int x1, int y1, int x2, int y2) {
  return sqrt(square(imRef(r, x1, y1)-imRef(r, x2, y2)) +
	      square(imRef(g, x1, y1)-imRef(g, x2, y2)) +
	      square(imRef(b, x1, y1)-imRef(b, x2, y2)));
}

/*
 * Segment an image
 *
 * Returns a color image representing the segmentation.
 *
 * im: image to segment.
 * sigma: to smooth the image.
 * c: constant for treshold function.
 * min_size: minimum component size (enforced by post-processing stage).
 * num_ccs: number of connected components in the segmentation.
 */
void segment_image_mat(Mat& img, Mat& output, float sigma, float c, int min_size, int *num_ccs, int* label) {
  int width = img.cols;
  int height = img.rows;

  vector<Mat> color; split(img,color);

  image<float> *r = new image<float>(width, height);
  image<float> *g = new image<float>(width, height);
  image<float> *b = new image<float>(width, height);

  // smooth each color channel  
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      imRef(r, x, y) = color[0].at<uchar>(y,x); //imRef(im, x, y).r;
      imRef(g, x, y) = color[1].at<uchar>(y,x); //imRef(im, x, y).g;
      imRef(b, x, y) = color[2].at<uchar>(y,x); //imRef(im, x, y).b;
    }
  }
  image<float> *smooth_r = smooth(r, sigma);
  image<float> *smooth_g = smooth(g, sigma);
  image<float> *smooth_b = smooth(b, sigma);
  delete r;
  delete g;
  delete b;
 
  // build graph
  edge *edges = new edge[width*height*4];
  int num = 0;
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      if (x < width-1) {
	edges[num].a = y * width + x;
	edges[num].b = y * width + (x+1);
	edges[num].w = diff(smooth_r, smooth_g, smooth_b, x, y, x+1, y);
	num++;
      }

      if (y < height-1) {
	edges[num].a = y * width + x;
	edges[num].b = (y+1) * width + x;
	edges[num].w = diff(smooth_r, smooth_g, smooth_b, x, y, x, y+1);
	num++;
      }

      if ((x < width-1) && (y < height-1)) {
	edges[num].a = y * width + x;
	edges[num].b = (y+1) * width + (x+1);
	edges[num].w = diff(smooth_r, smooth_g, smooth_b, x, y, x+1, y+1);
	num++;
      }

      if ((x < width-1) && (y > 0)) {
	edges[num].a = y * width + x;
	edges[num].b = (y-1) * width + (x+1);
	edges[num].w = diff(smooth_r, smooth_g, smooth_b, x, y, x+1, y-1);
	num++;
      }
    }
  }
  delete smooth_r;
  delete smooth_g;
  delete smooth_b;

  // segment
  universe *u = segment_graph(width*height, num, edges, c);

  // post process small components
  for (int i = 0; i < num; i++) {
    int a = u->find(edges[i].a);
    int b = u->find(edges[i].b);
    if ((a != b) && ((u->size(a) < min_size) || (u->size(b) < min_size)))
      u->join(a, b);
  }
  delete [] edges;
  *num_ccs = u->num_sets();

  //image<rgb> *output = new image<rgb>(width, height);

//**************************************************************************************************************************
#if 1
  //int label[width*height];
  int list[*num_ccs];
  int counter=0;
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int comp = u->find(y * width + x);
      int flag=-1;
      for (int k=0; k<= counter; k++) {
        if(comp==list[k])
          flag=k;
      }
      if(flag==-1){
        *(label+y * width + x) = counter;
        list[counter++]=comp;
	  } else {
      *(label+y * width + x)=flag;
      }
    }
  }
/*
for(int i=0; i< *(num_ccs) ; i++){
printf("\nsegment----------------------------------------  %d\n", i);
for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
		if(*(label+y * width + x) == i)
		printf("(%d, %d), ", x, y );
    }
}
}
*/
#endif
//************************************************************************************************************

  // pick random colors for each component
  rgb *colors = new rgb[width*height];
  for (int i = 0; i < width*height; i++)
    colors[i] = random_rgb();
  
  split(output,color);

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int comp = u->find(y * width + x);
      color[0].at<uchar>(y,x) = colors[comp].r;
      color[1].at<uchar>(y,x) = colors[comp].g;
      color[2].at<uchar>(y,x) = colors[comp].b;
    }
  }

  merge(color,output);
  delete [] colors;  
  delete u;

  return;
}

void find_mask(Mat& mask, int* label, int id){
	int width = mask.cols;
	int height = mask.rows;
	for(int y=0; y<height; y++){
		for(int x=0; x<width; x++){
			if( *(label + y*width + x) == id )
				mask.at<uchar>(y,x) = 255;
			else
				mask.at<uchar>(y,x) = 0;
		}
	}
	return;
}

int find_nonzero(Mat& canny_img, Mat& mask){
	//imshow("canny", canny_img);
	//imshow("mask", mask);
	Mat final; bitwise_and(mask,canny_img,final);
	//imshow("final", final);
	//waitKey(0);
	return countNonZero(final);
}


int find_transformedX(int x0, int y0, double H[3][3]){
	int temp;
	temp = (int) ( (H[0][0] * x0 + H[0][1] * y0 + H[0][2]) / \
		 		   (H[2][0] * x0 + H[2][1] * y0 + H[2][2]) );
	if (temp >= MAX_WIDTH)
		temp = MAX_WIDTH;
	else if(temp <= X_START)
		temp = X_START;
	return temp;
}

int find_transformedY(int x0, int y0, double H[3][3]){
	int temp;
	temp = (int) ( (H[1][0] * x0 + H[1][1] * y0 + H[1][2]) / \
		 		   (H[2][0] * x0 + H[2][1] * y0 + H[2][2]) );
	if (temp >= MAX_HEIGHT)
		temp = MAX_HEIGHT;
	else if(temp <= Y_START)
		temp = Y_START;
	return temp;
}


void findXYfromHomography(int* x0,int* x1,int* x2,int* x3,int* y0,int* y1,int* y2,int* y3, double H[3][3]) {
	*x0 = find_transformedX(X_START, Y_START, H);
	*y0 = find_transformedY(X_START, Y_START, H);
	*x1 = find_transformedX(MAX_WIDTH, Y_START, H);
	*y1 = find_transformedY(MAX_WIDTH, Y_START, H);
	*x2 = find_transformedX(X_START, MAX_HEIGHT, H);
	*y2 = find_transformedY(X_START, MAX_HEIGHT, H);
	*x3 = find_transformedX(MAX_WIDTH, MAX_HEIGHT, H);
	*y3 = find_transformedY(MAX_WIDTH, MAX_HEIGHT, H);

	//printf("%d - %d\n", *x0, *y0);
	//printf("%d - %d\n", *x1, *y1);
	//printf("%d - %d\n", *x2, *y2);
	//printf("%d - %d\n", *x3, *y3);
	return;
}


/**
 * @function main
 * @brief Main function
 */

// The function
Mat image_register(Mat& img1, Mat& img2){
	Mat img_object, img_scene;
	cvtColor(img1, img_object, CV_RGB2GRAY);
	cvtColor(img2, img_scene, CV_RGB2GRAY);

//-- Step 1: Detect the keypoints using SURF Detector
	int minHessian = 400;

	SurfFeatureDetector detector( minHessian );

	std::vector<KeyPoint> keypoints_object, keypoints_scene;

	detector.detect( img_object, keypoints_object );
	detector.detect( img_scene, keypoints_scene );

//-- Step 2: Calculate descriptors (feature vectors)
	SurfDescriptorExtractor extractor;

	Mat descriptors_object, descriptors_scene;

	extractor.compute( img_object, keypoints_object, descriptors_object );
	extractor.compute( img_scene, keypoints_scene, descriptors_scene );

//-- Step 3: Matching descriptor vectors using FLANN matcher
	FlannBasedMatcher matcher;
	std::vector< DMatch > matches;
	matcher.match( descriptors_object, descriptors_scene, matches );

	double max_dist = 0; double min_dist = 100;

//-- Quick calculation of max and min distances between keypoints
	for( int i = 0; i < descriptors_object.rows; i++ ) {
		double dist = matches[i].distance;
		if( dist < min_dist ) min_dist = dist;
		if( dist > max_dist ) max_dist = dist;
	}

	printf("-- Max dist : %f \n", max_dist );
	printf("-- Min dist : %f \n", min_dist );

//-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
	std::vector< DMatch > good_matches;

	for( int i = 0; i < descriptors_object.rows; i++ ) {
		if( matches[i].distance < max(2*min_dist, 0.3) ) {
			good_matches.push_back( matches[i]);
		}
	}
#if DRAW
	Mat img_matches;
	drawMatches( img_object, keypoints_object, img_scene, keypoints_scene,
               good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
	imshow("best matches", img_matches);
	//imwrite("correspondence.jpg", img_matches);
	waitKey(0);
#endif
//-- Localize the object from img_1 in img_2
	std::vector<Point2f> obj;
	std::vector<Point2f> scene;

	for( size_t i = 0; i < good_matches.size(); i++ ) {
//-- Get the keypoints from the good matches
		obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
		scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
	}

	Mat H = findHomography( obj, scene, RANSAC );
	double coeffs[3][3];
            
	double coeffsM[3*3];
	Mat coeffsMat(3, 3, CV_64F, (void *)coeffsM);
	H.convertTo(coeffsMat, coeffsMat.type());

	for(int i = 0; i < 3; ++i)
		for(int j = 0; j < 3; ++j)
			coeffs[i][j] = coeffsM[i*3+j];

	int x0,x1,x2,x3,y0,y1,y2,y3;
	findXYfromHomography(&x0,&x1,&x2,&x3,&y0,&y1,&y2,&y3, coeffs);

	vector<Point2f> src, dst;
	src.push_back(Point(x0, y0));
	src.push_back(Point(x1, y1));
	src.push_back(Point(x2, y2));
	src.push_back(Point(x3, y3));
	dst.push_back(Point(0,0));
	dst.push_back(Point(640,0));
	dst.push_back(Point(0,480));
	dst.push_back(Point(640,480));

	Mat P = getPerspectiveTransform(src,dst);

	Mat result, result_temp;
	warpPerspective(img1,result_temp, H, Size(img1.cols,img1.rows));
	//imshow("warped-img3-on-common.jpg", result_temp);
	//imwrite("warped-img3-on-common.jpg", result_temp);
	//waitKey(0);
	warpPerspective(result_temp, result, P, Size(img1.cols,img1.rows));

	//imshow("result", result);
	//waitKey(0);
	return result;
}


#endif
