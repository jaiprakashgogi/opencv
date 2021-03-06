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

#include <cstdio>
#include <cstdlib>
#include <image.h>
#include <misc.h>
#include <pnmfile.h>
#include "segment-mat.h"

int main(int argc, char **argv) {
  if (argc != 9) {
    fprintf(stderr, "usage: %s sigma k min input(jpeg) max_shift\n", argv[0]);
    return 1;
  }
  
  float sigma = atof(argv[1]);
  float k = atof(argv[2]);
  int min_size = atoi(argv[3]);
	int max_shift = atoi(argv[8]);

	Mat img1 = imread(argv[4], IMREAD_COLOR);
	Mat img2 = imread(argv[5], IMREAD_COLOR);
	Mat img3 = imread(argv[6], IMREAD_COLOR);
	Mat img4 = imread(argv[7], IMREAD_COLOR);

	Mat img12 = image_register(img1, img2);
	Mat img34 = image_register(img3, img4);
	Mat img1234 = image_register(img12, img34);

	Mat out1 = image_register(img1, img1234);
	Mat out2 = image_register(img2, img1234);
	Mat out3 = image_register(img3, img1234);
	Mat out4 = image_register(img4, img1234);
#if DRAW
	imshow("input1", img1);
	imshow("input2", img2);
	imshow("input3", img3);
	imshow("input4", img4);
	imshow("registered1.jpg", out1);
	imshow("registered2.jpg", out2);
	imshow("registered3.jpg", out3);
	imshow("registered4.jpg", out4);
	imshow("common.jpg", img1234);
	waitKey(0);
#endif
	
  printf("loading input image.\n");
	Mat gray_img1, gray_img2, gray_img3, gray_img4;
	cvtColor(out1, gray_img1, CV_RGB2GRAY);
	cvtColor(out2, gray_img2, CV_RGB2GRAY);
	cvtColor(out3, gray_img3, CV_RGB2GRAY);
	cvtColor(out4, gray_img4, CV_RGB2GRAY);
  Canny(gray_img1, gray_img1, 5, 50, 7); // min_thres, max_thres, aperture. use aperture 5/7 for more edges
  Canny(gray_img2, gray_img2, 5, 50, 7);
  Canny(gray_img3, gray_img3, 5, 50, 7);
  Canny(gray_img4, gray_img4, 5, 50, 7);

	int width = img1.cols;
	int height = img1.rows;

  //Mat input = imread(argv[6], IMREAD_COLOR);
	//img3.copyTo(input);
#if DRAW
  imshow("input", out3);	//-> hardcoded here
  waitKey(0);
#endif
  printf("processing\n");
  int num_ccs;
  Mat output(height, width, CV_8UC3);
  int *label;
  label = (int*)malloc(640*480*sizeof(int));
  segment_image_mat(out3, output, sigma, k, min_size, &num_ccs, label); //-> hardcoded here to img3

  printf("got %d components\n", num_ccs);
  printf("done! uff...thats hard work.\n");

  imshow("segmented", output);
	imwrite("segmented.jpg", output);
  //waitKey(0);
// masking each segment and ANDing with the canny image to find best focus magnitude
  int focus[num_ccs][4];
  Mat mask(height, width, CV_8UC1);
  for(int i=0; i< num_ccs; i++){
		find_mask(mask, label, i);
		focus[i][0] = find_nonzero(gray_img1, mask);
		focus[i][1] = find_nonzero(gray_img2, mask);
		focus[i][2] = find_nonzero(gray_img3, mask);
		focus[i][3] = find_nonzero(gray_img4, mask);
#if DRAW_LOW
		imshow("mask", mask);
		waitKey(0);
#endif
  }

// finding the best focus mask and storing them in best_focus array from the focus[num_ccs][SIZE] array
	int best_focus[num_ccs];
	for(int i=0; i< num_ccs; i++)
		best_focus[i] = 3;
	for(int i=0; i< num_ccs; i++){
		int max=0;
		for(int j=0; j< 4; j++){
			if(focus[i][j] >= max){
				max = focus[i][j];
				best_focus[i] = j;
			}
		}	
	}

#if DRAW
	for(int i=0; i< num_ccs; i++){
			printf("%d, %d, %d, %d --- %d\n", focus[i][0],focus[i][1],focus[i][2],focus[i][3], best_focus[i]);
	}
#endif

// Finding the all focussed image and depth map of the scene
	vector<Mat> color1; split(out1, color1);
	vector<Mat> color2; split(out2, color2);
	vector<Mat> color3; split(out3, color3);
	vector<Mat> color4; split(out4, color4);

	Mat AllFocus(height, width, CV_8UC3);
	Mat depthMap(height, width, CV_8UC1);
	vector<Mat> AllFocus_color; split(AllFocus, AllFocus_color);
	for(int y =0; y<height; y++) {
		for(int x=0; x<width; x++){
			int comp = best_focus[*(label+y*width+x)];
			switch (comp) {
				case 0:
					for(int i=0; i<3; i++){
						AllFocus_color[i].at<uchar>(y,x) = color1[i].at<uchar>(y,x);
					}
					depthMap.at<uchar>(y,x) = 191;
					break;
				case 1:
					for(int i=0; i<3; i++){
						AllFocus_color[i].at<uchar>(y,x) = color2[i].at<uchar>(y,x);
					}
					depthMap.at<uchar>(y,x) = 127;
					break;
				case 2:
					for(int i=0; i<3; i++){
						AllFocus_color[i].at<uchar>(y,x) = color3[i].at<uchar>(y,x);
					}
					depthMap.at<uchar>(y,x) = 63;
					break;
				default:
					for(int i=0; i<3; i++) {
						AllFocus_color[i].at<uchar>(y,x) = color4[i].at<uchar>(y,x);
					}
					depthMap.at<uchar>(y,x) = 0;
					break;
			}			
		}
	}
	merge(AllFocus_color, AllFocus);
	imshow("allfocus", AllFocus);
	imshow("depthMap", depthMap);
	imwrite("allfocus.jpg", AllFocus);
	imwrite("depthMap.jpg", depthMap);
	//waitKey(0);

//creating left and right images from the depth map
	Mat Left_img(height, width, CV_8UC3);
	vector<Mat> left_color; split(Left_img, left_color);
	Mat Right_img(height, width, CV_8UC3);
	vector<Mat> right_color; split(Right_img, right_color);
	for(int y=0; y<height; y++) {
		for(int x=0; x<width; x++) {
			int x_shift = (int)( (float)(depthMap.at<uchar>(y,x)) / 255.0 * max_shift);
			if( (x-x_shift) > 0 ){
				for(int i=0; i<3; i++){
					left_color[i].at<uchar>(y,x-x_shift) = AllFocus_color[i].at<uchar>(y,x);
				}
			}
			if( (x+x_shift) < 640 ){
				for(int i=0; i<3; i++){
					right_color[i].at<uchar>(y,x+x_shift) = AllFocus_color[i].at<uchar>(y,x);
				}
			}
		}
	}

#if 1
	merge(left_color, Left_img);
	merge(right_color, Right_img);
	imwrite("Left_image_with_holes.jpg", Left_img);
	imwrite("Right_image_with_holes.jpg", Right_img);
	waitKey(0);
#endif

// stitch the holes due to shift in image for both lest and right image
	for(int y=0; y<height; y++) {
		for(int x=0; x<width; x++) {
			for(int i=0; i<3; i++){
				left_color[i].at<uchar>(y,x) = (left_color[i].at<uchar>(y,x) == 0) ? (left_color[i].at<uchar>(y,x-1)) : (left_color[i].at<uchar>(y,x) );
			}
		}
		for(int x=(width -1); x>=0; x--) {
			for(int i=0; i<3; i++){
				right_color[i].at<uchar>(y,x) = (right_color[i].at<uchar>(y,x) == 0) ? (right_color[i].at<uchar>(y,x+1)) : (right_color[i].at<uchar>(y,x) );
			}
		}
	}
#if 1
	merge(left_color, Left_img);
	merge(right_color, Right_img);
	imshow("Left_image_after_stitching.jpg", Left_img);
	imshow("Right_image_after_stitching.jpg", Right_img);
	waitKey(0);
#endif

//creating the RED-CYAN anaglyph
	Mat anaglyph(height,width, CV_8UC3);
	vector<Mat> anaglyph_color; split(anaglyph, anaglyph_color);
	right_color[0].copyTo(anaglyph_color[0]);
	right_color[1].copyTo(anaglyph_color[1]);
	left_color[2].copyTo(anaglyph_color[2]);
	merge(anaglyph_color, anaglyph);
	imshow("3D image", anaglyph);
	imwrite("RED_CYAN_anaglyph.jpg", anaglyph);
	waitKey(0);

  return 0;
}

