#include <cstdio>
#include <string>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
using namespace cv;		using namespace std;

const int numImages = 40;
const int numPoints = 300;
const string imDir = "~/Dropbox/helicopter_rect_crop_images/";

struct imageData {
	// Mat points(numPoints, 2, CV_32FC1);
	imageData():correspondencesPrev(numPoints, 2, CV_32FC1),
				correspondencesNext(numPoints, 2, CV_32FC1){}
	Mat correspondencesPrev;
	Mat correspondencesNext;
};

// void nccPyramidMatch(Mat, Mat, Mat, imageData&);
Mat harris(Mat);

int main() {
	imageData currIm1Data, currIm2Data;
	Mat currIm1, currIm2;
	//read in images first, fix this
	vector<imageData> imageSetLeft;
	// imageData currIm1Data, currIm2Data;
	// for (int i = 0; i < numImages; i++) {
	// 	currIm1Data.image = imread(etc); //fix this
	// 	imageSetLeft.push_back(currIm1Data);
	// }


	printf("setting up first image\n");
	char imageLocation[] = "%s %s %04d %s";
	sprintf(imageLocation,imDir.c_str(),"left_rect_crop_",0,".tiff");
	currIm1 = imread(imageLocation,0);
	Mat corners = harris(currIm1);
	// currIm1Data.correspondencesNext = suppress(harris points); //fix this
	// currIm1Data.correspondencesPrev = NULL;
	// imageSetLeft.push_back(currIm1Data);
	// currIm2 = imread(sprintf("%s %s %04d %s",imDir,"left_rect_crop_",1,".tiff"), CV_LOAD_IMAGE_GRAYSCALE);
	// nccPyramidMatch(currIm1, currIm2, currIm1Data.correspondencesNext, currIm2Data);
	// imageSetLeft.push_back(currIm2Data);

	// for (int i = 2; i < numImages; i++) {
	// 	printf("processing images %d and %d\n", i, i+1);
	// 	currIm1Data = currIm2Data;
	// 	currIm1 = currIm2;
	// 	currIm2 = imread(sprintf("%s %s %04d %s",imDir,"left_rect_crop_",i,".tiff"), CV_LOAD_IMAGE_GRAYSCALE);
	// 	nccPyramidMatch(currIm1, currIm2, currIm1Data.correspondencesNext, currIm2Data);
	// 	imageSetLeft.push_back(currIm2Data);
	// }
}

// void nccPyramidMatch(Mat im1, Mat im2, Mat im1Pts, imageData& im2Data) {
// 
// }

//returns N-by-2 matrix of (x,y) harris corner coordinates
Mat harris(Mat im) {
	Mat window = Mat::zeros(3,3,CV_32FC1);
	Mat maxPts = Mat::zeros(im.size(), CV_8UC1);
	Mat currPoint = Mat::zeros(1,2,CV_32FC1);
	Mat harrisImage = Mat::zeros(im.size(), CV_32FC1);
	Point* currMax;

	cornerHarris(im, harrisImage, 3, 3, 0.04, BORDER_DEFAULT); //not sure about the k parameter

	int descHalfSize = 16;
	int windowHalfSize = 64;
	int halfSize = descHalfSize + windowHalfSize;
	int harrisWidth = harrisImage.cols;
	int harrisHeight = harrisImage.rows;
	int suppressSize = 3;

	//remove points too close to image border
	harrisImage.rowRange(Range(0,halfSize)) = Mat::zeros(halfSize,harrisWidth,CV_32FC1);
	harrisImage.rowRange(Range(harrisHeight-halfSize,harrisHeight)) = Mat::zeros(halfSize,harrisWidth,CV_32FC1);
	harrisImage.colRange(Range(0,halfSize)) = Mat::zeros(harrisHeight,halfSize,CV_32FC1);
	harrisImage.colRange(Range(harrisWidth-halfSize,harrisWidth)) = Mat::zeros(harrisHeight,halfSize,CV_32FC1);

	//non-max suppress
	for (int row = 0; row < harrisHeight-suppressSize+1; row++) {
		for (int col = 0; col < harrisWidth-suppressSize+1; col++) {
			window = harrisImage(Range(row,row+suppressSize),Range(col,col+suppressSize));
			minMaxLoc(window, NULL, NULL, NULL, currMax);
			maxPts.at<unsigned char>((*currMax).y+row,(*currMax).x+col) = 1;
		}
	}
	
	//extract coordinates of nonzero points (the max pts)
	Mat corners = Mat::zeros(countNonZero(maxPts),2,CV_32FC1);
	// findNonZero(maxPts,corners); /* finds all nonzero elements, but only in opencv>=2.4.4 */
	int rowIndex = 0;
	for (int row = 0; row < maxPts.rows; row++) {
		for (int col = 0; col < maxPts.cols; col++) {
			if (maxPts.at<unsigned char>(row,col) == 1) {
				corners.at<float>(rowIndex,1) = col;
				corners.at<float>(rowIndex,2) = row;
				rowIndex++;
			}
		}
	}
	return corners;
}
