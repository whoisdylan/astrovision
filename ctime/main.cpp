#include <cstdio>
#include <string>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
using namespace cv;		using namespace std;

const int numImages = 40;
const int numPoints = 300;
const char imDir[] = "/Users/dylan/Dropbox/helicopter_rect_crop_images/left_rect_crop_";
const char imExt[] = ".tiff";
const int imLocLength = strlen(imDir) + strlen(imExt) + 4;

struct imageData {
	// Mat points(numPoints, 2, CV_32FC1);
	imageData():correspondencesPrev(numPoints, 2, CV_32FC1),
				correspondencesNext(numPoints, 2, CV_32FC1){}
	Mat correspondencesPrev;
	Mat correspondencesNext;
};

// void nccPyramidMatch(Mat, Mat, Mat, imageData&);
Mat harris(Mat, vector<double>&);
Mat suppress(Mat, const vector<double>&);

int main() {
	imageData currIm1Data, currIm2Data;
	Mat currIm1, currIm2;
	vector<double> strengths;
	//read in images first, fix this
	vector<imageData> imageSetLefts;
	// imageData currIm1Data, currIm2Data;
	// for (int i = 0; i < numImages; i++) {
	// 	currIm1Data.image = imread(etc); //fix this
	// 	imageSetLefts.push_back(currIm1Data);
	// }


	printf("setting up first image\n");
	char imageLocation[imLocLength];
	sprintf(imageLocation, "%s%04d%s", imDir,0,imExt);
	currIm1 = imread(imageLocation,CV_LOAD_IMAGE_GRAYSCALE);
	Mat corners = harris(currIm1, strengths);
	currIm1Data.correspondencesNext = suppress(corners, const strengths);
	currIm1Data.correspondencesPrev = NULL;
	printf("%d corners found\n",corners.rows);
	namedWindow("fig", CV_WINDOW_AUTOSIZE);
	Mat currPlot1 = currIm1.clone();
	for (int i = 0; i < corners.rows; i++) {
		circle(currPlot1, Point(corners.at<int>(i,1),corners.at<int>(i,2)), 1, Scalar(255,0,0), 1, 8, 0);
		// circle(currPlot1,Point(500,500),5,Scalar(255,0,0),3,8,0);
	}
	resize(currPlot1,currPlot1,Size(round(.5*currPlot1.cols),round(.5*currPlot1.rows)),.5,.5,INTER_CUBIC);
	imshow("fig",currPlot1);
	waitKey(0);
	// imageSetLefts.push_back(currIm1Data);
	// currIm2 = imread(sprintf("%s %s %04d %s",imDir,"left_rect_crop_",1,".tiff"), CV_LOAD_IMAGE_GRAYSCALE);
	// nccPyramidMatch(currIm1, currIm2, currIm1Data.correspondencesNext, currIm2Data);
	// imageSetLefts.push_back(currIm2Data);

	// for (int i = 2; i < numImages; i++) {
	// 	printf("processing images %d and %d\n", i, i+1);
	// 	currIm1Data = currIm2Data;
	// 	currIm1 = currIm2;
	// 	currIm2 = imread(sprintf("%s %s %04d %s",imDir,"left_rect_crop_",i,".tiff"), CV_LOAD_IMAGE_GRAYSCALE);
	// 	nccPyramidMatch(currIm1, currIm2, currIm1Data.correspondencesNext, currIm2Data);
	// 	imageSetLefts.push_back(currIm2Data);
	// }
}

// void nccPyramidMatch(Mat im1, Mat im2, Mat im1Pts, imageData& im2Data) {
// 
// }

//returns N-by-2 matrix of (x,y) harris corner coordinates
Mat harris(Mat im, vector<double>& strengths) {
	Mat window = Mat::zeros(3,3,CV_32FC1);
	Mat maxPts = Mat::zeros(im.size(), CV_64FC1);
	Mat currPoint = Mat::zeros(1,2,CV_32FC1);
	Mat harrisImage = Mat::zeros(im.size(), CV_32FC1);
	Point currMax;
	double maxVal;

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
			minMaxLoc(window, 0, &maxVal, 0, &currMax, noArray());
			if (maxVal > .00001) {
				maxPts.at<double>((currMax).y+row,(currMax).x+col) = maxVal;
			}
		}
	}
	
	//extract coordinates of nonzero points (the max pts)
	Mat corners = Mat::zeros(countNonZero(maxPts),2,CV_32SC1);
	strengths.resize(corners.rows);
	// findNonZero(maxPts,corners); /* finds all nonzero elements, but only in opencv>=2.4.4 */
	int rowIndex = 0;
	for (int row = 0; row < maxPts.rows; row++) {
		for (int col = 0; col < maxPts.cols; col++) {
			maxVal = maxPts.at<double>(row,col);
			if (maxVal != 0) {
				corners.at<int>(rowIndex,1) = col;
				corners.at<int>(rowIndex,2) = row;
				strengths.push_back(maxVal);
				rowIndex++;
			}
		}
	}
	return corners;
}

Mat suppress(Mat corners, const vector<double>& strengths) {

}
