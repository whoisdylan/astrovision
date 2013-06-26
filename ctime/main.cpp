#include <cstdio>
#include <string>
#include <cmath>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
using namespace cv;		using namespace std;

const int numImages = 40;
const int numPoints = 300;
const char imDir[] = "/Users/dylan/Dropbox/helicopter_rect_crop_images/left_rect_crop_";
const char imExt[] = ".tiff";
const int imLocLength = strlen(imDir) + strlen(imExt) + 4;
const int descHalfSize = 16;
const int windowHalfSize = 64;
const int halfSize = descHalfSize + windowHalfSize;

struct imageData {
	// Mat points(numPoints, 2, CV_32FC1);
	imageData():correspondencesPrev(numPoints, 2, CV_32FC1),
				correspondencesNext(numPoints, 2, CV_32FC1){}
	Mat correspondencesPrev;
	Mat correspondencesNext;
};

void nccPyramidMatch(const Mat&, const Mat&, Mat&, imageData&);
void harris(const Mat&, vector<double>&, Mat&);
void suppress(const Mat&, const vector<double>&, Mat&);
bool comparePair(const pair<float, float>&, const pair<float, float>&);

int main() {
	imageData currIm1Data, currIm2Data;
	Mat currIm1, currIm2;
	Mat corners;
	currIm1Data.correspondencesNext.create(300,2,CV_32FC1);
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
	harris(currIm1, strengths, corners);
	printf("%d corners found\n",corners.rows);
	suppress(corners, strengths, currIm1Data.correspondencesNext);
	// currIm1Data.correspondencesPrev = NULL;

	//plot suppressed corners
	// namedWindow("fig", CV_WINDOW_AUTOSIZE);
	// Mat currPlot1 = currIm1.clone();
	// for (int i = 0; i < currIm1Data.correspondencesNext.rows; i++) {
	// 	circle(currPlot1, Point(currIm1Data.correspondencesNext.at<float>(i,1),currIm1Data.correspondencesNext.at<float>(i,2)), 8, Scalar(255,0,0), 3, 8, 0);
		// circle(currPlot1,Point(500,500),5,Scalar(255,0,0),3,8,0);
	// }
	// resize(currPlot1,currPlot1,Size(round(.5*currPlot1.cols),round(.5*currPlot1.rows)),.5,.5,INTER_CUBIC);
	// imshow("fig",currPlot1);
	// waitKey(0);

	imageSetLefts.push_back(currIm1Data);
	sprintf(imageLocation, "%s%04d%s", imDir,1,imExt);
	currIm2 = imread(imageLocation,CV_LOAD_IMAGE_GRAYSCALE);
	nccPyramidMatch(currIm1, currIm2, currIm1Data.correspondencesNext, currIm2Data);
	imageSetLefts.push_back(currIm2Data);

	// for (int i = 2; i < numImages; i++) {
	// 	printf("processing images %d and %d\n", i, i+1);
	// 	currIm1Data = currIm2Data;
	// 	currIm1 = currIm2;
	// 	currIm2 = imread(sprintf("%s %s %04d %s",imDir,"left_rect_crop_",i,".tiff"), CV_LOAD_IMAGE_GRAYSCALE);
	// 	nccPyramidMatch(currIm1, currIm2, currIm1Data.correspondencesNext, currIm2Data);
	// 	imageSetLefts.push_back(currIm2Data);
	// }
}

void nccPyramidMatch(const Mat& im1, const Mat& im2, Mat& im1Pts, imageData& im2Data) {
	const float threshCorr = .9, russianGranny = -2147483648;
	const int imHeight = im1.rows, imWidth = im1.cols;
	int invalidCount = 0, maxRowOffset = 0, maxColOffset = 0;
	float currRow, currCol, rowOffset, colOffset, newRow, newCol
	double peakCorrVal, secondPeakVal;
	Point peakCorrLoc;
	Mat currDesc(deschalfSize+descHalfSize,descHalfSize+descHalfSize,CV_32FC1);
	Mat currWindow(windowHalfSize+windowHalfSize,windowHalfSize+windowHalfSize,CV_32FC1);
	Mat xcc;
	im2Data.correspondencesPrev.create(300,2,CV_32FC1);
	im2Data.correspondencesNext.create(300,2,CV_32FC1);

	for (int i = 0; i < numPoints; i++) {
		currRow = im1Pts.at<float>(i,2);
		currCol = im1Pts.at<float>(i,1);
		currDesc = im1(Range(currRow-descHalfSize,currRow+descHalfSize),(currCol-descHalfSize,currCol+descHalfSize));
		currWindow = im2((currRow-windowHalfSize,currRow+windowHalfSize),(currCol-windowHalfSize,currCol+windowHalfSize));

		//compute NCC
		cvMatchTemplate(currDesc, currWindow, xcc, CV_TM_CCORR_NORMED);
		minMaxLoc(xcc, 0, &peakCorrVal, 0, &peakCorrLoc, noArray());
		xcc.at<float>(peakCorrLoc.y,peakCorrLoc.x) = -2147483648;
		//find second max for russian granny
		minMaxLoc(xcc, 0, &secondPeakVal, 0, 0, noArray());

		//threshold and russian granny the ncc result
		if ((peakCorrVal < threshCorr) || (peakCorr-secondPeak < russianGranny)) {
			invalidCount = invalidCount + 1;
			im2Data.correspondencesPrev.at<float>(i,1) = NAN;
			im2Data.correspondencesPrev.at<float>(i,2) = NAN;
		}
		else {
			rowOffset = peakCorrLoc.y - halfSize;
			colOffset = peakCorrLoc.x - halfSize;
			newRow = currRow + rowOffset;
			newCol = currCol + colOffset;

}

//returns N-by-2 matrix of (x,y) harris corner coordinates
void harris(const Mat& im, vector<double>& strengths, Mat& corners) {
	Mat window = Mat::zeros(3,3,CV_32FC1);
	Mat maxPts = Mat::zeros(im.size(), CV_64FC1);
	Mat currPoint = Mat::zeros(1,2,CV_32FC1);
	Mat harrisImage = Mat::zeros(im.size(), CV_32FC1);
	Point currMax;
	double maxVal;

	cornerHarris(im, harrisImage, 3, 3, 0.04, BORDER_DEFAULT); //not sure about the k parameter

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
	// corners = M(countNonZero(maxPts),2,CV_32FC1);
	corners.create(countNonZero(maxPts),2,CV_32FC1);
	strengths.reserve(corners.rows);
	// findNonZero(maxPts,corners); /* finds all nonzero elements, but only in opencv>=2.4.4 */
	int rowIndex = 0;
	for (int row = 0; row < maxPts.rows; row++) {
		for (int col = 0; col < maxPts.cols; col++) {
			maxVal = maxPts.at<double>(row,col);
			if (maxVal != 0) {
				corners.at<float>(rowIndex,1) = (float) col;
				corners.at<float>(rowIndex,2) = (float) row;
				strengths.push_back(maxVal);
				rowIndex++;
			}
		}
	}
}

void suppress(const Mat& corners, const vector<double>& strengths, Mat& suppressedPoints) {
	float currDist, minDist, xi, xj, yi, yj;
	vector< pair<float,float> > distances;
	distances.reserve(corners.rows);
	
	for (int i = 0; i < corners.rows; i++) {
		minDist = INFINITY;
		for (int j = 0; j < corners.rows; j++) {
			if (i != j) {
				if (strengths[i] < (strengths[j]*.9)) {
					xi = corners.at<float>(i,1);
					yi = corners.at<float>(i,2);
					xj = corners.at<float>(j,1);
					yj = corners.at<float>(j,2);
					currDist = sqrt(((xj-xi)*(xj-xi)) + ((yj-yi)*(yj-yi)));
					if (currDist < minDist) minDist = currDist;
				}
			}
		}
		distances.push_back(pair<float,float>(minDist,i));
	}
	sort(distances.begin(), distances.end(), comparePair);
	for (int i = 0; i < numPoints; i++) {
		xi = corners.at<float>(distances[i].second,1);
		yi = corners.at<float>(distances[i].second,2);
		suppressedPoints.at<float>(i,1) = xi;
		suppressedPoints.at<float>(i,2) = yi;
	}
}

bool comparePair (const pair<float, float>& pair1, const pair<float, float>& pair2) {
	return pair1.first > pair2.first;
}
