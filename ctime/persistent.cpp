#include <cstdio>

#include <string>
#include <list>
#include <cmath>
#include <iostream>
#include <fstream>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
using namespace cv;		using namespace std;

// #define SAVE_CORRESPONDENCES

const int numImages = 40;
const int numPoints = 300;
const char imDir[] = "/Users/dylan/Dropbox/helicopter_rect_crop_images/left_rect_crop_";
const char imDirR[] = "/Users/dylan/Dropbox/helicopter_rect_crop_images/right_rect_crop_";
const char imExt[] = ".tiff";
const int imLocLength = strlen(imDirR) + strlen(imExt) + 4;
/* for hongbin construction images */
// const char imDir[] = "/Users/dylan/Dropbox/Hongbin/construction_images/L_";
// const char imExt[] = ".png";
// const int imLocLength = strlen(imDir) + strlen(imExt) + 1;
const int descHalfSize = 16;
const int windowHalfSize = 64;
const int halfSize = descHalfSize + windowHalfSize;

struct imageData {
	imageData():correspondencesLL(),
				correspondencesLR(){}
	imageData(const imageData& other) {
		// other.correspondencesLL.copyTo(correspondencesLL);
		// other.correspondencesLR.copyTo(correspondencesLR);
		correspondencesLL = other.correspondencesLL;
		correspondencesLR = other.correspondencesLR;
	}
	imageData& operator=(const imageData& other) {
		// other.correspondencesLL.copyTo(correspondencesLL);
		// other.correspondencesLR.copyTo(correspondencesLR);
		correspondencesLL = other.correspondencesLL;
		correspondencesLR = other.correspondencesLR;
		return *this;
	}
	list< vector<Point2f> > correspondencesLL;
	list< vector<Point2f> > correspondencesLR;
};

void nccPyramidMatch(const Mat&, const Mat&, const Mat&, imageData&, imageData&);
void harris(const Mat&, vector<double>&, vector<Point2f>&);
void suppress(const vector<Point2f>&, const vector<double>&, list< vector<Point2f> >&);
bool comparePair(const pair<float, float>&, const pair<float, float>&);
void writeMat(const Mat&, const char *);

int main() {
	imageData currIm1Data, currIm2Data;
	Mat currIm1, currIm2, currImR;
	vector<Point2f> corners;
	// currIm1Data.correspondencesLL;
	// currIm1Data.correspondencesLR;
	vector<double> strengths;
	vector<imageData> imageSetLefts;
	imageSetLefts.reserve(numImages);

	cout << "setting up first image" << endl;
	char imageLocation[imLocLength];
	sprintf(imageLocation, "%s%04d%s", imDir,0,imExt);
	currIm1 = imread(imageLocation,CV_LOAD_IMAGE_GRAYSCALE);
	harris(currIm1, strengths, corners);
	// printf("%d corners found\n",(int) corners.size());
	suppress(corners, strengths, currIm1Data.correspondencesLL);
	sprintf(imageLocation, "%s%04d%s", imDir,1,imExt);
	currIm2 = imread(imageLocation,CV_LOAD_IMAGE_GRAYSCALE);
	sprintf(imageLocation, "%s%04d%s", imDirR,1,imExt);
	currImR = imread(imageLocation,CV_LOAD_IMAGE_GRAYSCALE);
	nccPyramidMatch(currIm1, currIm2, currImR, currIm1Data, currIm2Data);
	imageSetLefts.push_back(currIm1Data);

	/* save correspondence pairs */
	// char resultNextFile[47];
	// char resultPrevFile[47];
	// sprintf(resultNextFile, "%s%02d%s", "/Users/dylan/Dropbox/astromats/corrNextP", 1, ".txt");
	// sprintf(resultPrevFile, "%s%02d%s", "/Users/dylan/Dropbox/astromats/corrPrevP", 1, ".txt");
	// writeMat(currIm1Data.correspondencesNext, resultNextFile);
	// writeMat(currIm2Data.correspondencesPrev, resultPrevFile);
	// sprintf(resultNextFile, "%s%02d%s", "/Users/dylan/Dropbox/astromats/corrNextP", 2, ".txt");
	// writeMat(currIm2Data.correspondencesNext, resultNextFile);

	for (int i = 2; i < numImages; i++) {
		cout << "processing images " << i << " and " << i+1 << endl;
		currIm1Data = currIm2Data;
		currIm1 = currIm2;
		sprintf(imageLocation, "%s%04d%s", imDir,i,imExt);
		currIm2 = imread(imageLocation,CV_LOAD_IMAGE_GRAYSCALE);
		sprintf(imageLocation, "%s%04d%s", imDirR,i-1,imExt);
		currImR = imread(imageLocation,CV_LOAD_IMAGE_GRAYSCALE);
		nccPyramidMatch(currIm1, currIm2, currImR, currIm1Data, currIm2Data);
		imageSetLefts.push_back(currIm1Data);

		/* save correspondence pairs */
		// sprintf(resultPrevFile, "%s%02d%s", "/Users/dylan/Dropbox/astromats/corrPrevP", i, ".txt");
		// writeMat(currIm2Data.correspondencesPrev, resultPrevFile);
		// sprintf(resultNextFile, "%s%02d%s", "/Users/dylan/Dropbox/astromats/corrNextP", i+1, ".txt");
		// writeMat(currIm2Data.correspondencesNext, resultNextFile);
	}
	imageSetLefts.push_back(currIm2Data);
	//last image does not have left-right correspondences!
	cout << "finished processing images" << endl;

	/* save the correspondence matrix pairs */
	#ifdef SAVE_CORRESPONDENCES
	cout << "saving correspondence matrices" << endl;
	char resultNextFile[47];
	char resultPrevFile[47];
	for (int i = 1; i < numImages; i++) {
		// cout << "saving astromat " << i << endl;
		currIm1Data = imageSetLefts[i-1];
		currIm2Data = imageSetLefts[i];
		sprintf(resultNextFile, "%s%02d%s", "/Users/dylan/Dropbox/astromats/corrNextP", i, ".txt");
		sprintf(resultPrevFile, "%s%02d%s", "/Users/dylan/Dropbox/astromats/corrPrevP", i, ".txt");
		writeMat(currIm1Data.correspondencesNext, resultNextFile);
		writeMat(currIm2Data.correspondencesPrev, resultPrevFile);
	}
	#endif
	cout << "all done" << endl;
}

void nccPyramidMatch(const Mat& im1, const Mat& im2, const Mat& imR, imageData& im1Data, imageData& im2Data) {
	const float threshCorr = .9, russianGranny = -2147483648;
	const unsigned int imHeight = im1.rows, imWidth = im1.cols, descHalfSize2 = 4, windowHalfSize2 = 5;
	const unsigned int imScale = 4;
	// const unsigned int xDim, yDim;
	// const bool top, left;
	float invalidCount = 0, maxRowOffset = 0, maxColOffset = 0;
	float currIm2Row, currIm2Col;
	double rowOffset, colOffset, newRow, newCol, currRow, currCol;
	double rowOffsetR, colOffsetR, newRowR, newColR, currImRRow, currImRCol;
	double peakCorrVal, secondPeakVal, peakCorrValR, secondPeakValR;
	Point peakCorrLoc, peakCorrLocR;
	Mat currDesc(descHalfSize+descHalfSize,descHalfSize+descHalfSize,CV_8UC1);
	Mat newDesc(descHalfSize2+descHalfSize2,descHalfSize2+descHalfSize2,CV_8UC1);
	Mat descResize(descHalfSize+descHalfSize,descHalfSize+descHalfSize,CV_8UC1);
	Mat currWindow(windowHalfSize+windowHalfSize,windowHalfSize+windowHalfSize,CV_8UC1);
	Mat currWindowR(windowHalfSize+windowHalfSize,windowHalfSize+windowHalfSize,CV_8UC1);
	Mat newWindow(windowHalfSize2+windowHalfSize2,windowHalfSize2+windowHalfSize2,CV_8UC1);
	Mat windowResize((windowHalfSize2+windowHalfSize2)*2,(windowHalfSize2+windowHalfSize2)*2,CV_8UC1);
	Mat xcc, xcc2, xccR, xcc2R;
	// im2Data.correspondencesLR = im1Data.correspondencesLR;
	// im2Data.correspondencesLL = im1Data.correspondencesLL;

	// list< vector<Point2f> >::iterator itLR = im1Data.correspondencesLR.begin();
	// list< vector<Point2f> >::iterator it2 = im2Data.correspondencesLL.begin();
	for (list< vector<Point2f> >::iterator it = im1Data.correspondencesLL.begin(); it != im1Data.correspondencesLL.end(); it++/* , itLR++, it2++ */) {
		currCol = (double) round((*it).back().x);
		currRow = (double) round((*it).back().y);
		currDesc = im1(Range(currRow-descHalfSize,currRow+descHalfSize),Range(currCol-descHalfSize,currCol+descHalfSize));
		currWindow = im2(Range(currRow-windowHalfSize,currRow+windowHalfSize),Range(currCol-windowHalfSize,currCol+windowHalfSize));
		currWindowR = imR(Range(currRow-windowHalfSize,currRow+windowHalfSize),Range(currCol-windowHalfSize,currCol+windowHalfSize));

		//compute NCC
		matchTemplate(currWindow, currDesc, xcc, CV_TM_CCORR_NORMED);
		minMaxLoc(xcc, 0, &peakCorrVal, 0, &peakCorrLoc, Mat());
		xcc.at<float>((float) peakCorrLoc.y,(float) peakCorrLoc.x) = -2147483648;
		//find second max for russian granny, disabled for now
		// minMaxLoc(xcc, 0, &secondPeakVal, 0, 0, Mat());
		secondPeakVal = 0;

		//compute LR NCC
		matchTemplate(currWindowR, currDesc, xccR, CV_TM_CCORR_NORMED);
		minMaxLoc(xccR, 0, &peakCorrValR, 0, &peakCorrLocR, Mat());
		xccR.at<float>((float) peakCorrLocR.y,(float) peakCorrLocR.x) = -2147483648;
		//find second max for russian granny, disabled for now
		// minMaxLoc(xccR, 0, &secondPeakValR, 0, 0, Mat());
		secondPeakValR = 0;


		//threshold and russian granny the ncc result
		if ((peakCorrVal < threshCorr) || (peakCorrValR < threshCorr) || (peakCorrValR - secondPeakValR < russianGranny) || (peakCorrVal-secondPeakVal < russianGranny)) {
			invalidCount = invalidCount + 1;
			//put (0,0) onto im1 list to signify track is over
			(*it).push_back(Point2f(0,0));
			//remove the track from im2 list and im1LR list
			// itLR = im1Data.correspondencesLR.erase(itLR);
			// it2 = im2Data.correspondencesLL.erase(it2);
		}
		else {
			rowOffset = (double) (peakCorrLoc.y - (windowHalfSize - descHalfSize));
			colOffset = (double) (peakCorrLoc.x - (windowHalfSize - descHalfSize));
			newRow = currRow + rowOffset;
			newCol = currCol + colOffset;
			newDesc = im1(Range(currRow-descHalfSize2,currRow+descHalfSize2),Range(currCol-descHalfSize2,currCol+descHalfSize));
			newWindow = im2(Range(newRow-windowHalfSize2,newRow+windowHalfSize2),Range(newCol-windowHalfSize2,newCol+windowHalfSize2));
			resize(newDesc, descResize, Size(imScale*descHalfSize2,imScale*descHalfSize2), 0, 0, INTER_CUBIC);
			resize(newWindow, windowResize, Size(imScale*windowHalfSize2,imScale*windowHalfSize2), 0, 0, INTER_CUBIC);

			matchTemplate(windowResize, descResize, xcc2, CV_TM_CCORR_NORMED);
			minMaxLoc(xcc2, 0, 0, 0, &peakCorrLoc, Mat());
			//similar to old offset calculation method
			// rowOffset = ((double) peakCorrLoc.y)/((double) imScale) - (double) ((newRow - windowHalfSize2) - (currRow - descHalfSize2));
			// colOffset = ((double) peakCorrLoc.x)/((double) imScale) - (double) ((newCol - windowHalfSize2) - (currCol - descHalfSize2));
			//new subpixel offset calculation method
			rowOffset = ((double) peakCorrLoc.y)/((double) imScale) - (double) (windowHalfSize2 - descHalfSize2);
			colOffset = ((double) peakCorrLoc.x)/((double) imScale) - (double) (windowHalfSize2 - descHalfSize2);
			// rowOffset = 0; colOffset = 0;

			if (rowOffset > maxRowOffset) maxRowOffset = rowOffset;
			if (colOffset > maxColOffset) maxColOffset = colOffset;

			currIm2Row = (float) (newRow + rowOffset);
			currIm2Col = (float) (newCol + colOffset);

			//now do it all again for the LR correspondence!
			rowOffsetR = (double) (peakCorrLocR.y - (windowHalfSize - descHalfSize));
			colOffsetR = (double) (peakCorrLocR.x - (windowHalfSize - descHalfSize));
			newRowR = currRow + rowOffsetR;
			newColR = currCol + colOffsetR;
			newWindow = imR(Range(newRowR-windowHalfSize2,newRowR+windowHalfSize2),Range(newColR-windowHalfSize2,newColR+windowHalfSize2));
			resize(newWindow, windowResize, Size(imScale*windowHalfSize2,imScale*windowHalfSize2), 0, 0, INTER_CUBIC);

			matchTemplate(windowResize, descResize, xcc2R, CV_TM_CCORR_NORMED);
			minMaxLoc(xcc2R, 0, 0, 0, &peakCorrLocR, Mat());
			//similar to old offset calculation method
			// rowOffset = ((double) peakCorrLoc.y)/((double) imScale) - (double) ((newRow - windowHalfSize2) - (currRow - descHalfSize2));
			// colOffset = ((double) peakCorrLoc.x)/((double) imScale) - (double) ((newCol - windowHalfSize2) - (currCol - descHalfSize2));
			//new subpixel offset calculation method
			rowOffsetR = ((double) peakCorrLocR.y)/((double) imScale) - (double) (windowHalfSize2 - descHalfSize2);
			colOffsetR = ((double) peakCorrLocR.x)/((double) imScale) - (double) (windowHalfSize2 - descHalfSize2);
			// rowOffset = 0; colOffset = 0;

			currImRRow = (float) (newRowR + rowOffsetR);
			currImRCol = (float) (newColR + colOffsetR);

			//check if new point is outside of the tolerance frame
			if ((currIm2Row < (halfSize - 1)) || (currImRRow < (halfSize - 1)) || (currIm2Col < (halfSize - 1)) || (currImRCol < (halfSize - 1)) || (currIm2Row > (imHeight - halfSize)) || (currImRRow > (imHeight - halfSize)) || (currIm2Col > (imWidth - halfSize)) || (currImRCol > (imWidth - halfSize))) {
				invalidCount = invalidCount + 1;
				//put (0,0) on im1 LL list to signify end of track
				(*it).push_back(Point2f(0,0));
				//remove point form LR and im2 LL list
			}
			else {
				//give im2 list the previous track record
				// (*it2) = (*it);
				im2Data.correspondencesLL.push_back(*it);
				//update the track record
				// (*it2).push_back(Point2f(currIm2Col,currIm2Row));
				im2Data.correspondencesLL.back().push_back(Point2f(currIm2Col,currIm2Row));
				//give LR list the current track record
				// (*itLR) = (*it);
				im1Data.correspondencesLR.push_back(*it);
				//update the track record
				// (*itLR).push_back(Point2f(currImRCol,currImRRow));
				im1Data.correspondencesLR.back().push_back(Point2f(currIm2Col,currIm2Row));
			}
		}
	}

	//detect new points if necessary
	if (invalidCount != 0) {
		printf("acquiring new points\n");
		//try to find points intelligently (mostly disabled)
		// if (maxRowOffset < 0) {
		// 	// yDim = imHeight - halfSize - round(maxRowOffset);
		// 	top = false;
		// }
		// else {
		// 	// yDim = halfSize + round(maxRowOffset);
		// 	top = true;
		// }
		// if (maxColOffset < 0) {
		// 	// xDim = imWidth - halfSize - round(maxColOffset);
		// 	left = false;
		// }
		// else {
		// 	// xDim = halfSize + round(maxColOffset);
		// 	left = true;
		// }
		float distanceThreshold = 0;
		float currX, currY, currX2, currY2, currDist;
		int newPoints = 0;
		vector<double> im2Strengths, imRStrengths;
		vector<Point2f> im2Corners, imRCorners;
		list< vector<Point2f> > im2Suppressed, imRSuppressed;
		harris(im2, im2Strengths, im2Corners);
		harris(imR, imRStrengths, imRCorners);
		suppress(im2Corners, im2Strengths, im2Suppressed);
		suppress(imRCorners, imRStrengths, imRSuppressed);
		//find new points for im2
		for (list< vector<Point2f> >::iterator it2 = im2Suppressed.begin(); it2 != im2Suppressed.end(); it2++) {
			if (newPoints < invalidCount) {
				for (list< vector<Point2f> >::iterator it = im2Data.correspondencesLL.begin(); it != im2Data.correspondencesLL.end(); it++) {
					currX2 = (*it2)[0].x;
					currY2 = (*it2)[0].y;
					currX = (*it).back().x;
					currY = (*it).back().y;
					currDist = sqrt(((currX2-currX)*(currX2-currX)) + ((currY2-currY)*(currY2-currY)));
					if (currDist >= distanceThreshold) {
						vector<Point2f> tempVector;
						tempVector.push_back(Point2f(currX2,currY2));
						im2Data.correspondencesLL.push_back(tempVector);
						newPoints++;
						break;
					}
				}
			}
			else break;
		}
		//otherwise just add any point, caution may add identical points!
		if (newPoints != invalidCount) {
			printf("no new features, adding top suppressed points\n");
			list< vector<Point2f> >::iterator it2 = im2Suppressed.begin();
			for (int i = 0; i < (invalidCount-newPoints); i++) {
				vector<Point2f> tempVector;
				tempVector.push_back((*it2)[0]);
				im2Data.correspondencesLL.push_back(tempVector);
				it2++;
			}
		}

		//now do it for the right image!
		newPoints = 0;
		for (list< vector<Point2f> >::iterator it2 = imRSuppressed.begin(); it2 != imRSuppressed.end(); it2++) {
			if (newPoints < invalidCount) {
				for (list< vector<Point2f> >::iterator it = im1Data.correspondencesLR.begin(); it != im1Data.correspondencesLR.end(); it++) {
					currX2 = (*it2)[0].x;
					currY2 = (*it2)[0].y;
					currX = (*it).back().x;
					currY = (*it).back().y;
					currDist = sqrt(((currX2-currX)*(currX2-currX)) + ((currY2-currY)*(currY2-currY)));
					if (currDist >= distanceThreshold) {
						vector<Point2f> tempVector;
						tempVector.push_back(Point2f(currX2,currY2));
						im1Data.correspondencesLR.push_back(tempVector);
						newPoints++;
						break;
					}
				}
			}
			else break;
		}
		//otherwise just add any point, caution may add identical points!
		if (newPoints != invalidCount) {
			printf("no new features, adding top suppressed points\n");
			list< vector<Point2f> >::iterator it2 = imRSuppressed.begin();
			for (int i = 0; i < (invalidCount-newPoints); i++) {
				vector<Point2f> tempVector;
				tempVector.push_back((*it2)[0]);
				im1Data.correspondencesLR.push_back(tempVector);
				it2++;
			}
		}
	}
}

//returns N-by-2 matrix of (x,y) harris corner coordinates
void harris(const Mat& im, vector<double>& strengths, vector<Point2f>& corners) {
	Mat window = Mat::zeros(3,3,CV_32FC1);
	Mat maxPts = Mat::zeros(im.size(), CV_64FC1);
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
			minMaxLoc(window, 0, &maxVal, 0, &currMax, Mat());
			if (maxVal > .00001) {
				maxPts.at<double>(((double) currMax.y)+row,((double) currMax.x)+col) = maxVal;
			}
		}
	}
	
	//extract coordinates of nonzero points (the max pts)
	// corners = M(countNonZero(maxPts),2,CV_32FC1);
	corners.reserve(countNonZero(maxPts));
	strengths.reserve(corners.size());
	// findNonZero(maxPts,corners); /* finds all nonzero elements, but only in opencv>=2.4.4 */
	int rowIndex = 0;
	for (int row = 0; row < maxPts.rows; row++) {
		for (int col = 0; col < maxPts.cols; col++) {
			maxVal = maxPts.at<double>(row,col);
			if (maxVal != 0) {
				corners.push_back(Point2f(col,row));
				strengths.push_back(maxVal);
				rowIndex++;
			}
		}
	}
}

void suppress(const vector<Point2f>& corners, const vector<double>& strengths, list< vector<Point2f> >& suppressedPoints) {
	float currDist, minDist, xi, xj, yi, yj;
	vector< pair<float,float> > distances;
	distances.reserve(corners.size());
	
	for (int i = 0; i < (int) corners.size(); i++) {
		minDist = INFINITY;
		for (int j = 0; j < (int) corners.size(); j++) {
			if (i != j) {
				if (strengths[i] < (strengths[j]*.9)) {
					xi = corners[i].x;
					yi = corners[i].y;
					xj = corners[j].x;
					yj = corners[j].y;
					currDist = sqrt(((xj-xi)*(xj-xi)) + ((yj-yi)*(yj-yi)));
					if (currDist < minDist) minDist = currDist;
				}
			}
		}
		distances.push_back(pair<float,float>(minDist,i));
	}
	sort(distances.begin(), distances.end(), comparePair);
	for (int i = 0; i < numPoints; i++) {
		vector<Point2f> tempVector;
		tempVector.push_back(corners[(distances[i].second)]);
		suppressedPoints.push_back(tempVector);
	}
}

bool comparePair (const pair<float, float>& pair1, const pair<float, float>& pair2) {
	return pair1.first > pair2.first;
}

void writeMat(const Mat& matrix, const char *filename) {
	ofstream fout(filename);

	if (!fout) {
		cout << "error, file not opened" << endl;
	}
	else {
		for (int i = 0; i < matrix.rows; i++) {
			for (int j = 0; j < matrix.cols; j++) {
				fout << (float) matrix.at<float>(i,j) << "\t";
			}
			fout << endl;
		}
		fout.close();
	}
}
