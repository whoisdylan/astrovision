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
	//
	//
	//
	//plot correspondence pair
	// namedWindow("fig", CV_WINDOW_AUTOSIZE);
	// namedWindow("fig2", CV_WINDOW_AUTOSIZE);
	// Mat currPlot1 = currIm1.clone();
	// Mat currPlot2 = currIm2.clone();
	// for (int i = 0; i < numPoints; i++) {
	// 	if (!isnan(currIm1Data.correspondencesNext.at<float>(i,1))) {
	// 		circle(currPlot1, Point(currIm1Data.correspondencesNext.at<float>(i,1),currIm1Data.correspondencesNext.at<float>(i,2)), 8, Scalar(255,0,0), 3, 8, 0);
	// 	}
	// 	if (!isnan(currIm2Data.correspondencesPrev.at<float>(i,1))) {
	// 		// printf("currPt: %f\n", currIm2Data.correspondencesPrev.at<float>(i,1));
	// 		circle(currPlot2, Point(currIm2Data.correspondencesPrev.at<float>(i,1),currIm2Data.correspondencesPrev.at<float>(i,2)), 8, Scalar(255,0,0), 3, 8, 0);
	// 	}
	// }
	// resize(currPlot1,currPlot1,Size(round(.5*currPlot1.cols),round(.5*currPlot1.rows)),.5,.5,INTER_CUBIC);
	// resize(currPlot2,currPlot2,Size(round(.5*currPlot2.cols),round(.5*currPlot2.rows)),.5,.5,INTER_CUBIC);
	// imshow("fig",currPlot1);
	// imshow("fig2",currPlot2);
	// waitKey(0);
