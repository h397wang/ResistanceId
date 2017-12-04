// testWrappers.cpp

#include "testWrappers.h"

#include "../lib/utils.h"


IplImage* gpImg = NULL;

int gCannyThreshLow = 0;
int gCannyThreshHigh = 0;
int gContourMode = 0;


const char* gpWindowNameOutput = "Output";
const char* gpWindowNameOriginal = "Original";

void trackbarCallback_getResistorContours( int aUnused ) {
	CvSeq* vpContours = NULL;

	IplImage* gpImgTmp = cvCreateImage( 
		cvSize( gpImg->width, gpImg->height ),
		gpImg->depth,
		gpImg->nChannels
		);
	cvCopy( gpImg, gpImgTmp );

	vpContours = getResistorContours(
		gpImgTmp,
		gCannyThreshLow,
		gCannyThreshHigh,
		gContourMode
		);

	if( vpContours ) {
		cvDrawContours(
			gpImgTmp,
			vpContours,
			cvScalarAll(0), // external colour
			cvScalarAll(255), // hole colour
			100, // max level
			4 // line thickness
			);
	}

	cvShowImage( gpWindowNameOutput, gpImgTmp );
	cvShowImage( gpWindowNameOriginal, gpImg );

	printf("gCannyThreshLow: %d\n", gCannyThreshLow);
	printf("gCannyThreshHigh: %d\n", gCannyThreshHigh);
	printf("gContourMode: %d\n", gContourMode);
}

int test_getResistorContours( char* apImagePath ) {

	gpImg = cvLoadImage( apImagePath );

	if ( gpImg == NULL ) {
		printf( "Error: could not open image %s", apImagePath );
		return -1;
	}

	cvNamedWindow( gpWindowNameOutput );
	cvNamedWindow( gpWindowNameOriginal );
	
	cvCreateTrackbar(
		"Canny Threshold Low",
		gpWindowNameOutput,
		&gCannyThreshLow,
		255,
		trackbarCallback_getResistorContours
		);

	cvCreateTrackbar(
		"Canny Threshold High",
		gpWindowNameOutput,
		&gCannyThreshHigh,
		255,
		trackbarCallback_getResistorContours
		);

	cvCreateTrackbar(
		"Contour Mode",
		gpWindowNameOutput,
		&gContourMode,
		3,
		trackbarCallback_getResistorContours
		);

	printf("Contour Modes:\n");
	printf("CV_RETR_EXTERNAL\n");
	printf("CV_RETR_LIST\n");
	printf("CV_RETR_CCOMP\n");
	printf("CV_RETR_TREE\n");

	trackbarCallback_getResistorContours(0);
	cvWaitKey(0);
	return 0;
}

