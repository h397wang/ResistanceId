// utils.cpp


#include "utils.h"

CvBox2D* getResistorRoi( IplImage* apImg ) {
	CvBox2D* vpRoi;
	return vpRoi;
}

CvSeq* getResistorContours(
	IplImage* apImg,
	int aCannyThreshLow,
	int aCannyThreshHigh,
	int aContourMode
	) 
{

	CvMemStorage* vpStorage = cvCreateMemStorage(0);
	CvSeq* vpContours = NULL;

	IplImage* vpImgEdge = cvCreateImage( cvGetSize(apImg), 8, 1 );
	cvCvtColor( apImg, vpImgEdge, CV_BGR2GRAY );
	cvCanny(
		vpImgEdge,
		vpImgEdge,
		aCannyThreshLow,
		aCannyThreshHigh,
		3 // aperture size
		);

	int vNumContours = cvFindContours(
		vpImgEdge,
		vpStorage,
		&vpContours,
		sizeof(CvContour), // header size
		aContourMode
		);

	// Clean up
	cvClearMemStorage( vpStorage );

	return vpContours;
	
}