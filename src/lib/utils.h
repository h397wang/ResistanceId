// utils.h

#pragma once

#include "cv.h"
#include "highgui.h"

CvBox2D* getResistorRoi( IplImage* apImg );

CvSeq* getResistorContours(
	IplImage* apImg,
	int aCannyThreshLow,
	int aCannyThreshHigh,
	int aContourMode
	);