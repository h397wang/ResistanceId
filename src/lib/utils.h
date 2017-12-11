// utils.h

#pragma once

#include "cv.h"
#include "highgui.h"

#include <math.h>

void filterNoise(
    IplImage* apImgSrc,
    IplImage* apImgDst
    );

CvBox2D* getResistorRoi( IplImage* apImg );

CvSeq* getResistorContours(
	IplImage* apImg,
	int aCannyThreshLow,
	int aCannyThreshHigh,
	int aContourMode
	);