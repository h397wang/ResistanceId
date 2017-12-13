// utils.h

#pragma once

#include "cv.hpp"

#include <math.h>

typedef struct {
    CvPoint mStart;
    CvPoint mEnd; 
} lineStartEnd_t;

typedef struct {
    CvSeq* mpLines;
} lineGroup_t;

void filterNoise(
    IplImage* apImgSrc,
    IplImage* apImgDst
    );

CvBox2D* getResistorRoi(
    IplImage* apImg,
    int aHoughLineAccumThresh = 65 // determined through characterization, depends on img size
    );

lineStartEnd_t getLineFromPolar(
    CvSize aSize,
    float aRho,
    float aTheta
    );

lineStartEnd_t sortLines(
    CvSeq* vpLines
    );

CvSeq* getResistorContours(
	IplImage* apImg,
	int aCannyThreshLow,
	int aCannyThreshHigh,
	int aContourMode
	);
