// utils.h

#pragma once

#include "cv.hpp"

#include <math.h>

using namespace std;

#define DEBUG 1

typedef struct {
    CvPoint mStart;
    CvPoint mEnd; 
} lineStartEnd_t;

typedef struct {
    int mNumLines;
    float mRhoSum;
    float mThetaSum;
} lineGroup_t;

void filterNoise(
    IplImage* apImgSrc,
    IplImage* apImgDst
    );

CvBox2D getResistorRoi(
    IplImage* apImg,
    const int aHoughLineAccumThresh = 65 // determined through characterization, depends on img size
    );

CvBox2D clipCvBox2DToFit( 
    CvSize aSize,
    CvBox2D aBox2D
    );

void drawCvBox2D(
    IplImage* apImg,
    const CvBox2D aBox2D
    );

lineStartEnd_t getLineFromPolar(
    CvSize aSize,
    float aRho,
    float aTheta
    );

lineGroup_t getLineGroupOfInterest(
    CvSeq* apLines,
    float aRhoEps = 100,
    float aThetaEps = 0.1
    );

CvSeq* getResistorContours(
	IplImage* apImg,
	int aCannyThreshLow,
	int aCannyThreshHigh,
	int aContourMode
	);
