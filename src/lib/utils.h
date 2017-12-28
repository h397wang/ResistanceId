// utils.h

#pragma once

#include "cv.hpp"

#include <math.h>
#include <assert.h>

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

typedef struct {
    CvPoint mTopLeft;
    CvPoint mTopRight;
    CvPoint mBotLeft;
    CvPoint mBotRight;
} cvBox2DCorners_t;

void filterNoise(
    IplImage* apImgSrc,
    IplImage* apImgDst
    );

CvBox2D getResistorRoi(
    IplImage* apImg,
    const int aHoughLineAccumThresh = 70 // determined through characterization, depends on img size
    );

CvBox2D clipCvBox2DToFit( 
    CvSize aSize,
    CvBox2D aBox2D
    );

void drawCvBox2D(
    IplImage* apImg,
    const CvBox2D aBox2D
    );

void drawCvRect(
    IplImage* apImg,
    const CvRect aRect
    );

void drawHoughLines(
    IplImage* apImg,
    CvSeq* apLines,
    const int aHoughMethod
    );

template< typename T >
T getDegFromRad( T aRad );

template< typename T >
T getRadFromDeg( T aDeg );

void printImgValues( IplImage* apImg );

void printArrayValues( const CvMat* apMat );

void printCvBox2DValues( const CvBox2D aBox2D );

void rotateToAlignRoiAxis(
    IplImage* apImgSrc,
    IplImage* apImgDst,
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

int getResistorStrip(
    IplImage* apImgSrc,
    IplImage* apImgDst
    );

void equalizeColorDistribution(
    IplImage* apImgSrc,
    IplImage* apImgDst
    );

CvRect detectResistorBody( IplImage* apImg );

CvScalar detectResistorBodyHorizontalAnalysis( IplImage* apImgLab );

CvScalar detectResistorBodyVerticalAnalysis( IplImage* apImgLab );

vector<CvScalar> detectVertLines( IplImage* apImg );

int detectResistorValue(
    IplImage* apImg,
    IplImage* apImgTmp
    );

