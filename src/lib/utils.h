// utils.h

#pragma once

#include "cv.hpp"

#include <math.h>
#include <assert.h>
#include <cstdarg>

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

CvBox2D detectResistorRoiBox2D(
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

void printArrayValues( const CvMat* apMat );

void printImgValues( IplImage* apImg );

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

int getResistorStripImg(
    IplImage* apImgSrc,
    IplImage* apImgDst
    );

void equalizeLabColorDistribution(
    IplImage* apImgSrc,
    IplImage* apImgDst
    );

CvRect detectResistorBody( IplImage* apImg );

CvScalar detectResistorBodyHorizontalAnalysis( IplImage* apImgLab );

CvScalar detectResistorBodyVerticalAnalysis( IplImage* apImgLab );

vector<int> filterFalsePositiveEdges(
    IplImage* apImg,
    vector<int> aEdgeXPos
    );

vector<int> detectVertLines(
    IplImage* apImg,
    IplImage* apImgTmp = NULL
    );

typedef enum {
    CIE94,
    CIEDE2000,
} labColorDistanceMethod_t;

float calcLabColorDistance(
    CvScalar vColor1,
    CvScalar vColor2,
    labColorDistanceMethod_t aMethod
    );

template< typename T >
T eucNorm(
    T* apArray,
    int aNumVals
    );

template< typename T >
T eucNorm( int aNumVals, ... );

void trimRect(
    CvRect* apRect,
    const int aWidthReduction,
    const int aHeightReduction
    );

void getResistorStripImg(
    IplImage* apImg,
    IplImage* apImgResStrip,
    const CvBox2D aRoiBox2D
    );

CvScalar getDominantColor( IplImage* apImg );

int detectResistorValue(
    IplImage* apImg,
    IplImage* apImgTmp = NULL
    );

int median( CvMat* apMat );

CvScalar getMedianColor( IplImage* apImg );
