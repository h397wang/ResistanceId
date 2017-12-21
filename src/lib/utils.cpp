// utils.cpp

#include "utils.h"

void filterNoise(
    IplImage* apImgSrc,
    IplImage* apImgDst
    )
{
    bool vValidInput = (1
        && (apImgSrc->width == apImgDst->width)
        && (apImgSrc->height == apImgDst->height)
        && (apImgSrc->depth == apImgDst->depth)
        && apImgSrc->nChannels == apImgDst->nChannels
        );
    if (!vValidInput) {
        printf("Invalid args, must have same size, depth and channel.\n");
        return; 
    }
    
    // Median blur
    int vKernelSize = 3;
    cvSmooth(
        apImgSrc,
        apImgDst,
        CV_MEDIAN,
        vKernelSize
        );

    // Bilateral Gaussian cannot be done in place
    IplImage* vpImgFilteredBilateral = cvCreateImage( 
        cvGetSize( apImgSrc ),
        apImgSrc->depth,
        apImgSrc->nChannels
        );

    vKernelSize = 5;
    cvSmooth(
        apImgDst,
        vpImgFilteredBilateral, // cannot be done in place
        CV_BILATERAL,
        vKernelSize,
        vKernelSize
        ); 

    // Morphological opening
    int vNumIterations = 3;
    CvArr* vpTemp = NULL;
    IplConvKernel* vpKernel = NULL;
    cvMorphologyEx(
        vpImgFilteredBilateral,
        apImgDst,
        vpTemp,
        vpKernel,           
        CV_MOP_OPEN,
        vNumIterations
    );
}

/*
apImgSrc	- gray scale
apImgDst	- gray scale
*/
void filterForLineDetect(
    IplImage* apImgSrc,
    IplImage* apImgDst
    )
{
	bool vValidInput = (1
		&& (apImgSrc->nChannels == 1)
        && (apImgSrc->width == apImgDst->width)
        && (apImgSrc->height == apImgDst->height)
        && (apImgSrc->depth == apImgDst->depth)
        && apImgSrc->nChannels == apImgDst->nChannels
        );

    IplImage* vpImgFiltered = cvCreateImage( 
        cvGetSize( apImgSrc ),
        apImgSrc->depth,
        apImgSrc->nChannels
        );

    const int vKernelSize = 5;
    cvSmooth(
		apImgSrc,
		apImgDst,
		CV_BLUR,
		vKernelSize,
		vKernelSize
		);

    const int vNumErosions = 2;
    cvErode(
		apImgDst,
		apImgDst,
		NULL,
		vNumErosions
		);
}

CvBox2D* getResistorRoi(
	IplImage* apImg,
	int aHoughLineAccumThresh
	)
{
	IplImage* vpImageFiltered = cvCreateImage( 
		cvGetSize( apImg ),
		apImg->depth,
		apImg->nChannels
		);
	filterNoise(apImg, vpImageFiltered);

	IplImage* vpImgFilteredGray = cvCreateImage( 
		cvGetSize( apImg ),
		apImg->depth,
		1
		);
	cvCvtColor(
		vpImageFiltered,
		vpImgFilteredGray,
		CV_BGR2GRAY
		);

	filterForLineDetect(
    	vpImgFilteredGray,
    	vpImgFilteredGray
    	);

	IplImage* vpImgEdges = cvCreateImage( 
		cvGetSize( apImg ),
		apImg->depth,
		1
		);
	const double vCannyLowThresh = 20;
	const double vCannyUppThresh = 60; 
    cvCanny(
		vpImgFilteredGray,
		vpImgEdges,
		vCannyLowThresh,
		vCannyUppThresh
		);

	CvMemStorage* vpStorage = cvCreateMemStorage(0);
	const double vRho = 1;
	const double vTheta = 1 * M_PI / 180;
	const int vHoughMethod = CV_HOUGH_STANDARD; //CV_HOUGH_PROBABILISTIC
	CvSeq* vpLines = cvHoughLines2(
		vpImgEdges,
		vpStorage,
		vHoughMethod,
		vRho,	
		vTheta,
		aHoughLineAccumThresh
		);

	for ( int i = 0; i < vpLines->total; i++ ) {
		if ( vHoughMethod == CV_HOUGH_PROBABILISTIC ) {
			CvPoint* vpPoints = (CvPoint*) cvGetSeqElem( vpLines , i );
			cvLine(apImg, vpPoints[0], vpPoints[1], cvScalar(0, 0, 0));

		} else if ( vHoughMethod == CV_HOUGH_STANDARD ) {
			float* vpRhoTheta = (float*) cvGetSeqElem( vpLines , i );
			float vRho = vpRhoTheta[0];
			float vTheta = vpRhoTheta[1];
			lineStartEnd_t vLine = getLineFromPolar(cvGetSize( apImg ), vRho, vTheta );
			cvLine(apImg, vLine.mStart, vLine.mEnd, cvScalar(0, 0, 0));
		}	
	}

	lineGroup_t vLineGroupOfInterest = getLineGroupOfInterest(vpLines);
	lineStartEnd_t vLineOfInterest = getLineFromPolar(
		cvGetSize( apImg ),
		vLineGroupOfInterest.mRhoSum / vLineGroupOfInterest.mNumLines,
		vLineGroupOfInterest.mThetaSum / vLineGroupOfInterest.mNumLines
		);
	cvLine(apImg, vLineOfInterest.mStart, vLineOfInterest.mEnd, cvScalar(255, 255, 255));

	CvBox2D* vpRoi;
	return vpRoi;
}

lineStartEnd_t getLineFromPolar(
	CvSize aSize,
	float aRho,
	float aTheta
	)
{
	float vX = aRho * cos(aTheta);
	float vY = aRho * sin(aTheta);
	float vLineSlope =  - (vX / vY);
	int vYInter = round( vY - vLineSlope * vX );
	int vXinter = - round(vYInter / vLineSlope);
	int vYRightBorderInter = vY + vLineSlope * (aSize.width - vX);

	CvPoint vSrtPnt = cvPoint( 0, vYInter );
	CvPoint vEndPnt = cvPoint( aSize.width, vYRightBorderInter );

	bool vIsPlottable = cvClipLine(
		aSize,
		&vSrtPnt,
		&vEndPnt
		);

	lineStartEnd_t vLine;

	if (vIsPlottable) {
		vLine.mStart = vSrtPnt;
		vLine.mEnd = vEndPnt;		
	}

	return vLine;
}

lineGroup_t getLineGroupOfInterest(
    CvSeq* apLines,
    float aRhoEps,
    float aThetaEps
    )
{

	CvMemStorage* vpStorage = cvCreateMemStorage(0);

	// Create linked list of line groups
	CvSeq* vpLineGroups = cvCreateSeq(
		0,
		sizeof(CvSeq),
		sizeof(lineGroup_t),
		vpStorage
	);

	float* vpRhoTheta = (float*) cvGetSeqElem( apLines , 0 );
	float vRho = vpRhoTheta[0];
	float vTheta = vpRhoTheta[1];

	lineGroup_t vFirstLineGroup = { 1, vRho, vTheta };

	// Add the first line group
	cvSeqPush(
		vpLineGroups,		
		&vFirstLineGroup
	);

	// Go through the rest of the lines
	for ( int i = 1; i < apLines->total; i++ ) {
		vpRhoTheta = (float*) cvGetSeqElem( apLines , i );
		vRho = vpRhoTheta[0];
		vTheta = vpRhoTheta[1];

		// Check against each line group
		for ( int j = 0; j < vpLineGroups->total; j++ ) {
			lineGroup_t* vpCurrentLineGroup = (lineGroup_t*) cvGetSeqElem( vpLineGroups , j );
			float vRhoAvg = vpCurrentLineGroup->mRhoSum / vpCurrentLineGroup->mNumLines;
			float vThetaAvg = vpCurrentLineGroup->mThetaSum / vpCurrentLineGroup->mNumLines;
			
			if ( 1
				&& ( abs(vRho - vRhoAvg) < aRhoEps ) 
				&& ( abs(vTheta - vThetaAvg) < aThetaEps ) ) {
				// Add to the current line group
				vpCurrentLineGroup->mRhoSum += vRho;
				vpCurrentLineGroup->mThetaSum += vTheta;
				vpCurrentLineGroup->mNumLines++;
			} else {
				// Make a new line group

				lineGroup_t vNewLineGroup = { 1, vRho, vTheta };
				cvSeqPush( vpLineGroups, &vNewLineGroup );
			}
		}
	}

	// Get the line group to be returned
	lineGroup_t vLineGroupToRet =  * ((lineGroup_t*) cvGetSeqElem( vpLineGroups , 0 ) );
	for ( int i = 1; i < vpLineGroups->total; i++ ) {
		lineGroup_t* vpCurrentLineGroup = (lineGroup_t*) cvGetSeqElem( vpLineGroups , i );
		if ( vpCurrentLineGroup->mNumLines > vLineGroupToRet.mNumLines ) {
			vLineGroupToRet = * vpCurrentLineGroup;
		}
	}

	cvReleaseMemStorage( &vpStorage );

	return vLineGroupToRet; 
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