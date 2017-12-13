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
			lineStartEnd_t vLine;
			vLine = getLineFromPolar(cvGetSize( apImg ), vRho, vTheta );
			cvLine(apImg, vLine.mStart, vLine.mEnd, cvScalar(0, 0, 0));
		}	
	}


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

CvSeq* groupLines(
    CvSeq* apLines
    )
{
	

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