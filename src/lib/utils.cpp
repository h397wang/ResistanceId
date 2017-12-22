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

CvBox2D getResistorRoi(
	IplImage* apImg,
	const int aHoughLineAccumThresh
	)
{
	// No change to input image, make a copy
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

	// Show the lines that were found
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

	// Get line groups
	const lineGroup_t vLineGroupOfInterest = getLineGroupOfInterest(vpLines);
	const float vLineGroupAvgTheta = vLineGroupOfInterest.mThetaSum / vLineGroupOfInterest.mNumLines;
	const float vLineGroupAvgRho   = vLineGroupOfInterest.mRhoSum / vLineGroupOfInterest.mNumLines;

	const lineStartEnd_t vLineOfInterest = getLineFromPolar(
		cvGetSize( apImg ),
		vLineGroupAvgRho,
		vLineGroupAvgTheta
		);

	// Show dominant line
	cvLine(apImg, vLineOfInterest.mStart, vLineOfInterest.mEnd, cvScalar(255, 255, 255));

	// Add top and bottom parallel lines


	// Clip lines to image size
	// TODO: extract block below into function
	// TODO: not the best idea, because of symmetrical clipping, reconsider
	const CvPoint2D32f vRoiCenter = cvPoint2D32f(
		(vLineOfInterest.mStart.x + vLineOfInterest.mEnd.x) / 2,
		(vLineOfInterest.mStart.y + vLineOfInterest.mEnd.y) / 2
		);

	// Create return value
	// TODO: Bad heuristic for height value
	const float vRoiHeight = 0.3 * (float) apImg->height;
	const float vLineLength = sqrt( 0
		+ pow(abs(vLineOfInterest.mStart.x - vLineOfInterest.mEnd.x), 2)
		+ pow(abs(vLineOfInterest.mStart.y - vLineOfInterest.mEnd.y), 2)
		);
	const float vLineRadFromXAxis = fmodf(vLineGroupAvgTheta, M_PI / 2);

	//const float vWidthClipped = vRoiHeight / tan( vLineRadFromXAxis );
	const float vWidthClipped = vRoiHeight * sin( vLineRadFromXAxis );
	
	const CvSize2D32f vRoiSize = cvSize2D32f(
		vLineLength - vWidthClipped,
		vRoiHeight
		);
	CvBox2D vRoi;
	vRoi.center = vRoiCenter;
	vRoi.size = vRoiSize;
	vRoi.angle = vLineRadFromXAxis;

	CvBox2D vRoiClipped = clipCvBox2DToFit( 
		cvGetSize( apImg ),
		vRoi
		);

#if 0
	printf( "center: (%d, %d)\n", (int) vRoiCenter.x, (int) vRoiCenter.y );
	printf( "size: %d x %d\n", (int) vRoiSize.width, (int) vRoiSize.height );
	printf( "angle: %f\n", vRoi.angle );
#endif
	return vRoiClipped;
}

// TODO: extract duplicated code
CvBox2D clipCvBox2DToFit( 
	CvSize aSize,
	CvBox2D aBox2D
	)
{
	const int vWidthXProj = aBox2D.size.width * cos( aBox2D.angle ) / 2;
	const int vHeightYProj = aBox2D.size.height * cos( aBox2D.angle ) / 2;

	const int vWidthYProj = aBox2D.size.width * sin( aBox2D.angle ) / 2;
	const int vHeightXProj = aBox2D.size.height * sin( aBox2D.angle ) / 2;

	CvPoint vTopLeft = cvPoint(
		aBox2D.center.x - vWidthXProj + vHeightXProj,
		aBox2D.center.y - vHeightYProj - vWidthYProj
		);

	CvPoint vTopRight = cvPoint(
		aBox2D.center.x + vWidthXProj + vHeightXProj,
		aBox2D.center.y - vHeightYProj + vWidthYProj
		);

	CvPoint vBotLeft = cvPoint(
		aBox2D.center.x - vWidthXProj - vHeightXProj,
		aBox2D.center.y + vHeightYProj - vWidthYProj
		);

	CvPoint vBotRight = cvPoint(
		aBox2D.center.x + vWidthXProj - vHeightXProj,
		aBox2D.center.y + vHeightYProj + vWidthYProj
		);

	// All lines should be plottable
	cvClipLine(
		aSize,
		&vTopLeft,
		&vTopRight
		);

	cvClipLine(
		aSize,
		&vBotLeft,
		&vBotRight
		);
	
	const int vWidthTop = sqrt( 0
		+ pow( abs( vTopLeft.x - vTopRight.x ), 2 )
		+ pow( abs( vTopLeft.y - vTopRight.y ), 2 )
		);

	const int vWidthBot = sqrt( 0
		+ pow( abs( vBotLeft.x - vBotRight.x ), 2 )
		+ pow( abs( vBotLeft.y - vTopRight.y ), 2 )
		);

	const int vNewWidth = std::min( vWidthTop, vWidthBot );
	const int vNewOldWidthDelta = aBox2D.size.width - vNewWidth; // positive number
	printf( "vNewWidth: %d\n", vNewWidth );

	const CvPoint2D32f vNewCenter = cvPoint2D32f(
		aBox2D.center.x + vNewOldWidthDelta * cos( aBox2D.angle ) / 2,
		aBox2D.center.y + vNewOldWidthDelta * sin( aBox2D.angle ) / 2
		);
	const CvSize2D32f vNewSize = cvSize2D32f(
		vNewWidth,
		aBox2D.size.height
		);

	CvBox2D vBoxResized;
	vBoxResized.center = vNewCenter;
	vBoxResized.size = vNewSize;
	vBoxResized.angle = aBox2D.angle; // no change

	return vBoxResized;
}

void drawCvBox2D(
	IplImage* apImg,
	const CvBox2D aBox2D
	)
{	

	const int vWidthXProj = aBox2D.size.width * cos(aBox2D.angle) / 2;
	const int vHeightYProj = aBox2D.size.height * cos(aBox2D.angle) / 2;

	const int vWidthYProj = aBox2D.size.width * sin(aBox2D.angle) / 2;
	const int vHeightXProj = aBox2D.size.height * sin(aBox2D.angle) / 2;

	printf( "vWidthXProj: %d\n", vWidthXProj );
	printf( "vHeightYProj: %d\n", vHeightYProj );
	
	const CvPoint vTopLeft = cvPoint(
		aBox2D.center.x - vWidthXProj + vHeightXProj,
		aBox2D.center.y - vHeightYProj - vWidthYProj
		);

	const CvPoint vTopRight = cvPoint(
		aBox2D.center.x + vWidthXProj + vHeightXProj,
		aBox2D.center.y - vHeightYProj + vWidthYProj
		);

	const CvPoint vBotLeft = cvPoint(
		aBox2D.center.x - vWidthXProj - vHeightXProj,
		aBox2D.center.y + vHeightYProj - vWidthYProj
		);

	const CvPoint vBotRight = cvPoint(
		aBox2D.center.x + vWidthXProj - vHeightXProj,
		aBox2D.center.y + vHeightYProj + vWidthYProj
		);

	CvScalar vColor = cvScalar(255, 0, 255);
	cvLine(apImg, vTopLeft, vTopRight, vColor, 3);
	cvLine(apImg, vBotLeft, vBotRight, vColor, 3);
	cvLine(apImg, vTopLeft, vBotLeft, vColor, 3);
	cvLine(apImg, vTopRight, vBotRight, vColor, 3);

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