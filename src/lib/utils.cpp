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
        assert(0);
        return; 
    }
    
    // Median blur
    const int vKernelSizeMedian = 3;
    cvSmooth(
        apImgSrc,
        apImgDst,
        CV_MEDIAN,
        vKernelSizeMedian
        );

    // Bilateral Gaussian cannot be done in place
    IplImage* vpImgFilteredBilateral = cvCreateImage( 
        cvGetSize( apImgSrc ),
        apImgSrc->depth,
        apImgSrc->nChannels
        );

    const int vKernelSizeBilat = 5;
    cvSmooth(
        apImgDst,
        vpImgFilteredBilateral, // cannot be done in place
        CV_BILATERAL,
        vKernelSizeBilat,
        vKernelSizeBilat
        ); 

    // Morphological opening
    const int vNumIterations = 3;
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

    cvReleaseImage( &vpImgFilteredBilateral );
}

/*
apImgSrc    - gray scale
apImgDst    - gray scale
*/
void filterForLineDetect(
    IplImage* apImgSrc,
    IplImage* apImgDst
    )
{

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

   cvReleaseImage( &vpImgFiltered );
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
    // Noise filtering done on RGB image
    filterNoise(apImg, vpImageFiltered);

    // Line detection to be done on grayscale image
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
    #if 0
    cvLine(apImg, vLineOfInterest.mStart, vLineOfInterest.mEnd, cvScalar(255, 255, 255));
    #endif

    // Clip lines to image size
    // TODO: extract block below into function
    const CvPoint2D32f vRoiCenter = cvPoint2D32f(
        (vLineOfInterest.mStart.x + vLineOfInterest.mEnd.x) / 2,
        (vLineOfInterest.mStart.y + vLineOfInterest.mEnd.y) / 2
        );

    // Create return value
    // TODO: Bad heuristic for height value, hard code too
    const float vRoiHeight = 0.3 * (float) apImg->height;
    const float vLineLength = sqrt( 0
        + pow(abs(vLineOfInterest.mStart.x - vLineOfInterest.mEnd.x), 2)
        + pow(abs(vLineOfInterest.mStart.y - vLineOfInterest.mEnd.y), 2)
        );
    const float vLineRadFromXAxis = fmodf(vLineGroupAvgTheta, M_PI / 2);

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

    
    { // Clean up
    cvReleaseImage( &vpImageFiltered );
    cvReleaseImage( &vpImgFilteredGray );
    cvReleaseImage( &vpImgEdges );
    } // Clean up

    return vRoiClipped;
}

// TODO: Refactor to use void cvBoxPoints() instead?
cvBox2DCorners_t getBox2dCorners( CvBox2D aBox2D ) {
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

    cvBox2DCorners_t vCorners;
    vCorners.mTopLeft   = vTopLeft;
    vCorners.mTopRight  = vTopRight;
    vCorners.mBotLeft   = vBotLeft;
    vCorners.mBotRight  = vBotRight;

    return vCorners;
}

CvBox2D clipCvBox2DToFit( 
    CvSize aSize,
    CvBox2D aBox2D
    )
{
    cvBox2DCorners_t vCorners = getBox2dCorners( aBox2D );

    // All lines should be plottable
    cvClipLine(
        aSize,
        &(vCorners.mTopLeft),
        &(vCorners.mTopRight)
        );

    cvClipLine(
        aSize,
        &(vCorners.mBotLeft),
        &(vCorners.mBotRight)
        );
    
    const int vWidthTop = sqrt( 0
        + pow( abs( vCorners.mTopLeft.x - vCorners.mTopRight.x ), 2 )
        + pow( abs( vCorners.mTopLeft.y - vCorners.mTopRight.y ), 2 )
        );

    const int vWidthBot = sqrt( 0
        + pow( abs( vCorners.mBotLeft.x - vCorners.mBotRight.x ), 2 )
        + pow( abs( vCorners.mBotLeft.y - vCorners.mTopRight.y ), 2 )
        );

    const int vNewWidth = std::min( vWidthTop, vWidthBot );
    const int vNewOldWidthDelta = aBox2D.size.width - vNewWidth; // positive number

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
    cvBox2DCorners_t vCorners = getBox2dCorners( aBox2D );

    CvScalar vColor = cvScalar(255, 0, 255);
    cvLine(apImg, vCorners.mTopLeft, vCorners.mTopRight, vColor, 3);
    cvLine(apImg, vCorners.mBotLeft, vCorners.mBotRight, vColor, 3);
    cvLine(apImg, vCorners.mTopLeft, vCorners.mBotLeft, vColor, 3);
    cvLine(apImg, vCorners.mTopRight, vCorners.mBotRight, vColor, 3);

}

void drawHoughLines(
    IplImage* apImg,
    CvSeq* apLines,
    const int aHoughMethod
    )
{
    for ( int i = 0; i < apLines->total; i++ ) {
        if ( aHoughMethod == CV_HOUGH_PROBABILISTIC ) {
            CvPoint* vpPoints = (CvPoint*) cvGetSeqElem( apLines , i );
            cvLine(apImg, vpPoints[0], vpPoints[1], cvScalar(0, 0, 0));

        } else if ( aHoughMethod == CV_HOUGH_STANDARD ) {
            float* vpRhoTheta = (float*) cvGetSeqElem( apLines , i );
            float vRho = vpRhoTheta[0];
            float vTheta = vpRhoTheta[1];
            lineStartEnd_t vLine = getLineFromPolar(cvGetSize( apImg ), vRho, vTheta );
            cvLine(apImg, vLine.mStart, vLine.mEnd, cvScalar(155, 255, 0));
        }   
    }
}

void printArrayValues( CvMat* apMat ) {
    printf( "printArrayValues:\n" );
    printf( "Address: %p\n", (void*) apMat );
    for(int row = 0; row < apMat->rows; row++ ) {
        const float* ptr = (const float*)(apMat->data.ptr + row * apMat->step);
        for( int col = 0; col < apMat->cols; col++ ) {
            printf( "%.3f, ", *ptr );
        }
        printf("\n");
    }
}

void printCvBox2DValues( const CvBox2D aBox2D ) {
    printf( "center: (%f, %f)\n", aBox2D.center.x, aBox2D.center.y  );
    printf( "width: %f\n", aBox2D.size.width );
    printf( "height: %f\n", aBox2D.size.height );
    printf( "angle: %f\n", aBox2D.angle );

}

void printImgValues( IplImage* apImg ) {
    printf( "apImg->roi->xOffset: %d\n", apImg->roi->xOffset );
    printf( "apImg->roi->yOffset: %d\n", apImg->roi->yOffset );
    printf( "apImg->depth: %d\n", apImg->depth );
    printf( "apImg->width: %d\n", apImg->width );
    printf( "apImg->height: %d\n", apImg->height );
}

// floats and double types only
template< typename T >
T getDegFromRad( T aRad ) {
    T vDeg = aRad * 180 / M_PI;
    return vDeg;
}

template< typename T >
T getRadFromDeg( T aDeg ) {
    T vDeg = aDeg * M_PI / 180;
    return vDeg;
}

void rotateToAlignRoiAxis(
    IplImage* apImgSrc,
    IplImage* apImgDst,
    const CvBox2D aBox2D
    )
{   
    CvMat* vpAffineMat = cvCreateMat( 2, 3, CV_32F );
    const double vScale = 1.0;
    const double vRotAngleDeg = getDegFromRad( aBox2D.angle );
    cv2DRotationMatrix(
        aBox2D.center,
        vRotAngleDeg,
        vScale,
        vpAffineMat
    );
    cvWarpAffine(
        apImgSrc,
        apImgDst,
        vpAffineMat
        );
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
    //int vXinter = - round(vYInter / vLineSlope);
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

/*
Brief: 
Input:
    CvSeq* apLines - lines returned by hough transform 
    float aRhoEps - max difference in rho value for line to be considered part of line group
    float aThetaEps - max difference in theta value for line to be considered part of line group
Output:
    lineGroup_t vLineGroupToRet - line group that best represents the "direction" of the resistor strip
*/
lineGroup_t getLineGroupOfInterest(
    CvSeq* apLines,
    float aRhoEps,
    float aThetaEps
    )
{

    if ( apLines == NULL ) {
        printf("CvSeq pointer is NULL\n");
    }
    if ( apLines->total == 0 ) {
        printf("CvSeq contains no elements\n");
    }

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


// Unused
CvSeq* getResistorContours(
    IplImage* apImg,
    int aCannyThreshLow,
    int aCannyThreshHigh,
    int aContourMode
    ) 
{

    CvMemStorage* vpStorage = cvCreateMemStorage(0);
    CvSeq* vpContours = NULL;

    IplImage* vpImgEdge = cvCreateImage( cvGetSize(apImg), IPL_DEPTH_8U, 1 );
    cvCvtColor( apImg, vpImgEdge, CV_BGR2GRAY );
    const int vApertureSize = 3;
    cvCanny(
        vpImgEdge,
        vpImgEdge,
        aCannyThreshLow,
        aCannyThreshHigh,
        vApertureSize
        );

    cvFindContours(
        vpImgEdge,
        vpStorage,
        &vpContours,
        sizeof(CvContour),
        aContourMode
        );

    // Clean up
    cvClearMemStorage( vpStorage );

    return vpContours;
}

/*
Brief: convert input image to YCC color space and equalize the Y channel
Input:
    IplImage* apImgSrc - 
Output:
    IplImage* apImgDst -  
*/
void equalizeColorDistribution(
    IplImage* apImgSrc,
    IplImage* apImgDst
    )
{   
    assert( apImgSrc->nChannels == 3 );
    assert( apImgDst->nChannels == 3 );
    
    IplImage* vpImgYcc = cvCreateImage( 
        cvGetSize( apImgSrc ),
        apImgSrc->depth,
        3
        );
    cvCvtColor(apImgSrc, vpImgYcc, CV_RGB2YCrCb);

    IplImage* vpChannelY = cvCreateImage( cvGetSize(apImgSrc), apImgSrc->depth, 1 );
    IplImage* vpChannelCr = cvCreateImage( cvGetSize(apImgSrc), apImgSrc->depth, 1 );
    IplImage* vpChannelCb = cvCreateImage( cvGetSize(apImgSrc), apImgSrc->depth, 1 );
    
    cvSplit(
        vpImgYcc,
        vpChannelY,
        vpChannelCr,
        vpChannelCb,
        NULL
        );

    cvEqualizeHist( vpChannelY, vpChannelY);

    cvMerge(
        vpChannelY,
        vpChannelCr,
        vpChannelCb,
        NULL,
        vpImgYcc
        );

    cvCvtColor(vpImgYcc, apImgDst, CV_YCrCb2RGB);

    cvReleaseImage( &vpImgYcc );
    cvReleaseImage( &vpChannelY );
    cvReleaseImage( &vpChannelCr );
    cvReleaseImage( &vpChannelCb );
}

/*
Brief: locate roi containing resistor body from the resistor strip
Input:
    IplImage* apImg - cropped image containing resistor strip 
Output:
    CvRect vResistorBodyRect - describes bounds containign resistor body 
*/
CvRect detectResistorBody( IplImage* apImg ) {
    IplImage* vpImgLab = cvCreateImage(
        cvGetSize(apImg),
        apImg->depth,
        3
        );
    cvCvtColor(
        apImg,
        vpImgLab,
        CV_RGB2Lab
        );

    CvScalar vXBounds = detectResistorBodyHorizontalAnalysis( vpImgLab );
    CvScalar vYBounds = detectResistorBodyVerticalAnalysis( vpImgLab );

    CvRect vResistorBodyRect = cvRect(
        vXBounds.val[0],
        vYBounds.val[0],
        vXBounds.val[1] - vXBounds.val[0],
        vYBounds.val[1] - vYBounds.val[0]
        );

    return vResistorBodyRect;
}

/*
Brief: Horizontal analysis of resistor strip to locate x bounds of resistor body
Input:
    IplImage* apImg - cropped image containing resistor strip 
Output:
    vector<CvScalar> vXBounds - 2-tuple containing upper and lower x coordinates 
*/
CvScalar detectResistorBodyHorizontalAnalysis( IplImage* apImgLab ) {

    IplImage* vpChannelL = cvCreateImage( cvGetSize(apImgLab), apImgLab->depth, 1 );
    IplImage* vpChannelA = cvCreateImage( cvGetSize(apImgLab), apImgLab->depth, 1 );
    IplImage* vpChannelB = cvCreateImage( cvGetSize(apImgLab), apImgLab->depth, 1 );

    cvSplit(
        apImgLab,
        vpChannelL,
        vpChannelA,
        vpChannelB,
        NULL
        );

    vector<bool> vStripBelongsToRes = vector<bool>();
    const float sigmaThresh = 7.00;

    for( int col = 0; col < apImgLab->width; col++ ) {   
        const CvRect vVertStripRoi = cvRect( col, 0, 1, apImgLab->height );

        cvSetImageROI( vpChannelL, vVertStripRoi );
        cvSetImageROI( vpChannelA, vVertStripRoi );
        cvSetImageROI( vpChannelB, vVertStripRoi );
        
        CvScalar vMeanL = cvScalarAll(0);
        CvScalar vStdDevL = cvScalarAll(0);             
        CvScalar vMeanA = cvScalarAll(0);
        CvScalar vStdDevA = cvScalarAll(0);
        CvScalar vMeanB = cvScalarAll(0);
        CvScalar vStdDevB = cvScalarAll(0);
        
        cvAvgSdv(
            vpChannelL,
            &vMeanL,
            &vStdDevL
            );
        cvAvgSdv(
            vpChannelA,
            &vMeanA,
            &vStdDevA
            );
        cvAvgSdv(
            vpChannelB,
            &vMeanB,
            &vStdDevB
            );

        const float vColorDev = ( 0.1 * vStdDevL.val[0] )
            + ( 0.45 * vStdDevA.val[0] )
            + ( 0.45 * vStdDevB.val[0] );
        
        if ( vColorDev > sigmaThresh ) {
            vStripBelongsToRes.push_back(true);
        } else {
            vStripBelongsToRes.push_back(false);
        }

        cvResetImageROI( vpChannelL );
        cvResetImageROI( vpChannelA );
        cvResetImageROI( vpChannelB );
    }

    // If the distance between two marked columns is less than d0 pixels,
    // all the columns in between are marked as belonging to the resistor.
    const int d0 = 10;
    for( int i = 0; i < vStripBelongsToRes.size() - d0; i++ ) {
        if ( vStripBelongsToRes[i] && vStripBelongsToRes[i + d0] ) {
            for ( int j = i + 1; j < i + d0; j++ ) {
                vStripBelongsToRes[j] = true;
            }
        }
    }

    // Assumes a single area
    int vLowerXBound = 0;
    int vUpperXBound = 0;
    for( int i = 0; i < vStripBelongsToRes.size() - 1; i++ ) {
        if ( ! vStripBelongsToRes[i]
            && vStripBelongsToRes[i + 1] ) {
            vLowerXBound = i + 1;
        } else if ( vStripBelongsToRes[i]
            && ! vStripBelongsToRes[i + 1] ) {
            vUpperXBound = i + 1;
        }
    }

    CvScalar vXBounds = cvScalar( vLowerXBound, vUpperXBound, 0, 0 );
    return vXBounds;
}


/*
Brief: Vertical analysis of resistor strip to locate y bounds of resistor body
Input:
    IplImage* apImg - cropped image containing resistor strip 
Output:
    vector<CvScalar> vYBounds - 2-tuple containing upper and lower y coordinates 
*/
CvScalar detectResistorBodyVerticalAnalysis( IplImage* apImgLab ) {

    IplImage* vpChannelL = cvCreateImage( cvGetSize(apImgLab), apImgLab->depth, 1 );
    IplImage* vpChannelA = cvCreateImage( cvGetSize(apImgLab), apImgLab->depth, 1 );
    IplImage* vpChannelB = cvCreateImage( cvGetSize(apImgLab), apImgLab->depth, 1 );

    cvSplit(
        apImgLab,
        vpChannelL,
        vpChannelA,
        vpChannelB,
        NULL
        );

    vector<bool> vStripBelongsToRes = vector<bool>();      
    const float sigmaThresh = 7.00;

    for( int row = 0; row < apImgLab->height; row++ ) {   
        const CvRect vHoriStripRoi = cvRect( 0, row, apImgLab->width, 1 );

        cvSetImageROI( vpChannelL, vHoriStripRoi );
        cvSetImageROI( vpChannelA, vHoriStripRoi );
        cvSetImageROI( vpChannelB, vHoriStripRoi );
        
        CvScalar vMeanL = cvScalarAll(0);
        CvScalar vStdDevL = cvScalarAll(0);             
        CvScalar vMeanA = cvScalarAll(0);
        CvScalar vStdDevA = cvScalarAll(0);
        CvScalar vMeanB = cvScalarAll(0);
        CvScalar vStdDevB = cvScalarAll(0);
        
        cvAvgSdv(
            vpChannelL,
            &vMeanL,
            &vStdDevL
            );
        cvAvgSdv(
            vpChannelA,
            &vMeanA,
            &vStdDevA
            );
        cvAvgSdv(
            vpChannelB,
            &vMeanB,
            &vStdDevB
            );

        const float vColorDev = ( 0.1 * vStdDevL.val[0] )
            + ( 0.45 * vStdDevA.val[0] )
            + ( 0.45 * vStdDevB.val[0] );

        if ( vColorDev > sigmaThresh ) {
            vStripBelongsToRes.push_back(true);
        } else {
            vStripBelongsToRes.push_back(false);
        }

        cvResetImageROI( vpChannelL );
        cvResetImageROI( vpChannelA );
        cvResetImageROI( vpChannelB );
    }

    const int d0 = 5;
    for( int i = 0; i < vStripBelongsToRes.size() - d0; i++ ) {
        if ( vStripBelongsToRes[i] && vStripBelongsToRes[i + d0] ) {
            for ( int j = i + 1; j < i + d0; j++ ) {
                vStripBelongsToRes[j] = true;
            }
        }
    }

    // Assumes a single area
    int vLowerYBound = 0;
    int vUpperYBound = 0;
    for( int i = 0; i < vStripBelongsToRes.size() - 1; i++ ) {
        if ( ! vStripBelongsToRes[i]
            && vStripBelongsToRes[i + 1] ) {
            vLowerYBound = i + 1;
        } else if ( vStripBelongsToRes[i]
            && ! vStripBelongsToRes[i + 1] ) {
            vUpperYBound = i + 1;
        }
    }

    CvScalar vYBounds = cvScalar( vLowerYBound, vUpperYBound, 0, 0 );
    return vYBounds;
}

/*
Brief: 
Input:
    IplImage* apImg - cropped image containing only reduced resistor body, lab color space
Output:
    vector<CvScalar> vVertLines - 
*/
vector<int> detectVertLines( IplImage* apImg ) {

    IplImage* vpImgLab = cvCreateImage( 
        cvGetSize( apImg ),
        apImg->depth,
        apImg->nChannels
        );

    cvCvtColor(
        apImg,
        vpImgLab,
        CV_RGB2Lab
        );

    IplImage* vpMeanColColors = cvCreateImage( 
        cvSize( apImg->width, 1 ),
        apImg->depth,
        apImg->nChannels
        );

    for(int col = 0; col < vpImgLab->width; col++) {
        const CvRect vVertStrip = cvRect( col, 0, 1, vpImgLab->height );
        cvSetImageROI( vpImgLab, vVertStrip );
        const CvScalar vMeanColor = cvAvg( vpImgLab );
        cvResetImageROI( vpImgLab );

        printf("Mean Color %.2f, %.2f, %.2f \n", vMeanColor.val[0], vMeanColor.val[1], vMeanColor.val[2]);
        cvSet2D( vpMeanColColors, 0, col, vMeanColor );
    }

    const int vNumPrevColsToCmp = apImg->width / 25;

    vector<int> vVertLineXCoords = vector<int>();

    for(int i = vNumPrevColsToCmp; i < apImg->width; i++) {

        const CvRect vPrevColsRect = cvRect(
            i - vNumPrevColsToCmp,
            0,
            vNumPrevColsToCmp,
            apImg->height
            );
        cvSetImageROI( vpMeanColColors, vPrevColsRect );
        const CvScalar vMeanColor = cvAvg( vpMeanColColors );
        printf("Mean Color of prev block: %.2f, %.2f, %.2f \n", vMeanColor.val[0], vMeanColor.val[1], vMeanColor.val[2]);

        cvResetImageROI( vpMeanColColors );

        const CvScalar vCurrentColor = cvGet2D( vpMeanColColors, 0, i );
        float vColorDistance = calcLabColorDistance( vCurrentColor,  vMeanColor );
        printf("color distance: %.2f\n", vColorDistance);
        const float vColorDistThresh = 9.0;
        if ( (vVertLineXCoords.size() == 0 )
            || ( ( vColorDistance > vColorDistThresh )
            && ( i - vVertLineXCoords.back() > vNumPrevColsToCmp ) )
            ) {
            vVertLineXCoords.push_back(i);
        }
    }
    return vVertLineXCoords;
}

/*
Brief: 
Input:
    CvScalar vColor1 - 
    CvScalar vColor2 -    
Output:
    int vResistorValue - resistance of resistor (Ohms)
*/
float calcLabColorDistance(
    CvScalar vColor1,
    CvScalar vColor2
    )
{

    //const float k_L = 1;
    //const float k_C = 1;
    //const float k_H = 1;

    const float S_L = 1;
    const float S_C = 1 + 0.045 * eucNorm( & vColor1.val[1], 2 );
    const float S_H = 1 + 0.015 * eucNorm( & vColor1.val[1], 2 );
    const float deltaC_ab = eucNorm( & vColor1.val[1], 2 ) - eucNorm( & vColor2.val[1], 2 );
    const float deltaH_ab = sqrt( 0
        + pow( vColor1.val[1] - vColor2.val[1], 2 )
        + pow( vColor1.val[2] - vColor2.val[2], 2 )
        - pow( deltaC_ab, 2)
        );
    const float deltaL = vColor1.val[0] - vColor2.val[0];
    const float colorDiff = sqrt( 0
        + pow( deltaL / S_L, 2 )
        + pow( deltaC_ab / S_C, 2 )
        + pow( deltaH_ab / S_H, 2 )
        );

    return colorDiff;
}

// floats and doubles only
template< typename T >
T eucNorm(
    T* apArray,
    int aNumVals
    )
{   
    T vSum = 0;
    for ( int i = 0; i < aNumVals; i++ ) {
        vSum += pow( apArray[i], 2 );
    }
    const float vRet = sqrt(vSum);
    return vRet;
}

/*
Brief: Highest level function 
Input:
    IplImage* apImg - original image containing a resistor 
Output:
    int vResistorValue - resistance of resistor (Ohms)
*/
int detectResistorValue(
    IplImage* apImg,
    IplImage* apImgTmp // TEMP: for debugging and dev
    )
{

    const CvBox2D vRoiBox2D = getResistorRoi( apImg );
    IplImage* vpImgRotated = cvCreateImage( 
        cvGetSize( apImg ),
        apImg->depth,
        apImg->nChannels
        );
    rotateToAlignRoiAxis( apImg, vpImgRotated, vRoiBox2D );

    const CvRect vRoiRect = cvRect(
        vRoiBox2D.center.x - vRoiBox2D.size.width / 2,
        vRoiBox2D.center.y - vRoiBox2D.size.height / 2,
        vRoiBox2D.size.width,
        vRoiBox2D.size.height
        );

    // Extract roi into its own IplImage stucture
    // The reason we do this is because when the roi is set, filter functinos
    // "return" an image that has the size of the roi
    IplImage* vpImgResStrip = cvCreateImage( 
        cvSize( vRoiBox2D.size.width, vRoiBox2D.size.height ),
        apImg->depth,
        apImg->nChannels
        );

    cvSetImageROI( vpImgRotated, vRoiRect );
    cvCopy( vpImgRotated, vpImgResStrip );
    cvResetImageROI( vpImgRotated );

    // Median blur cannot be done in place so create new image
    IplImage* vpImgResStripSmoothed = cvCreateImage( 
        cvGetSize( vpImgResStrip ),
        vpImgResStrip->depth,
        vpImgResStrip->nChannels
        );
    cvCopy( vpImgResStrip, vpImgResStripSmoothed );
    
    // Not sure if this is truly neccary yet
    // We need a blur that doesn't ruin edges
    #if 1
    const int vKernelSize = 7;
    cvSmooth(
        vpImgResStrip,
        vpImgResStripSmoothed,
        CV_MEDIAN,
        vKernelSize
        );

    equalizeColorDistribution(
        vpImgResStripSmoothed,
        vpImgResStripSmoothed
        );
    #endif

    CvRect vResBodyRect = detectResistorBody( vpImgResStripSmoothed );

    // slightly reduce the area of the roi to reduce background noise
    const int vWidthReduction = 4 * vResBodyRect.width / 5;
    const int vHeightReduction = 4 * vResBodyRect.height / 5;
    vResBodyRect.x += vWidthReduction / 2;
    vResBodyRect.y += vHeightReduction / 2;
    vResBodyRect.width -= vWidthReduction / 2;
    vResBodyRect.height -= vHeightReduction / 2;

    IplImage* vpImgResBody = cvCreateImage( 
        cvSize( vResBodyRect.width, vResBodyRect.height ),
        vpImgResStripSmoothed->depth,
        vpImgResStripSmoothed->nChannels
        );
    cvSetImageROI( vpImgResStripSmoothed, vResBodyRect );
    cvCopy( vpImgResStripSmoothed, vpImgResBody );
    cvResetImageROI( vpImgResStripSmoothed );

    vector<int> vVertLines = detectVertLines( vpImgResBody );

    // TEMP: Debug/Dev
    apImgTmp->width = vRoiBox2D.size.width;
    apImgTmp->height = vRoiBox2D.size.height;
    cvCopy( vpImgResStripSmoothed, apImgTmp );  

    cvReleaseImage( &vpImgRotated );
    cvReleaseImage( &vpImgResStrip );
    cvReleaseImage( &vpImgResStripSmoothed );
    cvReleaseImage( &vpImgResBody );

    return 0;
}