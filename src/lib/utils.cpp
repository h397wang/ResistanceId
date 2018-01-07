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

CvBox2D detectResistorRoiBox2D(
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
    //printf( "apImg->roi->xOffset: %d\n", apImg->roi->xOffset );
    ///printf( "apImg->roi->yOffset: %d\n", apImg->roi->yOffset );
    printf( "apImg->depth: %d\n", apImg->depth );
    printf( "apImg->width: %d\n", apImg->width );
    printf( "apImg->height: %d\n", apImg->height );

    for( int y = 0; y < apImg->height; y++ ) {
        uchar* ptr = (uchar*) ( apImg->imageData + y * apImg->widthStep );
        for( int x = 0; x < apImg->width; x++ ) {
            printf("(%d, %d, %d)", ptr[3 * x + 0] , ptr[3 * x + 1] , ptr[3 * x + 2] );
        }
        printf("\n");
    }

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

    const int vWidthReduction = vResistorBodyRect.width / 4;
    const int vHeightReduction = vResistorBodyRect.height / 4;
    trimRect( &vResistorBodyRect, vWidthReduction, vHeightReduction );

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
    vector<int> aEdgeXPos - 
*/
vector<int> filterFalsePositiveEdges(
    IplImage* apImg,
    vector<int> aEdgeXPos
    )
{
    IplImage* vpImg = cvCreateImage( 
        cvGetSize( apImg ),
        apImg->depth,
        apImg->nChannels
        );
    cvCopy( apImg, vpImg );

    // Account for image boundaries
    vector<int>::iterator it = aEdgeXPos.begin();
    aEdgeXPos.insert( it, 0 );
    aEdgeXPos.push_back( apImg->width );
    for(int i = 0; i < aEdgeXPos.size() - 1; i++) {
        const CvRect vStrip = cvRect(
            aEdgeXPos[i],
            0,
            aEdgeXPos[i + 1],
            apImg->height
            );

        cvSetImageROI( vpImg, vStrip );
        cvSmooth(
            vpImg,
            vpImg,
            CV_MEDIAN,
            3
            );

        cvResetImageROI( vpImg );

    }

    cvReleaseImage( &vpImg );
    vector<int> vEdges = vector<int>();
    return vEdges;
}

/*
Brief: 
Input:
    IplImage* apImg - cropped image containing only reduced resistor body, lab color space
Output:
    vector<CvScalar> vVertLines - 
*/
vector<int> detectVertLines(
    IplImage* apImg,
    IplImage* apImgTmp
    )
{

    // Equalization to be done with RGB inputs as the function does conversion internally
    IplImage* vpImgEqualized = cvCreateImage( 
        cvGetSize( apImg ),
        apImg->depth,
        apImg->nChannels
        );
    equalizeColorDistribution(
        apImg,
        vpImgEqualized
        );

    IplImage* vpImgLab = cvCreateImage( 
        cvGetSize( apImg ),
        apImg->depth,
        apImg->nChannels
        );

    cvCvtColor(
        vpImgEqualized,
        vpImgLab,
        CV_RGB2Lab
        );

    const int vCols = 1;
    const int vRows = 5;
    const int vXAnchor = 0;
    const int vYAnchor = 2;
    IplConvKernel* vpKernel = cvCreateStructuringElementEx(
        vCols,
        vRows,
        vXAnchor,
        vYAnchor,
        CV_SHAPE_RECT
        );
    cvMorphologyEx(
        vpImgLab,
        vpImgLab,
        NULL,
        vpKernel,
        CV_MOP_OPEN,
        4
        );

    // TODO: needs to be done in vertical strips
    for ( int i = 0; i < 1; i ++ ) {
        cvSmooth(
            vpImgLab,
            vpImgLab,
            CV_MEDIAN,
            3
            );
    }

    for ( int i = 0; i < 1; i ++ ) {
        cvSmooth(
            vpImgLab,
            vpImgLab,
            CV_BLUR,
            7,
            1
            );
    }

    // detect background color of resistor body
    CvScalar vResBodyColor = getResBodyBgColor( vpImgLab );

    // subtract background color
    

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
        cvSet2D( vpMeanColColors, 0, col, vMeanColor );
    }

    const int vNumColsToCmpLhs = apImg->width / 25;
    const int vNumColsToCmpRhs = 1;  // default 1

    vector<int> vVertLineXCoords = vector<int>();

    for(int i = vNumColsToCmpLhs; i < apImg->width; i++) {

        const CvRect vColsLhsRect = cvRect(
            i - vNumColsToCmpLhs,
            0,
            vNumColsToCmpLhs,
            apImg->height
            );
        cvSetImageROI( vpMeanColColors, vColsLhsRect );
        const CvScalar vMeanColorLhs = cvAvg( vpMeanColColors );
        cvResetImageROI( vpMeanColColors );

        const CvRect vColsRhsRect = cvRect(
            i,
            0,
            vNumColsToCmpRhs,
            apImg->height
            );
        cvSetImageROI( vpMeanColColors, vColsRhsRect );
        const CvScalar vMeanColorRhs = cvAvg( vpMeanColColors );
        cvResetImageROI( vpMeanColColors );
 
        float vColorDistance = calcLabColorDistance( vMeanColorLhs,  vMeanColorRhs, CIE94 );
        //printf("color distance: %.2f\n", vColorDistance);
        const float vColorDistThresh = 7.0;

        if ( (vVertLineXCoords.size() == 0 )
            || ( ( vColorDistance > vColorDistThresh )
            && ( i - vVertLineXCoords.back() > vNumColsToCmpLhs ) )
            ) {
            vVertLineXCoords.push_back(i);
        }
    }

    if ( apImgTmp != NULL ) { 
        cvCvtColor(
            vpImgLab,
            apImgTmp,
            CV_Lab2RGB
            );
    }

    cvReleaseStructuringElement( &vpKernel );

    cvReleaseImage( &vpImgLab );
    cvReleaseImage( &vpMeanColColors );

    return vVertLineXCoords;
}


float calcLabColorDistance_CIE94(
    CvScalar vColor1,
    CvScalar vColor2
    )
{
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
    const float deltaE = sqrt( 0
        + pow( deltaL / S_L, 2 )
        + pow( deltaC_ab / S_C, 2 )
        + pow( deltaH_ab / S_H, 2 )
        );

    return deltaE;
}


float calcLabColorDistance_CIEDE2000(
    CvScalar vColor1,
    CvScalar vColor2
    )
{

    const float deltaL = vColor2.val[0] - vColor1.val[0];
    const float meanL = ( vColor2.val[0] + vColor1.val[0] ) / 2;
    const float meanC = ( vColor2.val[0] + vColor1.val[0] ) / 2;
    const float a_1_prime = vColor1.val[1]
        + ( vColor1.val[1] / 2 )
        * ( 1 - sqrt( pow( meanC, 7 ) / ( pow( meanC, 7 ) + pow( 25, 7 ) ) ) );
    const float a_2_prime = vColor2.val[1]
        + ( vColor2.val[1] / 2 )
        * ( 1 - sqrt( pow( meanC, 7 ) / ( pow( meanC, 7 ) + pow( 25, 7 ) ) ) );
    const float C_1_prime = eucNorm<float>( 2, a_1_prime, vColor1.val[2] );
    const float C_2_prime = eucNorm<float>( 2, a_2_prime, vColor2.val[2] );
    
    const float deltaE = 0;
    return deltaE;
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
    CvScalar aColor1,
    CvScalar aColor2,
    labColorDistanceMethod_t aMethod
    )
{
    float vColorDiff = 0;

    switch (aMethod) {
    case CIE94:
        vColorDiff = calcLabColorDistance_CIE94( aColor1, aColor2 );
        break;
    case CIEDE2000:
        vColorDiff = calcLabColorDistance_CIEDE2000( aColor1, aColor2 );     
        break;
    default:
        printf("Invalid argument aMethod\n");
        assert(0);
        break;
    }
    return vColorDiff;
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

template< typename T >
T eucNorm( int aNumVals, ... ) {
    va_list vNumList;
    va_start( vNumList, aNumVals ); 

    T vSum = 0;
    for ( int i = 0; i < aNumVals; i++ ) {
        T vNum = (T) va_arg( vNumList, double );
        vSum += pow( vNum, 2 );
    }
    va_end( vNumList );

    const float vRet = sqrt(vSum);
    return vRet;
}

void trimRect(
    CvRect* apRect,
    const int aWidthReduction,
    const int aHeightReduction
    )
{
    apRect->x += aWidthReduction / 2;
    apRect->y += aHeightReduction / 2;
    apRect->width -= aWidthReduction / 2;
    apRect->height -= aHeightReduction / 2;

    if ( aWidthReduction & 1 ) {
        apRect->width -= 1;        
    }
    if ( aHeightReduction & 1 ) {
        apRect->height -= 1;        
    }
}

void getResistorStripImg(
    IplImage* apImg,
    IplImage* apImgResStrip,
    const CvBox2D aRoiBox2D
    )
{
    IplImage* vpImgRotated = cvCreateImage( 
        cvGetSize( apImg ),
        apImg->depth,
        apImg->nChannels
        );
    rotateToAlignRoiAxis( apImg, vpImgRotated, aRoiBox2D );

    const CvRect vRoiRect = cvRect(
        aRoiBox2D.center.x - aRoiBox2D.size.width / 2,
        aRoiBox2D.center.y - aRoiBox2D.size.height / 2,
        aRoiBox2D.size.width,
        aRoiBox2D.size.height
        );
    
    cvSetImageROI( vpImgRotated, vRoiRect );
    // Assumes dst image is the correct size
    cvCopy( vpImgRotated, apImgResStrip );
    cvResetImageROI( vpImgRotated );

    cvReleaseImage( &vpImgRotated );
}


/*
Brief: Color of the resistor body should correspond to histogram bin with highest frequency 
        Currently only supports 3 channel images
Input:
    IplImage* apImg - original image containing the resistor body, only supports lab color space input?
Output:
    CvScalar vBgColor - average background color of resistor body
*/
// TODO: may be renamed... to a more generic description
CvScalar getResBodyBgColor( IplImage* apImg ) {

    // Split into seperate channels
    const int vNumChannels = 3;

    IplImage* vppChannels[vNumChannels];
    for ( int i = 0; i < vNumChannels; i++ ) {
        vppChannels[i] = cvCreateImage( cvGetSize(apImg), apImg->depth, 1 );
    }
    cvSplit(
        apImg,
        vppChannels[0],
        vppChannels[1],
        vppChannels[2],
        NULL
        );

    const int vNumBinsL = 8;
    const int vNumBinsA = 16;
    const int vNumBinsB = 16;
    
    const int vLowerBoundL = 0;
    const int vLowerBoundA = 0;
    const int vLowerBoundB = 0;
    
    const int vUpperBoundL = 255;
    const int vUpperBoundA = 255;
    const int vUpperBoundB = 255;

    int vpBinsPerChannel[vNumChannels] = { vNumBinsL, vNumBinsA, vNumBinsB };
    float vpRangeL[2] = { vLowerBoundL, vUpperBoundL };
    float vpRangeA[2] = { vLowerBoundA, vUpperBoundA };
    float vpRangeB[2] = { vLowerBoundB, vUpperBoundB };

    float* vppRanges[vNumChannels] = {
        vpRangeL,
        vpRangeA,
        vpRangeB
        };

    CvHistogram* vpHisto = cvCreateHist(
        vNumChannels,
        vpBinsPerChannel,
        CV_HIST_ARRAY,
        vppRanges,
        1
        );

    cvCalcHist( vppChannels, vpHisto );
    cvNormalizeHist( vpHisto, 1.0 );

    float vMinVal = 0;
    float vMaxVal = 0;
    
    int vpMinIndices[vNumChannels] = { 1, 1, 1 };
    int vpMaxIndices[vNumChannels] = { 1, 1, 1 };
    
    cvGetMinMaxHistValue(
        vpHisto,
        &vMinVal, 
        &vMaxVal, 
        vpMinIndices, 
        vpMaxIndices
        );

    printf("Max value: ");
    printf("%f\n", vMaxVal);

    printf("Min indices: ");    
    printf("%d, %d, %d\n",
        vpMinIndices[0],
        vpMinIndices[1],
        vpMinIndices[2]
        );

    printf("Max indices: ");    
    printf("%d, %d, %d\n",
        vpMaxIndices[0],
        vpMaxIndices[1],
        vpMaxIndices[2]
        );

    const int vValL = (float(vpMaxIndices[0]) + 0.5) * ( vpRangeL[1] - vpRangeL[0] + 1) / vNumBinsL;
    const int vValA = (float(vpMaxIndices[1]) + 0.5) * ( vpRangeA[1] - vpRangeA[0] + 1) / vNumBinsA;
    const int vValB = (float(vpMaxIndices[2]) + 0.5) * ( vpRangeB[1] - vpRangeB[0] + 1) / vNumBinsB;
    
    printf( "CvScalar: %d, %d, %d\n", vValL, vValA, vValB );
    CvScalar vBgColor = cvScalar( vValL, vValA, vValB );
    return vBgColor;
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

    const CvBox2D vRoiBox2D = detectResistorRoiBox2D( apImg );
    IplImage* vpImgResStrip = cvCreateImage( 
        cvSize( vRoiBox2D.size.width, vRoiBox2D.size.height ),
        apImg->depth,
        apImg->nChannels
        );
    getResistorStripImg(
        apImg,
        vpImgResStrip,
        vRoiBox2D
        );

    CvRect vResBodyRect = detectResistorBody( vpImgResStrip );

    IplImage* vpImgResBody = cvCreateImage( 
        cvSize( vResBodyRect.width, vResBodyRect.height ),
        apImg->depth,
        apImg->nChannels
        );
    cvSetImageROI( vpImgResStrip, vResBodyRect );
    cvCopy( vpImgResStrip, vpImgResBody );
    cvResetImageROI( vpImgResStrip );

    vector<int> vVertLines = detectVertLines( vpImgResBody );

    if ( apImgTmp != NULL ) {
        cvCopy( vpImgResBody, apImgTmp );  
    }

    cvReleaseImage( &vpImgResStrip );
    cvReleaseImage( &vpImgResBody );

    return 0;
}