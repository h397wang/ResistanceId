// testWrappers.cpp

#include "testWrappers.h"

IplImage* gpImg = NULL;
IplImage* gpImgFiltered = NULL;

int gCannyThreshLow = 110;
int gCannyThreshHigh = 150;
int gContourMode = CV_RETR_LIST;

int gHoughLineAccumThresh = 80;

using namespace cv;
using namespace std;

const char* gpWindowNameOutput = "Output";
const char* gpWindowNameOriginal = "Original";

static CvScalar gColors[] = {
    CvScalar(0,0,255),
    CvScalar(0,128,255),
    CvScalar(0,255,255),
    CvScalar(0,255,0),
    CvScalar(255,128,0),
    CvScalar(255,255,0),
    CvScalar(255,0,0),
    CvScalar(255,0,255),
};

void trackbarCallback_getResistorContours( int aUnused ) {
    CvSeq* vpContours = NULL;
    CvMoments vMoments;

    IplImage* gpImgTmp = cvCreateImage( 
        cvSize( gpImg->width, gpImg->height ),
        gpImg->depth,
        gpImg->nChannels
        );
    cvCopy( gpImg, gpImgTmp );

    vpContours = getResistorContours(
        gpImgTmp,
        gCannyThreshLow,
        gCannyThreshHigh,
        gContourMode
        );

    int seqCount = 0;
    CvSeq* vpCurrentSeq = vpContours;

    while (vpCurrentSeq != NULL) {
        if (vpCurrentSeq->total > 100) {
            printSeqInfo(vpCurrentSeq);

            cvMoments(
                vpCurrentSeq,
                &vMoments
                );
            printMomentInfo(&vMoments);

            cvDrawContours(
                gpImgTmp,
                vpCurrentSeq,
                cvScalarAll(0),     // external colour
                cvScalarAll(155),   // hole colour
                0,                  // max level
                4                   // line thickness
                );
        }
        vpCurrentSeq = vpCurrentSeq->h_next;
    }
    seqCount++;

    cvShowImage( gpWindowNameOutput, gpImgTmp );
    cvShowImage( gpWindowNameOriginal, gpImg );

    printf("gCannyThreshLow: %d\n", gCannyThreshLow);
    printf("gCannyThreshHigh: %d\n", gCannyThreshHigh);
    printf("gContourMode: %d\n", gContourMode);
}

int test_getResistorContours( char* apImagePath ) {

    gpImg = cvLoadImage( apImagePath );

    if ( gpImg == NULL ) {
        printf( "Error: could not open image %s", apImagePath );
        return -1;
    }

    cvNamedWindow( gpWindowNameOutput );
    cvNamedWindow( gpWindowNameOriginal );
    
    cvCreateTrackbar(
        "Canny Threshold Low",
        gpWindowNameOutput,
        &gCannyThreshLow,
        255,
        trackbarCallback_getResistorContours
        );

    cvCreateTrackbar(
        "Canny Threshold High",
        gpWindowNameOutput,
        &gCannyThreshHigh,
        255,
        trackbarCallback_getResistorContours
        );

    cvCreateTrackbar(
        "Contour Mode",
        gpWindowNameOutput,
        &gContourMode,
        3,
        trackbarCallback_getResistorContours
        );

    printf("Contour Modes:\n");
    printf("CV_RETR_EXTERNAL\n");
    printf("CV_RETR_LIST\n");
    printf("CV_RETR_CCOMP\n");
    printf("CV_RETR_TREE\n");

    trackbarCallback_getResistorContours(0);
    cvWaitKey(0);
    return 0;
}

int test_haarCascade( char* apImagePath ) {

    Mat frame = imread( apImagePath );

    //const char* vCascadeFile = "/Users/henrywang/Documents/SideProjects/opencv-3.2.0/data/haarcascades/haarcascade_eye.xml";
    const char* vCascadeFile = "/Users/henrywang/Documents/SideProjects/ComputerVisionProj/ResistanceId/data/data_24_24/cascade.xml";
    CascadeClassifier vDetect;
    if ( !vDetect.load(vCascadeFile) ) {
        printf("ERROR: Could not load cascade\n");
        return -1;
    }
    vector<Rect> vRois;

    vDetect.detectMultiScale(
        frame,
        vRois,
        1.1, // scale factor
        4,   // min neighbours?
        0
        |CASCADE_FIND_BIGGEST_OBJECT
        //|CASCADE_DO_ROUGH_SEARCH
        | CASCADE_SCALE_IMAGE,
        Size(24, 24),
        Size(200, 200)
        );

    for ( size_t i = 0; i < vRois.size(); i++ ) {
        Rect r = vRois[i];
        rectangle(
            frame,
            cvPoint(cvRound(r.x), cvRound(r.y)),
            cvPoint(cvRound((r.x + r.width-1)), cvRound((r.y + r.height-1))),
            gColors[0],
            3,
            8,
            0
            );
    }

#if 0
    CvMemStorage* vpStorage = cvCreateMemStorage(0);

    // Relative path from execution directory
    //const char* vCascadeFile = "/Users/henrywang/Documents/SideProjects/ComputerVisionProj/ResistanceId/data/data_24_24/cascade.xml";
    const char* vCascadeFile = "/Users/henrywang/Documents/SideProjects/opencv-3.2.0/data/haarcascades/haarcascade_eye.xml";

    CvHaarClassifierCascade* vpCascadeClassifier = NULL;
    vpCascadeClassifier = (CvHaarClassifierCascade*) cvLoad( vCascadeFile, 0, 0, 0 );
    if ( vpCascadeClassifier == NULL ) {
        printf( "Error: could not open casecade file %s", vCascadeFile );
        return -1;
    } 

    IplImage* vpImg = cvLoadImage( apImagePath ) {
    if ( vpImg == NULL ) {
        printf( "Error: could not open image %s", apImagePath );
        return -1;
    }

    CvSeq* vpRoiSeq = cvHaarDetectObjects(
        vpImg,
        vpCascadeClassifier,
        vpStorage,
        1.1,                        // scale factor
        3,                          // min neighbours
        CV_HAAR_DO_CANNY_PRUNING    // skip "flat" regions
        | CV_HAAR_SCALE_IMAGE ,    // performance optimization
        CvSize(24, 24)               // smallest region to search
    );

    for(int i = 0; i < (vpRoiSeq ? vpRoiSeq->total : 0); i++ ) {
        CvRect* vpRect = (CvRect*) cvGetSeqElem(vpRoiSeq, i);
        cvRectangle(
            vpImg,
            cvPoint(vpRect->x, vpRect->y),
            cvPoint(vpRect->x + vpRect->width, vpRect->y + vpRect->height),
            colors[i % 8]
            );
    }

    cvNamedWindow( gpWindowNameOriginal );
    cvShowImage( gpWindowNameOutput, vpImg );
    cvWaitKey(0);
#endif
    namedWindow(gpWindowNameOutput);
    imshow(gpWindowNameOutput, frame);
    waitKey(0);
    return 0;
}


void trackbarCallback_filterNoise( int aUnused ) {
    filterNoise( gpImg, gpImgFiltered );
    cvShowImage( gpWindowNameOriginal, gpImg );
    cvShowImage( gpWindowNameOutput, gpImgFiltered );
}

int test_filterNoise( char* apImagePath ) {

    gpImg = cvLoadImage( apImagePath );
    if ( gpImg == NULL ) {
        printf( "Error: could not open image %s", apImagePath );
        return -1;
    }

    gpImgFiltered = cvCreateImage( 
        cvSize( gpImg->width, gpImg->height ),
        gpImg->depth,
        gpImg->nChannels
        );

    cvCreateTrackbar(
        "Temp",
        gpWindowNameOutput,
        &gCannyThreshHigh,
        255,
        trackbarCallback_filterNoise
        );

    cvNamedWindow( gpWindowNameOriginal );
    cvNamedWindow( gpWindowNameOutput );
    trackbarCallback_filterNoise( 0 );
    cvWaitKey(0);
    return 0;
}

void trackbarCallback_detectResistorRoiBox2D( int aUnused ) {

    IplImage* vpImgTmp = cvCreateImage( 
        cvSize( gpImg->width, gpImg->height ),
        gpImg->depth,
        3
        );
    cvCopy( gpImg, vpImgTmp );
    
    CvBox2D vRoi = detectResistorRoiBox2D( vpImgTmp, gHoughLineAccumThresh );
    drawCvBox2D( vpImgTmp, vRoi );

    cvShowImage( gpWindowNameOriginal, gpImg );
    cvShowImage( gpWindowNameOutput, vpImgTmp );

    printf( "gHoughLineAccumThresh: %d\n", gHoughLineAccumThresh );
}

int test_detectResistorRoiBox2D( char* apImagePath ) {

    gpImg = cvLoadImage( apImagePath );
    if ( gpImg == NULL ) {
        printf( "Error: could not open image %s", apImagePath );
        return -1;
    }

    cvNamedWindow( gpWindowNameOriginal );
    cvNamedWindow( gpWindowNameOutput );

    cvCreateTrackbar(
        "Accumulator Threshold",
        gpWindowNameOutput,
        &gHoughLineAccumThresh,
        max( gpImg->width, gpImg->height ),
        trackbarCallback_detectResistorRoiBox2D
        );

    trackbarCallback_detectResistorRoiBox2D( 0 );
    cvWaitKey( 0 );
    return 0;
}

void trackbarCallback_detectResistorValue( int aUnused ) {

    IplImage* vpImgTmp = cvCreateImage( 
        cvSize( gpImg->width, gpImg->height ),
        gpImg->depth,
        3
        );
    cvCopy( gpImg, vpImgTmp );
    
    detectResistorValue( gpImg, vpImgTmp );

    cvShowImage( gpWindowNameOriginal, gpImg );
    cvShowImage( gpWindowNameOutput, vpImgTmp );
}

int test_detectResistorValue( char* apImagePath ) {
    
    gpImg = cvLoadImage( apImagePath );
    if ( gpImg == NULL ) {
        printf( "Error: could not open image %s", apImagePath );
        return -1;
    }

    cvNamedWindow( gpWindowNameOriginal );
    cvNamedWindow( gpWindowNameOutput );

    cvCreateTrackbar(
        "Placeholder",
        gpWindowNameOutput,
        &gHoughLineAccumThresh,
        1,
        trackbarCallback_detectResistorValue
        );

    trackbarCallback_detectResistorValue( 0 );
    cvWaitKey( 0 );
    return 0;
}

// TODO: Should be refactored, a lot of repeated code??
void trackbarCallback_detectVertLines( int aUnused ) {

    IplImage* vpImgTmp = cvCreateImage( 
        cvSize( gpImg->width, gpImg->height ),
        gpImg->depth,
        3
        );
    cvCopy( gpImg, vpImgTmp );
    
    vector<int> vVertLines = detectVertLines( gpImg, vpImgTmp);
   
    for( int i = 0; i < vVertLines.size(); i++ ) {
        cvLine( vpImgTmp,
            cvPoint( vVertLines[i], 0 ),
            cvPoint( vVertLines[i], vpImgTmp->height ),
            gColors[4]
            );
    }

    cvShowImage( gpWindowNameOriginal, gpImg );
    cvShowImage( gpWindowNameOutput, vpImgTmp );
}

int test_detectVertLines( char* apImagePath ) {
    
    gpImg = cvLoadImage( apImagePath );
    if ( gpImg == NULL ) {
        printf( "Error: could not open image %s", apImagePath );
        return -1;
    }

    cvNamedWindow( gpWindowNameOriginal );
    cvNamedWindow( gpWindowNameOutput );

    cvCreateTrackbar(
        "Placeholder",
        gpWindowNameOutput,
        &gHoughLineAccumThresh,
        1,
        trackbarCallback_detectVertLines
        );

    trackbarCallback_detectVertLines( 0 );
    cvWaitKey( 0 );
    return 0;
}

void trackbarCallback_detectResistorBody( int aUnused ) {

    IplImage* vpImgTmp = cvCreateImage( 
        cvSize( gpImg->width, gpImg->height ),
        gpImg->depth,
        3
        );
    cvCopy( gpImg, vpImgTmp );
    
    CvRect vCvRect = detectResistorBody( vpImgTmp );
    cvRectangle(
        vpImgTmp,
        cvPoint( vCvRect.x, vCvRect.y ),
        cvPoint( vCvRect.x + vCvRect.width, vCvRect.y + vCvRect.height ),
        cvScalar( 155, 0, 200 )
        );

    cvShowImage( gpWindowNameOriginal, gpImg );
    cvShowImage( gpWindowNameOutput, vpImgTmp );
}

int test_detectResistorBody( char* apImagePath ) {
    
    gpImg = cvLoadImage( apImagePath );
    if ( gpImg == NULL ) {
        printf( "Error: could not open image %s", apImagePath );
        return -1;
    }

    cvNamedWindow( gpWindowNameOriginal );
    cvNamedWindow( gpWindowNameOutput );

    cvCreateTrackbar(
        "Placeholder",
        gpWindowNameOutput,
        &gHoughLineAccumThresh,
        1,
        trackbarCallback_detectResistorBody
        );

    trackbarCallback_detectResistorBody( 0 );
    cvWaitKey( 0 );
    return 0;
}


// TODO: Move to utils, or create another file for print related functions
// ----------------------------------------------------------------------
// helpers
// ----------------------------------------------------------------------

// Assumes a sequence of CvPoint elements
void printSeqInfo( CvSeq* apSeq) {
    printf( "Number of elements in sequence: %d\n", apSeq->total );
    for( int i = 0; i < apSeq->total; ++i ) {
        CvPoint* vPoint = (CvPoint*) cvGetSeqElem( apSeq, i );
        printf("(%d,%d)\n", vPoint->x, vPoint->y );
    }
}

void printMomentInfo( CvMoments* apMoment ) {
    /*
    // spatial moments
    double m00, m10, m01, m20, m11, m02, m30, m21, m12, m03;
    // central moments
    double mu20, mu11, mu02, mu30, mu21, mu12, mu03;
    // m00 != 0 ? 1/sqrt(m00)
    */

    printf("Spatial Moments:\n");
    printf("\tm00: %.2f\n", apMoment->m00);
    printf("\tm10: %.2f\n", apMoment->m10);
    printf("\tm01: %.2f\n", apMoment->m01);
    printf("\tm11: %.2f\n", apMoment->m01);

    printf("Central Moments:\n");
    printf("\tmu20: %.2f\n", apMoment->mu20);
    printf("\tmu11: %.2f\n", apMoment->mu11);
    printf("\tmu02: %.2f\n", apMoment->mu02);
}


