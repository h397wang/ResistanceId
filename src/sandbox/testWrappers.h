// testWrappers.h

# pragma once

#include "../lib/utils.h"

void trackbarCallback_getResistorContours( int );
int test_getResistorContours( char* );

int test_haarCascade( char* );

void trackbarCallback_filterNoise( int );
int test_filterNoise( char* );

void trackbarCallback_getResistorRoi( int );
int test_getResistorRoi( char* );

// helpers
void printSeqInfo( CvSeq* );
void printMomentInfo( CvMoments* );