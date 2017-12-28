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

void trackbarCallback_detectResistorValue( int );
int test_detectResistorValue( char* );

void trackbarCallback_detectVertLines( int );
int test_detectVertLines( char* );

void trackbarCallback_detectResistorBody( int );
int test_detectResistorBody( char* apImagePath );

// helpers
void printSeqInfo( CvSeq* );
void printMomentInfo( CvMoments* );