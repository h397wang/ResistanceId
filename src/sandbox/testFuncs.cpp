// sandbox/main.cpp
#include <stdio.h>
#include <stdlib.h>

#include "testWrappers.h"

int main( int argc, char* argv[] ) {
    char* vpFilePath;

    if ( argc < 2 ) {
        printf("Usage: ./<exeName> <funcNum> <imagePath>\n");
        printf("Enter number for specified function:\n"
            "0: test_getResistorContours\n"
            "1: test_haarCascade\n"
            "2: test_filterNoise\n"
            "3: test_detectResistorRoiBox2D\n"
            "4: test_detectResistorValue\n"
            "5: test_detectVertLines\n"
            "6: test_detectResistorBody\n"
            );
        return -1;
    }
    
    if ( argc > 2 ) {
        vpFilePath = argv[2];
    }

    int vChoice = atoi( argv[1] );
    switch( vChoice ) {
    case 0:
        test_getResistorContours( vpFilePath );
        break;
    case 1:
        test_haarCascade( vpFilePath );
        break;
    case 2:
        test_filterNoise( vpFilePath );
        break;
    case 3:
        test_detectResistorRoiBox2D( vpFilePath );
        break;
    case 4:
        test_detectResistorValue( vpFilePath );
        break;
    case 5:
        test_detectVertLines( vpFilePath );
        break;
    case 6:
        test_detectResistorBody( vpFilePath );
        break;
    default:
        printf("Invalid option.\n");
        break;
    }
}

