// sandbox/main.cpp
#include <stdio.h>
#include <stdlib.h>

#include "cv.h"
#include "highgui.h"

#include "testWrappers.h"

int main( int argc, char* argv[] ) {
	char* vpFilePath;

	if (argc < 2) {
		printf("Usage: ./<exeName> <funcNum> <imagePath>\n");
		printf("Enter number for specified function:\n"
			"0: test_getResistorContours\n"
			);
		return -1;
	}
	
	if (argc > 2) {
		vpFilePath = argv[2];
	}

	int vChoice = atoi(argv[1]);
	switch(vChoice){
	case 0:
		test_getResistorContours(vpFilePath);
		break;
	default:
		break;
	}
}

