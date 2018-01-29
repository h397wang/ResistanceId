README.md


## Demo
![alt text](https://github.com/h397wang/ResistanceId/tree/master/demo/demo0.png)
![alt text](https://github.com/h397wang/ResistanceId/tree/master/demo/demo1.png)
![alt text](https://github.com/h397wang/ResistanceId/tree/master/demo/demo2.png)

## Reference
See the following paper:
Automated Resistor Classification by
Pascal Niklaus and Gian Ulli, from
Distributed Computing Group, Computer Engineering and Networks Laboratory, ETH Zürich

Pipeline Description

Key Assumptions
1. Low Noise Background
The image background should be quite plain (e.g a piece of paper).
2. Single Resistor
For simplicity there should only be one resistor present
3. Straight Wires
The wires connected to a resistor body are 

1. Resistor Localization
1.1 Noise Filtering
In this stage the filter attempts to get rid of camera artifacts (e.g salt and pepper noise) in preparation for the next step.

1.2 Hough Line Detection
The resistor body and its wires are not neccesarily perfectly horizontal and is assumed to be in any orientation. The Hough Line algorithm will detect various straight lines in the image based on arbitruary thresholding.

1.3 Statistical Analysis
The set of lines returned from the Hough transform are analyzed and "similar lines" are grouped together. Since

2. Color Ring Detection
Edge Detection
