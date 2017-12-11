from optparse import OptionParser
import glob
import os

if __name__ == '__main__':

	#TODO(hwang): Use absolute paths instead of this relative bs.
	cmd = "../build/cvutils/traincascade"
	cmd += " -data ../data"
	cmd += " -vec ../vec/out.vec"
	cmd += " -bg ../images/neg.txt"
	cmd += " -numPos 1000"
	cmd += " -numNeg 500"
	cmd += " -numStages 9"
	cmd += " -w 128"
	cmd += " -h 128"
	cmd += " -mode ALL"
	print(cmd)
	#os.system(cmd)
