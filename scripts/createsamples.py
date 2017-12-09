
from optparse import OptionParser
import glob
import os

if __name__ == "__main__":

	ext = "jpg"

	imgFiles = glob.glob("../images/pos/*.%s" % ext)

	for imgFile in imgFiles:
                imgFileName = os.path.basename(imgFile)
		cmd = "../build/cvutils/createsamples"
		cmd += " -img ../images/pos/%s" % imgFileName 		
		cmd += " -bg ../images/neg.txt" 
		cmd += " -maxxangle 0.5"
		cmd += " -maxyangle 0.5"
		cmd += " -maxzangle 0.2"
		cmd += " -bgcolor 240"
		cmd += " -bgthresh 15"
		cmd += " -vec ../vec/%s.vec" % imgFileName
		cmd += " -h 24"
		cmd += " -w 24"
		cmd += " -num 1000"
		os.system(cmd)
		print(cmd)

