from optparse import OptionParser
import glob
import os

if __name__ == '__main__':

	ext = "jpg"
	imgFiles = glob.glob("../images/pos/*.%s" % ext)

	for i in range(len(imgFiles)):
		cmd = "mv %s ../images/pos/r%d.%s" % (imgFiles[i], i, ext)
		os.system(cmd)
		print(cmd)
	print("image files renamed")
