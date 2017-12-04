from optparse import OptionParser
import glob
import os

if __name__ == '__main__':

	parser = OptionParser()

	parser.add_option(
		"-x",
		dest = "ext",
		action = "store",															
		help = "File type",
		default = "jpg"
		)
	
	(options, args) = parser.parse_args()

	imgFiles = glob.glob("neg/*.%s" % options.ext)

	f = open("neg.txt", "w")
	for i in range(len(imgFiles)):
		f.write("neg/img%d.%s\n" % (i, options.ext) )
		cmd = "mv %s neg/img%d.%s" % (imgFiles[i], i, options.ext)
		os.system(cmd)
	f.close()