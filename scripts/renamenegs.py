from optparse import OptionParser
import glob
import os

gExtTypes = [
	'jpg',
	'JPG',
	'png',
	'PNG',
	]

if __name__ == '__main__':

	parser = OptionParser()
	
	parser.add_option(
		"--dstExt",
		dest = "dstExt",
		action = "store",															
		help = "File type [jpg]",
		default = "jpg"
		)

	parser.add_option(
		"--baseName",
		dest = "baseName",
		action = "store",								
		help = "Base name of image files [neg]",
		default = "neg"
		)

	(options, args) = parser.parse_args()

	imgDir = os.path.abspath("../images")
	negImgDir = os.path.join(imgDir, "neg")
	tmpDir = os.path.join(imgDir, "tmp")
	if not os.path.exists(tmpDir):
		os.mkdir(tmpDir)
	else:
		cmd = "rm -f " + tmpDir + "/*"
		print(cmd)
		os.system(cmd)

	f = open(os.path.join(imgDir, "neg.txt"), "w")

	imgFiles = []
	for ext in gExtTypes:
		imgFiles.extend(glob.glob(negImgDir + "/*." + ext))

	for i in range(len(imgFiles)):
		numFormated = "%04d" % i
		imgRenamed = options.baseName + numFormated + '.' + options.dstExt
		
		tmpFilePath = os.path.join(tmpDir, imgRenamed)

		cmd = "cp " + imgFiles[i] + " " + tmpFilePath
		print(cmd)
		os.system(cmd)

	cmd = "rm -f " + negImgDir + "/*"
	print(cmd)
	os.system(cmd)

	tmpImgFiles = glob.glob(tmpDir + "/*." + options.dstExt)

	for i in range(len(tmpImgFiles)):
		
		fileName = os.path.basename(tmpImgFiles[i])
		srcFilePath = os.path.abspath(tmpImgFiles[i])
		dstFilePath = os.path.join(negImgDir, fileName)
		
		f.write("neg/" + fileName  + "\n")

		cmd = "cp " + srcFilePath + " " + dstFilePath
		print(cmd)
		os.system(cmd)
	
	f.close()
