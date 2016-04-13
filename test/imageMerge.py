import sys
import os

#This segment makes the images have transparent backdrops.
for i in range(1, 10):
	command = "convert images/after00"+str(i)+".png -transparent black images/after00"+str(i)+".png"
	os.system(command)

for i in range (10, 25):
	command = "convert images/after0"+str(i)+".png -transparent black images/after0"+str(i)+".png"
	os.system(command)

#This segment merges all of the images into a concise mapping.
command = "convert "
for i in range(1, 10):
	command += " images/after00"+str(i)+".png"

for i in range (10, 25):
	command += " images/after0"+str(i)+".png"

command += " -background none -flatten -transparent black images/RESULT.png"
os.system(command)

#Finally, this command opens the resulting image for viewing.
os.system("feh images/RESULT.png")
