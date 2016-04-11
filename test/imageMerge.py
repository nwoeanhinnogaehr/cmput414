import sys
import os

#transparency
for i in range(1, 10):
	command = "convert images/after00"+str(i)+".png -transparent black images/after00"+str(i)+".png"
	os.system(command)

for i in range (10, 25):
	command = "convert images/after0"+str(i)+".png -transparent black images/after0"+str(i)+".png"
	os.system(command)

#Merging
command = "convert "
for i in range(1, 10):
	command += " images/after00"+str(i)+".png"

for i in range (10, 25):
	command += " images/after0"+str(i)+".png"

#convert background image1 image2 image2 -background none -flatten result

command += " -background none -flatten -transparent black images/RESULT4.png"
os.system(command)

os.system("feh images/RESULT4.png")