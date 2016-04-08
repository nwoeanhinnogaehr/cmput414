import sys
import os

#transparency
for i in range(1, 10):
	command = "convert diff00"+str(i)+".png -transparent black diff00"+str(i)+".png"
	os.system(command)

for i in range (10, 31):
	command = "convert diff0"+str(i)+".png -transparent black diff0"+str(i)+".png"
	os.system(command)

#Merging
command = "convert "
for i in range(1, 10):
	command += " diff00"+str(i)+".png"

for i in range (10, 31):
	command += " diff0"+str(i)+".png"

#convert background image1 image2 image2 -background none -flatten result

command += " -background none -flatten -transparent black RESULT.png"
os.system(command)

os.system("feh RESULT.png")