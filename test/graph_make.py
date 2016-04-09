#!/usr/bin/python

import sys
import getopt
import os
import glob
import shutil
import matplotlib.pyplot as plt
from scipy import misc
import numpy as np
from itertools import combinations


def countChangedPixels(newpath, filePrefix):
    i = 0
    pixelList = [None]*101

    while True:
        imgPath = newpath+filePrefix+str(i).zfill(3)+'.png'
        try:
            face = misc.imread(imgPath)
        except IOError:
            if i == 0:
                i = 1
                continue
            else:
                break
        pixelList[i] =  np.count_nonzero(face)
        i += 1
    return pixelList


def createSavePlot(pixelList, figure):
    plt.figure(figure)
    plt.plot(pixelList)
    plt.ylabel('Pixels Changed')



def main(argv):
    inputfile = ''
    costFunction = []
    diffCostData = {}
    deltaCostData = {}
    try:
        opts, args = getopt.getopt(argv,"hi:o:, c:", ["ifile=", "costFunction="])
    except getopt.GetoptError:
        print 'test.py <object_file> -c cost_function_number'
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print 'test.py -i <inputfile> -o <outputfile>'
            sys.exit()
        elif opt in ("-i", "--ifile"):
            inputfile = arg
        elif opt in ("-c", "--costFunction"):
            costFunction.extend(arg)

    for i in xrange(1, len(costFunction)+1):
        print list(combinations(costFunction, i))

    testName = inputfile + "_" + '_'.join(sorted(costFunction))
    path = os.path.dirname(os.path.abspath(__file__)) + r'/images/'
    newpath = path + testName + r'/'
    if os.path.exists(newpath):
        shutil.rmtree(newpath)
    os.makedirs(newpath)
    for function in costFunction:
        functionpath = newpath + function + r'/'
        os.makedirs(functionpath)
        command = "./cmput414_bin ./" + inputfile + " " + function + " " + "s"
        os.system(command)
        files = glob.iglob(os.path.join(path, "*.png"))
        for file in files:
            if os.path.isfile(file):
                shutil.move(file, functionpath)
        diffCostData.update({function: countChangedPixels(functionpath, 'diff')})
        deltaCostData.update({function: countChangedPixels(functionpath, 'delta')})
    print diffCostData
    print deltaCostData

    graphpath = newpath + "graphs/"
    os.makedirs(graphpath)
    for i in xrange(1, len(costFunction)+1):
        for graph in list(combinations(costFunction, i)):
            for j in graph:
                createSavePlot(diffCostData[j], 0)
                createSavePlot(deltaCostData[j], 1)
            plt.figure(0)
            plt.savefig(graphpath+'_'.join(graph)+'diff_graph.png')
            plt.clf()
            plt.figure(1)
            plt.savefig(graphpath+'_'.join(graph)+'delta_graph.png')
            plt.clf()

if __name__ == "__main__":
   main(sys.argv[1:])
