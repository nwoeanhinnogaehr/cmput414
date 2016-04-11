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
from PIL import ImageFilter
from scipy.ndimage.filters import gaussian_filter


decimationCount = 101
costLabels = {
    '1': ('vector sum', 'b-'),
    '2': ('normals', 'g-'),
    '3': ('manhatten', 'r-'),
    '4': ('euclidean', 'c-'),
    '5': ('circulation angle sum, view weight', 'm-'),
    '6': ('circulation angle sum', 'y-'),
    '7': ('DFS angle sum', 'k-'),
    '1l': ('vector sum', 'b--'),
    '2l': ('normals', 'g--'),
    '3l': ('manhatten', 'r--'),
    '4l': ('euclidean', 'c--'),
    '5l': ('circulation angle sum, view weights', 'm--'),
    '6l': ('circulation angle sum', 'y--'),
    '7l': ('DFS angle sum', 'k--'),
    }




def countChangedPixels(newpath, filePrefix):
    i = 0
    pixelList = [None]*decimationCount

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



def readSurfDist(filePath):
    with open(filePath) as f:
        return [float(i) for i in f.read().splitlines()]





def createSavePlot(pixelList, figure, costFun):
    pixelList[:50] = pixelList[:50]
    x = np.arange(0, len(pixelList)*50, 50)
    plt.figure(figure)
    plt.plot(
        x,
        pixelList,
        costLabels[str(costFun)][1],
        label= costLabels[str(costFun)][0],
    )
    plt.ylabel('Pixels Changed')
    plt.xlabel('Decimations')
    if figure == 0:
        plt.title('Pixels Changed Between Original and Current Step')
    elif figure == 1:
        plt.title('Pixels Changed Between Steps')
    elif figure == 2:
        plt.title('Surface Distance Change Between Original and Current Step')

    legend = plt.legend(loc='upper left')

    for label in legend.get_texts():
        label.set_fontsize('large')

    for label in legend.get_lines():
        label.set_linewidth(1.5)  # the legend line width


def main(argv):
    inputfile = ''
    costFunction = []
    diffCostData = {}
    deltaCostData = {}
    surfaceDistData = {}
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

    costFunction = costFunction + [s + 'l' for s in costFunction]



    testName = inputfile + "_" + '_'.join(sorted(costFunction))
    path = os.path.dirname(os.path.abspath(__file__)) + r'/images/'
    newpath = path + testName + r'/'
    if os.path.exists(newpath):
        shutil.rmtree(newpath)
    os.makedirs(newpath)
    for function in costFunction:
        functionpath = newpath + function + r'/'
        os.makedirs(functionpath)
        if 'l' in function:
            command = "./cmput414_bin ./" + inputfile + " " + function[0] + " " + "ls"
        else:
            command = "./cmput414_bin ./" + inputfile + " " + function + " " + "s"
        os.system(command)
        files = glob.iglob(os.path.join(path, "*.png"))

        for file in files:
            if os.path.isfile(file):
                shutil.move(file, functionpath)
        os.rename(
            os.path.dirname(os.path.abspath(__file__)) + '/surface_distances',
            functionpath + 'surface_distances')
        diffCostData.update({function: countChangedPixels(functionpath, 'diff')})
        deltaCostData.update({function: countChangedPixels(functionpath, 'delta')})
        surfaceDistData.update({function: readSurfDist(functionpath + 'surface_distances')})


    graphpath = newpath + "graphs/"
    os.makedirs(graphpath)
    for i in xrange(1, len(costFunction)+1):
        for graph in list(combinations(costFunction, i)):
            for j in graph:
                # 0 is diff, 1 is delta, 2 surface distance
                createSavePlot(diffCostData[j], 0, j)
                createSavePlot(deltaCostData[j], 1, j)
                createSavePlot(surfaceDistData[j], 2, j)
            plt.figure(0)
            plt.savefig(graphpath+'_'.join(graph)+'diff_graph.png')
            plt.clf()
            plt.figure(1)
            plt.savefig(graphpath+'_'.join(graph)+'delta_graph.png')
            plt.clf()
            plt.figure(2)
            plt.savefig(graphpath+'_'.join(graph)+'surface_dist_graph.png')
            plt.clf()

if __name__ == "__main__":
   main(sys.argv[1:])
