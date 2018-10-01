from numpy import *
import numpy as np
import sys
import binascii
import os
import sys
import struct
import threading
import time
import random
import collections
import pickle
import threading
import sys
import select
import time
import math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy import signal

import tensorflow

from scipy.signal import argrelextrema
from math import sqrt
from numpy import concatenate
from matplotlib import pyplot
from pandas import read_csv
from pandas import DataFrame
from pandas import concat
from sklearn.preprocessing import MinMaxScaler
from sklearn.preprocessing import LabelEncoder
from sklearn.metrics import mean_squared_error
from keras.models import Sequential
from keras.layers import Dense, LSTM, SimpleRNN, CuDNNLSTM, RNN
from keras.optimizers import Adam
from keras.wrappers.scikit_learn import KerasRegressor
from keras import backend as kb
from keras import optimizers
import keras.backend as K
from keras.callbacks import ModelCheckpoint
from keras.callbacks import Callback
from keras.models import load_model
import tensorflow as tf
from keras.backend.tensorflow_backend import set_session

# the list of Name and the number of walking trials (each trial contains three strides)
subjectList = ['PoHsinnn', 'Winston', 'Jay', 'Ting', 'Thomas', 'Gene']
lengthList = [18, 13, 7, 7, 7, 19]

# featureLen denotes the number of features we used for the LSTM
sensorNum = 1

# define of states
STATIC = 0
MOVING = 400

def read_pickle(index, pathHead):
    '''
        This function read data from pickle files. We use a list to store the data at each timestamp. 
        At each timestamp, we use a dictionary to store the data. The data includes:
        * triaxial acceleration
        * triaxial angular rate
        * triaxial magnetometer data
        * the time interval between two successive BLE notifications
    '''
    
    # Open six .dat files:
    # fp1 stores the data from the left thigh
    # fp2 stores the data from the left shin
    # fp3 stores the data from the left shoe(ground truth)
    # fp4 stores the data from the left shoe
    # fp5 stores the data from the right shin
    # fp6 stores the data from the right thigh
    
    fp1 = open(pathHead + str(index) + 'thigh.dat', "rb")
    fp2 = open(pathHead + str(index) + 'shank.dat', "rb")
    fp3 = open(pathHead + str(index) + 'shoes.dat', "rb")
    fp4 = open(pathHead + str(index) + 'shoesData.dat', "rb")
    if "PoHsin" in pathHead:
        fp5 = open(pathHead + str(index) + 'rightShank.dat', "rb")
        fp6 = open(pathHead + str(index) + 'rightThigh.dat', "rb")
    else:
        fp5 = open(pathHead + str(index) + 'rightThigh.dat', "rb")
        fp6 = open(pathHead + str(index) + 'rightShank.dat', "rb")
    
    objs1, objs2, objs3, objs4, objs5, objs6 = ([] for i in range(6))
    
    while True:
        try:
            objs1.append(pickle.load(fp1, encoding='iso-8859-1'))
        except:
            break
    while True:
        try:
            objs2.append(pickle.load(fp2, encoding='iso-8859-1'))
        except:
            break
    while True:
        try:
            objs3.append(pickle.load(fp3, encoding='iso-8859-1'))
        except:
            break
    while True:
        try:
            objs4.append(pickle.load(fp4, encoding='iso-8859-1'))
        except:
            break
    while True:
        try:
            objs5.append(pickle.load(fp5, encoding='iso-8859-1'))
        except:
            break
    while True:
        try:
            objs6.append(pickle.load(fp6, encoding='iso-8859-1'))
        except:
            break
    
    # timestamp represents the the time interval between two successive BLE notifications on the HOST.
    timestamp, gx, gy, gz, ax, ay, az = ([] for i in range(7))
    timestamp2, gx2, gy2, gz2, ax2, ay2, az2 = ([] for i in range(7))
    timestamp4, gx4, gy4, gz4, ax4, ay4, az4 = ([] for i in range(7))
    timestamp5, gx5, gy5, gz5, ax5, ay5, az5 = ([] for i in range(7))
    timestamp6, gx6, gy6, gz6, ax6, ay6, az6 = ([] for i in range(7))
    
    
    # the ground truth from ultrasonic and two FSRs
    # distance = the distance between the left shoe and the wall
    # state = record the TO and HS state
    timestamp3, distance, state = ([] for i in range(3))


    for obj in objs1:
        timestamp.append(obj['timestamp'])
        gz.append(obj['Gyo'].item(0, 2))
        gy.append(obj['Gyo'].item(0, 1))
        gx.append(obj['Gyo'].item(0, 0))
        ax.append(obj['Acc'].item(0, 0))
        ay.append(obj['Acc'].item(0, 1))
        az.append(obj['Acc'].item(0, 2))

    for obj in objs2:
        timestamp2.append(obj['timestamp'])
        gz2.append(obj['Gyo'].item(0,2))
        gy2.append(obj['Gyo'].item(0, 1))
        gx2.append(obj['Gyo'].item(0, 0))
        ax2.append(obj['Acc'].item(0, 0))
        ay2.append(obj['Acc'].item(0, 1))
        az2.append(obj['Acc'].item(0, 2))

    for obj in objs3:
        timestamp3.append(obj['timestamp'])
        distance.append(obj['distance'])
        state.append(obj['state'])
        #label3.append(obj['state'])

    for obj in objs4:
        timestamp4.append(obj['timestamp'])
        gz4.append(obj['Gyo'].item(0, 2))
        gy4.append(obj['Gyo'].item(0, 1))
        gx4.append(obj['Gyo'].item(0, 0))
        ax4.append(obj['Acc'].item(0, 0))
        ay4.append(obj['Acc'].item(0, 1))
        az4.append(obj['Acc'].item(0, 2))
    
    for obj in objs5:
        timestamp5.append(obj['timestamp'])
        gz5.append(obj['Gyo'].item(0, 2))
        gy5.append(obj['Gyo'].item(0, 1))
        gx5.append(obj['Gyo'].item(0, 0))
        ax5.append(obj['Acc'].item(0, 0))
        ay5.append(obj['Acc'].item(0, 1))
        az5.append(obj['Acc'].item(0, 2))
    
    for obj in objs6:
        timestamp6.append(obj['timestamp'])
        gz6.append(obj['Gyo'].item(0, 2))
        gy6.append(obj['Gyo'].item(0, 1))
        gx6.append(obj['Gyo'].item(0, 0))
        ax6.append(obj['Acc'].item(0, 0))
        ay6.append(obj['Acc'].item(0, 1))
        az6.append(obj['Acc'].item(0, 2))
    
    # Make the timestamp distributes evenly, 
    # because the timestamp recorded on the HOST can't represent the time-interval between two successive BLE notifications.
    timestamp = np.linspace(0, 8, len(timestamp))
    timestamp2 = np.linspace(0, 8, len(timestamp2))
    timestamp4 = np.linspace(0, 8, len(timestamp4))
    timestamp5 = np.linspace(0, 8, len(timestamp5))
    timestamp6 = np.linspace(0, 8, len(timestamp6))
    
    # Apply low-pass filter
    # plt.plot(gx)
    gx, gy, gz, ax, ay, az = lowPass([gx, gy, gz, ax, ay, az])
    gx2, gy2, gz2, ax2, ay2, az2 = lowPass([gx2, gy2, gz2, ax2, ay2, az2])
    gx4, gy4, gz4, ax4, ay4, az4 = lowPass([gx4, gy4, gz4, ax4, ay4, az4])
    gx5, gy5, gz5, ax5, ay5, az5 = lowPass([gx5, gy5, gz5, ax5, ay5, az5])
    gx6, gy6, gz6, ax6, ay6, az6 = lowPass([gx6, gy6, gz6, ax6, ay6, az6])
    # plt.plot(gx)
    # plt.savefig("lowpass.png")
    
    sensorData1 = [timestamp, gx, gy, gz, ax, ay, az]
    sensorData2 = [timestamp2, gx2, gy2, gz2, ax2, ay2, az2]
    sensorData3 = [timestamp3, distance, state]
    sensorData4 = [timestamp4, gx4, gy4, gz4, ax4, ay4, az4]
    sensorData5 = [timestamp5, gx5, gy5, gz5, ax5, ay5, az5]
    sensorData6 = [timestamp6, gx6, gy6, gz6, ax6, ay6, az6]
    
    return [sensorData1, sensorData2, sensorData3, sensorData4, sensorData5, sensorData6]

def lowPass(dataRaw):
    # low-pass filter
    b, a = signal.butter(10, 0.3)
    returnList = []
    for dataList in dataRaw:
        dataFilt = signal.filtfilt(b, a, dataList)
        returnList.append(dataFilt[:])
        
    return returnList

def dataPreprocess(dataList):
    
    # declarations
    sensorData1 = np.array(dataList[0])
    sensorData2 = np.array(dataList[1])
    sensorData3 = np.array(dataList[2])
    sensorData4 = np.array(dataList[3])
    sensorData5 = np.array(dataList[4])
    sensorData6 = np.array(dataList[5])

    timeList1 = sensorData1[0]
    timeList2 = sensorData2[0]
    timeList4 = sensorData4[0]
    timeList5 = sensorData5[0]
    timeList6 = sensorData6[0]

    timeList3 = sensorData3[0]
    stateList = sensorData3[2]
    
    # --------------- get the timing of TO and HS events (ground truth) ---------------
    hsTime = []
    toTime = []    
    
    lastState = STATIC
    lastT = None
    for t, state in zip(timeList3, stateList):
        if state != lastState:
            if state == MOVING:
                # toe off event
                toTime.append(t)
            else:
                # heel strike event
                if t - lastT > 0.57:
                    hsTime.append(lastT + 0.00737)
                else:
                    hsTime.append(t)
                if len(hsTime) >= 3:
                    break
            lastState = state
        lastT = t
    
    # in timeList, find the timestamp that is most close to t, and return the index of that timestamp
    def getCloseIndex(t, timeList):
        minValue = 1000
        closeIndex = None
        for i, T in enumerate(timeList):
            if abs(t-T) < minValue:
                closeIndex = i
                minValue = abs(t-T)
        return closeIndex
    
    # heel strike
    hsIndex1 = []
    hsIndex2 = []
    hsIndex4 = []
    hsIndex5 = []
    hsIndex6 = []
    for t in hsTime:
        # left thigh
        closeIndex = getCloseIndex(t, timeList1)
        hsIndex1.append(closeIndex)
        # left shin
        closeIndex = getCloseIndex(t, timeList2)
        hsIndex2.append(closeIndex)
        # left shoe
        closeIndex = getCloseIndex(t, timeList4)
        hsIndex4.append(closeIndex)
        # right shin
        closeIndex = getCloseIndex(t, timeList5)
        hsIndex5.append(closeIndex)
        # right thigh
        closeIndex = getCloseIndex(t, timeList6)
        hsIndex6.append(closeIndex)
    
    # toe off
    toIndex1 = []
    toIndex2 = []
    toIndex4 = []
    toIndex5 = []
    toIndex6 = []
    for t in toTime:
        # left thigh
        closeIndex = getCloseIndex(t, timeList1)
        toIndex1.append(closeIndex)
        # left shin
        closeIndex = getCloseIndex(t, timeList2)
        toIndex2.append(closeIndex)
        # left shoe
        closeIndex = getCloseIndex(t, timeList4)
        toIndex4.append(closeIndex)
        # right shin
        closeIndex = getCloseIndex(t, timeList5)
        toIndex5.append(closeIndex)
        # right thigh
        closeIndex = getCloseIndex(t, timeList6)
        toIndex6.append(closeIndex)
    # ---------------------------------------------------------------------------------
    
    
    # --------------- normalize the data length of five sensors ---------------
    
    # select (gyroZ, accX, accY), and cut tail (the data after the third HS event are cutted out)
    x_seq1 = sensorData1[3:6, 0:hsIndex1[2]+12]
    x_seq2 = sensorData2[3:6, 0:hsIndex2[2]+12]
    x_seq4 = sensorData4[3:6, 0:hsIndex4[2]+12]
    x_seq5 = sensorData5[3:6, 0:hsIndex5[2]+12]
    x_seq6 = sensorData6[3:6, 0:hsIndex6[2]+12]
    
    # normalize data length between five sensors
    X_seq = []
    hsTimingIndex = []
    toTimingIndex = []
    indexPassed = 0
    timePassed = 0
    timeListNew = None
    '''
    * The walking data is consisted of:  START -> TO -> HS -> TO -> HS -> TO -> HS
    * We segment the walking data into: |START -> TO -> HS| |-> TO -> HS| |-> TO -> HS|
    * i = 0, 1, 2 deal with the first, second, third stride, respectively.
    * We segment each stride into two part: (lastHS -> TO) and (TO -> HS).
    * For each part, the walking data comes from five sensors, and the number of packets sent from these sensors are different.
    * Thus, we normalize the length of the walking data using the minimal number of packets among theses five sensors.
    '''
    for i in range(3):
        # normalize the first part (lastHS -> TO)
        if i == 0:
            segment1a = x_seq1[0:3, 0:toIndex1[0]].tolist()
            segment2a = x_seq2[0:3, 0:toIndex2[0]].tolist()
            segment4a = x_seq4[0:3, 0:toIndex4[0]].tolist()
            segment5a = x_seq5[0:3, 0:toIndex5[0]].tolist()
            segment6a = x_seq6[0:3, 0:toIndex6[0]].tolist()
            minSensorTo, lenTo = normalizeSegments([segment1a, segment2a, segment4a, segment5a, segment6a])
            timeListNew = eval("timeList" + str(minSensorTo) + "[0:" + str(lenTo) + "]")
        else:
            segment1a = x_seq1[0:3, hsIndex1[i-1]:toIndex1[i]].tolist()
            segment2a = x_seq2[0:3, hsIndex2[i-1]:toIndex2[i]].tolist()
            segment4a = x_seq4[0:3, hsIndex4[i-1]:toIndex4[i]].tolist()
            segment5a = x_seq5[0:3, hsIndex5[i-1]:toIndex5[i]].tolist()
            segment6a = x_seq6[0:3, hsIndex6[i-1]:toIndex6[i]].tolist()
            minSensorTo, lenTo = normalizeSegments([segment1a, segment2a, segment4a, segment5a, segment6a])
            startIndex = str(eval("hsIndex" + str(minSensorTo) + "[i-1]"))
            endIndex = str(eval("toIndex" + str(minSensorTo) + "[i]"))
            timeListNew = np.concatenate((timeListNew, eval("timeList" + str(minSensorTo) + "[" + startIndex + ":" + endIndex + "]")))
        # normalize the second part (TO -> HS)
        if i == 2:
            segment1b = x_seq1[0:3, toIndex1[i]:hsIndex1[i]+12].tolist()
            segment2b = x_seq2[0:3, toIndex2[i]:hsIndex2[i]+12].tolist()
            segment4b = x_seq4[0:3, toIndex4[i]:hsIndex4[i]+12].tolist()
            segment5b = x_seq5[0:3, toIndex5[i]:hsIndex5[i]+12].tolist()
            segment6b = x_seq6[0:3, toIndex6[i]:hsIndex6[i]+12].tolist()
            minSensorHs, lenHs = normalizeSegments([segment1b, segment2b, segment4b, segment5b, segment6b])
            startIndex = str(eval("toIndex" + str(minSensorTo) + "[i]"))
            endIndex = str(eval("hsIndex" + str(minSensorTo) + "[i]")+12)
            timeListNew = np.concatenate((timeListNew, eval("timeList" + str(minSensorTo) + "[" + startIndex + ":" + endIndex + "]")))
        else:
            segment1b = x_seq1[0:3, toIndex1[i]:hsIndex1[i]].tolist()
            segment2b = x_seq2[0:3, toIndex2[i]:hsIndex2[i]].tolist()
            segment4b = x_seq4[0:3, toIndex4[i]:hsIndex4[i]].tolist()
            segment5b = x_seq5[0:3, toIndex5[i]:hsIndex5[i]].tolist()
            segment6b = x_seq6[0:3, toIndex6[i]:hsIndex6[i]].tolist()
            minSensorHs, lenHs = normalizeSegments([segment1b, segment2b, segment4b, segment5b, segment6b])
            startIndex = str(eval("toIndex" + str(minSensorTo) + "[i]"))
            endIndex = str(eval("hsIndex" + str(minSensorTo) + "[i]"))
            timeListNew = np.concatenate((timeListNew, eval("timeList" + str(minSensorTo) + "[" + startIndex + ":" + endIndex + "]")))
        
        # append the first part (lastHS -> TO)
        for j in range(len(segment1a[0])):
            if sensorNum == 1:
                # use the sensor at the left shoe
                X_seq.append(np.array([segment4a[0][j], segment4a[1][j], segment4a[2][j]]))
            elif sensorNum == 3:
                # use the sensor at the left thigh, shin, and shoe
                X_seq.append(np.array([segment1a[0][j], segment1a[1][j], segment1a[2][j],  
                                       segment2a[0][j], segment2a[1][j], segment2a[2][j],  
                                       segment4a[0][j], segment4a[1][j], segment4a[2][j]]))
            elif sensorNum == 5:
                # use all five sensors
                X_seq.append(np.array([segment1a[0][j], segment1a[1][j], segment1a[2][j],  
                                       segment2a[0][j], segment2a[1][j], segment2a[2][j],  
                                       segment4a[0][j], segment4a[1][j], segment4a[2][j],
                                       segment5a[0][j], segment5a[1][j], segment5a[2][j],  
                                       segment6a[0][j], segment6a[1][j], segment6a[2][j]]))
        # append the second part (TO -> HS)
        for j in range(len(segment1b[0])):
            if sensorNum == 1:
                # use the sensor at the left shoe
                X_seq.append(np.array([segment4b[0][j], segment4b[1][j], segment4b[2][j]]))
            elif sensorNum == 3:
                # use the sensor at the left thigh, shin, and shoe
                X_seq.append(np.array([segment1b[0][j], segment1b[1][j], segment1b[2][j],  
                                       segment2b[0][j], segment2b[1][j], segment2b[2][j],  
                                       segment4b[0][j], segment4b[1][j], segment4b[2][j]]))
            elif sensorNum == 5:
                # use all five sensors
                X_seq.append(np.array([segment1b[0][j], segment1b[1][j], segment1b[2][j],  
                                       segment2b[0][j], segment2b[1][j], segment2b[2][j],  
                                       segment4b[0][j], segment4b[1][j], segment4b[2][j],
                                       segment5b[0][j], segment5b[1][j], segment5b[2][j],  
                                       segment6b[0][j], segment6b[1][j], segment6b[2][j]]))
            
        toTimingIndex.append(indexPassed + len(segment1a[0]))
        indexPassed = indexPassed + len(segment1a[0])
        
        if i == 2:
            hsTimingIndex.append(indexPassed + len(segment1b[0]) - 12)
            indexPassed = indexPassed + len(segment1b[0]) - 12
        else:
            hsTimingIndex.append(indexPassed + len(segment1b[0]))
            indexPassed = indexPassed + len(segment1b[0])
    
    X_seq = np.array(X_seq)
    
    # ------------ generate y sequences for LSTM -------------
    Y_seq = np.zeros((X_seq.shape[0], 3))
    for y in Y_seq:
        y[0] = 1
    for i in hsTimingIndex:
        Y_seq[i, 0] = 0.0
        Y_seq[i, 1] = 1.0
        Y_seq[i, 2] = 0.0
    for i in toTimingIndex:
        Y_seq[i, 0] = 0.0
        Y_seq[i, 1] = 0.0
        Y_seq[i, 2] = 1.0
    
    return X_seq, Y_seq

def normalizeSegments(segList):
    '''
    input - segList: A list contains five lists
    output - minSensor: The index of the list which has minimal length
    output - minLeng: The minimal length
    
    This function finds the shortest list among the five lists in segList, 
    and then resamples the other four lists with the minimal length.
    '''
    
    minLeng = 10000
    minSensor = None
    for i in range(5):
        if len(segList[i][0]) < minLeng:
            minLeng = len(segList[i][0])
            minSensor = i

    for i in range(5):
        if len(segList[i][0]) == minLeng:
            for j in range(3):
                segList[i][j] = np.array(segList[i][j])
        else:
            for j in range(3):
                segList[i][j] = signal.resample(segList[i][j], minLeng)
                
    if minSensor in [0, 1]:
        minSensor = minSensor + 1
    else:
        minSensor = minSensor + 2

    return minSensor, minLeng
    
def toFloat(myList):
    # convert the elements in myList into float32
    for i in range(len(myList)):
        myList[i] = [np.float32(x) for x in myList[i]]

def normalizeData(X_seq):
    # input - X_seq: A list with shape (sampleNum, length of a sample, featureLen)
    # For each sample, we normalize each of its feature into the range [0, 1] using the minima and maxima of these features
    maxList = [-1000]*featureLen
    minList = [1000]*featureLen
    for sample in X_seq:
        for i in range(featureLen):
            featureList = sample[:,i]
            for featureValue in featureList:
                if featureValue > maxList[i]:
                    maxList[i] = featureValue
                if featureValue < minList[i]:
                    minList[i] = featureValue
    rangeList = [maxV-minV for maxV, minV in zip(maxList, minList)]
    biasList = [-1*minV for minV in minList]
    
    for k in range(len(X_seq)):
        for i in range(featureLen):
            featureList = X_seq[k][:,i]
            for j in range(len(featureList)):
                featureList[j] = featureList[j] + biasList[i]
                featureList[j] = featureList[j] / rangeList[i]

def computeErrorTime(yTrue, yEst):
    '''
    input - yTrue: A list contains the ground truth. Its shape is (length of a sample, 3)
    input - yEst: A list contains the classification result of LSTM. Its shape is (length of a sample, 3)
    '''
    
    global maxIndexDisHs, maxIndexDisTo
    
    # get the classification results for HS and TO event, respectively.
    hsData = [a[1] for a in yEst]
    toData = [a[2] for a in yEst]
    # find local maxmima
    hsLocalMax = argrelextrema(np.asarray(hsData), np.greater, mode='wrap')[0]
    toLocalMax = argrelextrema(np.asarray(toData), np.greater, mode='wrap')[0]
    # pick the local maxima which is bigger than 0.5
    hsLocalMax = [x for x in hsLocalMax if hsData[x] > 0.5]
    toLocalMax = [x for x in toLocalMax if toData[x] > 0.5]
    
    # find the index of ground truth
    hsCorrectIndex = []
    toCorrectIndex = []
    for i, y in enumerate(yTrue):
        if y[1] >= 1.0:
            hsCorrectIndex.append(i)
        if y[2] >= 1.0:
            toCorrectIndex.append(i)
    
    # this function removes duplicate element in a list
    def removeDup(seq):
        seen = set()
        seen_add = seen.add
        return [x for x in seq if not (x in seen or seen_add(x))]

    # ----- Find successive local maxima. If they are close to each other, we take their average. -----
    hsIndex = []
    for i in range(len(hsLocalMax)):
        indexSum = hsLocalMax[i]
        num = 1.0
        # find other local maxima which are close to this local maximum
        for j in range(len(hsLocalMax)):
            if i == j:
                continue
            if abs(hsLocalMax[i]-hsLocalMax[j]) < 100:
                indexSum = indexSum + hsLocalMax[j]
                num = num + 1.0
        hsIndex.append(indexSum/num)
    hsIndex = removeDup(hsIndex)
    
    toIndex = []
    for i in range(len(toLocalMax)):
        indexSum = toLocalMax[i]
        num = 1.0
        for j in range(len(toLocalMax)):
            if i == j:
                continue
            if abs(toLocalMax[i]-toLocalMax[j]) < 100:
                indexSum = indexSum + toLocalMax[j]
                num = num + 1.0
        toIndex.append(indexSum/num)
    toIndex = removeDup(toIndex)
    
    # calculate the distance between ground truth (index) and the estimated index
    errorHsNumList = []
    if len(hsCorrectIndex) > len(hsIndex):
        for i in range(len(hsIndex)):
            minV = 1000
            for j in range(len(hsCorrectIndex)):
                if abs(hsIndex[i] - hsCorrectIndex[j]) < abs(minV):
                    minV = hsIndex[i] - hsCorrectIndex[j]
            errorHsNumList.append(minV)
            if abs(minV) > abs(maxIndexDisHs):
                maxIndexDisHs = minV
    else:
        for i in range(len(hsCorrectIndex)):
            minV = 1000
            for j in range(len(hsIndex)):
                if abs(hsIndex[j] - hsCorrectIndex[i]) < abs(minV):
                    minV = hsIndex[j] - hsCorrectIndex[i]
            errorHsNumList.append(minV)
            if abs(minV) > abs(maxIndexDisHs):
                maxIndexDisHs = minV
            
    errorToNumList = []
    if len(toCorrectIndex) > len(toIndex):
        for i in range(len(toIndex)):
            minV = 1000
            for j in range(len(toCorrectIndex)):
                if abs(toIndex[i] - toCorrectIndex[j]) < abs(minV):
                    minV = toIndex[i] - toCorrectIndex[j]
            errorToNumList.append(minV)
            if abs(minV) > abs(maxIndexDisTo):
                maxIndexDisTo = minV
    else:
        for i in range(len(toCorrectIndex)):
            minV = 1000
            for j in range(len(toIndex)):
                if abs(toIndex[j] - toCorrectIndex[i]) < abs(minV):
                    minV = toIndex[j] - toCorrectIndex[i]
            errorToNumList.append(minV)
            if abs(minV) > abs(maxIndexDisTo):
                maxIndexDisTo = minV
    
    # ----- calculate the detect loss -----
    # false-positive: hsIndex > 3 or toIndex > 3
    # false-negative: hsIndex < 3 or toIndex < 3
    detectLossHs = abs(3 - len(hsIndex))
    detectLossTo = abs(3 - len(toIndex))
    
    return errorHsNumList, errorToNumList, detectLossHs, detectLossTo

def detectLossTest(model, valiX, valiY):
    # Check whether all step events are detected successfully
    detectLoss = 0
    for i in range(len(valiX)):
        vali_X = valiX[i].reshape((1, valiX[i].shape[0], featureLen))
        predictions = model.predict(vali_X)
        # ----- compute error time -----
        hsError, toError, detectLossHs, detectLossTo = computeErrorTime(valiY[i], predictions[0])
        detectLoss = detectLoss + detectLossHs + detectLossTo

    # print("detect loss count: %d" % detectLoss)
    
    return detectLoss

def seperateData(X_seq, X_weights, Y_seq, k):
    # Seperate the walking data of the k-th volunteer for testing, the others for training and validation
    testNum = lengthList[k]
    trainNum = sampleNum - testNum
    
    testStartIndex = int(sum(lengthList[0:k]))
    testEndIndex = int(sum(lengthList[0:k+1]))
    
    testData = [X_seq[testStartIndex:testEndIndex], Y_seq[testStartIndex:testEndIndex]]
    trainData = [np.concatenate((X_seq[0:testStartIndex], X_seq[testEndIndex:])),
                 np.concatenate((X_weights[0:testStartIndex], X_weights[testEndIndex:])),
                 np.concatenate((Y_seq[0:testStartIndex], Y_seq[testEndIndex:]))]
    
#     print("trainData length before: %d" % len(trainData[0]))
    trainDataIndex = int(int(len(trainData[0])*0.9)/5)*5
    valiData = [[],[],[]]
    valiData[0] = trainData[0][trainDataIndex:]
    valiData[1] = trainData[1][trainDataIndex:]
    valiData[2] = trainData[2][trainDataIndex:]
#     print("valiData length: %d" % len(valiData[0]))
    
    trainData[0] = trainData[0][0:trainDataIndex]
    trainData[1] = trainData[1][0:trainDataIndex]
    trainData[2] = trainData[2][0:trainDataIndex]
#     print("trainData length after: %d" % len(trainData[0]))
    
    return trainData, testData, valiData

def meanErrorPrecisionRMSE(lossList):
    meanError = sum(lossList) / float(len(lossList))
    precisionSum = 0
    for loss in lossList:
        precisionSum = precisionSum + (loss-meanError)**2
    precisionMean = precisionSum / (float(len(lossList))-1.0)
    precision = math.sqrt(precisionMean)

    rmseSum = 0
    for loss in lossList:
        rmseSum = rmseSum + loss**2
    rmseMean = rmseSum / float(len(lossList))
    rmse = math.sqrt(rmseMean)
    
    return meanError, precision, rmse

if __name__ == '__main__':
    global featureLen
    
    # Control the proportion of GPU memory used in this process 
    config = tf.ConfigProto()
    config.gpu_options.per_process_gpu_memory_fraction = 0.2
    set_session(tf.Session(config=config))
    
    # Set featureLen according to sensorNum
    if sensorNum == 1:
        featureLen = 3
    elif sensorNum == 3:
        featureLen = 9
    elif sensorNum == 5:
        featureLen = 15
    else:
        print("sensorNum should be 1, 3, or 5")
    
    print("feature length: %d" % featureLen)
    print('Read data and preprocess ...')
    # -------------- read data and preprocess -------------- 
    X_seqs = []
    X_weights = []
    Y_seqs = []
    for j, name in enumerate(subjectList):
        dataLen = lengthList[j]
        pathHead = 'walking' + name + '/'
        for x in range(1, dataLen+1):
            dataList = read_pickle(x, pathHead)
            X_seq, Y_seq = dataPreprocess(dataList)
            toFloat(X_seq)
            Y_seq = [np.float32(i) for i in Y_seq]
            X_seqs.append(X_seq)
            # if we have fewer data for this volunteer, we give these data higher weight
            X_weights.append(19.0/float(dataLen))
            Y_seqs.append(Y_seq)
    X_seqs = np.array(X_seqs)
    X_weights = np.array(X_weights)
    Y_seqs = np.array(Y_seqs)
    sampleNum = X_seqs.shape[0]
    
    # --------- normalize data value----------
    normalizeData(X_seqs)
    for i in range(len(Y_seqs)):
        Y_seqs[i] = np.array(Y_seqs[i])
    
    hsErrorCV = []
    hsPrecisionCV = []
    hsRmseCV = []
    toErrorCV = []
    toPrecisionCV = []
    toRmseCV = []
    detectLossCV = 0
    
    for k in range(len(subjectList)):
        print("\nk = %d, test subject: %s" % (k, subjectList[k]))
        # --------- seperate data -----------
        trainData, testData, valiData = seperateData(X_seqs, X_weights, Y_seqs, k)
        
        # --------- shuffle data once -----------
        np.random.seed(11)
        rng_state = np.random.get_state()
        np.random.shuffle(trainData[0])
        np.random.set_state(rng_state)
        np.random.shuffle(trainData[1])
        np.random.set_state(rng_state)
        np.random.shuffle(trainData[2])
        
        trainX = trainData[0]
        trainW = trainData[1]
        trainY = trainData[2]
        testX = testData[0]
        testY = testData[1]
        valiX = valiData[0]
        valiW = valiData[1]
        valiY = valiData[2]
        print("trainData length: %d" % trainData[0].shape[0])
        print("valiData length: %d" % valiData[0].shape[0])
        print("testData length: %d" % testData[0].shape[0])
        
        # make the minimal weight be 1
        minW = min(trainW)
        trainW = [x / minW for x in trainW]
        
        # build and compile the LSTM model
        model = Sequential()
        model.add(CuDNNLSTM(20, input_shape=(None, featureLen), return_sequences=True))
        model.add(Dense(3, activation='softmax'))
        model.compile(loss='categorical_crossentropy', optimizer='adam', sample_weight_mode="temporal", metrics = ['categorical_accuracy'])

        # ----- give weights for training and validation -----
        # At the moment when step events(HS or TO) happen, we give then higher weight
        trainW_seqs = []
        for i in range(len(trainY)):
            trainW_seq = []
            eventWeight = (len(trainY[i])-6) / 3.0
            for j, yTrue in enumerate(trainY[i]):
                if yTrue.tolist() in [[0, 1.0, 0], [0, 0, 1.0]]:
                    trainW_seq.append(eventWeight * trainW[i])
                else:
                    trainW_seq.append(trainW[i])
            trainW_seqs.append(np.array([trainW_seq[:]]))

        valiW_seqs = []
        for i in range(len(valiY)):
            valiW_seq = []
            eventWeight = (len(valiY[i])-6) / 3.0
            for j, yTrue in enumerate(valiY[i]):
                if yTrue.tolist() in [[0, 1.0, 0], [0, 0, 1.0]]:
                    valiW_seq.append(eventWeight * valiW[i])
                else:
                    valiW_seq.append(valiW[i])
            valiW_seqs.append(np.array([valiW_seq[:]]))
        
        global maxIndexDisHs, maxIndexDisTo
        maxIndexDisHs = 0
        maxIndexDisTo = 0
        # -------------------- train --------------------
        tStart = time.time()
        meanLossList = []
        meanAccList = []
        meanValiLoss = []
        meanValiAcc = []
        minValiLoss = 10000
        maxValiAcc = -1
        minEpoch = 1
        lastAvgLoss = 0
        stopCount = 0
        stopEpoch = 101
        
        for epoch in range(101):
#             print("epoch no.%d" % epoch)
            lossList = []
            accList = []
            for i in range(len(trainX)):
                train_X = trainX[i].reshape((1, trainX[i].shape[0], featureLen))
                train_Y = trainY[i].reshape((1, trainY[i].shape[0], 3))
                metricsList = model.train_on_batch(train_X, train_Y, sample_weight = np.array(trainW_seqs[i]))
                lossList.append(metricsList[0])
                accList.append(metricsList[1])
            meanLossList.append(sum(lossList) / float(len(lossList)))
            meanAccList.append(sum(accList) / float(len(accList)))

            # ----- validation -----
            valiLossList = []
            valiAccList = []
            for i in range(len(valiX)):
                vali_X = valiX[i].reshape((1, valiX[i].shape[0], featureLen))
                vali_Y = valiY[i].reshape((1, valiY[i].shape[0], 3))
                metricsList = model.evaluate(vali_X, vali_Y, batch_size=1, sample_weight = np.array(valiW_seqs[i]), verbose=0)
                valiLossList.append(metricsList[0])
                valiAccList.append(metricsList[1])
            meanValiLoss.append(sum(valiLossList) / float(len(valiLossList)))
            meanValiAcc.append(sum(valiAccList) / float(len(valiAccList)))
            
            # check and save the best model every five-epoch 
            if epoch % 5 == 0:
                if meanValiLoss[-1] < minValiLoss:
                    # Check whether all step events are detected successfully. If not, don't save the model.
                    if detectLossTest(model, valiX, valiY) == 0:
                        minValiLoss = meanValiLoss[-1]
                        model.save('modelSpace/modelsForEventUI/model_k' + str(k) + '_best.h5')
                        minEpoch = epoch
                
                # early stopping: when the avgLoss is increasing five times successively, we stop training
                avgLoss = sum(meanValiLoss[-5:]) / float(len(meanValiLoss[-5:]))
                if avgLoss > lastAvgLoss:
                    stopCount = stopCount + 1
                else:
                    stopCount = 0
                lastAvgLoss = avgLoss
                if stopCount > 5:
                    stopEpoch = epoch
                    print("stopEpoch: %d" % epoch)
                    break
                    
            # save models for drawing the loss graph
            if epoch % 5 == 0:
                model.save('modelSpace/modelsForEventUI/model_k' + str(k) + '_e' + str(epoch) + '.h5')

        tEnd = time.time()
        print("Train time: %f sec" % (tEnd-tStart))

        # ----- draw training graph (Loss and Accuracy) -----
        plt.figure()
        plt.subplot(311)
        plt.plot(meanLossList, color='blue', label='train_loss')
        plt.plot(meanValiLoss, color='orange', label='validation_loss')
        plt.axvline(minEpoch, color='r')
        lgd1 = plt.legend(bbox_to_anchor=(1.05, 0.65), loc=2, borderaxespad=0.)
        plt.subplot(312)
        plt.plot(meanAccList, color='blue', label='train_acc')
        plt.plot(meanValiAcc, color='orange', label='validation_acc')
        plt.axvline(minEpoch, color='r')
        lgd2 = plt.legend(bbox_to_anchor=(1.05, 0.65), loc=2, borderaxespad=0.)

        # ------------------ end training ---------------------


        # -------------------- evaluate the performance through each five-epoch --------------------
        xListo = np.linspace(5, 100, 20)
        xList = [int(a) for a in xListo if a < stopEpoch]
        xList = [0] + xList
        
        lossCountListCE = []
        errorHsListCE = []
        errorToListCE = []
        # x denotes the epoch number
        for x in xList:
            errorHsNumList = []
            errorToNumList = []
            detectLoss = 0
            model = load_model('modelSpace/modelsForEventUI/model_k' + str(k) + '_e' + str(x) + '.h5')
            for i in range(len(testX)):
                test_X = testX[i].reshape((1, testX[i].shape[0], featureLen))
                predictions = model.predict(test_X)

                # ----- compute error time -----
                hsError, toError, detectLossHs, detectLossTo = computeErrorTime(testY[i], predictions[0])
                
                detectLoss = detectLoss + detectLossHs + detectLossTo
                errorHsNumList.extend(hsError)
                errorToNumList.extend(toError)
    
            lossCountListCE.append(detectLoss)
            errorHsListCE.append((sum(errorHsNumList) / float(len(errorHsNumList))))
            errorToListCE.append((sum(errorToNumList) / float(len(errorToNumList))))
        
        plt.subplot(313)
        plt.plot(xList, lossCountListCE, label='lossCountList')
        plt.plot(xList, errorHsListCE, label='errorHsList')
        plt.plot(xList, errorToListCE, label='errorToList')
        lgd3 = plt.legend(bbox_to_anchor=(1.05, 0.65), loc=2, borderaxespad=0.)
        plt.savefig('detectLossUI_'+str(k)+'.png', bbox_extra_artists=[lgd1, lgd2, lgd3], bbox_inches='tight')
        # ------------------------------------------------------------------------------------------
        
        # ---------------------------- evaluate the best epoch ----------------------------
        errorHsNumList = []
        errorToNumList = []
        detectLoss = 0
        maxIndexDisHs = 0
        maxIndexDisTo = 0
        try:
            model = load_model('modelSpace/modelsForEventUI/model_k' + str(k) + '_best.h5')
            for i in range(len(testX)):
                test_X = testX[i].reshape((1, testX[i].shape[0], featureLen))
                predictions = model.predict(test_X)

                # ----- compute error time -----
                hsError, toError, detectLossHs, detectLossTo = computeErrorTime(testY[i], predictions[0])
                hsErrorList = []
                toErrorList = []
                for hs in hsError:
                    hsErrorList.append("%.1f" % hs)
                for to in toError:
                    toErrorList.append("%.1f" % to)
                print("%s's test no.%d:" % (subjectList[k], i))
                print("hsError: ", hsErrorList, "lossNum: %d" % detectLossHs)
                print("toError: ", toErrorList, "lossNum: %d" % detectLossTo)
                detectLoss = detectLoss + detectLossHs + detectLossTo
                errorHsNumList.extend(hsError)
                errorToNumList.extend(toError)

            errorHsNumList = [a*0.008 for a in errorHsNumList]
            errorToNumList = [a*0.008 for a in errorToNumList]
            meanErrorHS, precisionHS, rmseHS = meanErrorPrecisionRMSE(errorHsNumList)
            meanErrorTO, precisionTO, rmseTO = meanErrorPrecisionRMSE(errorToNumList)

            print("max index distance HS: %d" % maxIndexDisHs)
            print("max index distance TO: %d" % maxIndexDisTo)
            print("HS mean error: %f" % meanErrorHS)
            print("HS precision: %f" % precisionHS)
            print("HS rmse: %f" % rmseHS)
            print("TO mean error: %f" % meanErrorTO)
            print("TO precision: %f" % precisionTO)
            print("TO rmse: %f" % rmseTO)
            print("detect loss count: %d" % detectLoss)

            detectLossCV = detectLossCV + detectLoss
            hsErrorCV.append(meanErrorHS)
            hsPrecisionCV.append(precisionHS)
            hsRmseCV.append(rmseHS)
            toErrorCV.append(meanErrorTO)
            toPrecisionCV.append(precisionTO)
            toRmseCV.append(rmseTO)
        except:
            print("don't have best model")
            detectLossCV = detectLossCV + 1000
        # ---------------------------------------------------------------------------------

    print("\nCV HS mean error: %f" % (sum(hsErrorCV) / float(len(hsErrorCV))))
    print("CV HS precision: %f" % (sum(hsPrecisionCV) / float(len(hsPrecisionCV))))
    print("CV HS rmse: %f" % (sum(hsRmseCV) / float(len(hsRmseCV))))
    
    print("\nCV TO mean error: %f" % (sum(toErrorCV) / float(len(toErrorCV))))
    print("CV TO precision: %f" % (sum(toPrecisionCV) / float(len(toPrecisionCV))))
    print("CV TO rmse: %f" % (sum(toRmseCV) / float(len(toRmseCV))))
    print("CV detect loss count: %d" % detectLossCV)
