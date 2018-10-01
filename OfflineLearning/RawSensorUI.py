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
from keras.layers import Dense, LSTM, SimpleRNN, CuDNNLSTM, RNN, Dropout
from keras.optimizers import Adam
from keras.wrappers.scikit_learn import KerasRegressor
from keras import backend as kb
from keras import optimizers
import keras.backend as K
from keras.callbacks import ModelCheckpoint, Callback, EarlyStopping
from keras.models import load_model
import tensorflow as tf
from keras.backend.tensorflow_backend import set_session

l3long = 28
L3 = 20

DegreeToRadians = 0.0174532925

subjectList = ['PoHsinnn', 'Winston', 'Jay', 'Ting', 'Thomas', 'Gene']
lengthList = [18, 13, 7, 7, 7, 19]

featureLen = 15

# define for classfication
STATIC = 0
MOVING = 400

def read_pickle(index, pathHead, recover=False):
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
    objs1 = []
    objs2 = []
    objs3 = []
    objs4 = []
    objs5 = []
    objs6 = []
    
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
            
#     print("data1 length:",len(objs1))
#     print("data2 length:",len(objs2))
#     print("data3 length:",len(objs3))
#     print("data4 length:",len(objs4))
#     print("data5 length:",len(objs5))
#     print("data6 length:",len(objs6))

    timestamp = list()
    timestampChip = []
    yaw1 = list()
    pitch1 = list()
    roll1 = list()
    gx= list()
    gy= list()
    gz= list()
    ax=list()
    ay=list()
    az=list()

    timestamp2 = list()
    timestampChip2 = []
    yaw2 = list()
    pitch2 = list()
    roll2 = list()
    gx2= list()
    gy2= list()
    gz2= list()
    ax2=list()
    ay2=list()
    az2=list()

    timestamp3 = list()
    timestampChip3 = []
    yaw3 = list()
    pitch3 = list()
    roll3 = list()
    gx3= list()
    gy3= list()
    gz3= list()
    ax3=list()
    ay3=list()
    az3=list()

    timestamp4 = list()
    timestampChip4 = []
    yaw4 = list()
    pitch4 = list()
    roll4 = list()
    gx4= list()
    gy4= list()
    gz4= list()
    ax4=list()
    ay4=list()
    az4=list()

    timestamp5 = list()
    timestampChip5 = []
    yaw5 = list()
    pitch5 = list()
    roll5 = list()
    gx5= list()
    gy5= list()
    gz5= list()
    ax5=list()
    ay5=list()
    az5=list()

    timestamp6 = list()
    timestampChip6 = []
    yaw6 = list()
    pitch6 = list()
    roll6 = list()
    gx6= list()
    gy6= list()
    gz6= list()
    ax6=list()
    ay6=list()
    az6=list()

    distance = []
    state = []

    for obj in objs1:
        timestamp.append(obj['timestamp'])
        timestampChip.append(obj['timestampChip'])
        yaw1.append(obj['Angle'].item(0,0))
        pitch1.append(obj['Angle'].item(0,1))
        roll1.append(obj['Angle'].item(0,2))
        gz.append(obj['Gyo'].item(0, 2))
        gy.append(obj['Gyo'].item(0, 1))
        gx.append(obj['Gyo'].item(0, 0))
        ax.append(obj['Acc'].item(0, 0))
        ay.append(obj['Acc'].item(0, 1))
        az.append(obj['Acc'].item(0, 2))

    for obj in objs2:
        timestamp2.append(obj['timestamp'])
        timestampChip2.append(obj['timestampChip'])
        yaw2.append(obj['Angle'].item(0,0))
        pitch2.append(obj['Angle'].item(0,1))
        roll2.append(obj['Angle'].item(0,2))
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
        timestampChip4.append(obj['timestampChip'])
        yaw4.append(obj['Angle'].item(0,0))
        pitch4.append(obj['Angle'].item(0,1))
        roll4.append(obj['Angle'].item(0,2))
        gz4.append(obj['Gyo'].item(0, 2))
        gy4.append(obj['Gyo'].item(0, 1))
        gx4.append(obj['Gyo'].item(0, 0))
        ax4.append(obj['Acc'].item(0, 0))
        ay4.append(obj['Acc'].item(0, 1))
        az4.append(obj['Acc'].item(0, 2))
    
    for obj in objs5:
        timestamp5.append(obj['timestamp'])
        timestampChip5.append(obj['timestampChip'])
        yaw5.append(obj['Angle'].item(0,0))
        pitch5.append(obj['Angle'].item(0,1))
        roll5.append(obj['Angle'].item(0,2))
        gz5.append(obj['Gyo'].item(0, 2))
        gy5.append(obj['Gyo'].item(0, 1))
        gx5.append(obj['Gyo'].item(0, 0))
        ax5.append(obj['Acc'].item(0, 0))
        ay5.append(obj['Acc'].item(0, 1))
        az5.append(obj['Acc'].item(0, 2))
    
    for obj in objs6:
        timestamp6.append(obj['timestamp'])
        timestampChip6.append(obj['timestampChip'])
        yaw6.append(obj['Angle'].item(0,0))
        pitch6.append(obj['Angle'].item(0,1))
        roll6.append(obj['Angle'].item(0,2))
        gz6.append(obj['Gyo'].item(0, 2))
        gy6.append(obj['Gyo'].item(0, 1))
        gx6.append(obj['Gyo'].item(0, 0))
        ax6.append(obj['Acc'].item(0, 0))
        ay6.append(obj['Acc'].item(0, 1))
        az6.append(obj['Acc'].item(0, 2))

    timestamp = np.linspace(0, 8, len(timestamp))
    timestamp2 = np.linspace(0, 8, len(timestamp2))
    # timestamp3 = np.linspace(0, 8, len(timestamp3))
    timestamp4 = np.linspace(0, 8, len(timestamp4))
    timestamp5 = np.linspace(0, 8, len(timestamp5))
    timestamp6 = np.linspace(0, 8, len(timestamp6))
    
    if recover:
        if index == 38:
            distance = [390.064888, 298.923233, 206.391445, 107.456669]
        if index == 35:
            distance = [384.904572, 285.173630, 180.766815, 79.574127]
        if index == 27:
            distance = [386.926575, 276.158875, 164.236950, 52.946915]
        if index == 13:
            distance = [382.478157, 272.477150, 164.161137, 56.034677]
        if index == 9:
            distance = [384.015724, 287.241974, 190.068005, 89.532480]
    
    gx, gy, gz, ax, ay, az = lowPass([gx, gy, gz, ax, ay, az])
    gx2, gy2, gz2, ax2, ay2, az2 = lowPass([gx2, gy2, gz2, ax2, ay2, az2])
    gx4, gy4, gz4, ax4, ay4, az4 = lowPass([gx4, gy4, gz4, ax4, ay4, az4])
    gx5, gy5, gz5, ax5, ay5, az5 = lowPass([gx5, gy5, gz5, ax5, ay5, az5])
    gx6, gy6, gz6, ax6, ay6, az6 = lowPass([gx6, gy6, gz6, ax6, ay6, az6])
    
    sensorData1 = [timestamp, gx, gy, gz, ax, ay, az]
    sensorData2 = [timestamp2, gx2, gy2, gz2, ax2, ay2, az2]
    sensorData3 = [timestamp3, distance, state]
    sensorData4 = [timestamp4, gx4, gy4, gz4, ax4, ay4, az4]
    sensorData5 = [timestamp5, gx5, gy5, gz5, ax5, ay5, az5]
    sensorData6 = [timestamp6, gx6, gy6, gz6, ax6, ay6, az6]
    
    return [sensorData1, sensorData2, sensorData3, sensorData4, sensorData5, sensorData6]

def lowPass(dataRaw):
    b, a = signal.butter(10, 0.3)
    returnList = []
    for dataList in dataRaw:
        dataFilt = signal.filtfilt(b, a, dataList)
        returnList.append(dataFilt[:])
        
    return returnList

def dataPreprocess(dataList):
    # distance
    sensorData1 = np.array(dataList[0])
    sensorData2 = np.array(dataList[1])
    sensorData3 = np.array(dataList[2])
    sensorData4 = np.array(dataList[3])
    sensorData5 = np.array(dataList[4])
    sensorData6 = np.array(dataList[5])
    
    disList = sensorData3[1]
    lastDis = disList[0]
    stepLengthList = []
    for dis in disList:
        if dis < lastDis:
            x = (lastDis - dis)
            correctionDis = -2.56564*math.pow(10, -8)*math.pow(x, 4) + 0.0000222831*math.pow(x, 3) - 0.00663392*math.pow(x, 2) + 1.85571*x - 27.5267
            stepLengthList.append(correctionDis)
            lastDis = dis

    # declare
    timeList1 = sensorData1[0]
    timeList2 = sensorData2[0]
    timeList4 = sensorData4[0]
    timeList5 = sensorData5[0]
    timeList6 = sensorData6[0]

    timeList3 = sensorData3[0]
    stateList = sensorData3[2]

    hsTime = []
    toTime = []
    
    # timing
    lastState = STATIC
    lastT = None
    for t, state in zip(timeList3, stateList):
        if state != lastState:
            if state == MOVING:
                # toe off event
                toTime.append(t)
            else:
                # heel strike event
#                 print(t - lastT)
                if t - lastT > 0.57:
                    hsTime.append(lastT + 0.00737)
                else:
                    hsTime.append(t)
                if len(hsTime) >= 3:
                    break
            lastState = state
        lastT = t

    # --------------- check result --------------
    # print(len(time1), len(time2), len(normalDataList1), len(normalDataList2))
    # plt.ion()
    # fig = plt.figure(figsize=(12,6))
    # f1 = fig.add_subplot(1, 1, 1)
    # leftGyroInt = [row[0] for row in normalDataList1]
    # rightGyroInt = [row[0] for row in normalDataList2]
    # gyro1 = [row[1] for row in normalDataList1]
    # gyro2 = [row[1] for row in normalDataList2]
    # accx1 = [row[2] for row in normalDataList1]
    # accx2 = [row[2] for row in normalDataList2]
    # accy1 = [row[3] for row in normalDataList1]
    # accy2 = [row[3] for row in normalDataList2]
    # # f1.plot(time1, leftGyroInt, 'blue', label = 'thigh gyro integration')
    # f1.plot(time2, rightGyroInt, 'm', label = 'shank gyro integration')
    # # f1.plot(time1, gyro1, 'y', label = 'thigh gyroZ')
    # f1.plot(time2, gyro2, 'b', label = 'shank gyro integration')
    # # f1.plot(time1, accx1, 'm', label = 'thigh accX')
    # f1.plot(time2, accx2, 'c', label = 'shank gyro integration')
    # # f1.plot(time1, accy1, 'r', label = 'thigh accY')
    # f1.plot(time2, accy2, 'r', label = 'shank gyro integration')
    # statLow = [60 if s != 0 else -60 for s in dataList[11]]
    # # f1.plot(dataList[10], statLow, 'y', label = 'state')
    # f1.legend(bbox_to_anchor=(1.1, 1), loc=1, borderaxespad=0.)
    # plt.show()
    # while True:
    #     plt.pause(0.0001)
    
    
    #  ------------ segment data for each stride ------------ 
    hsIndex1 = []
    hsIndex2 = []
    hsIndex4 = []
    hsIndex5 = []
    hsIndex6 = []
    # heel strike
    for t1 in hsTime:
        minValue = 1000
        index = None
        for i, t2 in enumerate(timeList1):
            if abs(t1-t2) < minValue and t2 > t1:
                minValue = abs(t1-t2)
                index = i
        hsIndex1.append(index)
        minValue = 1000
        index = None
        for i, t2, in enumerate(timeList2):
            if abs(t1-t2) < minValue and t2 > t1:
                minValue = abs(t1-t2)
                index = i
        hsIndex2.append(index)
        minValue = 1000
        for i, t2, in enumerate(timeList4):
            if abs(t1-t2) < minValue and t2 > t1:
                minValue = abs(t1-t2)
                index = i
        hsIndex4.append(index)
        minValue = 1000
        for i, t2, in enumerate(timeList5):
            if abs(t1-t2) < minValue and t2 > t1:
                minValue = abs(t1-t2)
                index = i
        hsIndex5.append(index)
        minValue = 1000
        for i, t2, in enumerate(timeList6):
            if abs(t1-t2) < minValue and t2 > t1:
                minValue = abs(t1-t2)
                index = i
        hsIndex6.append(index)
        
    toIndex1 = []
    toIndex2 = []
    toIndex4 = []
    toIndex5 = []
    toIndex6 = []
    # toe off
    for t1 in toTime:
        minValue = 1000
        index = None
        for i, t2 in enumerate(timeList1):
            if abs(t1-t2) < minValue and t2 < t1:
                minValue = abs(t1-t2)
                index = i
        toIndex1.append(index)
        minValue = 1000
        index = None
        for i, t2, in enumerate(timeList2):
            if abs(t1-t2) < minValue and t2 < t1:
                minValue = abs(t1-t2)
                index = i
        toIndex2.append(index)
        minValue = 1000
        for i, t2, in enumerate(timeList4):
            if abs(t1-t2) < minValue and t2 < t1:
                minValue = abs(t1-t2)
                index = i
        toIndex4.append(index)
        minValue = 1000
        for i, t2, in enumerate(timeList5):
            if abs(t1-t2) < minValue and t2 < t1:
                minValue = abs(t1-t2)
                index = i
        toIndex5.append(index)
        minValue = 1000
        for i, t2, in enumerate(timeList6):
            if abs(t1-t2) < minValue and t2 < t1:
                minValue = abs(t1-t2)
                index = i
        toIndex6.append(index)
    
    strideCount = 3
    x_seq1 = []
    x_seq2 = []
    x_seq4 = []
    x_seq5 = []
    x_seq6 = []
    for i in range(strideCount):
        startIndex = toIndex1[i]
        endIndex = hsIndex1[i]
        x_seq1.append(sensorData1[3:6, startIndex:endIndex+1].tolist())
        startIndex = toIndex2[i]
        endIndex = hsIndex2[i]
        x_seq2.append(sensorData2[3:6, startIndex:endIndex+1].tolist())
        startIndex = toIndex4[i]
        endIndex = hsIndex4[i]
        x_seq4.append(sensorData4[3:6, startIndex:endIndex+1].tolist())
        startIndex = toIndex5[i]
        endIndex = hsIndex5[i]
        x_seq5.append(sensorData5[3:6, startIndex:endIndex+1].tolist())
        startIndex = toIndex6[i]
        endIndex = hsIndex6[i]
        x_seq6.append(sensorData6[3:6, startIndex:endIndex+1].tolist())
        
    # --------------- check timimg synchronize between five cc2541 --------------- 
#     for t1, i1, i2, i4, i5, i6 in zip(hsTime, hsIndex1, hsIndex2, hsIndex4, hsIndex5, hsIndex6):
#         print("hsTime (sensor3) : ", t1)
#         print(i1, timeList1[i1])
#         print(i2, timeList2[i2])
#         print(i4, timeList4[i4])
#         print(i5, timeList5[i5])
#         print(i6, timeList6[i6])
#     for t1, i1, i2, i4, i5, i6 in zip(toTime, toIndex1, toIndex2, toIndex4, toIndex5, toIndex6):
#         print("toTime (sensor3) : ", t1)
#         print(i1, timeList1[i1])
#         print(i2, timeList2[i2])
#         print(i4, timeList4[i4])
#         print(i5, timeList5[i5])
#         print(i6, timeList6[i6])
    
#     np.set_printoptions(threshold=np.nan)
#     print(s1.shape)
#     print(s2.shape)
#     print(s4.shape)
#     print(s5.shape)
#     print(s6.shape)

#     fig = plt.figure(figsize=(12,6))
#     f1 = fig.add_subplot(1, 1, 1)
#     f1.plot(timeList1, sensorData1[3], label = '1')
#     f1.plot(timeList2, sensorData2[4], label = '2', linewidth=0.5)
#     f1.plot(timeList4, sensorData4[5], label = '4', linewidth=0.5)
#     f1.plot(timeList5, sensorData5[4], label = '5')
#     f1.plot(timeList6, sensorData6[3], label = '6')
#     f1.legend(bbox_to_anchor=(1.1, 1), loc=1, borderaxespad=0.)
#     plt.savefig("lstmFigBefore.png")
    
    # --------------- normalize the data length of five sensors ---------------

    x_seqs = [x_seq1, x_seq2, x_seq4, x_seq5, x_seq6]
    for i in range(strideCount):
        minLen = 1000
        for k in range(3):
            for j in range(5):
                if len(x_seqs[j][i][k]) < minLen:
                    minLen = len(x_seqs[j][i][k])
            for j in range(5):
                if len(x_seqs[j][i][k]) == minLen:
                    x_seqs[j][i][k] = np.array(x_seqs[j][i][k])
                else:
                    x_seqs[j][i][k] = signal.resample(x_seqs[j][i][k], minLen)
                    
#     for i in range(1, len(sensorData1)):
#         sensorData1[i], _ = signal.resample(sensorData1[i], 940, timeList1)
#     for i in range(1, len(sensorData2)):
#         sensorData2[i], _ = signal.resample(sensorData2[i], 940, timeList2)
#     for i in range(1, len(sensorData4)):
#         sensorData4[i], _ = signal.resample(sensorData4[i], 940, timeList4)
#     for i in range(1, len(sensorData5)):
#         sensorData5[i], _ = signal.resample(sensorData5[i], 940, timeList5)
#     for i in range(1, len(sensorData6)):
#         sensorData6[i], _ = signal.resample(sensorData6[i], 940, timeList6)
#     sensorData1[0], timeList1 = signal.resample(sensorData1[0], 940, timeList1)
#     sensorData2[0], timeList2 = signal.resample(sensorData2[0], 940, timeList2)
#     sensorData4[0], timeList4 = signal.resample(sensorData4[0], 940, timeList4)
#     sensorData5[0], timeList5 = signal.resample(sensorData5[0], 940, timeList5)
#     sensorData6[0], timeList6 = signal.resample(sensorData6[0], 940, timeList6)
    
#     fig = plt.figure(figsize=(12,6))
#     f1 = fig.add_subplot(1, 1, 1)
#     print(sensorData1[3])
#     print(len(timeList1), len(sensorData1[3]))
#     f1.plot(timeList1, sensorData1[3], label = '1')
#     f1.plot(timeList2, sensorData2[4], label = '2', linewidth=0.5)
#     f1.plot(timeList4, sensorData4[5], label = '4', linewidth=0.5)
#     f1.plot(timeList5, sensorData5[4], label = '5')
#     f1.plot(timeList6, sensorData6[3], label = '6')
#     f1.legend(bbox_to_anchor=(1.1, 1), loc=1, borderaxespad=0.)
#     plt.savefig("lstmFigAfter.png")
#     x_seq = np.concatenate([s1, s2, s4, s5, s6], axis = -1)
#     strideCount = 3

#     np.set_printoptions(threshold=np.nan)
    
    # ------------ generate x sequences for LSTM -------------
    X_seq = []
    for i in range(strideCount):
        s1 = np.array([np.array(a) for a in zip(x_seqs[0][i][0], x_seqs[0][i][1], x_seqs[0][i][2])])
        s2 = np.array([np.array(a) for a in zip(x_seqs[1][i][0], x_seqs[1][i][1], x_seqs[1][i][2])])
        s4 = np.array([np.array(a) for a in zip(x_seqs[2][i][0], x_seqs[2][i][1], x_seqs[2][i][2])])
        s5 = np.array([np.array(a) for a in zip(x_seqs[3][i][0], x_seqs[3][i][1], x_seqs[3][i][2])])
        s6 = np.array([np.array(a) for a in zip(x_seqs[4][i][0], x_seqs[4][i][1], x_seqs[4][i][2])])
        x_seq = np.concatenate([s1, s2, s4, s5, s6], axis = -1)
#         x_seq = np.concatenate([s1, s2, s4], axis = -1)
#         x_seq = np.concatenate([s4], axis = -1)
        X_seq.append(x_seq)
    X_seq = np.array(X_seq)
    # ------------ generate y sequences for LSTM -------------
    Y_seq = np.array(stepLengthList[0:strideCount])
    
#     print(x_seqs[0].shape)
#     print(x_seqs[1].shape)
#     print(y_seqs)
    
    return X_seq, Y_seq

def toFloat(myList):
    for i in range(len(myList)):
        myList[i] = [np.float32(x) for x in myList[i]]

def seperateData(X_seq, X_weights, Y_seq, i):
    testNum = lengthList[i]*3
    trainNum = sampleNum - testNum
    
#     print("trainNum: %d" % trainNum)
#     print("testNum: %d" % testNum)
    
    index = 0
    for x in range(i):
        index = index + lengthList[x]*3
    testData = [X_seq[index:index+testNum], Y_seq[index:index+testNum]]
    trainData = [np.concatenate((X_seq[0:index], X_seq[index+testNum:])),
                 np.concatenate((X_weights[0:index], X_weights[index+testNum:])),
                 np.concatenate((Y_seq[0:index], Y_seq[index+testNum:]))]
    
#     print("trainData index range: %d~%d, %d~%d" % (0, index, index+testNum, sampleNum))
#     print("testData index range: %d~%d" % (index, index+testNum))
    
#     print("trainData length before: %d" % len(trainData[0]))
    np.random.seed(13)
    rng_state = np.random.get_state()
    np.random.shuffle(trainData[0])
    np.random.set_state(rng_state)
    np.random.shuffle(trainData[1])
    np.random.set_state(rng_state)
    np.random.shuffle(trainData[2])

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

def normalizeData(X_seq):
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

def normalizeDataLen(X_seq):
    global minLength
    minLength = 1000
    paddedLen = []
    for sample in X_seq:
        if len(sample) < minLength:
            minLength = len(sample)
    
    feature = [[] for i in range(featureLen)]
    for i in range(len(X_seq)):
#         print(np.array(X_seq[i]).shape)
        for j in range(featureLen):
            feature[j] = np.interp(np.linspace(0, 1, minLength), np.linspace(0, 1, len(X_seq[i][:, j])), X_seq[i][:, j])
#             feature[j] = signal.resample(X_seq[i][:, j], minLength)
        featureInv = list(map(list, zip(*feature)))
        X_seq[i] = np.array(featureInv[:])
#         print(np.array(X_seq[i]).shape)


if __name__ == '__main__':
    global sampleNum
    
    f = open("RawSensorUI_neuron32_epoch1200_featureLen15.txt", 'w')
    sys.stdout = f
    
    config = tf.ConfigProto()
    config.gpu_options.per_process_gpu_memory_fraction = 0.2
    set_session(tf.Session(config=config))
    
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
            if name == 'PoHsin':
                dataList = read_pickle(x, pathHead, recover=True)
            else:
                dataList = read_pickle(x, pathHead)
            X_seq, Y_seq = dataPreprocess(dataList)
            for i in range(3):
                toFloat(X_seq[i])
            Y_seq = [np.float32(i) for i in Y_seq]
            for i in range(3):
                X_seqs.append(X_seq[i])
                X_weights.append(19.0/float(dataLen))
                Y_seqs.append(Y_seq[i])
    X_seqs = np.array(X_seqs)
    X_weights = np.array(X_weights)
    Y_seqs = np.array(Y_seqs)
    sampleNum = X_seqs.shape[0]
    
#     # X_seqs.shape = (sampleNum, length(time_step), featureNum)
#     # Y_seqs.shape = (sampleNum)
    
    # --------- normalize data length and value----------
#     plt.plot([a for a in X_seqs[0][:, 0]])
#     plt.plot([a for a in X_seqs[0][:, 1]])
#     plt.savefig("testShape.png")
#     print(abcde)
    
    normalizeDataLen(X_seqs)
    normalizeData(X_seqs)
#     plt.plot([a for a in X_seqs[0][:, 0]])
#     plt.plot([a for a in X_seqs[0][:, 1]])
#     plt.plot([a for a in X_seqs[0][:, 2]])
#     plt.savefig("testShape.png")
    
    # k for cross validation
    meanerrorList = []
    precisionList = []
    rmseList = []
    
    for k in range(len(subjectList)):
        global testloss, testPrecision
        lossList = []
        testLoss = []
        testPrecision = []
        # --------- seperate data -----------
        trainData, testData, valiData = seperateData(X_seqs, X_weights, Y_seqs, k)
        trainX = trainData[0]
        trainW = trainData[1]
        trainY = trainData[2]
        testX = testData[0]
        testY = testData[1]
        valiX = valiData[0]
        valiW = valiData[1]
        valiY = valiData[2]
        print("\ntrainData length: %d" % trainData[0].shape[0])
        print("valiData length: %d" % valiData[0].shape[0])
        print("testData length: %d" % testData[0].shape[0])

        trainX = np.stack(trainX)
        testX = np.stack(testX)
        valiX = np.stack(valiX)
        
        def precisionM(y_true, y_pred):
            return K.sqrt(K.mean(((y_pred-y_true) - K.mean(y_pred-y_true))**2))
        
        class TestCallback(Callback):
            global testLoss, testPrecision
            def __init__(self, test_data):
                self.test_data = test_data

            def on_epoch_end(self, epoch, logs={}):
                x, y = self.test_data
                evalResult = self.model.evaluate(x, y, verbose=0)
                testLoss.append(evalResult[0])
                testPrecision.append(evalResult[1])
#                 print('\nTesting loss: {}, acc: {}\n'.format(evalResult[0], evalResult[1]))
        
#         class EarlyStoppingCallback(Callback):
#             def __init__(self, vali_data):
#                 self.vali_data = vali_data
#                 self.stopCount = 0
#                 self.valiLoss = []
#                 self.valiPrecision = []
#                 self.lastAvgLoss = 0

#             def on_epoch_end(self, epoch, logs={}):
#                 x, y = self.vali_data
#                 evalResult = self.model.evaluate(x, y, verbose=0)
#                 self.valiLoss.append(evalResult[0])
#                 self.valiPrecision.append(evalResult[1]) 
# #                 if epoch % 3 == 0:
#                 avgLoss = sum(self.valiLoss[-10:]) / float(len(self.valiLoss[-10:]))
#                 if avgLoss > self.lastAvgLoss:
#                     self.stopCount = self.stopCount + 1
#                 else:
#                     self.stopCount = 0
#                 self.lastAvgLoss = avgLoss
#                 if self.stopCount >= 5:
#                     print("stopEpoch: %d" % epoch)
#                     self.model.stop_training = True
                    
        model = Sequential()
        model.add(CuDNNLSTM(32, input_shape=(minLength, featureLen)))
#         model.add(Dropout(0.2))
        model.add(Dense(1))
#         adam = optimizers.Adam(clipnorm=1.0)
        model.compile(loss='mse', optimizer='adam', metrics=[precisionM])
    
        checkpointer = ModelCheckpoint(filepath='modelsRawSensorUI/weights' + str(k) + '.hdf5', 
                                       monitor='val_loss', verbose=0, save_best_only=True)
        
        tStart = time.time()
        history = model.fit(trainX, trainY, batch_size=5, epochs=1200, verbose=0, 
                            sample_weight=trainW,
                            validation_data=(valiX, valiY, valiW), 
                            callbacks=[checkpointer, TestCallback((testX, testY))])
        
        tEnd = time.time()
        print("Train time: %f sec" % (tEnd-tStart))
        
        history.history['loss'][0:200] = [300] * 200
        history.history['val_loss'][0:200] = [300] * 200
        testLoss[0:200] = [300] * 200
        
        plt.figure()
        plt.subplot(211)
        plt.plot(history.history['loss'], label = 'train_loss')
        plt.plot(history.history['val_loss'], label = 'validation_loss')
        plt.plot(testLoss, label = 'test_loss')
        plt.axvline(np.argmin(history.history['val_loss']), color='r')
        lgd = plt.legend(bbox_to_anchor=(1.05, 0.65), loc=2, borderaxespad=0.)
#         plt.savefig('lossFigRawSensorUI' + str(k) + '.png', bbox_extra_artists=[lgd], bbox_inches='tight')
        plt.subplot(212)
        plt.plot(history.history['precisionM'], label = 'train_precision')
        plt.plot(history.history['val_precisionM'], label = 'validation_precision')
        plt.plot(testPrecision, label = 'test_precision')
        plt.axvline(np.argmin(history.history['val_loss']), color='r')
        lgd = plt.legend(bbox_to_anchor=(1.05, 0.65), loc=2, borderaxespad=0.)
        plt.savefig('figRawSensorUI_neuron32_epoch1200_featureLen15_' + str(k) + '.png', bbox_extra_artists=[lgd], bbox_inches='tight')

        print("min val_loss: %f, epoch: %d" % 
              (min(history.history['val_loss']), (np.argmin(history.history['val_loss'])+1)))
        print("min val_precision: %f, epoch: %d" % 
              (min(history.history['val_precisionM']), (np.argmin(history.history['val_precisionM'])+1)))
        
        model = load_model('modelsRawSensorUI/weights' + str(k) + '.hdf5', custom_objects = {'precisionM': precisionM})
        predictions = model.predict(testX)
        print("k = %d" % k)
        for i in range(len(testY)):
            print("estimated: %f, ground truth: %f" % (predictions[i][0], testY[i]))
            errorDis = predictions[i][0] - testY[i]
            lossList.append(errorDis)
        
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

        print("\nmeanError: %f" % meanError)
        print("precision: %f" % precision)
        print("rmse: %f" % rmse)
        
        meanerrorList.append(meanError)
        precisionList.append(precision)
        rmseList.append(rmse)
    
    print("\nAverage MeanError: %f" % (sum(meanerrorList) / float(len(meanerrorList))))
    print("nAverage Precision: %f" % (sum(precisionList) / float(len(precisionList))))
    print("nAverage Rmse: %f" % (sum(rmseList) / float(len(rmseList))))
    
#     f.close()
    