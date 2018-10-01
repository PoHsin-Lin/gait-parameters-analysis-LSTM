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

l3long = 28
L3 = 20

DegreeToRadians = 0.0174532925

subjectList = ['PoHsinnn', 'Fast', 'Slow']
lengthList = [18, 30, 22]

featureLen = 3

# define for classfication
STATIC = 0
MOVING = 400

def read_pickle(index, pathHead):
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
#     if len(objs1) < 970 or len(objs2) < 970 or len(objs4) < 970 or len(objs5) < 970 or len(objs6) < 970:
#         print(index)

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
            
#     plt.plot(gx)
    gx, gy, gz, ax, ay, az = lowPass([gx, gy, gz, ax, ay, az])
    gx2, gy2, gz2, ax2, ay2, az2 = lowPass([gx2, gy2, gz2, ax2, ay2, az2])
    gx4, gy4, gz4, ax4, ay4, az4 = lowPass([gx4, gy4, gz4, ax4, ay4, az4])
    gx5, gy5, gz5, ax5, ay5, az5 = lowPass([gx5, gy5, gz5, ax5, ay5, az5])
    gx6, gy6, gz6, ax6, ay6, az6 = lowPass([gx6, gy6, gz6, ax6, ay6, az6])
#     plt.plot(gx)
#     plt.savefig("lowpass.png")
#     print(sensorData1[10])
    
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

    # --------------- normalize the data length of five sensors ---------------
    
    # ----- the timing difference between five sensor < 0.008s -----
#     for i in range(3):
#         maxi = max(sensorData1[0][toIndex1[i]], sensorData2[0][toIndex2[i]], 
#                sensorData4[0][toIndex4[i]], sensorData5[0][toIndex5[i]], sensorData6[0][toIndex6[i]])
#         mini = min(sensorData1[0][toIndex1[i]], sensorData2[0][toIndex2[i]], 
#                sensorData4[0][toIndex4[i]], sensorData5[0][toIndex5[i]], sensorData6[0][toIndex6[i]])
#         print("toe off:")
#         if maxi - mini >= 0.008:
#             print("-------------------- %f --------------------" % (maxi-mini))
#         print("sensor1 time: %f, index: %d" % (sensorData1[0][toIndex1[i]], toIndex1[i]))
#         print("sensor2 time: %f, index: %d" % (sensorData2[0][toIndex2[i]], toIndex2[i]))
#         print("sensor4 time: %f, index: %d" % (sensorData4[0][toIndex4[i]], toIndex4[i]))
#         print("sensor5 time: %f, index: %d" % (sensorData5[0][toIndex5[i]], toIndex5[i]))
#         print("sensor6 time: %f, index: %d" % (sensorData6[0][toIndex6[i]], toIndex6[i]))
#         maxi = max(sensorData1[0][hsIndex1[i]], sensorData2[0][hsIndex2[i]], 
#                sensorData4[0][hsIndex4[i]], sensorData5[0][hsIndex5[i]], sensorData6[0][hsIndex6[i]])
#         mini = min(sensorData1[0][hsIndex1[i]], sensorData2[0][hsIndex2[i]], 
#                sensorData4[0][hsIndex4[i]], sensorData5[0][hsIndex5[i]], sensorData6[0][hsIndex6[i]])
#         print("heel strike:")
#         if maxi - mini >= 0.008:
#             print("-------------------- %f --------------------" % (maxi-mini))
#         print("sensor1 time: %f, index: %d" % (sensorData1[0][hsIndex1[i]], hsIndex1[i]))
#         print("sensor2 time: %f, index: %d" % (sensorData2[0][hsIndex2[i]], hsIndex2[i]))
#         print("sensor4 time: %f, index: %d" % (sensorData4[0][hsIndex4[i]], hsIndex4[i]))
#         print("sensor5 time: %f, index: %d" % (sensorData5[0][hsIndex5[i]], hsIndex5[i]))
#         print("sensor6 time: %f, index: %d" % (sensorData6[0][hsIndex6[i]], hsIndex6[i]))
    
    # ----- cut tail -----
    x_seq1 = sensorData1[3:6, 0:hsIndex1[2]+12]
    x_seq2 = sensorData2[3:6, 0:hsIndex2[2]+12]
    x_seq4 = sensorData4[3:6, 0:hsIndex4[2]+12]
    x_seq5 = sensorData5[3:6, 0:hsIndex5[2]+12]
    x_seq6 = sensorData6[3:6, 0:hsIndex6[2]+12]
    
    # ----- normalize data length between five sensors -----
    
#     print(x_seq1[0].shape)
#     plt.plot(x_seq1[0])
    
    X_seq = []
    hsTimingIndex = []
    toTimingIndex = []
    indexPassed = 0
    timePassed = 0
    timeListNew = None
    for i in range(3):
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
            
        for j in range(len(segment1a[0])):
            X_seq.append(np.array([segment4a[0][j], segment4a[1][j], segment4a[2][j]]))
#             X_seq.append(np.array([segment1a[0][j], segment1a[1][j], segment1a[2][j],  
#                                    segment2a[0][j], segment2a[1][j], segment2a[2][j],  
#                                    segment4a[0][j], segment4a[1][j], segment4a[2][j],
#                                    segment5a[0][j], segment5a[1][j], segment5a[2][j],  
#                                    segment6a[0][j], segment6a[1][j], segment6a[2][j]]))
        for j in range(len(segment1b[0])):
            X_seq.append(np.array([segment4b[0][j], segment4b[1][j], segment4b[2][j]]))
#             X_seq.append(np.array([segment1b[0][j], segment1b[1][j], segment1b[2][j],  
#                                    segment2b[0][j], segment2b[1][j], segment2b[2][j],  
#                                    segment4b[0][j], segment4b[1][j], segment4b[2][j],
#                                    segment5b[0][j], segment5b[1][j], segment5b[2][j],  
#                                    segment6b[0][j], segment6b[1][j], segment6b[2][j]]))
            
        toTimingIndex.append(indexPassed + len(segment1a[0]))
        indexPassed = indexPassed + len(segment1a[0])
        
        if i == 2:
            hsTimingIndex.append(indexPassed + len(segment1b[0]) - 12)
            indexPassed = indexPassed + len(segment1b[0]) - 12
        else:
            hsTimingIndex.append(indexPassed + len(segment1b[0]))
            indexPassed = indexPassed + len(segment1b[0])

#     for i in range(3):
#         print("toTime[i]", toTime[i])
#         print("timeListNew[toTimingIndex[i]]", timeListNew[toTimingIndex[i]])
#         if abs(toTime[i] - timeListNew[toTimingIndex[i]]) >= 0.016:
#             print("------------------------------", abs(toTime[i] - timeListNew[toTimingIndex[i]]))
            
#         print("hsTime[i]", hsTime[i])
#         print("timeListNew[hsTimingIndex[i]]", timeListNew[hsTimingIndex[i]])
#         if abs(hsTime[i] - timeListNew[hsTimingIndex[i]]) >= 0.016:
#             print("------------------------------", abs(hsTime[i] - timeListNew[hsTimingIndex[i]]))
            
    
    X_seq = np.array(X_seq)
    
    # ------------ generate y sequences for RNN -------------
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
    for i in range(len(myList)):
        myList[i] = [np.float32(x) for x in myList[i]]

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
    
#     print("maxList", maxList)
#     print("minList", minList)

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

def computeErrorTime(yTrue, yEst, debug=False):
    global maxIndexDisHs, maxIndexDisTo
    
    hsCorrectIndex = []
    toCorrectIndex = []
    
    hsData = [a[1] for a in yEst]
    toData = [a[2] for a in yEst]
    hsLocalMax = argrelextrema(np.asarray(hsData), np.greater, mode='wrap')[0]
    toLocalMax = argrelextrema(np.asarray(toData), np.greater, mode='wrap')[0]
    
    hsLocalMax = [x for x in hsLocalMax if hsData[x] > 0.5]
    toLocalMax = [x for x in toLocalMax if toData[x] > 0.5]
    
    for i, y in enumerate(yTrue):
        if y[1] >= 1.0:
            hsCorrectIndex.append(i)
        if y[2] >= 1.0:
            toCorrectIndex.append(i)

    def removeDup(seq):
        seen = set()
        seen_add = seen.add
        return [x for x in seq if not (x in seen or seen_add(x))]

#     print("hsLocalMax", hsLocalMax)
    hsIndex = []
    for i in range(len(hsLocalMax)):
        indexSum = hsLocalMax[i]
        num = 1.0
        for j in range(len(hsLocalMax)):
            if i == j:
                continue
            if abs(hsLocalMax[i]-hsLocalMax[j]) < 100:
                indexSum = indexSum + hsLocalMax[j]
                num = num + 1.0
        hsIndex.append(indexSum/num)
    hsIndex = removeDup(hsIndex)
    if debug:
        print("hsIndex", hsIndex)
    
#     print("toLocalMax", toLocalMax)
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
    if debug:
        print("toIndex", toIndex)
    
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
    
    detectLossHs = abs(3 - len(hsIndex))
    detectLossTo = abs(3 - len(toIndex))
    
    return errorHsNumList, errorToNumList, detectLossHs, detectLossTo

def detectLossTest(model, valiX, valiY):
    global maxIndexDisHs, maxIndexDisTo
    detectLoss = 0
    maxIndexDisHs = -1000
    maxIndexDisTo = -1000
    for i in range(len(valiX)):
        vali_X = valiX[i].reshape((1, valiX[i].shape[0], featureLen))
        predictions = model.predict(vali_X)
        # ----- compute error time -----
        hsError, toError, detectLossHs, detectLossTo = computeErrorTime(valiY[i], predictions[0])
        detectLoss = detectLoss + detectLossHs + detectLossTo

    # print("detect loss count: %d" % detectLoss)
    
    return detectLoss

def seperateTrainValiTest(X_seq, X_weight, Y_seq, index, subject):
    totalNum = lengthList[subject]
    testNum = int(round(totalNum*0.17))
    valiNum = int(round(totalNum*0.17))
    testIndex = index * testNum
    valiIndex = testIndex + testNum
    trainIndex = valiIndex + valiNum
    
    testData = [X_seq[testIndex:testIndex+testNum], Y_seq[testIndex:testIndex+testNum]]
    if index == 5:
        valiData = [X_seq[0:valiNum], X_weight[0:valiNum], Y_seq[0:valiNum]]
        trainData = [X_seq[valiNum:testIndex], X_weight[valiNum:testIndex], Y_seq[valiNum:testIndex]]
    else:
        valiData = [X_seq[valiIndex:valiIndex+valiNum], X_weight[valiIndex:valiIndex+valiNum], Y_seq[valiIndex:valiIndex+valiNum]]
        trainData = [np.concatenate((X_seq[0:testIndex], X_seq[trainIndex:])), 
                     np.concatenate((X_weight[0:testIndex], X_weight[trainIndex:])), 
                     np.concatenate((Y_seq[0:testIndex], Y_seq[trainIndex:]))]
    
    return trainData, testData, valiData

def seperateData(X_seq, X_weight, Y_seq, k):
    pohsinNum = lengthList[0]
    fastNum = lengthList[1]
    slowNum = lengthList[2]
    
    pohsinTrain, pohsinTest, pohsinVali = seperateTrainValiTest(X_seq[0:lengthList[0]], 
                                                                X_weight[0:lengthList[0]], 
                                                                Y_seq[0:lengthList[0]], k, 0)
    fastTrain, fastTest, fastVali = seperateTrainValiTest(X_seq[lengthList[0]:sum(lengthList[0:2])], 
                                                          X_weight[lengthList[0]:sum(lengthList[0:2])], 
                                                          Y_seq[lengthList[0]:sum(lengthList[0:2])], k, 1)
    slowTrain, slowTest, slowVali = seperateTrainValiTest(X_seq[sum(lengthList[0:2]):sum(lengthList[0:3])], 
                                                          X_weight[sum(lengthList[0:2]):sum(lengthList[0:3])], 
                                                          Y_seq[sum(lengthList[0:2]):sum(lengthList[0:3])], k, 2)
    
#     print("k = %d" % k)
#     print("pohsinTrain[0].length = %d" % len(pohsinTrain[0]))
#     print("pohsinTest[0].length = %d" % len(pohsinTest[0]))
#     print("pohsinVali[0].length = %d" % len(pohsinVali[0]))
#     print("fastTrain[0].length = %d" % len(fastTrain[0]))
#     print("fastTest[0].length = %d" % len(fastTest[0]))
#     print("fastVali[0].length = %d" % len(fastVali[0]))
#     print("slowTrain[0].length = %d" % len(slowTrain[0]))
#     print("slowTest[0].length = %d" % len(slowTest[0]))
#     print("slowVali[0].length = %d" % len(slowVali[0]))
    
    trainData = [np.concatenate((pohsinTrain[0], fastTrain[0], slowTrain[0])),
                 np.concatenate((pohsinTrain[1], fastTrain[1], slowTrain[1])), 
                 np.concatenate((pohsinTrain[2], fastTrain[2], slowTrain[2]))]
    testData = [np.concatenate((pohsinTest[0], fastTest[0], slowTest[0])), 
                np.concatenate((pohsinTest[1], fastTest[1], slowTest[1]))]
    valiData = [np.concatenate((pohsinVali[0], fastVali[0], slowVali[0])), 
                np.concatenate((pohsinVali[1], fastVali[1], slowVali[1])), 
                np.concatenate((pohsinVali[2], fastVali[2], slowVali[2]))]
    testIndex = [len(pohsinTest[0]), len(fastTest[0]), len(slowTest[0])]
    
    return trainData, testData, valiData, testIndex

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
    global sampleNum
    
    f = open("LstmStepEventSpeed_epoch100_featureLen3_stateful.txt", 'w')
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
            dataList = read_pickle(x, pathHead)
            X_seq, Y_seq = dataPreprocess(dataList)
            toFloat(X_seq)
            Y_seq = [np.float32(i) for i in Y_seq]
            X_seqs.append(X_seq)
            X_weights.append(21.0/float(dataLen))
            Y_seqs.append(Y_seq)
    X_seqs = np.array(X_seqs)
    X_weights = np.array(X_weights)
    Y_seqs = np.array(Y_seqs)
    sampleNum = X_seqs.shape[0]
    
    # --------- normalize data value----------
    
    normalizeData(X_seqs)
    for i in range(len(Y_seqs)):
        Y_seqs[i] = np.array(Y_seqs[i])
    
    hsErrorCV_PoHsin = []
    hsPrecisionCV_PoHsin = []
    hsRmseCV_PoHsin = []
    toErrorCV_PoHsin = []
    toPrecisionCV_PoHsin = []
    toRmseCV_PoHsin = []
    detectLossCV_PoHsin = 0
    hsErrorCV_Fast = []
    hsPrecisionCV_Fast = []
    hsRmseCV_Fast = []
    toErrorCV_Fast = []
    toPrecisionCV_Fast = []
    toRmseCV_Fast = []
    detectLossCV_Fast = 0
    hsErrorCV_Slow = []
    hsPrecisionCV_Slow = []
    hsRmseCV_Slow = []
    toErrorCV_Slow = []
    toPrecisionCV_Slow = []
    toRmseCV_Slow = []
    detectLossCV_Slow = 0
    for k in range(2):
        print("\nk = %d"% (k))
        lossList = []
        # --------- seperate data -----------
        trainData, testData, valiData, testIndex = seperateData(X_seqs, X_weights, Y_seqs, k)
        
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

        minW = min(trainW)
        trainW = [x / minW for x in trainW]

        model = Sequential()
        model.add(LSTM(20, batch_input_shape=(1, None, featureLen), return_sequences=True, stateful=True))
        model.add(Dense(3, activation='softmax'))
        model.compile(loss='categorical_crossentropy', optimizer='adam', sample_weight_mode="temporal", metrics = ['categorical_accuracy'])

        # ----- construct temporal training and validation weight -----
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
        # -----------------------------------------------

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
                model.reset_states()
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
                model.reset_states()
                valiLossList.append(metricsList[0])
                valiAccList.append(metricsList[1])
            meanValiLoss.append(sum(valiLossList) / float(len(valiLossList)))
            meanValiAcc.append(sum(valiAccList) / float(len(valiAccList)))
            
            if epoch % 5 == 0:
                avgLoss = sum(meanValiLoss[-5:]) / float(len(meanValiLoss[-5:]))
                if avgLoss > lastAvgLoss:
                    stopCount = stopCount + 1
                else:
                    stopCount = 0
                lastAvgLoss = avgLoss
                if meanValiLoss[-1] < minValiLoss:
                    if detectLossTest(model, valiX, valiY) == 0:
                        minValiLoss = meanValiLoss[-1]
                        model.save('modelsForEventSpeed_T2/model_k' + str(k) + '_best.h5')
                        minEpoch = epoch
                if stopCount > 5:
                    stopEpoch = epoch
                    print("stopEpoch: %d" % epoch)
                    break
            if epoch % 5 == 0:
                model.save('modelsForEventSpeed_T2/model_k' + str(k) + '_e' + str(epoch) + '.h5')
            # ------------------------

        tEnd = time.time()
        print("Train time: %f sec" % (tEnd-tStart))

        # ----- draw training graph -----
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


        # -------------------- test --------------------
        
#         stopEpoch = 101
        xListo = np.linspace(5, 100, 20)
        xList = [int(a) for a in xListo if a < stopEpoch]
        xList = [0] + xList
#         xList = [0, 2, 4, 6, 8, 10]
        
        lossCountListCE = []
        errorHsListCE = []
        errorToListCE = []
        
        for x in xList:
            global maxIndexDisHs, maxIndexDisTo
            errorHsNumList = []
            errorToNumList = []
            maxIndexDisHs = -1000
            maxIndexDisTo = -1000
            detectLoss = 0
            model = load_model('modelsForEventSpeed_T2/model_k' + str(k) + '_e' + str(x) + '.h5')
            for i in range(len(testX)):
                test_X = testX[i].reshape((1, testX[i].shape[0], featureLen))
                predictions = model.predict(test_X)
                model.reset_states()
                # ----- compute error time -----
                hsError, toError, detectLossHs, detectLossTo = computeErrorTime(testY[i], predictions[0])
#                 hsErrorList = []
#                 toErrorList = []
#                 for hs in hsError:
#                     hsErrorList.append("%.1f" % hs)
#                 for to in toError:
#                     toErrorList.append("%.1f" % to)
#                 print("%s's test no.%d:" % (subjectList[k], i))
#                 print("hsError: ", hsErrorList, "lossNum: %d" % detectLossHs)
#                 print("toError: ", toErrorList, "lossNum: %d" % detectLossTo)
                
                detectLoss = detectLoss + detectLossHs + detectLossTo
                errorHsNumList.extend(hsError)
                errorToNumList.extend(toError)

                # ----- print results -----
    #             indexHsTo = []
    #             for j, yTrue in enumerate(testY[i]):
    #                 if yTrue.tolist() in [[0, 1.0, 0], [0, 0, 1.0]]:
    #                     indexHsTo.append(j)
    #             lastPrintFlag = None
    #             for j, (yEst, yTrue) in enumerate(zip(predictions[0], testY[i])):
    #                 printFlag = True
    #                 for index in indexHsTo:
    #                     if abs(j-index) <= 10:
    #                         printFlag = True
    #                 if printFlag:
    #                     yEstList = []
    #                     yTrueList = []
    #                     for e, t in zip(yEst, yTrue):
    #                         yEstList.append("%.4f" % e)
    #                         yTrueList.append("%.4f" % t)
    #                     print("yEst:", yEstList, "yTrue:", yTrueList, "index:", j)
    #                 if lastPrintFlag == True and printFlag == False:
    #                     print("\n")
    #                 lastPrintFlag = printFlag
    
            lossCountListCE.append(detectLoss)
            errorHsListCE.append((sum(errorHsNumList) / float(len(errorHsNumList))))
            errorToListCE.append((sum(errorToNumList) / float(len(errorToNumList))))
        
#         plt.figure()
        plt.subplot(313)
        plt.plot(xList, lossCountListCE, label='lossCountList')
        plt.plot(xList, errorHsListCE, label='errorHsList')
        plt.plot(xList, errorToListCE, label='errorToList')
        lgd3 = plt.legend(bbox_to_anchor=(1.05, 0.65), loc=2, borderaxespad=0.)
        plt.savefig('detectLossSpeed_2_'+str(k)+'.png', bbox_extra_artists=[lgd1, lgd2, lgd3], bbox_inches='tight')
        
        # evaluate target epoch
        global maxIndexDisHs, maxIndexDisTo
        maxIndexDisHs = 0
        maxIndexDisTo = 0
        errorHsNumListPoHsin = []
        errorToNumListPoHsin = []
        errorHsNumListFast = []
        errorToNumListFast = []
        errorHsNumListSlow = []
        errorToNumListSlow = []
        detectLossPoHsin = 0
        detectLossFast = 0
        detectLossSlow = 0
        try:
            model = load_model('modelsForEventSpeed_T2/model_k' + str(k) + '_best.h5')
            for i in range(len(testX)):
                test_X = testX[i].reshape((1, testX[i].shape[0], featureLen))
                predictions = model.predict(test_X)
                model.reset_states()
                # ----- compute error time -----
                hsError, toError, detectLossHs, detectLossTo = computeErrorTime(testY[i], predictions[0], debug = True)
                
                if i < testIndex[0]:
                    # pohsin
                    print("PoHsin's test %d" % i)
                    errorHsNumListPoHsin.extend(hsError)
                    errorToNumListPoHsin.extend(toError)
                    detectLossPoHsin = detectLossPoHsin + detectLossHs + detectLossTo
                elif i >= testIndex[0] and i < sum(testIndex[0:2]):
                    # fast
                    print("Fast's test %d" % i)
                    errorHsNumListFast.extend(hsError)
                    errorToNumListFast.extend(toError)
                    detectLossFast = detectLossFast + detectLossHs + detectLossTo
                else:
                    # slow
                    print("Slow's test %d" % i)
                    errorHsNumListSlow.extend(hsError)
                    errorToNumListSlow.extend(toError)
                    detectLossSlow = detectLossSlow + detectLossHs + detectLossTo
                
                hsErrorList = []
                toErrorList = []
                for hs in hsError:
                    hsErrorList.append("%.1f" % hs)
                for to in toError:
                    toErrorList.append("%.1f" % to)
                print("%d's test no.%d:" % (k, i))
                print("hsError: ", hsErrorList, "lossNum: %d" % detectLossHs)
                print("toError: ", toErrorList, "lossNum: %d" % detectLossTo)
#                 detectLoss = detectLoss + detectLossHs + detectLossTo
#                 errorHsNumList.extend(hsError)
#                 errorToNumList.extend(toError)
            
            errorHsNumListPoHsin = [a*0.008 for a in errorHsNumListPoHsin]
            errorToNumListPoHsin = [a*0.008 for a in errorToNumListPoHsin]
            errorHsNumListFast = [a*0.008 for a in errorHsNumListFast]
            errorToNumListFast = [a*0.008 for a in errorToNumListFast]
            errorHsNumListSlow = [a*0.008 for a in errorHsNumListSlow]
            errorToNumListSlow = [a*0.008 for a in errorToNumListSlow]
            
            meanErrorHSp, precisionHSp, rmseHSp = meanErrorPrecisionRMSE(errorHsNumListPoHsin)
            meanErrorTOp, precisionTOp, rmseTOp = meanErrorPrecisionRMSE(errorToNumListPoHsin)
            meanErrorHSf, precisionHSf, rmseHSf = meanErrorPrecisionRMSE(errorHsNumListFast)
            meanErrorTOf, precisionTOf, rmseTOf = meanErrorPrecisionRMSE(errorToNumListFast)
            meanErrorHSs, precisionHSs, rmseHSs = meanErrorPrecisionRMSE(errorHsNumListSlow)
            meanErrorTOs, precisionTOs, rmseTOs = meanErrorPrecisionRMSE(errorToNumListSlow)
            
            print("max index distance HS: %d" % maxIndexDisHs)
            print("max index distance TO: %d" % maxIndexDisTo)
            print("HS mean error PoHsin: %f" % meanErrorHSp)
            print("HS precision PoHsin: %f" % precisionHSp)
            print("HS rmse PoHsin: %f" % rmseHSp)
            print("HS mean error Fast: %f" % meanErrorHSf)
            print("HS precision Fast: %f" % precisionHSf)
            print("HS rmse Fast: %f" % rmseHSf)
            print("HS mean error Slow: %f" % meanErrorHSs)
            print("HS precision Slow: %f" % precisionHSs)
            print("HS rmse Slow: %f" % rmseHSs)
            print("TO mean error PoHsin: %f" % meanErrorTOp)
            print("TO precision PoHsin: %f" % precisionTOp)
            print("TO rmse PoHsin: %f" % rmseTOp)
            print("TO mean error Fast: %f" % meanErrorTOf)
            print("TO precision Fast: %f" % precisionTOf)
            print("TO rmse Fast: %f" % rmseTOf)
            print("TO mean error Slow: %f" % meanErrorTOs)
            print("TO precision Slow: %f" % precisionTOs)
            print("TO rmse Slow: %f" % rmseTOs)
            print("detect loss count PoHsin: %d" % detectLossPoHsin)
            print("detect loss count Fast: %d" % detectLossFast)
            print("detect loss count Slow: %d" % detectLossSlow)
            
            detectLossCV_PoHsin = detectLossCV_PoHsin + detectLossPoHsin
            detectLossCV_Fast = detectLossCV_Fast + detectLossFast
            detectLossCV_Slow = detectLossCV_Slow + detectLossSlow
            hsErrorCV_PoHsin.append(meanErrorHSp)
            hsPrecisionCV_PoHsin.append(precisionHSp)
            hsRmseCV_PoHsin.append(rmseHSp)
            toErrorCV_PoHsin.append(meanErrorTOp)
            toPrecisionCV_PoHsin.append(precisionTOp)
            toRmseCV_PoHsin.append(rmseTOp)
            
            hsErrorCV_Fast.append(meanErrorHSf)
            hsPrecisionCV_Fast.append(precisionHSf)
            hsRmseCV_Fast.append(rmseHSf)
            toErrorCV_Fast.append(meanErrorTOf)
            toPrecisionCV_Fast.append(precisionTOf)
            toRmseCV_Fast.append(rmseTOf)
            
            hsErrorCV_Slow.append(meanErrorHSs)
            hsPrecisionCV_Slow.append(precisionHSs)
            hsRmseCV_Slow.append(rmseHSs)
            toErrorCV_Slow.append(meanErrorTOs)
            toPrecisionCV_Slow.append(precisionTOs)
            toRmseCV_Slow.append(rmseTOs)
        except:
            print("don't have best model")
            detectLossCV_PoHsin = detectLossCV_PoHsin + 1000
            detectLossCV_Fast = detectLossCV_Fast + 1000
            detectLossCV_Slow = detectLossCV_Slow + 1000

    print("\nCV HS mean error PoHsin: %f" % (sum(hsErrorCV_PoHsin) / float(len(hsErrorCV_PoHsin))))
    print("CV HS precision PoHsin: %f" % (sum(hsPrecisionCV_PoHsin) / float(len(hsPrecisionCV_PoHsin))))
    print("CV HS rmse PoHsin: %f" % (sum(hsRmseCV_PoHsin) / float(len(hsRmseCV_PoHsin))))
    print("\nCV TO mean error PoHsin: %f" % (sum(toErrorCV_PoHsin) / float(len(toErrorCV_PoHsin))))
    print("CV TO precision PoHsin: %f" % (sum(toPrecisionCV_PoHsin) / float(len(toPrecisionCV_PoHsin))))
    print("CV TO rmse PoHsin: %f" % (sum(toRmseCV_PoHsin) / float(len(toRmseCV_PoHsin))))
    print("CV detect loss count PoHsin: %d" % detectLossCV_PoHsin)
    
    print("\nCV HS mean error Fast: %f" % (sum(hsErrorCV_Fast) / float(len(hsErrorCV_Fast))))
    print("CV HS precision Fast: %f" % (sum(hsPrecisionCV_Fast) / float(len(hsPrecisionCV_Fast))))
    print("CV HS rmse Fast: %f" % (sum(hsRmseCV_Fast) / float(len(hsRmseCV_Fast))))
    print("\nCV TO mean error Fast: %f" % (sum(toErrorCV_Fast) / float(len(toErrorCV_Fast))))
    print("CV TO precision Fast: %f" % (sum(toPrecisionCV_Fast) / float(len(toPrecisionCV_Fast))))
    print("CV TO rmse Fast: %f" % (sum(toRmseCV_Fast) / float(len(toRmseCV_Fast))))
    print("CV detect loss count Fast: %d" % detectLossCV_Fast)
    
    print("\nCV HS mean error Slow: %f" % (sum(hsErrorCV_Slow) / float(len(hsErrorCV_Slow))))
    print("CV HS precision Slow: %f" % (sum(hsPrecisionCV_Slow) / float(len(hsPrecisionCV_Slow))))
    print("CV HS rmse Slow: %f" % (sum(hsRmseCV_Slow) / float(len(hsRmseCV_Slow))))
    print("\nCV TO mean error Slow: %f" % (sum(toErrorCV_Slow) / float(len(toErrorCV_Slow))))
    print("CV TO precision Slow: %f" % (sum(toPrecisionCV_Slow) / float(len(toPrecisionCV_Slow))))
    print("CV TO rmse Slow: %f" % (sum(toRmseCV_Slow) / float(len(toRmseCV_Slow))))
    print("CV detect loss count Slow: %d" % detectLossCV_Slow)

    f.close()
