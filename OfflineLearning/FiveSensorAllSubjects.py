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
from scipy import signal
import matplotlib.pyplot as plt

DegreeToRadians = 0.0174532925

# the length of a shoe
l3long = 28
# the length of the grounded part of the shoe at the moment of toe-off
L3 = 20

# definition of states
STATIC = 0
MOVING = 400

# ------------------- The data of six volunteers -------------------

# the list of Name, Leg length, and the number of walking trials (each trial contains three strides)
subjectList = ['PoHsinnn', 'Winston', 'Jay', 'Ting', 'Thomas', 'Gene']
legList = [(39.5, 45), (40, 49), (37, 47), (40, 52), (36.5, 47), (38.5, 48)]
lengthList = [18, 13, 7, 7, 7, 19]
scaleList = [(1.17, 1.10, 0.9), (1.23, 1.10, 0.7), (1.23, 1.10, 0.77), (0.9, 1.23, 0.7), (1.17, 1.10, 0.9), (1.03, 1.17, 0.7)]

'''
    minP, minQ, minR is the parameters that will be multiplied by L1, L2, L3, seperately. 
    They are used to scale the measured length(L1, L2, L3) to proper lengthes(l1, l2, l3) for our Mechanical Model.
    
    L1 = thigh length, L2 = shin length, L3 = shoe length. L1 and L2 are stored in legList.
    
    l1 = L1*minP
    l2 = L2*minQ
    l3 = L3*minR
    
    For each volunteer, minP, minQ, and minR are tuned using the walking data of other five volunteers. 
    We do this to simulate the scenario that new volunteers come, we don't have to tune these three parameters for them.
'''

# ------------------------------------------------------------------

# ------------------- The data of three walking speeds of the same volunteer -------------------

# subjectList = ['PoHsinnn', 'Fast', 'Slow']
# legList = [(39.5, 45), (39.5, 45), (39.5, 45)]
# lengthList = [18, 30, 22]
# scaleList = [(1.23, 1.10, 0.7), (1.23, 1.10, 0.7), (1.23, 1.10, 0.7)]

# ----------------------------------------------------------------------------------------------

def read_pickle(index, pathHead, recover=False):
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
            objs1.append(pickle.load(fp1))
        except:
            break
    while True:
        try:
            objs2.append(pickle.load(fp2))
        except:
            break
    while True:
        try:
            objs3.append(pickle.load(fp3))
        except:
            break
    while True:
        try:
            objs4.append(pickle.load(fp4))
        except:
            break
    while True:
        try:
            objs5.append(pickle.load(fp5))
        except:
            break
    while True:
        try:
            objs6.append(pickle.load(fp6))
        except:
            break

    # timestamp represents the the time interval between two successive BLE notifications on the HOST.
    # gyro1-6 are the Z-axis of gyroscope data
    timestamp, gyro1 = ([] for i in range(2))
    timestamp2, gyro2 = ([] for i in range(2))
    timestamp4, gyro4 = ([] for i in range(2))
    timestamp5, gyro5 = ([] for i in range(2))
    timestamp6, gyro6 = ([] for i in range(2))

    # the ground truth from ultrasonic and two FSRs
    # distance = the distance between the left shoe and the wall
    # state = record the TO and HS state
    timestamp3, distance, state = ([] for i in range(3))
    
    for obj in objs1:
        timestamp.append(obj['timestamp'])
        gyro1.append(obj['Gyo'].item(0,2))

    for obj in objs2:
        timestamp2.append(obj['timestamp'])
        gyro2.append(obj['Gyo'].item(0,2))

    for obj in objs3:
        timestamp3.append(obj['timestamp'])
        distance.append(obj['distance'])
        state.append(obj['state'])
    
    for obj in objs4:
        timestamp4.append(obj['timestamp'])
        gyro4.append(obj['Gyo'].item(0,2))
    
    for obj in objs5:
        timestamp5.append(obj['timestamp'])
        gyro5.append(obj['Gyo'].item(0,2))
        
    for obj in objs6:
        timestamp6.append(obj['timestamp'])
        gyro6.append(obj['Gyo'].item(0,2))
    
    # Integrate the Z-axis gyroscope data
    gyroList1 = integrationList(gyro1)
    gyroList2 = integrationList(gyro2)
    gyroList4 = integrationList(gyro4)
    gyroList5 = integrationList(gyro5)
    gyroList6 = integrationList(gyro6)
    
    # Make the timestamp distributes evenly, 
    # because the timestamp recorded on the HOST can't represent the time-interval between two successive BLE notifications.
    timestamp = np.linspace(0, 8, len(timestamp))
    timestamp2 = np.linspace(0, 8, len(timestamp2))
    timestamp4 = np.linspace(0, 8, len(timestamp4))
    timestamp5 = np.linspace(0, 8, len(timestamp5))
    timestamp6 = np.linspace(0, 8, len(timestamp6))
    
    # The gyroscope's Z-axis on the right leg and the left leg are opposite.
    # Transfer the gyroscope data from right leg to the same direction as the left leg.
    for i in range(len(gyroList5)):
        gyroList5[i] = gyroList5[i] * -1
    for i in range(len(gyroList6)):
        gyroList6[i] = gyroList6[i] * -1
    
    # some distance data are lossed, recover them.
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
        
    return [timestamp, gyroList1, timestamp2, gyroList2, timestamp4, gyroList4, timestamp5, gyroList5, timestamp6, gyroList6, timestamp3, distance, state]

def integrationList(dataRaw):
    summ = 0
    result = []
    
    # low-pass filter
    b, a = signal.butter(10, 0.3)
    data = signal.filtfilt(b, a, dataRaw)
    
    # integration
    t = 8.0/len(data)
    for i in range(len(data)-1):
        summ = summ + data[i] * t
        summ = summ + (data[i+1]-data[i])*t/2
        result.append(summ/DegreeToRadians)
        
    return result

def dataPreprocess(dataList):

    # calculate the distance (ground truth)
    disList = dataList[11]
    lastDis = disList[0]
    stepLengthList = []
    for dis in disList:
        if dis < lastDis:
            x = (lastDis - dis)
            # Calibrate the reading of ultrasonic sensor.
            correctionDis = -2.56564*math.pow(10, -8)*math.pow(x, 4) + 0.0000222831*math.pow(x, 3) - 0.00663392*math.pow(x, 2) + 1.85571*x - 27.5267
            stepLengthList.append(correctionDis)
            lastDis = dis

    # declare
    timeList1 = dataList[0][0:-1]
    timeList2 = dataList[2][0:-1]
    timeList4 = dataList[4][0:-1]
    timeList5 = dataList[6][0:-1]
    timeList6 = dataList[8][0:-1]
    
    leftThighPitch = dataList[1]
    leftShankPitch = dataList[3]
    shoePitch = dataList[5]
    rightShankPitch = dataList[7]
    rightThighPitch = dataList[9]

    timeList3 = dataList[10]
    stateList = dataList[12]

    hsTime = []
    toTime = []
    
    # get the timing of TO and HS events (ground truth)
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
            lastState = state
        lastT = t
    
    # these lists are used to store the angles at each joints at TO and HS moments
    rightThighHsPitch = []
    rightShankHsPitch = []
    leftThighHsPitch = []
    leftShankHsPitch = []
    shoeHsPitch = []

    rightThighToPitch = []
    rightShankToPitch = []
    leftThighToPitch = []
    leftShankToPitch = []
    shoeToPitch = []
    
    # in (timeList, pitchList), find the timestamp that is most close to t, and return the pitch at that timestamp
    def getClosePitch(t, timeList, pitchList):
        minValue = 1000
        closePitch = None
        for T, pitch in zip(timeList, pitchList):
            if abs(t-T) < minValue:
                closePitch = pitch
                minValue = abs(t-T)
        return closePitch
    
    # get the angles at joints at heel-strike timings
    for t in hsTime:
        # left thigh
        closePitch = getClosePitch(t, timeList1, leftThighPitch)
        leftThighHsPitch.append(closePitch)
        # left shin
        closePitch = getClosePitch(t, timeList2, leftShankPitch)
        leftShankHsPitch.append(closePitch)
        # left shoe
        closePitch = getClosePitch(t, timeList4, shoePitch)
        shoeHsPitch.append(closePitch)
        # right shin
        closePitch = getClosePitch(t, timeList5, rightShankPitch)
        rightShankHsPitch.append(closePitch)
        # right thigh
        closePitch = getClosePitch(t, timeList6, rightThighPitch)
        rightThighHsPitch.append(closePitch)

    # get the angles at joints at toe-off timings
    for t in toTime:
        # left thigh
        closePitch = getClosePitch(t, timeList1, leftThighPitch)
        leftThighToPitch.append(closePitch)
        # left shin
        closePitch = getClosePitch(t, timeList2, leftShankPitch)
        leftShankToPitch.append(closePitch)
        # left shoe
        closePitch = getClosePitch(t, timeList4, shoePitch)
        shoeToPitch.append(closePitch)
        # right shin
        closePitch = getClosePitch(t, timeList5, rightShankPitch)
        rightShankToPitch.append(closePitch)
        # right thigh
        closePitch = getClosePitch(t, timeList6, rightThighPitch)
        rightThighToPitch.append(closePitch)

    HsAngleList = [leftThighHsPitch, leftShankHsPitch, shoeHsPitch, rightShankHsPitch, rightThighHsPitch]
    ToAngleList = [leftThighToPitch, leftShankToPitch, shoeToPitch, rightShankToPitch, rightThighToPitch]
        
    return stepLengthList, hsTime, HsAngleList, toTime, ToAngleList

def lengthEstimate(HsAngleList, ToAngleList, x):
    stepCount = 3
    lengthList = []
    
    for i in range(stepCount):
        # ---------- calculate D1 ----------
        if ToAngleList[0][i] < 0:
            gamma = ((180-abs(ToAngleList[1][i])) + abs(ToAngleList[0][i])) * DegreeToRadians
        else:
            gamma = ((180-abs(ToAngleList[1][i])) - abs(ToAngleList[0][i])) * DegreeToRadians
        d1 = math.sqrt(l1**2 + l2**2 - 2*l1*l2*math.cos(gamma))
        phi1 = math.asin(math.sin(gamma)*l1/d1)
        beta = (abs(ToAngleList[2][i]) + (90 - abs(ToAngleList[1][i]))) * DegreeToRadians
        d2 = math.sqrt(d1**2 + l3**2 - 2*d1*l3*math.cos(beta+phi1))
        alpha1 = math.asin(math.sin(gamma)*l2/d1)
        alpha2 = math.asin(math.sin(beta+phi1)*l3/d2)
        alpha = ToAngleList[0][i] * DegreeToRadians
        if alpha < 0:
            if alpha2 > alpha1:
                alpha3 = abs(alpha) - (alpha2 - alpha1)
            else:
                alpha3 = (alpha1-alpha2) + abs(alpha)
        else:
            alpha3 = (alpha1-alpha2) - abs(alpha)
        theta4 = 90*DegreeToRadians - alpha3
        D1 = d2 * math.cos(theta4)
        
        # ---------- calculate D2 ----------
        # correct unreasonable angles
        if ToAngleList[3][i] >= ToAngleList[4][i]:
            gamma3 = 180 * DegreeToRadians
            d4 = l1 + l2
            phi3 = 0
        else:
            if ToAngleList[3][i] < 0:
                gamma3 = (180 - abs(ToAngleList[4][i]) - abs(ToAngleList[3][i])) * DegreeToRadians
            else:
                gamma3 = (180 - abs(ToAngleList[4][i]) + abs(ToAngleList[3][i])) * DegreeToRadians
            d4 = math.sqrt(l1**2 + l2**2 - 2*l1*l2*math.cos(gamma3))
            phi3 = math.asin(math.sin(gamma3)*l1/d4)
        # correct unreasonable angles
        if HsAngleList[3][i] > 0:
            HsAngleList[3][i] = -1
        if HsAngleList[4][i] > 0:
            HsAngleList[4][i] = -1
        if HsAngleList[3][i] > HsAngleList[4][i]:
            gamma2 = 180 * DegreeToRadians
            d3 = l1 + l2
            phi2 = 0
        else:
            gamma2 = (180 - abs(HsAngleList[3][i]) + abs(HsAngleList[4][i])) * DegreeToRadians
            d3 = math.sqrt(l1**2 + l2**2 - 2*l1*l2*math.cos(gamma2))
            phi2 = math.asin(math.sin(gamma2)*l1/d3)
        # correct unreasonable angles
        if ToAngleList[3][i] < 0:
            beta2 = (abs(HsAngleList[3][i]) - abs(ToAngleList[3][i])) * DegreeToRadians
        else:
            beta2 = (abs(HsAngleList[3][i]) + abs(ToAngleList[3][i])) * DegreeToRadians
        theta = phi3 + (beta2 - phi2)
        D2 = math.sqrt(d3**2 + d4**2 - 2*d3*d4*math.cos(theta))
        
        # ---------- calculate D3 ----------
        # correct unreasonable angles
        if HsAngleList[1][i] > HsAngleList[0][i]:
            gamma4 = 180 * DegreeToRadians
            d5 = l1 + l2
            phi4 = 0
        else:
            gamma4 = (180 - abs(HsAngleList[0][i]) + abs(HsAngleList[1][i])) * DegreeToRadians
            d5 = math.sqrt(l1**2 + l2**2 - 2*l1*l2*math.cos(gamma4))
            phi4 = math.asin(math.sin(gamma4)*l2/d5)
        theta2 = abs(HsAngleList[0][i]) * DegreeToRadians - phi4
        theta3 = 90*DegreeToRadians - theta2
        D3 = d5 * math.cos(theta3)

        SL = D1 + D2 + D3 + l3long - (l3long-l3)
        
        # correct the first stride and the last stride
        if i == 0:
            SL = SL * 1.05
        elif i == 2:
            SL = SL * 0.95
        lengthList.append(SL)
        
    return lengthList

def meanErrorPrecisionRMSE(lossList):
    # calculate the mean error, precision, and RMSE
    
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
    global l1, l2, l3
    
    # ---------- read files and preprocess ----------
    x_seqList = []
    for i, name in enumerate(subjectList):
        dataLen = lengthList[i]
        pathHead = 'walking' + name + '/'
        for x in range(1, dataLen+1):
            if name == 'PoHsin':
                dataList = read_pickle(x, pathHead, recover=True)
            else:
                dataList = read_pickle(x, pathHead)
            groundLength, hsTime, HsAngleList, toTime, ToAngleList = dataPreprocess(dataList)
            x_seqList.append([groundLength, HsAngleList, ToAngleList])
    totalLen = sum(lengthList)
    
    # for statistic 
    gMin = 1000
    gMax = -1000
    gSum = 0
    eMin = 1000
    eMax = -1000
    eSum = 0
    
    # These lists store the performance of six volunteers one by one 
    meanErrorList = []
    precisionList = []
    rmseList = []
    
    volunteerIndex = 0
    previousIndex = 0
    lossList = []
    
    for x in range(0, totalLen):
        # check the data belongs to which volunteer
        if x < lengthList[0]:
            volunteerIndex = 0
        elif x >= lengthList[0] and x < sum(lengthList[0:2]):
            volunteerIndex = 1
        elif x >= sum(lengthList[0:2]) and x < sum(lengthList[0:3]):
            volunteerIndex = 2
        elif x >= sum(lengthList[0:3]) and x < sum(lengthList[0:4]):
            volunteerIndex = 3
        elif x >= sum(lengthList[0:4]) and x < sum(lengthList[0:5]):
            volunteerIndex = 4
        elif x >= sum(lengthList[0:5]) and x < sum(lengthList[0:6]):
            volunteerIndex = 5
        
        # when switch to the next volunteer, calculate the performance of previous volunteer from lossList
        if previousIndex != volunteerIndex or x == totalLen-1:
            meanError, precision, rmse = meanErrorPrecisionRMSE(lossList)
            lossList = []
            meanErrorList.append(meanError)
            precisionList.append(precision)
            rmseList.append(rmse)
        
        print(subjectList[volunteerIndex], x)
        # scale l1, l2, l3
        l1 = legList[volunteerIndex][0] * scaleList[volunteerIndex][0]
        l2 = legList[volunteerIndex][1] * scaleList[volunteerIndex][1]
        l3 = L3 * scaleList[volunteerIndex][2]
        
        groundLength = x_seqList[x][0]
        estimatedLength = lengthEstimate(x_seqList[x][1], x_seqList[x][2], x)

        print("#%d gound truth: %s" % (x, groundLength[0:3]))
        print("#%d estimated length: %s" % (x, estimatedLength))
        
        # calculate the error of the three strides
        for j in range(3):
            errorDis = estimatedLength[j] - groundLength[j]
            print("Distance error (cm) = %f" % errorDis)
            lossList.append(errorDis)

            #---------- statistic counting ---------- 
            if groundLength[j] > gMax:
                gMax = groundLength[j]
            if groundLength[j] < gMin:
                gMin = groundLength[j]
            if estimatedLength[j] > eMax:
                eMax = estimatedLength[j]
            if estimatedLength[j] < eMin:
                eMin = estimatedLength[j]
            gSum = gSum + groundLength[j]
            eSum = eSum + estimatedLength[j]
            
        previousIndex = volunteerIndex
    
    # Performance of each volunteers
    print("\n ----- Performance of each volunteers -----")
    for i, name in enumerate(subjectList):
        print("%s:" % name)
        print("Mean error: %f" % meanErrorList[i])
        print("Precision: %f" % precisionList[i])
        print("RMSE: %f\n" % rmseList[i])
        
    # Weighted Average
    print("\n ----- Weighted Average -----")
    meanErrorSum = 0
    for i, meanError in enumerate(meanErrorList):
        meanErrorSum = meanErrorSum + meanError * lengthList[i]
    precisionSum = 0
    for i, precision in enumerate(precisionList):
        precisionSum = precisionSum + precision * lengthList[i]
    rmseSum = 0
    for i, rmse in enumerate(rmseList):
        rmseSum = rmseSum + rmse * lengthList[i]

    print("Mean error: %f" % (meanErrorSum / float(sum(lengthList))))
    print("Precision: %f" % (precisionSum / float(sum(lengthList))))
    print("RMSE: %f" % (rmseSum / float(sum(lengthList))))

    print("\nground truth max: %f" % gMax)
    print("ground truth min: %f" % gMin)
    print("estimated max: %f" % eMax)
    print("estimated min: %f" % eMin)
    print("ground average: %f" % (gSum / float(sum(lengthList))))
    print("estimated average: %f\n" % (eSum / float(sum(lengthList))))
    