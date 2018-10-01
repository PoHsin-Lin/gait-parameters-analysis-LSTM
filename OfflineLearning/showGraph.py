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
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# device01: 3c:cd:40:18:c2:07   down lower leg
# device02: 3c:cd:40:18:c1:8e   top  thigh
# device03: 3c:cd:40:18:c0:4f   shoe instep

DegreeToRadians = 0.0174532925
STATIC = -60
MOVING = 60

fileName = 'pohsin.png'

def read_pickle(index):
    # 1 2 3 7 10 12 20 26 27 28 29 32 36 38 45 48 49 50 53
    pathHead = 'walkingPoHsin/'
    fp1 = open(pathHead + str(index) + 'thigh.dat', "rb")
    fp2 = open(pathHead + str(index) + 'shank.dat', "rb")
    # fp3 = open(pathHead + str(index) + 'reWriteShoes.dat', "rb")
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
    
    print "data1 length:",len(objs1)
    print "data2 length:",len(objs2)
    print "data3 length:",len(objs3)
    print "data4 length:",len(objs4)
    print "data5 length:",len(objs5)
    print "data6 length:",len(objs6)

    timestamp = list()
    timestampChip = []
    yaw1 = list()
    pitch1 = list()
    roll1 = list()
    gx= list()
    gy= list()
    gz= list()
    mx=list()
    my=list()
    mz=list()
    ax=list()
    ay=list()
    az=list()
    gyro1 = []
    #label=list()

    timestamp2 = list()
    timestampChip2 = []
    yaw2 = list()
    pitch2 = list()
    roll2 = list()
    gx2= list()
    gy2= list()
    gz2= list()
    mx2=list()
    my2=list()
    mz2=list()
    ax2=list()
    ay2=list()
    az2=list()
    gyro2 = []
    #label2=list()

    timestamp3 = list()
    timestampChip3 = []
    yaw3 = list()
    pitch3 = list()
    roll3 = list()
    gx3= list()
    gy3= list()
    gz3= list()
    mx3=list()
    my3=list()
    mz3=list()
    ax3=list()
    ay3=list()
    az3=list()
    #label3=list()

    timestamp4 = list()
    timestampChip4 = []
    yaw4 = list()
    pitch4 = list()
    roll4 = list()
    gx4= list()
    gy4= list()
    gz4= list()
    mx4=list()
    my4=list()
    mz4=list()
    ax4=list()
    ay4=list()
    az4=list()
    gyro4 = []

    timestamp5 = list()
    timestampChip5 = []
    yaw5 = list()
    pitch5 = list()
    roll5 = list()
    gx5= list()
    gy5= list()
    gz5= list()
    mx5=list()
    my5=list()
    mz5=list()
    ax5=list()
    ay5=list()
    az5=list()
    gyro5 = []

    timestamp6 = list()
    timestampChip6 = []
    yaw6 = list()
    pitch6 = list()
    roll6 = list()
    gx6= list()
    gy6= list()
    gz6= list()
    mx6=list()
    my6=list()
    mz6=list()
    ax6=list()
    ay6=list()
    az6=list()
    gyro6 = []

    distance = []
    state = []

    for obj in objs1:
        timestamp.append(obj['timestamp'])
        timestampChip.append(obj['timestampChip'])
        yaw1.append(obj['Angle'].item(0,0))
        pitch1.append(obj['Angle'].item(0,1))
        roll1.append(obj['Angle'].item(0,2))
        gyro1.append(obj['Gyo'].item(0,2))
        gy.append(obj['Gyo'].item(0, 1))
        gx.append(obj['Gyo'].item(0, 0))

    for obj in objs2:
        timestamp2.append(obj['timestamp'])
        timestampChip2.append(obj['timestampChip'])
        yaw2.append(obj['Angle'].item(0,0))
        pitch2.append(obj['Angle'].item(0,1))
        roll2.append(obj['Angle'].item(0,2))
        gyro2.append(obj['Gyo'].item(0,2))
        gy2.append(obj['Gyo'].item(0, 1))
        gx2.append(obj['Gyo'].item(0, 0))

    for obj in objs3:
        timestamp3.append(obj['timestamp'])
        distance.append(obj['distance'])
        state.append(obj['state'])

    for obj in objs4:
        timestamp4.append(obj['timestamp'])
        timestampChip4.append(obj['timestampChip'])
        yaw4.append(obj['Angle'].item(0,0))
        pitch4.append(obj['Angle'].item(0,1))
        roll4.append(obj['Angle'].item(0,2))
        gyro4.append(obj['Gyo'].item(0,2))
        gy4.append(obj['Gyo'].item(0, 1))
        gx4.append(obj['Gyo'].item(0, 0))
        ax4.append(obj['Acc'].item(0, 0))
        ay4.append(obj['Acc'].item(0, 1))
    
    for obj in objs5:
        timestamp5.append(obj['timestamp'])
        timestampChip5.append(obj['timestampChip'])
        yaw5.append(obj['Angle'].item(0,0))
        pitch5.append(obj['Angle'].item(0,1))
        roll5.append(obj['Angle'].item(0,2))
        gyro5.append(obj['Gyo'].item(0,2))
        gy5.append(obj['Gyo'].item(0, 1))
        gx5.append(obj['Gyo'].item(0, 0))
    
    for obj in objs6:
        timestamp6.append(obj['timestamp'])
        timestampChip6.append(obj['timestampChip'])
        yaw6.append(obj['Angle'].item(0,0))
        pitch6.append(obj['Angle'].item(0,1))
        roll6.append(obj['Angle'].item(0,2))
        gyro6.append(obj['Gyo'].item(0,2))
        gy6.append(obj['Gyo'].item(0, 1))
        gx6.append(obj['Gyo'].item(0, 0))

    sumT = 0
    for i in range(len(timestampChip)):
        sumT = sumT + timestampChip[i]
        timestampChip[i] = sumT
#     print(sumT/len(timestampChip))
    sumT = 0
    for i in range(len(timestampChip2)):
        sumT = sumT + timestampChip2[i]
        timestampChip2[i] = sumT
    sumT = 0
    for i in range(len(timestampChip4)):
        sumT = sumT + timestampChip4[i]
        timestampChip4[i] = sumT
    sumT = 0
    for i in range(len(timestampChip5)):
        sumT = sumT + timestampChip5[i]
        timestampChip5[i] = sumT
    sumT = 0
    for i in range(len(timestampChip6)):
        sumT = sumT + timestampChip6[i]
        timestampChip6[i] = sumT
    
    stateLow = [50 if s != 0 else -50 for s in state]
    # lastState = STATIC
    # insertNum = 0
    # for i, (t, s) in enumerate(zip(timestamp3, statLow)):
    #     if lastState == 40 and s != 40:
    #         statLow.insert(i+insertNum, -60)
    #         timestamp3.insert(i+insertNum, timestamp3[(i+insertNum)-1]+0.005)
    #         distance.insert(i+insertNum, distance[i+insertNum-1])
    #         insertNum = insertNum + 1
    #     lastState = s
    
    gyroList1 = integrationList(gyro1, timestampChip)
    gyroList2 = integrationList(gyro2, timestampChip2)
    gyroList4 = integrationList(gyro4, timestampChip4)
    gyroList5 = integrationList(gyro5, timestampChip5)
    gyroList6 = integrationList(gyro6, timestampChip6)
    
    for i in range(len(gyroList5)):
        gyroList5[i] = gyroList5[i] * -1
    for i in range(len(gyroList6)):
        gyroList6[i] = gyroList6[i] * -1
    
    timestamp = np.linspace(0, 8, len(timestamp))
    timestamp2 = np.linspace(0, 8, len(timestamp2))
    # timestamp3 = np.linspace(0, 8, len(timestamp3))
    timestamp4 = np.linspace(0, 8, len(timestamp4))
    timestamp5 = np.linspace(0, 8, len(timestamp5))
    timestamp6 = np.linspace(0, 8, len(timestamp6))

    plt.ion()  # enable interactivity
    fig = plt.figure(figsize=(12,6))
    f1 = fig.add_subplot(1, 1, 1)
    ax = f1.axes
#     ax.set_xticks(np.arange(0, 10, 0.2))
#     ax.set_yticks(np.arange(50, -50, 11))
    ax.grid(True)
    
    plt.plot(timestamp3, stateLow, 'y', label = 'state')
    
#     plt.plot(timestamp4, ax4, label = "ax")
#     plt.plot(timestamp4, ay4, label = "ay")
#     plt.plot(timestamp4, gyro4, label = "gz")
    plt.plot(timestamp[0:-1], gyroList1, label = 'left thigh gyro integration')
    plt.plot(timestamp2[0:-1], gyroList2, label = 'left shank gyro integration')
    plt.plot(timestamp4[0:-1], gyroList4, 'gray', label = 'left shoe gyro integration')
    plt.plot(timestamp5[0:-1], gyroList5, label = 'right shank gyro integration')
    plt.plot(timestamp6[0:-1], gyroList6, 'g', label = 'right thigh gyro integration')

    plt.plot(timestamp3, distance, 'y', label = 'distance')

    lgd3 = plt.legend(bbox_to_anchor=(1.05, 0.65), loc=2, borderaxespad=0.)

    plt.savefig(fileName, bbox_extra_artists=[lgd3], bbox_inches='tight')
#     plt.show()
#     while True:
#         plt.pause(0.0001)

    # ---------- rewrite data back to file -----------
#     dictList = []
#     for i, obj in enumerate(objs3):
#         dataDict=collections.OrderedDict()
#         dataDict['timestamp'] = obj['timestamp']
#         if abs(obj['distance'] - 63) < 1:
#             dataDict['distance'] = 54
#         else:
#             dataDict['distance'] = obj['distance']
#         dataDict['state'] = obj['state']
#         dictList.append(dataDict)
#     fileName = str(index) + 'shoes'
#     reWrite(dictList[:], fileName)

# define for classfication
STATIC = 0
MOVING = 400

def integrationList(data, tData):
    summ = 0
    result = []
    t = 8.0/len(data)
#     print(t)
    for i in range(len(data)-1):
#         t = tData[i+1]-tData[i]
        summ = summ + data[i] * t
        summ = summ + (data[i+1]-data[i])*t/2
        result.append(summ/DegreeToRadians)

    return result

def moveSum(data):
    summ = 0
    tempList = []
    resultList = []
    for i, d in enumerate(data):
        if i < 4:
            tempList.append(d)
        elif i == 4:
            tempList.append(d)
            for j in range(5):
                resultList.append(sum(tempList)/5.0)    
        else:
            tempList.pop(0)
            tempList.append(d)
            resultList.append(sum(tempList)/5.0)
            # print(tempList)

    return resultList


def reWrite(list_item, fileName):
    fp = open(fileName+'.dat', "wb")
    for item_ob in list_item:    #pickle dump
        pickle.dump(item_ob, fp)
    fp.close()
    print("finish saving file: %s" % fileName)

if __name__ == '__main__':
#     index = np.linspace(1, 5, 5)
#     index = [int(a) for a in index]
#     for i in index:
#         read_pickle(i)
        
#     plt.legend(bbox_to_anchor=(1.1, 1), loc=1, borderaxespad=0.)
#     plt.savefig(fileName)
    read_pickle(sys.argv[1])

