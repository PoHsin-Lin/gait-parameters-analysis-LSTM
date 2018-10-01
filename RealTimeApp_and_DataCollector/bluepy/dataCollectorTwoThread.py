import binascii
import os
import sys
import btle
import struct
import threading
import time
import json,httplib
from numpy import *
import numpy as np
import collections
import pickle
from numpy import sqrt
import sys
import select
import myOpenGL
import Madgwick
import Mahony
import scipy.interpolate

imuNodeNum = 6
ifaceOffset = 1
dongleNum = 4

acc_divider = 4095.999718
gyro_divider = 65.500002
DEG2RAD = 0.01745329251

lock = threading.Lock()
mainLock = threading.Lock()

save_data_flag="notReady"
savedFileNum = 0

deviceList = []
ultrasonicNode = None
ultrasonicNodeRunning = False

# define for classfication
STATIC = 0
MOVING = 400
toeoffEnable = False

# class -----------------------------------------------------------------------------------

class myNode:
    def __init__(self):
        self.Peripheral = None
        self.accBias = [0.0,0.0,0.0]
        self.gyroBias = [0.0,0.0,0.0]
        self.magBias = [0.0,0.0,0.0]
        self.magScale = [0.0,0.0,0.0]
        self.magCalibration = [0.0,0.0,0.0]
        self.dataDicts=[]
        # for sensor data drawing
        self.xList = np.empty((2000,))
        self.yList = np.empty((2000,))
        self.zList = np.empty((2000,))
        self.index = 0
        # for madgwick filter
        self.madgwick = Madgwick.MadgwickClass()
        self.mahony = Mahony.MahonyClass()
        # for cube
        self.nodeCube = myOpenGL.myCube(x_axis = 90)
        self.alreadyDraw = False
        self.lastTime = -1
        # for count packet number
        self.count = 0
        # for window draw
        self.windowNumber = None
        # for packet loss handle
        # self.lossTime = -1
        # self.lastTenData = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #                     [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #                     [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #                     [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #                     [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #                     [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #                     [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #                     [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #                     [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        #                     [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]

# functions -------------------------------------------------------------------------------

def scanThread():
    global ultrasonicNode, ultrasonicNodeRunning
    scanner = btle.Scanner(ifaceOffset)
    iface = 0
    while True :
        if len(deviceList) >= imuNodeNum and ultrasonicNode != None:
            print("Devices all connected")
            return
            # time.sleep(5)
            # continue

        print("Still scanning...")
        devcies = scanner.scan(timeout = 3)
        
        # Michael's shoes nodes: "3c:cd:40:18:c1:7d" or dev.addr == "3c:cd:40:0b:c0:16": 

        # thigh: 3c:cd:40:18:c1:8e
        # shank: 3c:cd:40:18:c2:07
        # shoeUltra : 3c:cd:40:0b:c0:f8
        # shoeData: 3c:cd:40:18:c0:4f
        # rightShank: 3c:cd:40:18:c3:46
        # rightThigh: 3c:cd:40:18:c2:0b
        # rightShoe:  3c:cd:40:18:c1:04

        for dev in devcies:
            if dev.addr in ["3c:cd:40:18:c2:0b", "3c:cd:40:18:c3:46", "3c:cd:40:18:c0:4f", "3c:cd:40:18:c1:8e", "3c:cd:40:18:c2:07", "3c:cd:40:0b:c0:f8", "3c:cd:40:18:c1:04"]: 
                if dev.addr == "3c:cd:40:18:c1:04" and len(deviceList) != 5:
                    continue
                elif dev.addr == "3c:cd:40:18:c2:0b" and len(deviceList) != 4:
                    continue
                elif dev.addr == "3c:cd:40:18:c3:46" and len(deviceList) != 3:
                    continue
                elif dev.addr == "3c:cd:40:18:c0:4f" and ultrasonicNode == None:
                    continue
                elif dev.addr == "3c:cd:40:0b:c0:f8" and len(deviceList) != 2:
                   continue
                elif dev.addr == "3c:cd:40:18:c2:07" and len(deviceList) != 1:
                    continue
                print("devcies %s (%s) , RSSI = %d dB" % (dev.addr , dev.addrType , dev.rssi))
                #Try to create connection
                try:
                    iface = (iface+1)%dongleNum
                    print("iface = %d" % (iface + ifaceOffset))
                    connNode = myNode()
                    connNode.Peripheral = btle.Peripheral(dev.addr , dev.addrType , iface + ifaceOffset)
                    connNode.Peripheral.setDelegate(btle.DefaultDelegate())
                    accBias = binascii.b2a_hex(connNode.Peripheral.readCharacteristic(0x40))
                    gyroBias = binascii.b2a_hex(connNode.Peripheral.readCharacteristic(0x43))
                    magBias = binascii.b2a_hex(connNode.Peripheral.readCharacteristic(0x46))
                    magScale = binascii.b2a_hex(connNode.Peripheral.readCharacteristic(0x49))
                    magCalibration = binascii.b2a_hex(connNode.Peripheral.readCharacteristic(0x4C))
                    calibrationData = [accBias[0:8], accBias[8:16], accBias[16:24]]
                    connNode.accBias = Uint8Tofloat(calibrationData)
                    calibrationData = [gyroBias[0:8], gyroBias[8:16], gyroBias[16:24]]
                    connNode.gyroBias = Uint8Tofloat(calibrationData)
                    calibrationData = [magBias[0:8], magBias[8:16], magBias[16:24]]
                    connNode.magBias = Uint8Tofloat(calibrationData)
                    calibrationData = [magScale[0:8], magScale[8:16], magScale[16:24]]
                    connNode.magScale = Uint8Tofloat(calibrationData)
                    calibrationData = [magCalibration[0:8], magCalibration[8:16], magCalibration[16:24]]
                    connNode.magCalibration = Uint8Tofloat(calibrationData)
                    
                    print("accBias: ",connNode.accBias)
                    print("gyroBias: ",connNode.gyroBias)
                    print("magBias: ",connNode.magBias)
                    print("magScale: ",connNode.magScale)
                    print("magCalibration: ",connNode.magCalibration)
                    print("connect successfully")

                except:
                    iface = (iface-1)%dongleNum
                    print("failed connection")
                    break

                #Try to get Service , Characteristic and set notification
                try:
                    # need to add 0000fed0-0000-1000-8000-00805f9b34fb
                    service = connNode.Peripheral.getServiceByUUID("0000FED0-0000-1000-8000-00805f9b34fb")
                    char = service.getCharacteristics("0000FED7-0000-1000-8000-00805f9b34fb")[0] 
                    connNode.Peripheral.writeCharacteristic(char.handle + 2,struct.pack('<bb', 0x01, 0x00),True)
                    if dev.addr == "3c:cd:40:0b:c0:f8":
                        ultrasonicNode = connNode
                        ultrasonicNodeRunning = False
                    else:
                        mainLock.acquire()
                        deviceList.append(connNode)
                        mainLock.release()
                except:
                    print("get service, characteristic or set notification failed")
                    break


def write_to_txt(list_item, fileName):
    fp = open(fileName+'.dat', "wb")
    for item_ob in list_item:    #pickle dump
        pickle.dump(item_ob, fp)
    fp.close()
    print("finish saving file: %s" % fileName)

def detectInputKey():
    global save_data_flag, ultrasonicNode, tStart
    while True:
        if save_data_flag == "notReady":
            print "waiting for 'S' to start collect data"
        i,o,e =select.select([sys.stdin], [], [], 60)
        for s in i:
            if s == sys.stdin:
                input = sys.stdin.readline()
                if input=="S\n":
                    print "start to collect data after 3 seconds"
                    time.sleep(3)
                    print "-----------------------------now start collect data"
                    print "-----------------------------now start collect data"
                    print "-----------------------------now start collect data"
                    save_data_flag="ready"
                    tStart = time.time()
                    time.sleep(8)
                    print "now end collect data-----------------------------"
                    print "now end collect data-----------------------------"
                    print "now end collect data-----------------------------"
                    for i, node in enumerate(deviceList):
                        print("count: ", i, node.count)
                    if ultrasonicNode != None:
                        print("count for ultrasonicNode: ", ultrasonicNode.count)
                    save_data_flag="finish"
                elif input=="U\n":
                    triggerUltrasonic()

def triggerUltrasonic():
    global ultrasonicNode
    # readLock.acquire()
    disSum = 0.0
    preValue = -1
    varience = 0
    validCount = 0.0
    # turn off notification before trigger ultrasonic
    service = ultrasonicNode.Peripheral.getServiceByUUID("0000FED0-0000-1000-8000-00805f9b34fb")
    char = service.getCharacteristics("0000FED7-0000-1000-8000-00805f9b34fb")[0] 
    ultrasonicNode.Peripheral.writeCharacteristic(char.handle + 2,struct.pack('<bb', 0x00, 0x00),True)
    disList = []
    for k in range(4):
        try:
            service = ultrasonicNode.Peripheral.getServiceByUUID("0000FED0-0000-1000-8000-00805f9b34fb")
            char = service.getCharacteristics("0000FED6-0000-1000-8000-00805f9b34fb")[0] 
            ultrasonicNode.Peripheral.writeCharacteristic(char.handle + 1 , struct.pack('<b', 0x01) , False)

            time.sleep(0.050)

            char = service.getCharacteristics("0000FED5-0000-1000-8000-00805f9b34fb")[0] 
            dis = ultrasonicNode.Peripheral.readCharacteristic(char.handle+1)

            floatVal = struct.unpack('f',dis)[0]
            print("floatVal = ", floatVal)
            disList.append(floatVal)
            if floatVal > 400:
                continue
            else:
                
                validCount = validCount + 1
                disSum = disSum + floatVal
                if preValue != -1:
                    varience = varience + abs(floatVal - preValue)
                preValue = floatVal
                time.sleep(0.060)

        except:
            print("exception for input U")
            if ultrasonicNode != None:
                ultrasonicNode.Peripheral.disconnect()
                ultrasonicNode = None
            # readLock.release()
    # readLock.release()
    if abs(disList[0] - disList[1]) > 20:
        disList = disList[1:4]

    if validCount != 0:
        print("average distance = %f" % (sum(disList)/float(len(disList))))
        print("varience = %f" % varience)

    # turn on notification
    service = ultrasonicNode.Peripheral.getServiceByUUID("0000FED0-0000-1000-8000-00805f9b34fb")
    char = service.getCharacteristics("0000FED7-0000-1000-8000-00805f9b34fb")[0] 
    ultrasonicNode.Peripheral.writeCharacteristic(char.handle + 2,struct.pack('<bb', 0x01, 0x00),True)

    if validCount != 0:
        return (sum(disList)/float(len(disList)))
    else:
        return 0

# def doInterpolation(lastTenData, packets):
#     print(packets)
#     results = []
#     for i in range(6):
#         x = [float(a) for a in range(9)]
#         x.append(9.0+packets)
#         y = [a[i] for a in lastTenData]
#         y_interp = scipy.interpolate.interp1d(x, y)
#         results.append([])
#         for j in range(packets):
#             results[-1].append(y_interp(9.0+j))
        
#     returnList = []
#     for i in range(packets):
#         returnList.append([x[i] for x in results])
#     return returnList

# def getUltraSonicDistance(node):

#     service = node.Peripheral.getServiceByUUID("0000FED0-0000-1000-8000-00805f9b34fb")
#     char = service.getCharacteristics("0000FED5-0000-1000-8000-00805f9b34fb")[0] 
#     dis = node.Peripheral.readCharacteristic(char.handle+1)
#     floatVal = struct.unpack('f',dis)[0]

#     return floatVal

def imuThread(node, i):
    global save_data_flag, fp, deviceList, mydraw, runningThreadNum, savedFileNum, lock, toeoffEnable
    print("imuThread start")

    # for Gyro Average
    global sensorStopMax
    sensorStopMax = [1.252, 0.055]
    # 1.2519, 0.0544

    # periodSum = 0.0
    # packets = 0.0
    # packetsPerSecond = 280
    lastSerial = None
    file_num = 1
    savedFlag = False

    while True:
        # try:
        if node.Peripheral.waitForNotifications(0.05):
            rawdata= [node.Peripheral.gyrodata[0:4],node.Peripheral.gyrodata[4:8],node.Peripheral.gyrodata[8:12],node.Peripheral.gyrodata[12:16],node.Peripheral.gyrodata[16:20],node.Peripheral.gyrodata[20:24],node.Peripheral.gyrodata[24:28],node.Peripheral.gyrodata[28:32]]
            shortdata = Uint4Toshort(rawdata)
            calibratedData = dataCalibration(shortdata, i)
            # if node.lastTime == -1:
            #     calibratedData[9] = 0.003448
            # else:
            #     calibratedData[9] = time.time() - node.lastTime
            # node.lastTime = time.time()

            # ---------- Serial number check ---------- 
            # if lastSerial != None:
            #     if lastSerial == 32759:
            #         if shortdata[6] != 0 and shortdata[6] != 32759:
            #             # pass
            #             print("imuThread: %d, serialNum loss: %d" % (i ,32759+shortdata[6]-lastSerial))
            #     else:
            #         if shortdata[6]-lastSerial > 1:
            #             # pass
            #             print("imuThread: %d, serialNum loss: %d" % (i, shortdata[6]-lastSerial))
            # lastSerial = shortdata[6]
            
            accRSS = calibratedData[0]**2 + calibratedData[1]**2 + calibratedData[2]**2
            gyroRSS = calibratedData[3]**2 + calibratedData[4]**2 + calibratedData[5]**2

            # if i == 2:
            #     #print(calibratedData[0], calibratedData[1], calibratedData[2
            #     if calibratedData[0] < 0.95:
            #         toeoffEnable = True
            #     else:
            #         toeoffEnable = False
            toeoffEnable = True
            # compute average period in 1s (from the passed 3 seconds)
            # periodSum = periodSum + calibratedData[9]
            # packets = packets + 1.0
            # if periodSum >= 3.0:
            #     packetsPerSecond = packets / 3.0
            #     periodSum = 0.0
            #     packets = 0.0

            # save last ten packets
            # for j in range(9):
            #     node.lastTenData[j] = node.lastTenData[j+1]
            # node.lastTenData[9] = calibratedData[0:6]

            # check whether data loss happened
            # if node.lossTime != -1:
            #     lossDuration = time.time() - node.lossTime + 0.05
            #     lossPackets = int(round(lossDuration * packetsPerSecond))
            #     interData = doInterpolation(node.lastTenData, lossPackets)
            #     packetsPeriod = 1.0/packetsPerSecond
            #     # if gyroRSS < sensorStopMax[1]:
            #     #     for data in interData:
            #     #         node.madgwick.MadgwickAHRSupdateIMU(0,0,0,data[3],data[4],data[5],packetsPeriod)
            #     #         node.nodeCube.angle = node.madgwick.quatern2euler()
            #     #     node.madgwick.MadgwickAHRSupdateIMU(0,0,0,calibratedData[0],-calibratedData[1],-calibratedData[2],calibratedData[9])
            #     #     node.nodeCube.angle = node.madgwick.quatern2euler()
            #     # else:
            #     for data in interData:
            #         node.madgwick.MadgwickAHRSupdateIMU(data[0],data[1],data[2],data[3],data[4],data[5],packetsPeriod)
            #         node.nodeCube.angle = node.madgwick.quatern2euler()
            #     node.madgwick.MadgwickAHRSupdateIMU(calibratedData[3],-calibratedData[4],-calibratedData[5],calibratedData[0],-calibratedData[1],-calibratedData[2],packetsPeriod)
            #     node.nodeCube.angle = node.madgwick.quatern2euler()
            #     node.lossTime = -1
            # else:

            if gyroRSS < sensorStopMax[1]:
                for k in range(5):
                    node.madgwick.MadgwickAHRSupdateIMU(0,0,0,calibratedData[0],-calibratedData[1],-calibratedData[2],calibratedData[6]/5.0)
                node.nodeCube.angle = node.madgwick.quatern2euler()
                # print("stop")
            else:
                for k in range(5):

                    node.madgwick.MadgwickAHRSupdateIMU(calibratedData[3],-calibratedData[4],-calibratedData[5],calibratedData[0],-calibratedData[1],-calibratedData[2],calibratedData[6]/5.0)
                node.nodeCube.angle = node.madgwick.quatern2euler()
                # print("moveing")
            # if i == 1:
            #     print(node.nodeCube.angle)

            # node.mahony.MahonyIMU6Dof(calibratedData[3],-calibratedData[4],-calibratedData[5],calibratedData[0],-calibratedData[1],-calibratedData[2],calibratedData[9])
            # node.mahony.MahonyAHRSupdate(calibratedData[3],-calibratedData[4],-calibratedData[5],calibratedData[0],-calibratedData[1],-calibratedData[2], calibratedData[7], -calibratedData[6], calibratedData[8],calibratedData[9])
            # node.nodeCube.angle = node.mahony.quatern2euler()

            # for drawing the data
            # node.xList[node.index] = calibratedData[0]
            # node.yList[node.index] = calibratedData[1]
            # node.zList[node.index] = calibratedData[2]
            # node.index = node.index + 1
            # if node.index >= 1997:
            #     node.index = 0

            #save data per hundred packet received
            if save_data_flag == "notReady":
                notReadyFlag = True
                savedFlag = False
            if save_data_flag == "ready":
                if notReadyFlag == True:
                    notReadyFlag = False
                    node.count = 0
                dataDict=collections.OrderedDict()
                dataDict['Acc'] = mat([calibratedData[0],calibratedData[1],calibratedData[2]])
                dataDict['Gyo'] = mat([calibratedData[3],calibratedData[4],calibratedData[5]])
                # dataDict['Mag'] = mat([calibratedData[6],calibratedData[7],calibratedData[8]])
                dataDict['Angle'] = mat(node.nodeCube.angle)
                dataDict['timestamp'] = time.time() - tStart
                dataDict['timestampChip'] = calibratedData[6]
                # if len(node.dataDicts) != 0 and i == 1:
                #     print(dataDict['timestamp'] - node.dataDicts[-1]['timestamp'], calibratedData[6])
                node.dataDicts.append(dataDict)
                node.count = node.count + 1
        else:
            # print("imuThread %d waitForNotification fail in 0.05s" % i)
            # if node.lossTime == -1:
            #     node.lossTime = time.time()
            continue

        # except:
        #     print("waitForNotification exception")
        #     # mydraw.removeDevice(node)
        #     deviceList.remove(node)
        #     node.Peripheral.disconnect()
        #     runningThreadNum = runningThreadNum - 1
        #     return

        if save_data_flag == "finish" and savedFlag == False:
            savedFlag = True
            if i == 0:
                fileName = str(file_num) + 'thigh'
            elif i == 1:
                fileName = str(file_num) + 'shank'
            elif i == 2:
                fileName = str(file_num) + 'shoesData'
            elif i == 3:
                fileName = str(file_num) + 'rightShank'
            elif i == 4:
                fileName = str(file_num) + 'rightThigh'
            elif i == 5:
                fileName = str(file_num) + 'rightShoe'
            else:
                print("fileIndex error")
            print("start saving file: %s" % fileName)
            t =threading.Thread(target = write_to_txt, args=( [node.dataDicts[:], fileName]))  #[data_chunk] make  data_chunk as a arguement
            t.start()
            
            node.dataDicts = []
            file_num = file_num + 1
            lock.acquire()
            savedFileNum = savedFileNum + 1
            if savedFileNum == (imuNodeNum+1):
                save_data_flag = "notReady"
                savedFileNum = 0
            lock.release()


def ultrasonicThread(node):
    global save_data_flag, deviceList, savedFileNum, lock
    print("ultrasonicThread start")

    currentState = STATIC
    currentDistance = None
    triggerUltrasonicFlag = False
    file_num = 1
    savedFlag = False
    tS = None
    while True:
        # try:
        # readLock.acquire()
        if node.Peripheral.waitForNotifications(0.05):
            rawdata= [node.Peripheral.gyrodata[0:4]]
            shortdata = Uint4Toshort(rawdata)

            # for FSR interrupt handle
            frontInterrupt = bool(shortdata[0] & 0x8000)
            backInterrupt = bool(shortdata[0] & 0x4000)

            if frontInterrupt == False and backInterrupt == False:
                pass
            elif frontInterrupt == True:# and backInterrupt == False:
                # print("signal: static -> moving")
                if currentState != MOVING and toeoffEnable:
                    currentState = MOVING
                    print("state change: ground -> flying")
            elif frontInterrupt == False and backInterrupt == True:
                # print("signal: moving -> static")
                if currentState != STATIC:
                    currentState = STATIC
                    print("state change: flying -> ground, triggerUltrasonic")
                    triggerUltrasonicFlag = True
            else:
                print("WARN: no data in static phase")

            # for drawing the data
            # node.xList[node.index] = calibratedData[0]
            # node.yList[node.index] = calibratedData[1]
            # node.zList[node.index] = calibratedData[2]
            # node.index = node.index + 1
            # if node.index >= 1997:
            #     node.index = 0

            #save data per hundred packet received
            if save_data_flag == "notReady":
                notReadyFlag = True
                savedFlag = False
            if save_data_flag == "ready":
                if notReadyFlag == True:
                    notReadyFlag = False
                    node.count = 0
                dataDict=collections.OrderedDict()
                dataDict['state'] = currentState
                dataDict['distance'] = currentDistance
                dataDict['timestamp'] = time.time() - tStart
                # if len(node.dataDicts) != 0:
                #     print(dataDict['timestamp'] - node.dataDicts[-1]['timestamp'])
                node.dataDicts.append(dataDict)
                node.count = node.count + 1
            if triggerUltrasonicFlag:
                currentDistance = triggerUltrasonic()
                triggerUltrasonicFlag = False
        else:
            # print("ultrasonic waitForNotification fail in 0.05s")
            # readLock.release()
            # if node.lossTime == -1:
            #     node.lossTime = time.time()
            continue

        # except:
        #     print("waitForNotification exception")
        #     node.Peripheral.disconnect()
        #     ultrasonicNode = None
        #     # readLock.acquire(False)
        #     # readLock.release()
        #     return

        if save_data_flag == "finish" and savedFlag == False:
            savedFlag = True
            fileName = str(file_num) + 'shoes'
            print("start saving file: %s" % fileName)
            t =threading.Thread(target = write_to_txt, args=( [node.dataDicts[:], fileName]))  #[data_chunk] make  data_chunk as a arguement
            t.start()
            
            node.dataDicts = []
            file_num = file_num + 1
            lock.acquire()
            savedFileNum = savedFileNum + 1
            if savedFileNum == (imuNodeNum+1):
                save_data_flag = "notReady"
                savedFileNum = 0
            lock.release()


def dataCalibration(rawdata, i):

    # mag = [None]*3
    # mag[0] = rawdata[6]*0.15*deviceList[i].magCalibration[0] - deviceList[i].magBias[0]
    # mag[1] = rawdata[7]*0.15*deviceList[i].magCalibration[1] - deviceList[i].magBias[1]
    # mag[2] = rawdata[8]*0.15*deviceList[i].magCalibration[2] - deviceList[i].magBias[2]

    # mag[0] *= deviceList[i].magScale[0]
    # mag[1] *= deviceList[i].magScale[1]
    # mag[2] *= deviceList[i].magScale[2]
    
    acc = [None]*3
    acc[0] = rawdata[0]/acc_divider - deviceList[i].accBias[0]
    acc[1] = rawdata[1]/acc_divider - deviceList[i].accBias[1]
    acc[2] = rawdata[2]/acc_divider - deviceList[i].accBias[2]

    gyro = [None]*3
    gyro[0] = (rawdata[3]/gyro_divider - deviceList[i].gyroBias[0])*DEG2RAD
    gyro[1] = (rawdata[4]/gyro_divider - deviceList[i].gyroBias[1])*DEG2RAD
    gyro[2] = (rawdata[5]/gyro_divider - deviceList[i].gyroBias[2])*DEG2RAD

    # secondData = rawdata[7] & (~0xC000)
    secondData = rawdata[7] / 1000000.0

    return [acc[0],acc[1],acc[2],gyro[0],gyro[1],gyro[2],secondData]

def Uint4Toshort(tenData):
    #print(threeData)
    retVal =[]
    
    for i, data in enumerate(tenData):
    #(data)
        i = 0
        byteArray = []
        while(i != 4):
            byteArray.append(int(data[i:i+2], 16))
        #print(int(data, 16))
            i=i+2

        b = ''.join(chr(i) for i in byteArray)
        if i == 9:
            retVal.append(struct.unpack('<H',b)[0])
        else:
            retVal.append(struct.unpack('<h',b)[0])
    return retVal

def Uint8Tofloat(threeData):
    #print(threeData)
    retVal =[]
    
    for data in threeData:
    #(data)
        i = 0
        byteArray = []
        while(i != 8):
            byteArray.append(int(data[i:i+2], 16))
        #print(int(data, 16))
            i=i+2

        b = ''.join(chr(i) for i in byteArray)
        retVal.append(struct.unpack('<f',b)[0])
    return retVal


def drawFig():
    global line1, line2, line3, line4, line5, line6
    X = np.linspace(0,2000, 2000)
    Y = np.linspace(-100, 100, 2000)

    plt.ion()
    # graph = plt.plot(X,Y)[0]
    # plt.hold(True)

    fig = plt.figure(figsize=(15,10))
    ax1 = fig.add_subplot(3, 2, 1)
    ax2 = fig.add_subplot(3, 2, 3)
    ax3 = fig.add_subplot(3, 2, 5)
    ax4 = fig.add_subplot(3, 2, 2)
    ax5 = fig.add_subplot(3, 2, 4)
    ax6 = fig.add_subplot(3, 2, 6)
    
    ax1.set_ylabel('accX')
    ax1.set_xlim(0, 2000)
    ax1.set_ylim(-5, 5)
    line1 = Line2D(X, Y, color = 'blue')
    ax1.add_line(line1)

    ax2.set_ylabel('accY')
    ax2.set_xlim(0, 2000)
    ax2.set_ylim(-5, 5)
    line2 = Line2D(X, Y, color = 'red')
    ax2.add_line(line2)

    ax3.set_ylabel('accZ')
    ax3.set_xlim(0, 2000)
    ax3.set_ylim(-5, 5)
    line3 = Line2D(X, Y, color = 'green')
    ax3.add_line(line3)

    ax4.set_ylabel('accX')
    ax4.set_xlim(0, 2000)
    ax4.set_ylim(-5, 5)
    line4 = Line2D(X, Y, color = 'blue')
    ax4.add_line(line4)

    ax5.set_ylabel('accY')
    ax5.set_xlim(0, 2000)
    ax5.set_ylim(-5, 5)
    line5 = Line2D(X, Y, color = 'red')
    ax5.add_line(line5)

    ax6.set_ylabel('accZ')
    ax6.set_xlim(0, 2000)
    ax6.set_ylim(-5, 5)
    line6 = Line2D(X, Y, color = 'green')
    ax6.add_line(line6)

    plt.hold(True)
    plt.show()

    
def updateFig(i):
    global line1, line2, line3, line4, line5, line6
    if i == 0:
        line1.set_ydata(list(deviceList[i].xList[deviceList[i].index:])+list(deviceList[i].xList[:deviceList[i].index]))
        line2.set_ydata(list(deviceList[i].yList[deviceList[i].index:])+list(deviceList[i].yList[:deviceList[i].index]))
        line3.set_ydata(list(deviceList[i].zList[deviceList[i].index:])+list(deviceList[i].zList[:deviceList[i].index]))
    elif i == 1:
        line4.set_ydata(list(deviceList[i].xList[deviceList[i].index:])+list(deviceList[i].xList[:deviceList[i].index]))
        line5.set_ydata(list(deviceList[i].yList[deviceList[i].index:])+list(deviceList[i].yList[:deviceList[i].index]))
        line6.set_ydata(list(deviceList[i].zList[deviceList[i].index:])+list(deviceList[i].zList[:deviceList[i].index]))

    plt.draw()
    plt.pause(0.0001)

if __name__ == '__main__':
    global ultrasonicNode, ultrasonicNodeRunning, mydraw, runningThreadNum

    scanTh =  threading.Thread(target = scanThread)
    scanTh.start()

    inputTh =  threading.Thread(target = detectInputKey)
    inputTh.start()

    # mydraw = myOpenGL.myDraw(deviceList)
    runningThreadNum = 0

    while True:
        mainLock.acquire()
        if runningThreadNum < len(deviceList):
            for i in range(len(deviceList) - runningThreadNum):
                newThread =  threading.Thread(target = imuThread, args=[deviceList[-(i+1)], runningThreadNum])
                newThread.start()
                runningThreadNum = runningThreadNum + 1
                # if len(deviceList) == 1:
                #    mydraw.addFirstDevice()
        mainLock.release()

        mainLock.acquire()
        if ultrasonicNode != None and ultrasonicNodeRunning == False:
            newThread =  threading.Thread(target = ultrasonicThread, args=[ultrasonicNode])
            newThread.start()
            ultrasonicNodeRunning = True
        mainLock.release()
        time.sleep(1)


    # # drawFig()

    # # while True:
    # #     updateFig(0)
    # #     updateFig(1)
