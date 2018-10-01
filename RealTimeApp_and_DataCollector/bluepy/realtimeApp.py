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
import demoDrawer

os.environ["CUDA_DEVICE_ORDER"] = "PCI_BUS_ID"
os.environ["CUDA_VISIBLE_DEVICES"] = ""
from keras.models import load_model, Sequential
from keras.layers import LSTM, CuDNNLSTM, Dense

imuNodeNum = 5
ifaceOffset = 1
dongleNum = 3

acc_divider = 4095.999718
gyro_divider = 65.500002
DEG2RAD = 0.01745329251
DegreeToRadians = 0.01745329251

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

dataList = [[], [], [], [], []]
maxList = [7.94564, 3.21446, 4.58520]
minList = [-9.04163, -1.19848, -1.37290]
# maxList = [7.94564, 3, 3]
# minList = [-9.04163, -1.19848, -1.37290]
angleList = [0, 0, 0, 0, 0]

L1 = 39.5
L2 = 45
L3 = 20
l3long = 28

minP = 1.03
minQ = 1.17
minR = 0.7

collectGyroZ = False

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
    scanner = btle.Scanner(ifaceOffset)
    iface = 0
    while True :
        if len(deviceList) >= imuNodeNum:
            print("Devices all connected")
            return
            # time.sleep(5)
            # continue

        print("Still scanning...")
        devcies = scanner.scan(timeout = 3)
        
        # Michael's shoes nodes: "3c:cd:40:18:c1:7d" or dev.addr == "3c:cd:40:0b:c0:16": 

        # thigh: 3c:cd:40:18:c1:8e
        # shank: 3c:cd:40:18:c2:07
        # shoeData: 3c:cd:40:18:c0:4f
        # rightShank: 3c:cd:40:18:c3:46
        # rightThigh: 3c:cd:40:18:c2:0b

        for dev in devcies:
            if dev.addr in ["3c:cd:40:18:c2:0b", "3c:cd:40:18:c3:46", "3c:cd:40:18:c0:4f", "3c:cd:40:18:c1:8e", "3c:cd:40:18:c2:07"]: 
                if dev.addr == "3c:cd:40:18:c2:0b" and len(deviceList) != 4:
                    continue
                elif dev.addr == "3c:cd:40:18:c3:46" and len(deviceList) != 3:
                    continue
                elif dev.addr == "3c:cd:40:18:c0:4f" and len(deviceList) != 2:
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
                    mainLock.acquire()
                    deviceList.append(connNode)
                    mainLock.release()
                except:
                    print("get service, characteristic or set notification failed")
                    break


def detectInputKey():
    global save_data_flag, tStart, angleList, xposition, yposition, lastEvent
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
                    angleList = [0, 0, 0, 0, 0]
                    save_data_flag="ready"
                    tStart = time.time()
                    while len(xposition) != 0:
                        xposition.pop(0)
                        yposition.pop(0)
                    xposition.append(0)
                    yposition.append(0)
                    lastEvent = 'HS'
                    time.sleep(30)
                    print "now end collect data-----------------------------"
                    print "now end collect data-----------------------------"
                    print "now end collect data-----------------------------"
                    for device in deviceList:
                        print(device.count)
                        device.count = 0
                    save_data_flag="finish"


def imuThread(node, i):
    global save_data_flag, fp, deviceList, runningThreadNum, savedFileNum, lock, dataList, collectGyroZ, angleList
    print("imuThread start: %d" % i)

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
    counter = 0
    gyroList = []
    accXList = []
    accYList = []
    angle = 0
    lastTimestamp = time.time()
    lastGyro = 0
    gyroInte = []

    while True:
        # try:
        if node.Peripheral.waitForNotifications(0.05):
            rawdata= [node.Peripheral.gyrodata[0:4],node.Peripheral.gyrodata[4:8],node.Peripheral.gyrodata[8:12],node.Peripheral.gyrodata[12:16],node.Peripheral.gyrodata[16:20],node.Peripheral.gyrodata[20:24],node.Peripheral.gyrodata[24:28],node.Peripheral.gyrodata[28:32]]
            shortdata = Uint4Toshort(rawdata)
            calibratedData = dataCalibration(shortdata, i)
            
            accRSS = calibratedData[0]**2 + calibratedData[1]**2 + calibratedData[2]**2
            gyroRSS = calibratedData[3]**2 + calibratedData[4]**2 + calibratedData[5]**2

            if gyroRSS < sensorStopMax[1]:
                for k in range(5):
                    node.madgwick.MadgwickAHRSupdateIMU(0,0,0,calibratedData[0],-calibratedData[1],-calibratedData[2],calibratedData[6]/5.0)
                node.nodeCube.angle = node.madgwick.quatern2euler()
                # print("stop")
            else:
                for k in range(5):
                    node.madgwick.MadgwickAHRSupdateIMU(calibratedData[3],-calibratedData[4],-calibratedData[5],calibratedData[0],-calibratedData[1],-calibratedData[2],calibratedData[6]/5.0)
                node.nodeCube.angle = node.madgwick.quatern2euler()

            if save_data_flag == "notReady":
                notReadyFlag = True
                savedFlag = False
            if save_data_flag == "ready":

                if i == 3 or i == 4:
                    calibratedData[5] = calibratedData[5] * -1
                
                # step events
                gyroZ = (calibratedData[5] - minList[0]) / (maxList[0] - minList[0])
                accX = (calibratedData[0] - minList[1]) / (maxList[1] - minList[1])
                accY = (calibratedData[1] - minList[2]) / (maxList[2] - minList[2])
                gyroList.append(gyroZ)
                accXList.append(accX)
                accYList.append(accY)
                if len(gyroList) >= 5:
                    gyroAvg = sum(gyroList) / float(len(gyroList))
                    accXAvg = sum(accXList) / float(len(accXList))
                    accYAvg = sum(accYList) / float(len(accYList))
                    gyroList.pop(0)
                    accXList.pop(0)
                    accYList.pop(0)
                    dataList[i].append([gyroAvg, accXAvg, accYAvg])

                if i == 3 or i == 4:
                    angleList[i] = node.nodeCube.angle[1] * -1
                elif i == 2:
                    angleList[i] = node.nodeCube.angle[1]-90
                else:
                    angleList[i] = node.nodeCube.angle[1]

                node.count = node.count + 1
                # stride lengths
                # gyroInte.append(calibratedData[5])
                # if len(gyroInte) >= 5:
                #     gyroInteAvg = sum(gyroInte) / float(len(gyroInte))
                #     gyroInte.pop(0)
                #     # currentTimestamp = time.time()
                #     # period = currentTimestamp - lastTimestamp
                #     period = 0.008
                #     angleList[i] = angleList[i] + calibratedData[5] * period
                #     angleList[i] = angleList[i] + abs(calibratedData[5] - lastGyro) * period / 2.0
                #     # lastTimestamp = currentTimestamp
                #     lastGyro = gyroInteAvg
                #     # print(period)

                # if i == 0:
                #     print(angleList)

                # time.sleep(0.2)
                # if notReadyFlag == True:
                #     notReadyFlag = False
                #     node.count = 0
                # dataDict=collections.OrderedDict()
                # dataDict['Acc'] = mat([calibratedData[0],calibratedData[1],calibratedData[2]])
                # dataDict['Gyo'] = mat([calibratedData[3],calibratedData[4],calibratedData[5]])
                # dataDict['timestamp'] = time.time() - tStart
                # dataDict['timestampChip'] = calibratedData[6]
                # node.dataDicts.append(dataDict)
                # node.count = node.count + 1
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

        # if save_data_flag == "finish" and savedFlag == False:
        #     savedFlag = True
        #     if i == 0:
        #         fileName = str(file_num) + 'thigh'
        #     elif i == 1:
        #         fileName = str(file_num) + 'shank'
        #     elif i == 2:
        #         fileName = str(file_num) + 'shoesData'
        #     elif i == 3:
        #         fileName = str(file_num) + 'rightShank'
        #     elif i == 4:
        #         fileName = str(file_num) + 'rightThigh'
        #     elif i == 5:
        #         fileName = str(file_num) + 'rightShoe'
        #     else:
        #         print("fileIndex error")
        #     print("start saving file: %s" % fileName)
        #     t =threading.Thread(target = write_to_txt, args=( [node.dataDicts[:], fileName]))  #[data_chunk] make  data_chunk as a arguement
        #     t.start()
            
        #     node.dataDicts = []
        #     file_num = file_num + 1
        #     lock.acquire()
        #     savedFileNum = savedFileNum + 1
        #     if savedFileNum == (imuNodeNum+1):
        #         save_data_flag = "notReady"
        #         savedFileNum = 0
        #     lock.release()

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

def lengthEstimate(HsAngleList, ToAngleList):
    global sum01, sum02, max01, max02, min01, min02, leng

    if ToAngleList[0] < 0:
        gamma = ((180-abs(ToAngleList[1])) + abs(ToAngleList[0])) * DegreeToRadians
    else:
        gamma = ((180-abs(ToAngleList[1])) - abs(ToAngleList[0])) * DegreeToRadians
    d1 = math.sqrt(l1**2 + l2**2 - 2*l1*l2*math.cos(gamma))
    phi1 = math.asin(math.sin(gamma)*l1/d1)
    beta = (abs(ToAngleList[2]) + (90 - abs(ToAngleList[1]))) * DegreeToRadians
    d2 = math.sqrt(d1**2 + l3**2 - 2*d1*l3*math.cos(beta+phi1))
    alpha1 = math.asin(math.sin(gamma)*l2/d1)
    alpha2 = math.asin(math.sin(beta+phi1)*l3/d2)
    alpha = ToAngleList[0] * DegreeToRadians
    if alpha < 0:
        if alpha2 > alpha1:
            alpha3 = abs(alpha) - (alpha2 - alpha1)
        else:
            alpha3 = (alpha1-alpha2) + abs(alpha)
    else:
        alpha3 = (alpha1-alpha2) - abs(alpha)
    theta4 = 90*DegreeToRadians - alpha3
    D1 = d2 * math.cos(theta4)
    
    # correct unusuall scheme
    if ToAngleList[3] >= ToAngleList[4]:
        gamma3 = 180 * DegreeToRadians
        d4 = l1 + l2
        phi3 = 0
    else:
        if ToAngleList[3] < 0:
            gamma3 = (180 - abs(ToAngleList[4]) - abs(ToAngleList[3])) * DegreeToRadians
        else:
            gamma3 = (180 - abs(ToAngleList[4]) + abs(ToAngleList[3])) * DegreeToRadians
        d4 = math.sqrt(l1**2 + l2**2 - 2*l1*l2*math.cos(gamma3))
        phi3 = math.asin(math.sin(gamma3)*l1/d4)
    
    if HsAngleList[3] > HsAngleList[4]:
        gamma2 = 180 * DegreeToRadians
        d3 = l1 + l2
        phi2 = 0
    else:
        gamma2 = (180 - abs(HsAngleList[3]) + abs(HsAngleList[4])) * DegreeToRadians
        d3 = math.sqrt(l1**2 + l2**2 - 2*l1*l2*math.cos(gamma2))
        phi2 = math.asin(math.sin(gamma2)*l1/d3)
    
    if ToAngleList[3] < 0:
        beta2 = (abs(HsAngleList[3]) - abs(ToAngleList[3])) * DegreeToRadians
    else:
        beta2 = (abs(HsAngleList[3]) + abs(ToAngleList[3])) * DegreeToRadians
    theta = phi3 + (beta2 - phi2)
    D2 = math.sqrt(d3**2 + d4**2 - 2*d3*d4*math.cos(theta))
#         print("gamma2", gamma2 / DegreeToRadians)
#         print("d3", d3)
#         print("gamma3", gamma3 / DegreeToRadians)
#         print("d4", d4)
#         print("beta2", beta2 / DegreeToRadians)
#         print("phi2", phi2 / DegreeToRadians)
#         print("phi3", phi3 / DegreeToRadians)
#         print("theta", theta / DegreeToRadians)
#         print("D2: ", D2)
    if HsAngleList[1] > HsAngleList[0]:
#             print("HsAngle[1] > HsAngle[0]", abs(HsAngleList[1] - HsAngleList[0]))
        gamma4 = 180 * DegreeToRadians
        d5 = l1 + l2
        phi4 = 0
    else:
        gamma4 = (180 - abs(HsAngleList[0]) + abs(HsAngleList[1])) * DegreeToRadians
        d5 = math.sqrt(l1**2 + l2**2 - 2*l1*l2*math.cos(gamma4))
        phi4 = math.asin(math.sin(gamma4)*l2/d5)
    theta2 = abs(HsAngleList[0]) * DegreeToRadians - phi4
    theta3 = 90*DegreeToRadians - theta2
    D3 = d5 * math.cos(theta3)
#         print("gamma4", gamma4 / DegreeToRadians)
#         print("d5", d5)
#         print("phi4", phi4 / DegreeToRadians)
#         print("theta2", theta2 / DegreeToRadians)
#         print("theta3", theta3 / DegreeToRadians)
    # print("D1: ", D1)
    # print("D2: ", D2)
    # print("D3: ", D3)
    SL = D1 + D2 + D3 + l3long - (l3long-l3)
    return SL

def stepEventThread():
    global dataList, angleList, lastEvent
    print("stepEventThread start")
    
    model = load_model('model_event_speed_featureLen3.h5')
    heelTimer = 0
    toeTimer = 0
    toAngleList = None
    hsAngleList = None
    lastEvent = 'HS'
    while True:
        if len(dataList[2]) > 0:
            data = np.array(dataList[2].pop(0))
            # print(data)
            test_X = data.reshape((1, 1, 3))
            predictions = model.predict_on_batch(test_X)
            # print(predictions)
            if predictions[0][0][1] > 0.9:
                if lastEvent != 'TO':
                    continue
                if time.time()-heelTimer > 0.5:
                    heelTimer = time.time()
                    print("heel-strike")
                    # print("predictions[0][0][1]: %f" % predictions[0][0][1])
                    lastEvent = 'HS'
                    hsAngleList = angleList[:]
                    # hsAngleList = [a/DegreeToRadians for a in hsAngleList]
                    if toAngleList != None and hsAngleList != None:
                        estimatedLength = lengthEstimate(hsAngleList, toAngleList)
                        print(estimatedLength)
                        xposition.append(xposition[-1]+estimatedLength)
                        yposition.append(0)
            if predictions[0][0][2] > 0.5:
                if lastEvent != 'HS':
                    continue
                if time.time()-toeTimer > 0.5:
                    toeTimer = time.time()
                    print("toe-off")
                    # print("predictions[0][0][2]: %f" % predictions[0][0][2])
                    lastEvent = 'TO'
                    toAngleList = angleList[:]
                    # toAngleList = [a/DegreeToRadians for a in toAngleList]
        # hsError, toError, detectLossHs, detectLossTo = computeErrorTime(testY[i], predictions[0], debug = True)
    # print("finish")

if __name__ == '__main__':
    global mydraw, runningThreadNum, l1, l2, l3, xposition, yposition

    scanTh =  threading.Thread(target = scanThread)
    scanTh.start()

    inputTh =  threading.Thread(target = detectInputKey)
    inputTh.start()

    stepEventTh =  threading.Thread(target = stepEventThread)
    stepEventTh.start()

    runningThreadNum = 0

    l1 = L1*minP
    l2 = L2*minQ
    l3 = L3*minR

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
        if len(deviceList) >= imuNodeNum:
            break
        time.sleep(1)

    xposition = [0]
    yposition = [0]
    mydemoDrawer = demoDrawer.demoDrawer(xposition, yposition)
    