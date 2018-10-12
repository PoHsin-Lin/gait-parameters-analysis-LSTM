import numpy as np
import sys
import time
import pickle
import math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy import signal
from scipy.signal import argrelextrema
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

# the list of Name and the number of walking trials (each trial contains three strides)
subjectList = ['PoHsinnn', 'Winston', 'Jay', 'Ting', 'Thomas', 'Gene']
lengthList = [18, 13, 7, 7, 7, 19]

# featureLen denotes the number of features we used for the LSTM
featureLen = 15

# define of states
STATIC = 0
MOVING = 400

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
    
    # Apply low-pass filter
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
    
    # calculate the distance (ground truth)
    disList = sensorData3[1]
    lastDis = disList[0]
    strideLengthList = []
    for dis in disList:
        if dis < lastDis:
            x = (lastDis - dis)
            # Calibrate the reading of ultrasonic sensor
            correctionDis = -2.56564*math.pow(10, -8)*math.pow(x, 4) + 0.0000222831*math.pow(x, 3) - 0.00663392*math.pow(x, 2) + 1.85571*x - 27.5267
            strideLengthList.append(correctionDis)
            lastDis = dis

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
#                 print(t - lastT)
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
    
    # segment the walking data, which contains three strides, into strides
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
        
    # --------------- normalize the data length of five sensors ---------------
    # For each stride, the walking data comes from five sensors, and the number of packets sent from these sensors are different.
    # Thus, we normalize the length of the walking data using the minimal number of packets among theses five sensors.
    x_seqs = [x_seq1, x_seq2, x_seq4, x_seq5, x_seq6]
    # three strides
    for i in range(strideCount):
        minLen = 1000
        # gyroZ, accX, accY
        for k in range(3):
            # five sensors
            for j in range(5):
                if len(x_seqs[j][i][k]) < minLen:
                    minLen = len(x_seqs[j][i][k])
            for j in range(5):
                if len(x_seqs[j][i][k]) == minLen:
                    x_seqs[j][i][k] = np.array(x_seqs[j][i][k])
                else:
                    x_seqs[j][i][k] = signal.resample(x_seqs[j][i][k], minLen)
    
    # ------------ generate x sequences for LSTM -------------
    X_seq = []
    for i in range(strideCount):
        #                                              0 here denotes the first sensor
        #                                                 i here denotes the i-th stride
        #                                                    0 here denotes the first axis: gyroZ
        s1 = np.array([np.array(a) for a in zip(x_seqs[0][i][0], x_seqs[0][i][1], x_seqs[0][i][2])])
        s2 = np.array([np.array(a) for a in zip(x_seqs[1][i][0], x_seqs[1][i][1], x_seqs[1][i][2])])
        s4 = np.array([np.array(a) for a in zip(x_seqs[2][i][0], x_seqs[2][i][1], x_seqs[2][i][2])])
        s5 = np.array([np.array(a) for a in zip(x_seqs[3][i][0], x_seqs[3][i][1], x_seqs[3][i][2])])
        s6 = np.array([np.array(a) for a in zip(x_seqs[4][i][0], x_seqs[4][i][1], x_seqs[4][i][2])])
        # x_seq is a ndarray with shape (1, 15)
        x_seq = np.concatenate([s1, s2, s4, s5, s6], axis = -1)
        X_seq.append(x_seq)
    X_seq = np.array(X_seq)
    
    # ------------ generate y sequences for LSTM -------------
    Y_seq = np.array(strideLengthList[0:strideCount])
    
    return X_seq, Y_seq

def toFloat(myList):
    # convert the elements in myList into float32
    for i in range(len(myList)):
        myList[i] = [np.float32(x) for x in myList[i]]

def seperateData(X_seq, X_weights, Y_seq, i):
    # Seperate the walking data of the i-th volunteer for testing, the others for training and validation
    testNum = lengthList[i]*3
    
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

def normalizeDataLen(X_seq):
    # input - X_seq: A list with shape (sampleNum, length of a sample, featureLen), this function makes "length of a sample" the same for all samples
    # we normalize the length of the each sample using the minimal number of sample lengths among all samples.
    global minLength
    minLength = 1000
    for sample in X_seq:
        if len(sample) < minLength:
            minLength = len(sample)
    
    feature = [[] for i in range(featureLen)]
    for i in range(len(X_seq)):
        # print(np.array(X_seq[i]).shape)
        for j in range(featureLen):
            #            np.interp(target X-axis length        , original X-axis length                , original Y-axis data)
            feature[j] = np.interp(np.linspace(0, 1, minLength), np.linspace(0, 1, len(X_seq[i][:, j])), X_seq[i][:, j])
        # convert the shape of "feature" list from (minLength, featureLen) to (featureLen, minLength)
        featureInv = list(map(list, zip(*feature)))
        X_seq[i] = np.array(featureInv[:])
        # print(np.array(X_seq[i]).shape)

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
    global sampleNum
    
    # Control the proportion of GPU memory used in this process 
    config = tf.ConfigProto()
    config.gpu_options.per_process_gpu_memory_fraction = 0.3
    set_session(tf.Session(config=config))
    
    print("feature length: %d" % featureLen)
    print('Read data and preprocess ...')
    # -------------- read data and preprocess -------------- 
    X_seqs = []
    X_weights = []
    Y_seqs = []
    
    for j, name in enumerate(subjectList):
        dataLen = lengthList[j]
        pathHead = 'walkingDataSpace/walking' + name + '/'
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
                # if we have fewer data for this volunteer, we give these data higher weight
                X_weights.append(19.0/float(dataLen))
                Y_seqs.append(Y_seq[i])
    X_seqs = np.array(X_seqs)
    X_weights = np.array(X_weights)
    Y_seqs = np.array(Y_seqs)
    sampleNum = X_seqs.shape[0]
    
    # X_seqs.shape = (sampleNum, length of a sample, featureNum)
    # Y_seqs.shape = (sampleNum)
    
    # --------- normalize data length and value----------
    normalizeDataLen(X_seqs)
    normalizeData(X_seqs)
    
    # for statistic
    meanerrorList = []
    precisionList = []
    rmseList = []
    
    # k for cross validation
    for k in range(len(subjectList)):
        global testloss, testPrecision
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
        
        # convert the shape of trainX, testX, and valiX from (sampleNum, ) to (sampleNum, minSampleLength, featureLen)
        trainX = np.stack(trainX)
        testX = np.stack(testX)
        valiX = np.stack(valiX)
        
        # make the minimal weight be 1
        minW = min(trainW)
        trainW = np.array([x / minW for x in trainW])
        
        # self-defined metrics
        def precisionM(y_true, y_pred):
            return K.sqrt(K.mean(((y_pred-y_true) - K.mean(y_pred-y_true))**2))

        # build and compile the LSTM model
        model = Sequential()
        model.add(CuDNNLSTM(32, input_shape=(minLength, featureLen)))
        model.add(Dense(1))
        model.compile(loss='mse', optimizer='adam', metrics=[precisionM])
    
        # save the model with lowest validation loss
        checkpointer = ModelCheckpoint(filepath='modelSpace/modelsRawSensorUI/weights' + str(k) + '.hdf5', 
                                       monitor='val_loss', verbose=0, save_best_only=True)
        
        # self-defined call-back function
        class TestCallback(Callback):
            global testLoss, testPrecision
            def __init__(self, test_data):
                self.test_data = test_data

            def on_epoch_end(self, epoch, logs={}):
                x, y = self.test_data
                evalResult = self.model.evaluate(x, y, verbose=0)
                testLoss.append(evalResult[0])
                testPrecision.append(evalResult[1])
                # print('\nTesting loss: {}, acc: {}\n'.format(evalResult[0], evalResult[1]))
        
        # -------------------- train --------------------
        tStart = time.time()
        history = model.fit(trainX, trainY, batch_size=5, epochs=1200, verbose=0, 
                            sample_weight=trainW,
                            validation_data=(valiX, valiY, valiW), 
                            callbacks=[checkpointer, TestCallback((testX, testY))])
        
        tEnd = time.time()
        print("Train time: %f sec" % (tEnd-tStart))
        # ------------------ end training ---------------------
        
        # ----- draw training graph (loss and precision) -----
        # The first 200 losses are very large that makes the loss variation from 201 to 1200 unobservable.
        # Thus, we change them to 300.
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
        
        plt.subplot(212)
        plt.plot(history.history['precisionM'], label = 'train_precision')
        plt.plot(history.history['val_precisionM'], label = 'validation_precision')
        plt.plot(testPrecision, label = 'test_precision')
        plt.axvline(np.argmin(history.history['val_loss']), color='r')
        lgd = plt.legend(bbox_to_anchor=(1.05, 0.65), loc=2, borderaxespad=0.)
        plt.savefig('resultSpace/figRawSensorUI_neuron32_epoch1200_featureLen15_' + str(k) + '.png', bbox_extra_artists=[lgd], bbox_inches='tight')

        print("min val_loss: %f, epoch: %d" % 
              (min(history.history['val_loss']), (np.argmin(history.history['val_loss'])+1)))
        print("min val_precision: %f, epoch: %d" % 
              (min(history.history['val_precisionM']), (np.argmin(history.history['val_precisionM'])+1)))
        
        # ---------------------------- evaluate the best model ----------------------------
        model = load_model('modelSpace/modelsRawSensorUI/weights' + str(k) + '.hdf5', custom_objects = {'precisionM': precisionM})
        predictions = model.predict(testX)
        lossList = []
        print("k = %d" % k)
        for i in range(len(testY)):
            print("estimated: %f, ground truth: %f" % (predictions[i][0], testY[i]))
            errorDis = predictions[i][0] - testY[i]
            lossList.append(errorDis)
        
        meanError, precision, rmse = meanErrorPrecisionRMSE(lossList)

        print("\nmeanError: %f" % meanError)
        print("precision: %f" % precision)
        print("rmse: %f" % rmse)
        
        meanerrorList.append(meanError)
        precisionList.append(precision)
        rmseList.append(rmse)
    
    # Weighted Average
    print("\n ----- Weighted Average -----")
    meanErrorSum = 0
    for i, meanError in enumerate(meanerrorList):
        meanErrorSum = meanErrorSum + meanError * (lengthList[i]*3)
    precisionSum = 0
    for i, precision in enumerate(precisionList):
        precisionSum = precisionSum + precision * (lengthList[i]*3)
    rmseSum = 0
    for i, rmse in enumerate(rmseList):
        rmseSum = rmseSum + rmse * (lengthList[i]*3)

    print("Mean error: %f" % (meanErrorSum / float(sum(lengthList)*3)))
    print("Precision: %f" % (precisionSum / float(sum(lengthList)*3)))
    print("RMSE: %f" % (rmseSum / float(sum(lengthList)*3)))
    
    groundTruthMean = sum(Y_seqs) / float(len(Y_seqs))
    groundTruthLoss = [l - groundTruthMean for l in Y_seqs]
    meanError, precision, rmse = meanErrorPrecisionRMSE(groundTruthLoss)
    print("\n ----- The precision and RMSE of ground truth -----")
    print("Mean error: %f" % meanError)
    print("Precision: %f" % precision)
    print("RMSE: %f" % rmse)
    
    