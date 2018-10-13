# Gait parameters analysis based on leg-and-shoe-mounted EcoIMU and LSTM

This repository contains **369 strides of walking data**, **a step event detector**, and **two stride length estimators**.

#### Walking Data
The walking data is collected from six volunteers equipped with leg-and-shoe-mounted [EcoIMU](https://epl.tw/ecomini/). We attach five EcoIMUs to human body and collect motion data, including triaxial accelerations and triaxial angular rates, through BLE at a 200 samples per second data rate.

![Collecting the walking data with EcoIMU through BLE](https://drive.google.com/uc?id=1pmaJ2iuqjzIB8484h0qPcC60nP3A7AxC)

#### Step Event Detector
The step event detector detects HS (heel-strike) and TO (toe-off) events through a neural network constructed with LSTM cells. The input is 6-axis IMU data collected from one left-shoe-mounted EcoIMU.

We implement this detector in *StepEvent_LSTM.py*.

#### Stride Length Estimator
We propose two stride length estimators, which use *Mechanical Model* and *LSTM*, separately, to estimate stride lengths. 
* **Mechanical Model**: use the z-axis of gyroscope data to obtain the angles at joints through integration, and then we combine these angles with leg length and shoe length to calculate stride lengths. We implement this estimator in *StrideLength_MechanicalModel.py*.
* **LSTM**: use 6-axis IMU data collected from all of the five leg-and-shoe-mounted EcoIMUs as the inputs to a neural network constructed with LSTM cells. We implement this estimator in *StrideLength_LSTM.py*.


## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

### Prerequisites

* install [cuDNN](https://docs.nvidia.com/deeplearning/sdk/cudnn-install/index.html)
* install the following packages:
```javascript
pip install numpy scipy matplotlib keras
pip install --upgrade tensorflow-gpu==1.5.0
```

### Clone and Run

Clone this repository and run the corresponding programs. The result will be printed in the terminal, and for the programs that use LSTM, graphs that show the **loss** and **accuracy** during traing, validation, and testing can be found in **resultSpace** folder.

```
git clone https://github.com/PoHsin-Lin/gait-parameters-analysis-LSTM.git
python3 StepEvent_LSTM.py
python3 StrideLength_MechanicalModel.py
python3 StrideLength_LSTM.py
```

### More Information

For more details about the methods and the performance, please see the attached *thesis.pdf*.