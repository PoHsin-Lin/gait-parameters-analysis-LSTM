# gait-parameters-analysis-LSTM

This repository contains a set of walking data, a step event detector, and two stride length estimators.

The walking data is collected from six volunteers equipped with leg-and-shoe-mounted [EcoIMU](https://epl.tw/ecomini/). We attach five EcoIMUs to human body and collect motion data, including triaxial accelerations and triaxial angular rates, through BLE.

![Collecting the walking data with EcoIMU through BLE](https://drive.google.com/uc?id=1pmaJ2iuqjzIB8484h0qPcC60nP3A7AxC)


## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Prerequisites

* install [cuDNN](https://docs.nvidia.com/deeplearning/sdk/cudnn-install/index.html)
* install the following packages:
```javascript
pip install numpy scipy matplotlib keras
pip install --upgrade tensorflow-gpu==1.5.0
```

### Installing

A step by step series of examples that tell you how to get a development env running

Say what the step will be

```
Give the example
```

And repeat

```
until finished
```

End with an example of getting some data out of the system or using it for a little demo
