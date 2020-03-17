![YDLIDAR](image/YDLidar.jpg  "YDLIDAR")

# YDLidar C++ SDK

YDLidar SDK provides the implementation of control commands and Laser scan data transmission via serial interface, as well as the C/C++ API. The basic structure of YDLidar SDK is shown as below:

![YDLidar SDK Architecture](image/sdk_architecture.png)

Please refer to the [YDLidar SDK Communication Protocol](include/ydlidar_protocol.h) sourcecode for further information. LaserScan supports Laser Scan Data transmission, while Command handler receives and sends control commands. And the C++ API is based on Command and LaserScan Hander.

## Installation
### Dependencies
YDLidar SDK requires [CMake 2.8.2+](https://cmake.org/). You can install it using apt:
```
sudo apt install cmake pkg-config
```
### Compile YDLidar SDK Example Project
In the directory where you want the project to be located, run the following commands to compile the project:
```
git clone https://github.com/MoritzTockner/PiDLidar
cd PiDLidar
cmake -D BUILD_CPP_EXAMPLE=ON .
make
```

## Run YDLidar SDK Example
The following configuation is used for the X4 Lidar Sensor and can be found in the [Specifications](#Specifications) Section. The lidar scan frequency (the frequency with which the sensor should rotate) that can be entered here has no effect. At least for the X4 Model. The real scan frequency is per default a little bit below 8 Hz and can only be modified with an external PWM signal (see [Datasheet](1) - page 5 & 6). 

```
./ydlidar_test
```

```
__   ______  _     ___ ____    _    ____
\ \ / /  _ \| |   |_ _|  _ \  / \  |  _ \ 
 \ V /| | | | |    | || | | |/ _ \ | |_) | 
  | | | |_| | |___ | || |_| / ___ \|  _ <
  |_| |____/|_____|___|____/_/   \_\_| \_\ 

Baudrate:
0. 115200
1. 128000
2. 153600
3. 230400
4. 512000
Please select the lidar baudrate:1
Whether the Lidar is one-way communication[yes/no]:no
Whether the Lidar is a TOF Lidar [yes/no]:no
Please enter the lidar scan frequency[5-12]:8
YDLidar SDK initializing
YDLidar SDK has been initialized
[YDLIDAR]:SDK Version: 1.4.6
LiDAR successfully connected
[YDLIDAR]:Lidar running correctly ! The health status: good
[YDLIDAR] Connection established in [/dev/ttyUSB0][128000]:
Firmware version: 1.5
Hardware version: 1
Model: X4
Serial: 2019050500001341
LiDAR init success!
[YDLIDAR INFO] Current Sampling Rate : 5K
[YDLIDAR INFO] Now YDLIDAR is scanning ......
Scan received[1584432655212153000]: 605 ranges is [8.278145]Hz
Scan received[1584432655333865000]: 602 ranges is [8.319468]Hz
Scan received[1584432655454265000]: 600 ranges is [8.347245]Hz
Scan received[1584432655574265000]: 599 ranges is [8.361204]Hz
Scan received[1584432655694065000]: 600 ranges is [8.347245]Hz
Scan received[1584432655814065000]: 599 ranges is [8.361204]Hz
Scan received[1584432655933865000]: 600 ranges is [8.347245]Hz
Scan received[1584432656053865000]: 600 ranges is [8.347245]Hz
Scan received[1584432656173865000]: 599 ranges is [8.361204]Hz
Scan received[1584432656293665000]: 600 ranges is [8.347245]Hz
Scan received[1584432656413665000]: 598 ranges is [8.375209]Hz
Scan received[1584432656533265000]: 600 ranges is [8.347245]Hz
Scan received[1584432656653265000]: 599 ranges is [8.361204]Hz
Scan received[1584432656773065000]: 599 ranges is [8.361204]Hz
...
```

## The API
The most important API classes and structures for controlling the sensor are:

`LaserPoint`: Data structure that stores the range, distance and intensity (not used for the X4 Model) of a measured point.

`LaserConfig`: Internally used by the `LaserScan` class.

`LaserScan`: Contains a `LaserConfig`, multiple scanned `LaserPoints` and a corresponding timestamp `stamp`

`CYdLidar`: the main Sensor class with following member variables 
* `MaxRange`: maximum sample range. Everything that exceeds this range is set to 0
* `MinRange`: minimum sample range. Everything that is below this range is set to 0
* `MaxAngle`: maximum scan angle in degree (max 180)
* `MinAngle`: minimum scan angle in degree (min -180)
* `SampleRate`: sample rate in kHz
* `ScanFrequency`: rotations of the sensor per second. Default is 10 but without external PWM its around 7-8 Hz for the X4 Model. This is set to 8 in the example project because it's used for calculating the sleep time of the worker thread
* `FixedResolution`: ???
* `Reversion`: determines where 0° e.g. the front of the sensor is. False --> the side where the motor is located on the sensor, True --> the opposite side.
* `Inverted`: determines the direction of positive angle. True --> counter clockwise, False --> clockwise.
* `AutoReconnect`: ???
* `SerialBaudrate`: baudrate of the serial communication to the sensor
* `AbnormalCheckCount`: ???
* `SerialPort`: string with the serial port where the sensor is connected
* `IgnoreArray`: ???
* `OffsetTime`: ???
* `SingleChannel`: ???
* `LidarType`: 1 --> Triangulation, 0 --> Time of Flight

which can be accessed via setters and getters. The member variables:
* `SoftVersion`
* `HardwareVersion`
* `SerialNumber`
* `isAngleOffsetCorrected`
* `AngleOffset`

are read-only and can only be accessed with their corresponding getters.
The sensor can be controlled with the following methods:
* `initialize`: Initializes the LIDAR with the configured parameters. A few lines of device information are printed and the sensor starts spinning and stops again after a very short time
* `doProcessSimple`: returns true when the samples of a full rotation are ready and stores their information in a `LaserScan` (around 600 `LaserPoints`)
* `turnOn`: starts the sensor rotation and sampling. It has to be initialized first
* `turnOff`: stops the sensor rotation and sampling
* `disconnecting`: the sensor can't be started anymore after disconnecting. First it has to be initialized again

The corresponding sourcecode can be found in [include/ydlidar_protocol.h](include/ydlidar_protocol.h) (`LaserPoint`, `LaserConfig` and `LaserScan`) and [include/CYdLidar.h](include/CYdLidar.h) (`CYdLidar`). A python wrapper exists for all these classes and their methods (see the following section).

# YDLidar Python SDK
## Installation
### Dependencies
YDLidar SDK requires [CMake 2.8.2+](https://cmake.org/). You can install it using apt:
```
sudo apt install cmake pkg-config
```
The GUI example project depends on:
* Python 3.x
* PyQt5
* numpy (used version 1.12.1)
* pyqtgraph (used version 0.10.0)

At the time of writing this, when installing pyqtgraph with pip3, a newer numpy version (1.18.0) is also installed but an error occured when i tried to use pyqtgraph with the new numpy version (or just numpy alone). I already had numpy 1.12.1 installed, so my solution to the problem was to install pyqtgraph with its depencies (numpy 1.18.0) and then uninstall the new numpy version, so that the 1.12.1 is used.
```
pip3 install numpy==1.12.1
pip3 install pyqtgraph
pip3 uninstall numpy
```

### Create the PiDLidar Python Package
In the PiDLidar directory, run the following commands to install the python package:
```
cd PiDLidar
pip3 install .
```

### Run the PiDLidar Example 
Navigate to the python folder and execute the [gui.py](python/gui.py) script.
```
cd PiDLidar/python
python gui.py
```

After some status messages, the GUI will appear with the following control elements:
* **Start Sensor Button**: calls `CYdLidar.turnOn()` to start sampling and displaying the samples in the graph
* **Stop Sensor Button**: calls `CYdLidar.turnOff()` to stop the sensor
* **Exit Application Button**: calls `CYdLidar.turnOff()` followed by `CYdLidar.disconnecting()` to safely close the connection and exits the application
* **Slider**: enables manually adjustment of the graph scaling. Ranges from 1 (most left) to 10 metres (most right)
* **Enable auto-scaling Ticker**: scaling is automatically adjusted to the most distant sample currently available when enabled. The slider can't be changed manually when auto-scaling is activated

![GUI](image/GUI.png  "GUI")

## The API
See the C++ SDK API.


# Sensor Specifications 
The following specifications are gathered from the [Datasheet](1).

| LIDAR | Model | Baudrate | SampleRate / kHz | Range / m | ScanFrequency / Hz | Intenstiy / Bits | SingleChannel | Voltage / V |
| :---- | :---: | :------: | :--------------: | :-------: | :----------------: | :--------------: | :-----------: | :---------: |
| X4    |   6   |  128000  |        5         | 0.12 - 10 |       5 - 12       |      false       |     false     |  4.8 - 5.2  |

   Note: ScanFrequency (the rotational frequency of the Sensor) can only be changed via an external PWM signal (see [Datasheet](1) - page 5, 6).
   
# Useful Links

* [Pybind11 Documentation](https://pybind11.readthedocs.io/en/stable/)
* [YDLidar X4 Resources](http://www.ydlidar.com/service_support/download.html?gid=5)
   * [Datasheet](1)
   * [User Manual](http://www.ydlidar.com/Public/upload/files/2019-12-18/YDLIDAR%20X4%20User%20Manual.pdf)
   * [Development Manual](http://www.ydlidar.com/Public/upload/files/2019-12-18/YDLIDAR%20X4%20Development%20Manual.pdf)
   * [Windows Tool](http://www.ydlidar.com/dowfile.html?cid=2&type=5)
   * [SDK Github](https://github.com/YDLIDAR/sdk)
* [PyQtGraph Documentation](http://pyqtgraph.org/documentation/)
* [PyQt5 API Documentation](https://doc.qt.io/qtforpython/index.html#)

Datasheet, User Manual and Development Manual can also be found in the [doc](doc) folder.

# Licence

The SDK itself is licensed under BSD [license](license)

[1]:http://www.ydlidar.com/Public/upload/files/2019-12-18/YDLIDAR%20X4%20Datasheet.pdf

