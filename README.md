# SFERA

This repository regroups several modules to calibrate dual back-to-back fisheye event cameras and operates event-based target tracking of fast moving objects. This repository will be updated with new branch at each new release of the framework.

The source code of the present version was also used to generate the results of our submitted ICRA25 paper "A new Stereo Fisheye Event Camera for Fast Drone Detection and Tracking". The bibtex of the article will be provided later.

For any information do not hesitate to contact me: maxime.robic@u-picardie.fr.

## Dependencies

These modules are meant to work with Metavision SDK, so first step is to install it: https://docs.prophesee.ai/4.6.2/index.html

The current version of the code has been validated with the following version of core dependencies:

| Library       | Version       |
| ------------- | ------------- |
| MetavisionSDK | 4.6.2         |
| OpenCV        | 4.9.0         |
| PCL           | 1.14.1        |

MetavisionSDK is used for the connection with the cameras and for the event processing part. 
OpenCV is used for the calibration step, classic mathematical operations, and some displaying features. 
PointCloudLibrary (PCL) is used for the 3D sperical view. 

## Usage
Some tips to compile and run the different codes provided.

### Calibration
This part refers to the Calibration folder.

### Viewer & Tracking
This part refers to the Omnitracking folder.

Compilation:

```
cd SFERA/Omnitracking/
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make
```

Viewer:

```
./SFERA_viewer {SerialPortMaster} {SerialPortSlave}

```
Tracking:
```
./SFERA_omnitracking {SerialPortMaster} {SerialPortSlave}
```
Note that the codes can also be used with prerecorded .raw data. In this case, one can change {SerialPortMaster} {SerialPortSlave} by the file address of the data, *e.g.* {/master.raw} {/slave.raw}.

## Acknowledgment
The bayesian estimator is adapted from the work of Prof. Ivan Marković on omnidirectional RGB, whose source code can be found here: https://bitbucket.org/unizg-fer-lamor/odyn-tracker/src/master/
