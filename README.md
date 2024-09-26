# SFERA

This repository regroups several modules to calibrate dual back-to-back fisheye event cameras and operates event-based target tracking of fast moving objects. This repository will be updated with new branch at each new release of the framework.

The source code of the present version was also used to generate the results of our submitted ICRA25 paper "A new Stereo Fisheye Event Camera for Fast Drone Detection and Tracking". The bibtex of the article will be provided later.

For any information do not hesitate to contact us: maxime.robic@u-picardie.fr or daniel.rodrigues.da.costa@u-picardie.fr.

## Dependencies

These modules are meant to work with Metavision SDK, so first step is to install it: https://docs.prophesee.ai/4.6.2/index.html

The current version of the code has been validated on Ubuntu 22.04.4 (but the portage to Windows is straightforward) with the following version of core dependencies:

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

**Compilation:**

```
cd SFERA/Calibration/
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make
```

**Step 1 - Acquisition and Detection:**
The following script is used to record an image and the points detected on a calibration pattern (such as a checkerboard) for calibration purposes:

```
./step_one_primitive_detection_recording --output-dir {OutputFilePath} --output-name {OutputName} --rows {NumberOfLines} --cols {NumberOfcolumns} --number-pose {PoseNumber} -s {SerialPortMaster} 

```
_Description of Parameters:_

* `--output-dir {OutputFilePath}` : Specifies the directory where the acquired image and detection results will be saved.
* `--output-name {OutputName}` : Defines the name of the output files (both the image and detected points).
* `--rows {NumberOfLines}` : Indicates the number of horizontal lines (rows) on the calibration pattern (i.e., how many rows of points are visible on the pattern).
* `--cols {NumberOfColumns}` : Indicates the number of vertical lines (columns) on the calibration pattern.
* `--number-pose {PoseNumber}` : Refers to the pose number or shot number in a series of images for calibration (useful when multiple poses are required).
* `-s {SerialPortMaster}` : Specifies the serial port of the master camera or device used for image acquisition.

Note that for greater simplicity, we recommend using our interface `./First_step.sh`, which automates the entire acquisition procedure for both cameras.
You simply need to follow the instructions provided in the prompt. During the process, you will be prompted to press the `R` key to record the current camera pose. 
Once the pose is successfully recorded, press `Q` to exit the acquisition for the current pose, which will automatically execute the code for the next pose.

**Step 2 - Matching of coordinates (optional):**
The following script is used to match the pixel coordinates with the 3D world coordinates. It is optional since *omnicalib* has already a coordinates matching algorithm embedded so you can just provide the images acquired in step 1, as it is done currently. However, it is also possible to run step 2 and provide *omnicalib* with the results of step 2 (*i.e.* object & image points). We recommend to use this technique if you are not satisfied with the primitive detection of OpenCV. 

```
./step_two_sort {InputTextFilePath} {OutputTextFilePath} {InputImageFilePath} {DistanceBetween2PointsOfInterest} {NumberOfLines} {NumberOfcolumns}
	
```
_Description of Parameters:_

* `{InputTextFilePath}` : Path to the input text file that contains the raw detection data (coordinates of points detected on the calibration pattern).
* `{OutputTextFilePath}` : Path to the output text file where the sorted detection data will be saved.
* `{InputImageFilePath}` : Path to the input image file, which corresponds to the original calibration image used in the detection phase.
* `{DistanceBetween2PointsOfInterest}` : The physical distance (in real-world units) between two points of interest on the calibration pattern. 
* `{NumberOfLines}` : The number of horizontal lines (rows) on the calibration pattern, helping to organize the points correctly.
* `{NumberOfColumns}` : The number of vertical lines (columns) on the calibration pattern, used to sort the detected points in the correct order.

Note that for greater simplicity, we recommend using our interface `./Second_step.sh`, which automates the entire matching procedure for each cameras. 
During the process, you will be requested to press `Y` to confirm the order of the detected points displayed on the image, or `N` to redo the point matching. 
If you press `N`, you will be required to manually select 4 points that correspond to the corners of the calibration pattern in a specific order, as described in the prompt.

**Step 3 - Estimation of camera parameters:**


### Viewer & Tracking
This part refers to the Omnitracking folder.

**Compilation:**

```
cd SFERA/Omnitracking/
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make
```

**Viewer:**

```
./SFERA_viewer {SerialPortMaster} {SerialPortSlave}

```
**Tracking:**
```
./SFERA_omnitracking {SerialPortMaster} {SerialPortSlave}
```
Note that the codes can also be used with prerecorded .raw data. In this case, one can change {SerialPortMaster} {SerialPortSlave} by the file address of the data, *e.g.* {/master.raw} {/slave.raw} and set the value of the bool *live_camera* to *false*.

## Acknowledgment
The bayesian estimator is adapted from the work of Prof. Ivan Marković on omnidirectional RGB, whose source code can be found here: https://bitbucket.org/unizg-fer-lamor/odyn-tracker/src/master/
