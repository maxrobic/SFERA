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
./step_two_sort {InputTextFilePath} {OutputTextFilePath} {InputImageFilePath} {SquareSide} {NumberOfLines} {NumberOfcolumns}
	
```
_Description of Parameters:_

* `{InputTextFilePath}` : Path to the input text file that contains the raw detection data (coordinates of points detected on the calibration pattern).
* `{OutputTextFilePath}` : Path to the output text file where the sorted detection data will be saved.
* `{InputImageFilePath}` : Path to the input image file, which corresponds to the original calibration image used in the detection phase.
* `{SquareSide}` : The physical distance (in mm) of a side of the square of the calibration pattern. 
* `{NumberOfLines}` : The number of horizontal lines (rows) on the calibration pattern, helping to organize the points correctly.
* `{NumberOfColumns}` : The number of vertical lines (columns) on the calibration pattern, used to sort the detected points in the correct order.

Note that for greater simplicity, we recommend using our interface `./Second_step.sh`, which automates the entire matching procedure for each cameras. 
During the process, you will be requested to press `Y` to confirm the order of the detected points displayed on the image, or `N` to redo the point matching. 
If you press `N`, you will be required to manually select 4 points that correspond to the corners of the calibration pattern in a specific order, as described in the prompt.

**Step 3 - Estimation of camera parameters:**

This last step takes advantage of OpenCV's *omnicalib* to obtain the calibration parameters of each camera. Please run step 1 (and optionnaly step 2 as mentionned before) so you have images (and object/image points) of the calibration pattern available. It is also possible to give step 3 with your own calibration pattern images (or points). Please follow the instructions below:

```
./step_three_omnicalibration -w={NumberOfColums} -h={NumberOfLines} -s={SquareSide} -fp={FixedPrincipal} -fs={FixedSkew} -id={idCam} -nPose={PoseNumber} {imagelist.xml}

```

_Description of Parameters:_

* `{NumberOfColumns}` : The number of vertical lines (columns) on the calibration pattern.
* `{NumberOfLines}` : The number of horizontal lines (rows) on the calibration pattern.
* `{SquareSize}` : The physical distance (in mm) of a side of a square of the calibration pattern. (default value = 1.0)
* `{FixedPrincipal}` : A boolean to fix the principal point to half the dimension of the camera. (default value = false)
* `{FixedSkew}` : A boolean to fix the skew parameter to 1. (default value = false)
* `{idCam}` : The id of the camera you wish to calibrate, 1 for Master, 2 for Slave.
* `{PoseNumber}` : Refers to the pose number or shot number in a series of images for calibration (if you have k images for each camera, put k)
* `{imagelist.xml}`: Optional string, a .xml file with the list of images paths in string format (default value = ""). If you do not specified this parameter, the algorithm targets directly the folder created by step 1 with the recorded images.

Example of what to run if you have recorded 7 poses with step 1 on each camera with arbitrary parameters:

```
./step_three_omnicalibration -w=6 -h=9 -s=34 -fs=true -id=1 -nPose=7

./step_three_omnicalibration -w=6 -h=9 -s=34 -fs=true -id=2 -nPose=7

```

The output of the calibration are then stored in param_camera1.xml & param_camera2.xml.

### Viewer & Tracking
This part refers to the Omnitracking folder. It allows you to run a spherical view of the sensor with accumulated map, and to run the tracker and see the tracker result in this spherical view.

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

**Output:**

Here you can see the output of the omnitracking algorithm (video taken from our submission to ICRA25):


https://github.com/user-attachments/assets/c6c31c25-1d72-4c7c-955a-0b03c0df0251



## Acknowledgment
The bayesian estimator is adapted from the work of Prof. Ivan Marković on omnidirectional RGB, whose source code can be found here: https://bitbucket.org/unizg-fer-lamor/odyn-tracker/src/master/
