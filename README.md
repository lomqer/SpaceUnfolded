# Space Unfolded
Space Unfolded is a single point-of-view projector calibration system.
It is an OpenCV and CGAL-based solution for correcting projected images to the shape of the projection area.
This repository compiles into a DLL file with exported **calibrate(const char\*, const int\*)** function, which will run the calibration on the images from provided data folder.

Its two arguments are:
* const char* **dataFolder** - Name of the folder with captured images
* const int* **contourPoints** - 4 sets of coordinates of the captured images area we want to display on, in form [x1, y1, x2, y2, x3, y3, x4, y4]

Example capture and calibration process can be found inside [the examples folder.](examples/SimpleCalibrationInPython)

Below we can see the calibration effects we can expect when using this solution on a room corner.

**Before calibration**
![Uncalibrated Projection](img/uncalibrated.jpg)


**After calibration**
![Calibrated Projection](img/calibrated.jpg)
