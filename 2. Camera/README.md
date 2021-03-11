# Camera

- Fuse camera images together with lidar point cloud data. 
- Extract object features from camera images in order to estimate object motion and orientation. 
- Classify objects from camera images in order to apply a motion model. 
- Project the camera image into three dimensions. 
- Fuse the projection into three dimensions to fuse with lidar data.

## Project: Track Object In 3D Space
Detect and track objects from the benchmark KITTI dataset. Classify those objects and project them into three dimensions. Fuse those projections together with lidar data to create 3D objects to track over time.

## Dependencies for Running Locally
* cmake >= 3.7
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
