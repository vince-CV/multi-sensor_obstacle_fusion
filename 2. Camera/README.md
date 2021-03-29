# Camera

- Fuse camera images together with lidar point cloud data. 
- Extract object features from camera images in order to estimate object motion and orientation. 
- Classify objects from camera images in order to apply a motion model. 
- Project the camera image into three dimensions. 
- Fuse the projection into three dimensions to fuse with lidar data.


## Collision Detection
Detect other cars on the road using raw lidar data from real self-driving car, Carla. Implement custom RANSAC and euclidean clustering algorithms.

With calculating the distance
- Cloud Point with CVM (constant velocity model)

Without calculate the distance: it is possible to measure the time to collision by observing relative height change on the image sensor.
- Deep Neural Network with Bounding Box
- Keypoint detection (distinctive texture patterns)


## Combining 2D Camera and 3D Lidar
1. Combine the tracked feature points within the camera images with the 3D Lidar points. Geometrically project the Lidar points into the camera.
2. Homogeneous coordinates (齐次坐标系/投影坐标系) to map Lidar point cloud data into 2D image plane. (Camera & Lidar calibration using Intrinsic & Extrinsic matrix)
<img src="images/transfer.png" width="600" height="150" />
3. Using DNN model to detect vehicles in an image to properly cluster and combine 2D feature tracks and 3D lidar points.
4. 2D feature tracks + 3D Lidar points = 3D vehicle tracks.




### Project 1: Tracking 2D Image Featrues
Objective: Identify and track reliable and stable features through a sequence of images.

* First, load images, setting up data structures and putting everything into a ring buffer to optimize memory load. 
* Then, integrate several keypoint detectors such as HARRIS, FAST, BRISK and SIFT and compare them with regard to number of keypoints and speed. 
* Next, descriptor extraction and matching using brute force and also the FLANN approach. 
* Last, test the various algorithms in different combinations and compare them with regard to some performance measures. 

<img src="2D_Feature_Matching/images/O_B.png" width="1000" height="150" />

### Project 2: Track Object In 3D Space
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
