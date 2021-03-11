# Lidar

- Process raw lidar data with filtering, segmentation, and clustering to detect other vehicles on the road. 
- Implementation Ransac with planar model fitting to segment point clouds. 
- Implementation Euclidean clustering with a KD-Tree to cluster and distinguish vehicles and obstacles.


## Project: Lidar Obstacle Detection
Detect other cars on the road using raw lidar data from real self-driving car, Carla. Implement custom RANSAC and euclidean clustering algorithms.

### Point Cloud Segmentation
Planar model to segment point cloud data using RANSAC.

Ransac, Random Sample Consensus, and is a method for detecting outliers in data.
- each iteration randomly picks subset of points
- fit a model to the points
- iteration with the most inliers to the model is best

### Point Cloud Clustering
Perform Euclidean clustering, and build KD-Tree to do efficient nearest neighbor search for clustering.

Euclidean Clustering, based on nearest neighbor search. KD-Tree KD-Tree speeds up look up time from O(n) to O(log(n)) (tree allows you to better break up your search space.) By grouping points into regions in a KD-Tree, it can avoid calculating distance for possibly thousands of points just because they are not considered in a close enough region.






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
