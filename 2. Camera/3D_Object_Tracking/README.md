# Camera 2D Object Tracking: Camera & Lidar Fusion




### MP.1 Match 3D Objects
Implemented the method "matchBoundingBoxes". The alogorithm is to related bounding boxes in previous and current frame, by using the knowledge from feature matching.
The first step is to generate a 2D container, the col and row are the bounding box index from previous and current frame respectively. Then loop through all matched features (in `cv::DMatch`), check its `queryIdx` & `trainIdx` to get the corresponding bounding box's index which the keypoint belongs to, and increase the 2D container accordingly. After construct the 2D container, complete the `map<int, int>` but choosing the best matching which has the highest number of keypoint correspondences from each row.

### MP.2  Compute Lidar-based TTC
To compute the time-to-collision based on Lidar, this part would follow the Constant Velocity Model and calculate the TTC from:
```cpp
TTC = (d1 * dT) / (d0 - d1);
```
Apart from preparing the Lidar Point clouds, it is important to make it robust against outliers which could results in faulty estimation. To tackle the outliers problem, there are mant approaches on hands, such as PCA, DBSCAN... but in this project a statistic method has been leveraged. The algorithm is simple: 
1. calucate `mean` & standard derivative `std`.
2. choose a threshold as `mean - n * std` for outlier removal. (n = 1, 2, or 3, in this project n = 2)
<img src="images/outlier.png" width="1000" height="150" />
After outlier removal, the closest lidar point to the ego vehicle is picked for TTC calculation. 
