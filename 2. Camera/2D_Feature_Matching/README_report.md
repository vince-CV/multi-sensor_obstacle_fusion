# Camera 2D Feature Matching Writeup

This project focus on the feature matching techniques on image data, which consists of feature/keypiont detector, keypoint descriptor and descriptor matching.

To fulfill the requirements of this project, we could discuss from three aspects:
1. Keypoint detection: `SHITOMASI`, `HARRIS`, `FAST`, `BRISK`, `AKAZE`, `SIFT`.
2. Keypoint description: `BRISK`, `ORB`, `FREAK`, `AKAZE`, `SIFT`.
3. Descriptor matching: `Brute Force`, `FLANN`.

### MP.1 Data Buffer Optimization
Firstly make sure the 1. image buffer size, 2. FIFO machanism. Therefore, in the loop using `vector::erase()` to remove the beginning element in the vector containter, and `vector::push_back` to add a new image into the buffer.

### MP.2 Keypoint Detection
Implemented `HARRIS`, `FAST`, `BRISK`, `ORB`, `AKAZE`, and `SIFT` in _matching2D.cpp_ using opencv API. `detect()` method is called for keypoint detection.

### MP.3 Keypoint Removal
Generally there are two demands of keypoint removal: 1. using bounding box to only focus on the pixle region of the preceding vehicle; 2. only retain the keypoints with better qualities so that release the burden of computing.
Option 1 is implemented by using `std::vector::remove_if` with a predefined `cv::Rect` rectangle. Option 2 is ready by using `vector::erase()` on a sorted keypoints container.

### MP.4 Keypoint Descriptors
Implemented descriptors `BRIEF`, `ORB`, `FREAK`, `AKAZE` and `SIFT` in _matching2D.cpp_ using opencv API. `extract()` method is called for keypoint description.

### MP.5 Descriptor Matching
There are two matching options: 1. Brute_Force (readily-available), 2. FLANN. (value type change required because the poential opencv bug). FLANN is implemenetd using opencv API `cv::DescriptorMatcher::create()`.  

### MP.6 Descriptor Distance Ratio
This part is to implement KNN matching for descriptor distance ratio test in _matching2D.cpp_, which keeps the best 2 matches per keypoint.


### MP.7 Performance Evaluation 1
Plot 1: The number of keypoints from different detector.
| Detector  | `SHITOMASI` |  `HARRIS`  |   `ORB`   |    `FAST`   |    `SIFT`   |   `BRISK`   |   `AKAZE`   |
|:---------:|:-----------:|:----------:|:---------:|:-----------:|:-----------:|:-----------:|:-----------:|
|  Frame 1  |     125     |     15     |    92     |     419     |     138     |     264     |     166     |
|  Frame 2  |     118     |     13     |    102    |     427     |     132     |     282     |     157     |
|  Frame 3  |     123     |     16     |    106    |     404     |     124     |     282     |     161     |
|  Frame 4  |     120     |     14     |    113    |     423     |     137     |     277     |     155     |
|  Frame 5  |     120     |     21     |    109    |     386     |     134     |     297     |     163     |
|  Frame 6  |     113     |     32     |    125    |     414     |     140     |     279     |     164     |
|  Frame 7  |     114     |     14     |    130    |     418     |     137     |     289     |     173     |
|  Frame 8  |     123     |     27     |    129    |     406     |     148     |     272     |     175     |
|  Frame 9  |     111     |     21     |    127    |     396     |     159     |     266     |     177     |
|  Frame 10 |     112     |     23     |    128    |     401     |     137     |     254     |     179     |

And here are the visualized results of keypoint detector:
**SHI-TOMASI** keypoint detection:

<img src="images/Shi-Tomasi.png" width="660" height="200" />

**HARRIS** keypoint detection:

<img src="images/Harris.png" width="660" height="200" />

**FAST** keypoint detection:

<img src="images/FAST.png" width="660" height="200" />

**SIFT** keypoint detection:

<img src="images/SIFT.png" width="660" height="200" />

**ORB** keypoint detection:

<img src="images/ORB.png" width="660" height="200" />

**BRISK** keypoint detection:

<img src="images/BRISK.png" width="660" height="200" />

**AKAZE** keypoint detection:

<img src="images/AKAZE.png" width="660" height="200" />


### MP.8 Performance Evaluation 2


### MP.9 Performance Evaluation 3







