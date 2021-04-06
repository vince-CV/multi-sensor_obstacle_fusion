
#include <numeric>
#include "matching2D.hpp"

using namespace std;

// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType)
{
    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    if (matcherType.compare("MAT_BF") == 0)
    {
        //int normType = cv::NORM_HAMMING;
        //matcher = cv::BFMatcher::create(normType, crossCheck);
        int normType = descriptorType.compare("DES_BINARY") == 0 ? cv::NORM_HAMMING : cv::NORM_L2;
        matcher = cv::BFMatcher::create(normType, crossCheck);
    }
    else if (matcherType.compare("MAT_FLANN") == 0)
    {
        //potential bug in the current implementation of the OpenCV, which requires a conversion of the binary descriptors into floating point vectors
        if (descSource.type() != CV_32F)
        { 
          descSource.convertTo(descSource, CV_32F);
          descRef.convertTo(descRef, CV_32F);
        }
        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    }

    // perform matching task
    if (selectorType.compare("SEL_NN") == 0)
    { // nearest neighbor (best match)
        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
    }
    else if (selectorType.compare("SEL_KNN") == 0)
    { 	// k nearest neighbors (k=2)
        std::vector<std::vector<cv::DMatch>> knn_matches;
        matcher->knnMatch(descSource, descRef, knn_matches, 2);
      
        double minDescDistRatio = 0.8;
        for (auto it = knn_matches.begin(); it != knn_matches.end(); ++it)
        {
          if ((*it)[0].distance < minDescDistRatio * (*it)[1].distance)
          {
            matches.push_back((*it)[0]);
          }
        }
    }
}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType)
{
    // select appropriate descriptor
    cv::Ptr<cv::DescriptorExtractor> extractor;
  
    if (descriptorType.compare("BRISK") == 0)
    {
        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }
    else if (descriptorType == "SIFT") 
    {
      	extractor = cv::xfeatures2d::SIFT::create();
    } 
    else if (descriptorType == "ORB") 
    {
      	extractor = cv::ORB::create();
    } 
    else if (descriptorType == "BRIEF") 
    {
      	extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
    } 
    else if (descriptorType == "AKAZE") 
    {
      	extractor = cv::AKAZE::create();
    } 
    else if (descriptorType == "FREAK") 
    {
      	extractor = cv::xfeatures2d::FREAK::create();
    }
    else 
    {
      	return;
    }

    extractor->compute(img, keypoints, descriptors);
  
    // perform feature description
    double t = (double)cv::getTickCount();
    extractor->compute(img, keypoints, descriptors);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    //cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl;
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
void detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0; // max. permissible overlap between two features in %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints

    double qualityLevel = 0.01; // minimal accepted quality of image corners
    double k = 0.04;

    // Apply corner detection
    double t = (double)cv::getTickCount();
    vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

    // add corners to result vector
    for (auto it = corners.begin(); it != corners.end(); ++it)
    {

        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        keypoints.push_back(newKeyPoint);
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    //cout << "Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Shi-Tomasi Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

void detKeypointsHarris(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // hyper-parameter
    int blockSize = 2;
    int apertureSize = 3;
    double k = 0.04;

    cv::Mat dst = cv::Mat::zeros(img.size(), CV_32FC1);
    cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);

    cv::Mat dst_norm, dst_norm_scaled;
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1);
    cv::convertScaleAbs(dst_norm, dst_norm_scaled);


    // NMS
    const int nms_window_size = 2 * apertureSize + 1;  // 7 x 7
    const int rows = dst_norm.rows;
    const int cols = dst_norm.cols;

    int minResponse = 100;

    // Store the resulting points in a vector of cv::KeyPoints
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
        int centre_r = -1;
        int centre_c = -1;

        // The max value is set to the minimum response
        // We should have keypoints that exceed this first
        unsigned char max_val = static_cast<unsigned char>(minResponse);
        for (int x = -nms_window_size; x <= nms_window_size; x++) {
            for (int y = -nms_window_size; y <= nms_window_size; y++) {
            if ((i + x) < 0 || (i + x) >= rows) { continue; }
            if ((j + y) < 0 || (j + y) >= cols) { continue; }
            const unsigned char val =
                dst_norm_scaled.at<unsigned char>(i + x, j + y);
            if (val > max_val) {
                max_val = val;
                centre_r = i + x;
                centre_c = j + y;
            }
            }
        }

        // If the largest value was at the centre, remember this keypoint
        if (centre_r == i && centre_c == j) {
            keypoints.emplace_back(j, i, 2 * apertureSize, -1, max_val);
        }
        }
    }
    //clock_t t;
    //t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    //cout << "Harris detection with n=" << keypoints.size() << " keypoints in "
    //    << 1000 * t / 1.0 << " ms" << endl;

    // visualize results
    if (bVis) {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1),
                        cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Harris Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cout << "Press any key to continue\n";
        cv::waitKey(0);
    }



}

void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorType, bool bVis)
{
  cv::Ptr<cv::FeatureDetector> detector;
  
  if (detectorType == "SIFT")       { detector = cv::xfeatures2d::SIFT::create(); }
  else if (detectorType == "ORB") 	{ detector = cv::ORB::create(); }
  else if (detectorType == "BRISK") { detector = cv::BRISK::create(); }
  else if (detectorType == "AKAZE") { detector = cv::AKAZE::create(); }
  else if (detectorType == "FAST")  { detector = cv::FastFeatureDetector::create(); }
  else { return ; }
  
  detector->detect(img, keypoints);

  if (bVis) 
  {
    cv::Mat Imshow = img.clone();
    cv::drawKeypoints(img, keypoints, Imshow, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    imshow("KeyPoint Detection "+detectorType, Imshow);
    cv::waitKey(0);
  }

}