/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{
    //SHITOMASI, HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
    string detectorType = "FAST";

    //BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT
    string descriptorType = "ORB";

    string descriptorValueType = descriptorType != "SIFT" ? "DES_BINARY" : "DES_HOG";

    string matcherType  = "MAT_FLANN";        // MAT_BF, MAT_FLANN
    string selectorType = "SEL_kNN";       // SEL_NN, SEL_KNN

    bool bVis = true;

    /* INIT VARIABLES AND DATA STRUCTURES */


    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time




    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
    {
        ostringstream imgNumber;
        imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
        string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;


        cv::Mat img, imgGray;
        img = cv::imread(imgFullFilename);
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);


        DataFrame frame;
        frame.cameraImg = imgGray;
        if(dataBuffer.size()==dataBufferSize) dataBuffer.erase(dataBuffer.begin());
        dataBuffer.push_back(frame);

        //cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;


        vector<cv::KeyPoint> keypoints; 

        if (detectorType.compare("SHITOMASI") == 0)
        {
            detKeypointsShiTomasi(keypoints, imgGray, bVis);
        }
        else
        {
          if (detectorType.compare("HARRIS") == 0)
          {
            detKeypointsHarris(keypoints, imgGray, bVis);
          }
          else
          {
            detKeypointsModern(keypoints, imgGray, detectorType, bVis);
          }
        }

        bool bFocusOnVehicle = true;
        cv::Rect vehicleRect(535, 180, 180, 150);
        if (bFocusOnVehicle)
        {
            // this piece of code is inspried from: https://github.com/rayryeng/Udacity_Sensor_Fusion_Nanodegree/tree/master/SFND_Camera/SFND_2D_Feature_Matching/src
            keypoints.erase(remove_if(keypoints.begin(), keypoints.end(), [&vehicleRect](const cv::KeyPoint& kpt) 
                                {
                                  return !vehicleRect.contains(kpt.pt);
                                }),
                      keypoints.end());
        }

        cout<<"# of detected Keypoints within ROI = "<<keypoints.size()<<endl;


        bool bLimitKpts = true;
        if (bLimitKpts)
        {
            int maxKeypoints = 50;

            if (detectorType.compare("SHITOMASI") == 0)
            { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
            }
            cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
            //cout << " NOTE: Keypoints have been limited!" << endl;
        }


        (dataBuffer.end() - 1)->keypoints = keypoints;

        //cout << "#2 : DETECT KEYPOINTS done" << endl;


        // -> BRISK, BRIEF, ORB, FREAK, AKAZE, SIF
        cv::Mat descriptors;
        descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);
        
        (dataBuffer.end() - 1)->descriptors = descriptors;

        //cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

        if (dataBuffer.size() > 1) // wait until at least two images have been processed
        {
            vector<cv::DMatch> matches;
           
            matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                             (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                             matches, descriptorValueType, matcherType, selectorType);

            (dataBuffer.end() - 1)->kptMatches = matches;

            cout<<"# of matched Keypoints = "<<matches.size() << endl; cout<<endl;

            //cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;

            // visualize matches between current and previous image
            
            if (bVis)
            {
                cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                                (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                                matches, matchImg,
                                cv::Scalar::all(-1), cv::Scalar::all(-1),
                                vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                string windowName = "Matching keypoints between two camera images";
                cv::namedWindow(windowName, 7);
                cv::imshow(windowName, matchImg);
                //cout << "Press key to continue to next image" << endl;
                cv::waitKey(0); // wait for key to be pressed
            }

        }

    } 

    return 0;
}
