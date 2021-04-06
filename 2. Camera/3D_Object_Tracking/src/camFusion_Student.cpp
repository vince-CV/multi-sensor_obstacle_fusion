
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        // pixel coordinates
        pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0); 
        pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0); 

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}


void showLidar(std::vector<LidarPoint> lidarPoints, string name, double res)
{

  		cv::Size worldSize(20, 20);
  		cv::Size imageSize(500, 500);
  		cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));
  
		cv::RNG rng(0);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = lidarPoints.begin(); it2 != lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            cv::circle(topviewImg, cv::Point(x, y), 1, currColor, -1);
        }

  		double new_res = (-res * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, new_res), cv::Point(imageSize.width, new_res), cv::Scalar(255, 0, 0));  
        cv::imshow(name, topviewImg);
  		//cv::waitKey();

}

/* 
* The show3DObjects() function below can handle different output image sizes, but the text output has been manually tuned to fit the 2000x2000 size. 
* However, you can make this function work for other sizes too.
* For instance, to use a 1000x1000 size, adjusting the text positions by dividing them by 2.
*/
void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize,  bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        //cv::RNG rng(it1->boxID);
        //cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));
        cv::Scalar currColor = cv::Scalar(0,0,0);

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }
        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50),  cv::FONT_HERSHEY_PLAIN, 4, currColor, 2, cv::LINE_AA);
      //putText(visImg, str, cv::Point2f(80, 50), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0,0,255), 2, cv::LINE_AA);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_HERSHEY_PLAIN, 4, currColor, 2, cv::LINE_AA); 
     // putText(visImg, str, cv::Point2f(80, 50), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0,0,255), 2, cv::LINE_AA);
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    //cv::resize(topviewImg, topviewImg, cv::Size(), 0.3, 0.3);
    //cv::imshow(windowName, topviewImg);
   
    if(bWait)
    {
        //cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches, cv::Mat image1, cv::Mat image2)
{
    boundingBox.keypoints.clear();
    boundingBox.kptMatches.clear();
  
    vector<cv::KeyPoint> enclosed_kptsPrev, enclosed_kptsCurr;
    vector<cv::DMatch> enclosed_kptMatches, inlier_kptMatches;
  
    vector<cv::Point2f> pt1, pt2;

    for(auto match: kptMatches)
    {
        if(boundingBox.roi.contains(kptsCurr[match.trainIdx].pt))
        {
            pt1.push_back( kptsPrev[match.queryIdx].pt);
            pt2.push_back( kptsCurr[match.trainIdx].pt);
            enclosed_kptMatches.push_back( match );
          
            enclosed_kptsPrev.push_back(kptsPrev[match.queryIdx]);
            enclosed_kptsCurr.push_back(kptsCurr[match.trainIdx]);
        }
    }
  
    //cv::Mat imageMatches;
    //drawMatches(image1, kptsPrev, image2, kptsCurr, enclosed_kptMatches, imageMatches, cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 0) );
    //if ((imageMatches.cols > 1000) || (imageMatches.rows > 1000)) { resize(imageMatches, imageMatches, cv::Size(), 0.5, 0.5); }
    //imshow("Object Detection before", imageMatches);
    //cv::waitKey(0);
 
  
    vector<uchar> inliers(pt1.size(), 0);
    cv::Mat H = cv::findHomography(pt1, pt2, inliers, cv::RANSAC, 1.0);

    vector<uchar>::const_iterator itIn = inliers.begin();
    vector<cv::DMatch>::const_iterator itM = enclosed_kptMatches.begin();

    for (; itIn != inliers.end(); ++itIn, ++itM)
    {
        if (*itIn)
        {
          inlier_kptMatches.push_back(*itM);
        }
    }
   
    boundingBox.kptMatches = inlier_kptMatches;
  
    for(auto match: inlier_kptMatches)
    {
        boundingBox.keypoints.push_back( kptsCurr[match.trainIdx] );
    }
  
    //cv::Mat imageMatches1;
    //drawMatches(image1, kptsPrev, image2, kptsCurr, boundingBox.kptMatches, imageMatches1, cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 0) );
    //if ((imageMatches1.cols > 1000) || (imageMatches1.rows > 1000)) { resize(imageMatches1, imageMatches1, cv::Size(), 0.5, 0.5); }
    //imshow("feature matching", imageMatches1);
    //cv::waitKey(0);
   
   
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
        
		vector<double> distRatios; 
  
  		if (kptMatches.size() == 0)
		{
			TTC = 0.0;
			return;
		}
  
		for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
		{
			cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
			cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

			for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
			{ 
				double minDist = 100.0; 
              
				cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
				cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);
              
				double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
				double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

				if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
				{ 
					double distRatio = distCurr / distPrev;
					distRatios.push_back(distRatio);
				}
			} 
		}    
       
		// only continue if list of distance ratios is not empty
		if (distRatios.size() == 0)
		{
			TTC = 0.0;
			return;
		}


		//double DistRatio = std::accumulate(distRatios.begin(), distRatios.end(), 0.0) / distRatios.size();
        
		sort(distRatios.begin(), distRatios.end());
		double medianRatio = distRatios[int(distRatios.size() / 2)];

		double dT = 1 / frameRate;
		TTC = -dT / (1 - medianRatio);
}


double getThreshold(const vector<double> &xpts, const double &margin)
{
      
  	double mean =  std::accumulate(std::begin(xpts), std::end(xpts), 0.0) / xpts.size(); 
    
    double accum  = 0.0;  
    std::for_each(std::begin(xpts), std::end(xpts), [&](const double val) {  
    	accum  += (val-mean)*(val-mean); 
    });   
  
    double std = sqrt(accum/(xpts.size()-1)); 
    
    return mean - std*margin;
}

void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev, std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    //std::vector<LidarPoint> lidarPointsPrev = lidarPointsPrevv;
    //std::vector<LidarPoint> lidarPointsCurr = lidarPointsCurrv;
  
  
  	double laneWidth = 4.0; // assumed width of the ego lane
  
    vector<double> xprev, xcurr;
  	for (auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); ++it)
    {
        if(it->y*it->y <= laneWidth)  
        {
            xprev.push_back(it->x);
        }
    }
    for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it)
    {
        if(it->y*it->y <= laneWidth)
        {
            xcurr.push_back(it->x);
        }
    }
  
    double thresholdXprev = getThreshold(xprev, 2);
    double thresholdXcurr = getThreshold(xcurr, 2);
  
    //cout<<"thresholdXprev = "<<thresholdXprev<<endl;
    //cout<<"thresholdXcurr = "<<thresholdXcurr<<endl;

    double dT = 1.0 / frameRate;                // time between two measurements in seconds
    double minXPrev = 1e9, minXCurr = 1e9;      // find closest distance to Lidar points within ego lane

    //cout<<"before erase = "<<lidarPointsPrev.size()<<endl;
    //showLidar(lidarPointsPrev, "before", thresholdXprev);
    vector<LidarPoint>::iterator itor = lidarPointsPrev.begin();
    while( itor != lidarPointsPrev.end() )
    {
        if( (abs(itor->y) > laneWidth / 2.0) || (itor->x < thresholdXprev) )
        {
          itor = lidarPointsPrev.erase(itor);
        }
        else
        {
          itor++;
        }
    }
    //showLidar(lidarPointsPrev, "after", thresholdXprev);
    //cout<<"after erase = "<<lidarPointsPrev.size()<<endl; cout<<endl;
    //cv::waitKey();
 
    //cout<<"before erase = "<<lidarPointsCurr.size()<<endl;
    //showLidar(lidarPointsCurr, "before", thresholdXcurr);
    vector<LidarPoint>::iterator itor_ = lidarPointsCurr.begin();
    while( itor_ != lidarPointsCurr.end() )
    {
        if( (abs(itor_->y) > laneWidth / 2.0) || (itor_->x < thresholdXcurr) )
        {
          itor_ = lidarPointsCurr.erase(itor_);
        }
        else
        {
          itor_++;
        }
    }
    //showLidar(lidarPointsCurr, "after", thresholdXcurr);
    //cout<<"after erase = "<<lidarPointsCurr.size()<<endl;
    //cv::waitKey();
  
  
  	 for (auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); ++it)
    {
        minXPrev = minXPrev > it->x ? it->x : minXPrev;
    }

    for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it)
    {
        minXCurr = minXCurr > it->x ? it->x : minXCurr;
    }
  
    //cout<<"minXPrev = "<<minXPrev<<endl;
    //cout<<"minXCurr = "<<minXCurr<<endl;


    // compute TTC from both measurements
    TTC = minXCurr * dT / (minXPrev - minXCurr);
  
}


vector<int> encloseBouningBoxes(const cv::Point &pt, const DataFrame &Frame)
{
    vector<int> output;
    for(int i = 0; i<Frame.boundingBoxes.size(); i++){
        if(Frame.boundingBoxes[i].roi.contains(pt)){
            output.push_back(Frame.boundingBoxes[i].boxID);
        }
    }
  
    return output;
}

void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    int row = prevFrame.boundingBoxes.size();
    int col = currFrame.boundingBoxes.size();
    
    vector<vector<int>> table(row, vector<int>(col, 0));
  
    for(auto match: matches)
    {
        cv::Point prevPt = prevFrame.keypoints[match.queryIdx].pt;
        cv::Point currPt = currFrame.keypoints[match.trainIdx].pt;
      
        vector<int> prevBoxesPresent = encloseBouningBoxes(prevPt, prevFrame);
        vector<int> currBoxesPresent = encloseBouningBoxes(currPt, currFrame);
      
        for (auto i : prevBoxesPresent){
            for (auto j : currBoxesPresent){
                table[i][j] += 1;
            }
        }
    }
  
  	for(int i = 0; i < row; i++)
    {
        auto maxPosition = max_element(table[i].begin(), table[i].end());
        int j = maxPosition - table[i].begin(); 
      
        if(table[i][j]!=0)
        {
          bbBestMatches.insert(make_pair(i, j));
        }
    }
  
    /*
  	for(int i = 0; i < row; i++){
        for(int j = 0; j < col; j++){
            cout<<table[i][j]<<" ";
        }
        cout<<endl;
    }
   */
}
