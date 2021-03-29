// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


using namespace std;

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

template<typename PointT>
void ProcessPointClouds<PointT>::Proximity(std::vector<std::vector<float>> points, int index, KdTree* tree, float distanceTol, std::vector<int> &cluster, std::vector<bool> &visited)
{
	visited[index] = true;
	cluster.push_back(index);

	std::vector<int> nearby = tree->search(points[index], distanceTol);

	for (int i : nearby)
	{
		if(!visited[i])
		{
			Proximity(points, i, tree, distanceTol, cluster, visited);
		}
	}

}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;
	std::vector<bool> visited(points.size(), false);


	for(int i = 0; i<points.size(); i++)
	{
		if(visited[i]==false)
		{
			vector<int> cluster;
			Proximity(points, i, tree, distanceTol, cluster, visited);
			clusters.push_back(cluster);
		}
	}
 
	return clusters;

}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    typename pcl::PointCloud<PointT>::Ptr cloud_filtered ( new pcl::PointCloud<PointT> );
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*cloud_filtered);

    typename pcl::PointCloud<PointT>::Ptr cloud_boxed ( new pcl::PointCloud<PointT> );
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloud_filtered);
    region.filter(*cloud_boxed);


    vector<int> index;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1,   1));
    roof.setMax(Eigen::Vector4f ( 2.6,  1.7, -0.4, 1));
    roof.setInputCloud(cloud_boxed);
    roof.filter(index);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point:index)
        inliers->indices.push_back(point);
    
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_boxed);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_boxed);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_boxed;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstacle(new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr plane(new pcl::PointCloud<PointT> ());

    for (int index : inliers->indices)
    {
        plane->points.push_back(cloud->points[index]);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstacle);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacle, plane);

    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	// For max iterations 
	while(maxIterations--)
	{
		std::unordered_set<int> inliersCurrent;

		while(inliersCurrent.size()<3)
		{
			// Randomly sample subset and fit line
			inliersCurrent.insert(rand()%(cloud->points.size()));
		}

		auto loc = inliersCurrent.begin();
		//int dot1 =  rand()%size;
		//int dot2 =  rand()%size;

		float x1 = cloud->points[*loc].x;
		float y1 = cloud->points[*loc].y;
		float z1 = cloud->points[*loc].z;
		loc++;
		float x2 = cloud->points[*loc].x;
		float y2 = cloud->points[*loc].y;
		float z2 = cloud->points[*loc].z;
		loc++;
		float x3 = cloud->points[*loc].x;
		float y3 = cloud->points[*loc].y;
		float z3 = cloud->points[*loc].z;

		float i = (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);
		float j = (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
		float k = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);

		float a = i;
		float b = j;
		float c = k;
		float d = -(i*x1+j*y1+k*z1);

		for( int idx = 0; idx < cloud->points.size(); idx++ )
		{
			// Measure distance between every point and fitted line
			PointT p = cloud->points[idx];
			float distance = fabs(a*p.x + b*p.y + c*p.z + d)/sqrt(a*a + b*b + c*c);

			// If distance is smaller than threshold count it as inlier
			if (distance <= distanceTol)
			{
				inliersCurrent.insert(idx);
			}
		}

		if(inliersCurrent.size() > inliersResult.size())
		{
			inliersResult = inliersCurrent;
		}
		
	}


    pcl::PointIndices::Ptr inliersIndex(new pcl::PointIndices); 

    for (auto it = inliersResult.begin(); it != inliersResult.end(); ++it)
    {
        inliersIndex->indices.push_back(*it);
    }

 
	// Return indicies of inliers from fitted line with most inliers
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliersIndex,cloud);
	
    return segResult;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in this function to find inliers for the cloud.
    // Create the segmentation object
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};

    pcl::SACSegmentation<PointT> seg;
    
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);

    seg.setInputCloud(cloud);
    
    // Segment the largest planar component from the input cloud.
    seg.segment(*inliers, *coefficients);

    if(inliers->indices.size() == 0){ std::cout << "Fail to estimate a planar model for the given dataset." << std::endl; }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);

    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;

    ec.setClusterTolerance (clusterTolerance); 
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    for(pcl::PointIndices getIndices:cluster_indices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

        for(int index:getIndices.indices)
        {
            cloudCluster->points.push_back(cloud->points[index]);
        }

        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        clusters.push_back(cloudCluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::ClusteringKD(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    vector<vector<float>> points;

    KdTree* tree = new KdTree;
  
    for (int i=0; i<cloud->points.size(); i++) 
    {
        const PointT pt = cloud->points[i];
        const std::vector<float> vec = {pt.x, pt.y, pt.z };

    	tree->insert(vec, i); 
        points.push_back(vec);
    }

    const std::vector<std::vector<int>> clusters_vec = euclideanCluster(points, tree, clusterTolerance);

    delete tree;
    tree = NULL;

    // Render clusters
  	// int clusterId = 0;

  	for(const std::vector<int> cluster : clusters_vec)
  	{
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

  		for(const int index: cluster)
        {
  			cloudCluster->points.push_back(cloud->points[index]);
        }

        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        clusters.push_back(cloudCluster);
        
  	}


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}