// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


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
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr filtered_cloud (new pcl::PointCloud<PointT>);
    // create filtering object
    pcl::VoxelGrid<PointT> fil;
    fil.setInputCloud(cloud);
    // leaf size of 1cm
    fil.setLeafSize(filterRes, filterRes, filterRes);
    fil.filter(*filtered_cloud);
    
    // set region of interest
    typename pcl::PointCloud<PointT>::Ptr region_cloud (new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(filtered_cloud);
    region.filter(*region_cloud);
    
    // remove the top of the car
    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f (2.6, 1.7, -0.1, 1));
    roof.setInputCloud(region_cloud);
    // keep results of crop box in indices
    roof.filter(indices);
    // create extractor object and filter indices to remove the top of the car
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point : indices)
        inliers->indices.push_back(point);
    
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(region_cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*region_cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return region_cloud;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr cloud_p (new pcl::PointCloud<PointT>), cloud_f (new pcl::PointCloud<PointT>);
    // create extract object 
    pcl::ExtractIndices<PointT> extractor;
    extractor.setInputCloud(cloud);

    // extract planar object
    extractor.setIndices(inliers);
    extractor.setNegative(false);
    extractor.filter(*cloud_p);

    // extract obstacles
    extractor.setNegative(true);
    extractor.filter(*cloud_f);
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
    std::cerr << "PointCloud representing obstacle component: " << cloud_f->width * cloud_f->height << " data points." << std::endl;
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud_f, cloud_p);
    return segResult;
}

template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(uint id, const std::vector<std::vector<float>>& points, std::vector<bool> &processed, std::vector<int> &cluster, KdTree* tree, float distanceTol)
{
	// mark point as processes
	processed[id] = true;
	// add the point to the cluster
	cluster.push_back(id);
	// find nearest neighbours
	std::vector<int> neighbours = tree->search(points[id], distanceTol);
	// iterate through neighbours and find their neighbours 
	for(auto idx : neighbours)
	{
		if (!processed[idx])
			clusterHelper(idx, points, processed, cluster, tree, distanceTol);
	}
}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol, int minSize, int maxSize)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;
	// index in the vector of points and each node of the KdTree is the same
	std::vector<bool> processed (points.size(), false);
	for (uint i = 0; i < points.size(); i++)
	{
		if (!processed[i])
		{	
			// create cluster for the point
			std::vector<int> cluster;
			// find neighbours for the cluster
			clusterHelper(i, points, processed, cluster, tree, distanceTol);
            //std::cout << "Found cluster of size " << cluster.size() << std::endl;
			// keep the found cluster
            if (cluster.size() >= minSize && cluster.size() <= maxSize)
			    clusters.push_back(cluster);
		}
	}
	return clusters;
}

template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
    // For max iterations
    for (size_t i =0; i < maxIterations; i++)
    {
		std::unordered_set<int> inliers;

		while (inliers.size() < 3)
			inliers.insert(rand()%cloud->points.size());

		// get the pointer to begining of inliers
		auto it = inliers.begin();
		
		// get random points
		PointT p1 = cloud->points[*it];
		it++;
		PointT p2 = cloud->points[*it];
		it++;
		PointT p3 = cloud->points[*it];

		// get plane coefficients
		float A = (p2.y-p1.y)*(p3.z-p1.z) - (p2.z-p1.z)*(p3.y-p1.y);
		float B = (p2.z-p1.z)*(p3.x-p1.x) - (p2.x-p1.x)*(p3.z-p1.z);
		float C = (p2.x-p1.x)*(p3.y-p1.y) - (p2.y-p1.y)*(p3.x-p1.x);
		float D = -(A*p1.x+B*p1.y+C*p1.z);

		// Measure distance between every point and fitted plane
		for (int i = 0; i < cloud->points.size(); i++)
		{
			PointT p = cloud->points[i];
			if (std::fabs(A*p.x + B*p.y + C*p.z + D)/(std::sqrt(A*A+B*B+C*C)) < distanceTol)
			{
				// If distance is smaller than threshold count it as inlier
				inliers.insert(i);
			}
		}

        // keep the model with highest number of inliers
		if (inliers.size() > inliersResult.size())
		{
			inliersResult = inliers;
		}
    }

	return inliersResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // USING PCL LIBRARY SEGMENTATION
    // TODO:: Fill in this function to find inliers for the cloud.
    //pcl::PointIndices::Ptr inliers {new pcl::PointIndices()};
    //pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients()}; 

    // create pcl segmentation object
    //pcl::SACSegmentation<PointT> seg;
    //seg.setOptimizeCoefficients (true);
    //seg.setModelType(pcl::SACMODEL_PLANE);
    //seg.setMethodType(pcl::SAC_RANSAC);
    //seg.setMaxIterations(maxIterations);
    //seg.setDistanceThreshold(distanceThreshold);

    // segment the largest planar componenet from the remaining cloud
    //seg.setInputCloud(cloud);
    //seg.segment(*inliers, *coefficients);

    // segment using implemented RANSAC
    std::unordered_set<int> inliers_set = this->RansacPlane(cloud, maxIterations, distanceThreshold);
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices()};

    for (auto index : inliers_set)
        inliers->indices.push_back(index);

    // see if segmentation was successful
    if (inliers->indices.size () == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    // print the time take for segmentation
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    
    // gather results in a std::pair and return
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
    //typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    //tree->setInputCloud(cloud);
    // create a vector of indices to store cluster indices
    //std::vector<pcl::PointIndices> cluster_indices;
    // create euclidean cluster extraction object
    //typename pcl::EuclideanClusterExtraction<PointT> ec;
    //ec.setClusterTolerance(clusterTolerance);
    //ec.setMinClusterSize(minSize);
    //ec.setMaxClusterSize(maxSize);
    //ec.setSearchMethod(tree);
    //ec.setInputCloud(cloud);
    //extract the clusters
    //ec.extract(cluster_indices);

    // Use implemented euclidean clustering
    // create a vector of point from point cloud
    std::vector<std::vector<float>> points;
    // create KdTree 
    KdTree* tree = new KdTree;
    // Create a list of points and add to tree
    for (int i = 0; i < cloud->points.size(); i++) 
    {
        std::vector<float> point {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
        points.push_back(point);
        tree->insert(point,i); 
    }
    
    std::vector<std::vector<int>> indices = euclideanCluster(points, tree, clusterTolerance, minSize, maxSize); 

    // separate clusters from point indices
    for (auto ind : indices)
    {
        // create a point cloud to store points of each cluster
        typename pcl::PointCloud<PointT>::Ptr cluster_cloud (new pcl::PointCloud<PointT>);
        for (int idx : ind)
        {
            // add each point to the point cloud
            cluster_cloud->points.push_back(cloud->points[idx]);
        }
        //std::cout << "Cluster with " << cluster_cloud->points.size() << " points added." << std::endl;
        clusters.push_back(cluster_cloud);
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