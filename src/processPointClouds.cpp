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
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud
                                                                            , float filterRes
                                                                            , Eigen::Vector4f minPoint
                                                                            , Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    // Create the filtering object
    pcl::VoxelGrid<PointT> vg;
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);
    vg.setLeafSize (filterRes, filterRes, filterRes);
    vg.setInputCloud (cloud);
    vg.filter (*cloudFiltered);

    typename pcl::PointCloud<PointT>::Ptr cloudROI (new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter (*cloudROI);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudROI;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers
                                                                                                                                , typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>());

    for(int index : inliers->indices)
        planeCloud->points.push_back(cloud->points[index]);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter(*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud
                                                                                                                                , int maxIterations
                                                                                                                                , float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Use PCL's built in segmentation function to find inliers for the cloud.
    // pcl::SACSegmentation<PointT> seg;
    // pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    // pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};

    // seg.setOptimizeCoefficients(true);
    // seg.setModelType(pcl::SACMODEL_PLANE);
    // seg.setMethodType(pcl::SAC_RANSAC);
    // seg.setMaxIterations(maxIterations);
    // seg.setDistanceThreshold(distanceThreshold);

    // // Segment the largest planar component from the input cloud
    // seg.setInputCloud(cloud);
    // seg.segment(*inliers, *coefficients);
    // if(inliers->indices.size() == 0)
    // {
    //     std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    // }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult;

    segResult = RansacPlane(cloud, maxIterations, distanceThreshold);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud
                                                                                                                            , int maxIterations
                                                                                                                            , float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	while(maxIterations--)
	{
		// Randomly pick two points
		std::unordered_set<int> inliers;
		while (inliers.size() < 3)
			inliers.insert(rand()%(cloud->points.size()));

		float x1, x2, x3, y1, y2, y3, z1, z2, z3;

		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;
		
		// Equation of a plane through three points
		float a = ((y2-y1)*(z3-z1)-(z2-z1)*(y3-y1));
		float b = ((z2-z1)*(x3-x1)-(x2-x1)*(z3-z1));
		float c = ((x2-x1)*(y3-y1)-(y2-y1)*(x3-x1));
		float d =  (-(a*x1+b*y1+c*z1));

		for (int index = 0; index < cloud->points.size(); index++)
		{
			if(inliers.count(index)>0)
				continue;

		    PointT point = cloud->points[index];
			float x4 = point.x;
			float y4 = point.y;
			float z4 = point.y;

			float distance = fabs(a*x4+b*y4+c*z4+d)/sqrt(a*a+b*b+c*c);

			if(distance <= distanceTol)
				inliers.insert(index);
		}

		if(inliers.size() > inliersResult.size())
		{
			inliersResult = inliers;
		}

	}	

	typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliersResult.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudInliers, cloudOutliers);

	return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud
                                                                                        , float clusterTolerance
                                                                                        , int minSize
                                                                                        , int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    // Use PCL's built in clustering function to perform euclidean clustering to group detected obstacles
    // http://pointclouds.org/documentation/tutorials/cluster_extraction.php
    // Creating the KdTree object for the search method of the extraction
    // typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    // tree->setInputCloud (cloud);

    // std::vector<pcl::PointIndices> clusterIndices;
    // pcl::EuclideanClusterExtraction<PointT> ec;
    // ec.setClusterTolerance (clusterTolerance); 
    // ec.setMinClusterSize (minSize);
    // ec.setMaxClusterSize (maxSize);
    // ec.setSearchMethod (tree);
    // ec.setInputCloud (cloud);
    // ec.extract (clusterIndices);  
 
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    clusters = euclideanCluster(cloud, clusterTolerance, minSize, maxSize);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud
                                                                                                , float distanceTol
                                                                                                , int minSize
                                                                                                , int maxSize)
{
	KdTree<PointT>* tree = new KdTree<PointT>;
  
    // Do Kd-tree inseration
    for (int i=0; i<cloud->points.size(); i++) 
    	tree->insert(cloud->points[i],i); 
    std::cout << "Done kd-tree insertion" << std::endl;


	std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
	std::vector<bool> processed(cloud->points.size(), false);

	int i = 0;
	while (i < cloud->points.size())
	{
		if(processed[i])
		{
			i++;
			continue;
		}

		std::vector<int> clusterIndices;
		proximity(i, cloud, clusterIndices, processed, tree, distanceTol);

        if (clusterIndices.size() >= minSize && clusterIndices.size() <= maxSize)
        {
            typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);
          
            for (int index : clusterIndices)
                cloudCluster->points.push_back(cloud->points[index]);
    
            cloudCluster->width = cloudCluster->points.size ();
            cloudCluster->height = 1;
            cloudCluster->is_dense = true;

            clusters.push_back(cloudCluster);
            std::cout << "PointCloud representing the Cluster: " << cloudCluster->points.size () << " data points." << std::endl;

        }
        else
        {
            for (int index : clusterIndices)
                processed[index] = false;
        }
        
		i++;
	}
 
 	// return list of indices for each cluster
	return clusters;

}


template<typename PointT>
void ProcessPointClouds<PointT>::proximity(int indice 
				                        , typename pcl::PointCloud<PointT>::Ptr cloud
                                        , std::vector<int>& cluster
                                        , std::vector<bool>& processed 
                                        , KdTree<PointT>* tree 
                                        , float distanceTol)
{
	processed[indice] = true;
	cluster.push_back(indice);

	std::vector<int> nearest = tree->search(cloud->points[indice], distanceTol);

	for(int id : nearest)
	{
		if(!processed[id])
			proximity(id, cloud, cluster, processed, tree, distanceTol);
	}
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