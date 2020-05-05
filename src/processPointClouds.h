// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
//#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include <unordered_set>
#include "render/box.h"


template <typename PointT>
struct Node
{
	PointT point;
	int id;
	Node<PointT>* left;
	Node<PointT>* right;

	Node(PointT arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

template <typename PointT>
struct KdTree
{
	Node<PointT>* root;

	KdTree(): root(NULL){}

	void insertNode(Node<PointT>** node, uint depth, PointT& point, int id)
	{
		if(*node == NULL) // empty tree
		{
			*node = new Node<PointT>(point, id);
		}
		else
		{
			// Calculate current depth
			uint current_depth = depth % 3;
			if(point.data[current_depth] < ((*node)->point.data[current_depth]))
			{
				insertNode(&(*node)->left, depth+1, point, id);
			}
			else
			{
				insertNode(&(*node)->right, depth+1, point, id);
			}	
		}		
	}

	void insert(PointT& point, int id)
	{
		// insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertNode(&root, 0, point, id);
	}

	void searchHelper(PointT& target, Node<PointT>* node, uint depth, float distanceTol, std::vector<int>& ids)
	{
		if (node != NULL)
		{
			if((node->point.x >= (target.x - distanceTol)) && (node->point.x <= (target.x + distanceTol))
			&& (node->point.y >= (target.y - distanceTol)) && (node->point.y <= (target.y + distanceTol))
            && (node->point.z >= (target.z - distanceTol)) && (node->point.z <= (target.z + distanceTol)))
			{   // within box centered by target, then check the distance
				float distance = sqrt ((node->point.x - target.x) * (node->point.x - target.x) 
									 + (node->point.y - target.y) * (node->point.y - target.y)
                                     + (node->point.z - target.z) * (node->point.z - target.z));
				if (distance <= distanceTol)
				{
					ids.push_back(node->id);
				}
			}

            uint current_depth = depth % 3;
			if (target.data[current_depth] - distanceTol < node->point.data[current_depth])
			{
				searchHelper(target, node->left, depth+1, distanceTol, ids);
			}			
			if (target.data[current_depth] + distanceTol > node->point.data[current_depth])
			{
				searchHelper(target, node->right, depth+1, distanceTol, ids);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(PointT& target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}	
};

template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);

    void proximity(int indice, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster, std::vector<bool>& processed, KdTree<PointT>* tree, float distanceTol);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, float distanceTol, int minSize, int maxSize);

};
#endif /* PROCESSPOINTCLOUDS_H_ */