/*
 * StemPointDetection.cpp
 *
 *  Created on: 05.02.2015
 *      Author: hackenberg
 */

#include "StemPointDetection.h"

StemPointDetection::StemPointDetection(PointCloudI::Ptr cloud,
		std::vector<bool> stem_pts_old, float max_distance, float max_intens) {
	this->cloud = cloud;
	this->stem_pts_old = stem_pts_old;
	this->max_intens = max_intens;
	this->max_distance = max_distance;
	clusters.reset(new pcl::IndicesClusters);

	copyCloud();
	pcl::ConditionalEuclideanClustering<PointI> cec(true);
	cec.setInputCloud(cloud_temp);
	cec.setConditionFunction(&customRegionGrowing);
	cec.setClusterTolerance(max_distance);
	cec.setMinClusterSize(100);
	cec.segment(*clusters);
	resetStemPoints();

}

StemPointDetection::~StemPointDetection() {
	// TODO Auto-generated destructor stub
}

void StemPointDetection::copyCloud() {
	cloud_temp.reset(new PointCloudI);
	pcl::copyPointCloud(*cloud, *cloud_temp);
	for (size_t i = 0; i < cloud_temp->points.size(); i++) {
		if (stem_pts_old.at(i)) {

	// std::cout << "fooasdasdas \n";	  
	 cloud_temp->points[i].intensity = 0;
			
		} else {
	 
			cloud_temp->points[i].intensity = 255;
		}
	}
}

bool customRegionGrowing(const PointI& p1, const PointI& p2,
		float squared_distance) {
	float dist = 0.05;
	if (squared_distance < dist) {
		if (p1.intensity < 0.5)
			return (true);
		if (p2.intensity < 0.5)
			return (true);
	}
	return false;
}

void StemPointDetection::resetStemPoints() {
	int index_largest_cluster = -1;
	size_t max_size =0;
	for (size_t i = 0; i < clusters->size (); ++i)
	  {
		if((*clusters)[i].indices.size ()>max_size)
		{std::cout << (*clusters)[i].indices.size () << "\n";
			max_size= (*clusters)[i].indices.size ();
			index_largest_cluster = i;
		}

	  }
	std::cout<< "index largest cluster" << index_largest_cluster<< "\n";
	std::cout<< "largest cluster size" << (*clusters)[index_largest_cluster].indices.size()<< "\n";
	stem_pts_new.clear();
	for (size_t i = 0; i < stem_pts_old.size(); i++) {
		stem_pts_new.push_back(false);
	}
	if(index_largest_cluster>-1)
	{
	for (size_t j = 0; j < (*clusters)[index_largest_cluster].indices.size(); ++j){
		stem_pts_new[(*clusters)[index_largest_cluster].indices[j]] = true;
	//std::cout<< "ffoo\n";
	  
	}
	}
}
