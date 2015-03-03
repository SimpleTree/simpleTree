/*
 * StemPointDetection.h
 *
 *  Created on: 05.02.2015
 *      Author: hackenberg
 */

#ifndef STEMPOINTDETECTION_H_
#define STEMPOINTDETECTION_H_
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
typedef pcl::PointXYZRGBA PointD;
typedef pcl::PointCloud<PointD> PointCloudD;
typedef pcl::PointXYZINormal PointI;
typedef pcl::PointCloud<PointI> PointCloudI;
typedef pcl::PointCloud<pcl::PrincipalCurvatures> CurvatureCloud;
bool
	customRegionGrowing(const PointI& p1, const PointI& p2, float squared_distance);
class StemPointDetection {
public:
	StemPointDetection(PointCloudI::Ptr cloud,	std::vector<bool> stem_pts_old
			, float max_distance =0.05, float max_intens =1);
	virtual ~StemPointDetection();

	const std::vector<bool>& getStemPtsNew() const {
		return stem_pts_new;
	}

	void setStemPtsNew(const std::vector<bool>& stemPtsNew) {
		stem_pts_new = stemPtsNew;
	}

private:
	PointCloudI::Ptr cloud;
	std::vector<bool> stem_pts_old;
	std::vector<bool> stem_pts_new;
	PointCloudI::Ptr cloud_temp;
	pcl::IndicesClustersPtr clusters;
	float max_distance;
	float max_intens;
	void copyCloud();

	void
	resetStemPoints();

};

#endif /* STEMPOINTDETECTION_H_ */
