/*
 * eigenvalueestimator.h
 *
 *  Created on: Feb 1, 2015
 *      Author: hackenberg
 */

#ifndef EIGENVALUEESTIMATOR_H_
#define EIGENVALUEESTIMATOR_H_

#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl/common/centroid.h>
#include <Eigen/Eigenvalues>
#include <pcl/console/time.h>

typedef pcl::PointXYZINormal PointI;
typedef pcl::PointCloud<PointI> PointCloudI;
class EigenValueEstimator
{
  public:
    EigenValueEstimator (PointCloudI::Ptr cloud_in, std::vector<float>& e1, std::vector<float>& e2, std::vector<float>& e3
    		,std::vector<bool>& isStem, float range, float lambda1_upper = 0.1, float lambda2_lower = 0.35);
    virtual
    ~EigenValueEstimator ();
  private:
    PointI centroid;
    PointCloudI::Ptr cloud;
    pcl::console::TicToc tt;

};

#endif /* EIGENVALUEESTIMATOR_H_ */
