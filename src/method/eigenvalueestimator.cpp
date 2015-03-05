/*
 * eigenvalueestimator.cpp
 *
 *  Created on: Feb 1, 2015
 *      Author: hackenberg
 */

#include "eigenvalueestimator.h"

EigenValueEstimator::EigenValueEstimator (PointCloudI::Ptr cloud_in,
                                          std::vector<float>& e1, std::vector<float>& e2, std::vector<float>& e3
                                          ,std::vector<bool>& isStem,float range
                                          , float lambda1_upper, float lambda2_lower)
{
  tt.tic ();
  e1.clear();
  e2.clear();
  e3.clear();
  this->cloud = cloud_in;
  pcl::octree::OctreePointCloudSearch<PointI> octree (0.01f);

    octree.setInputCloud (cloud);
    octree.addPointsFromInputCloud ();

  for(size_t i = 0; i < cloud->points.size(); i++)
  {
    PointI p = cloud->points.at(i);
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    octree.radiusSearch (p, range, pointIdxRadiusSearch, pointRadiusSquaredDistance);

    // Placeholder for the 3x3 covariance matrix at each surface patch
    Eigen::Matrix3f covariance_matrix;
    // 16-bytes aligned placeholder for the XYZ centroid of a surface patch
    Eigen::Vector4f xyz_centroid;

    // Estimate the XYZ centroid
    pcl::compute3DCentroid<PointI> (*cloud, pointIdxRadiusSearch, xyz_centroid);

    // Compute the 3x3 covariance matrix
    pcl::computeCovarianceMatrix (*cloud, pointIdxRadiusSearch, xyz_centroid, covariance_matrix);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen (covariance_matrix);
    float lambda1,lambda2,lambda3,sum;
    lambda1 = eigen.eigenvalues()[0];
    lambda2 = eigen.eigenvalues()[1];
    lambda3 = eigen.eigenvalues()[2];
    sum = lambda1+lambda2+lambda3;
    lambda1 /= sum;
    lambda2 /= sum;
    lambda3 /= sum;

    e1.push_back(lambda1);
    e2.push_back(lambda2);
    e3.push_back(lambda3);
    if(lambda1_upper>=lambda1&&lambda2_lower<=lambda2)
    {
    	isStem.push_back(true);
    } else {
    	isStem.push_back(false);
    }



//    PointI p = cloud->points.at(i);
//    std::vector<int> pointIdxRadiusSearch;
//    std::vector<float> pointRadiusSquaredDistance;
//    PointCloudI::Ptr neighbors (new PointCloudI);
//    if (octree.radiusSearch (p, range, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
//     {
//       for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
//         neighbors->points.push_back( cloud->points[ pointIdxRadiusSearch[i] ]);
//     }
//    // Placeholder for the 3x3 covariance matrix at each surface patch
//    Eigen::Matrix3f covariance_matrix;
//    // 16-bytes aligned placeholder for the XYZ centroid of a surface patch
//    Eigen::Vector4f xyz_centroid;
//
//    // Estimate the XYZ centroid
//    pcl::compute3DCentroid<PointI> (*neighbors, xyz_centroid);
//
//    // Compute the 3x3 covariance matrix
//    pcl::computeCovarianceMatrix (*neighbors, xyz_centroid, covariance_matrix);
//    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen (covariance_matrix);
//    float lambda1,lambda2,lambda3,sum;
//    lambda1 = eigen.eigenvalues()[0];
//    lambda2 = eigen.eigenvalues()[1];
//    lambda3 = eigen.eigenvalues()[2];
//    sum = lambda1+lambda2+lambda3;
//    lambda1 /= sum;
//    lambda2 /= sum;
//    lambda3 /= sum;
//
//    e1.push_back(lambda1);
//    e2.push_back(lambda2);
//    e3.push_back(lambda3);
   }
  std::cout << "PCA in " << tt.toc () / 1000 << " seconds." << std::endl;

}

EigenValueEstimator::~EigenValueEstimator ()
{
  // TODO Auto-generated destructor stub
}
