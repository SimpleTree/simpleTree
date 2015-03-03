/*
 * SphereFollowing.h
 *
 *  Created on: 06.11.2014
 *      Author: hackenberg
 */

#ifndef SPHEREFOLLOWING_H
#define SPHEREFOLLOWING_H

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <boost/smart_ptr/shared_ptr.hpp>
#include <boost/smart_ptr/weak_ptr.hpp>
#endif



#include <pcl/console/time.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree_search.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "../method/set_coefficients.h"
#include <iostream>
#include <limits>
#include <vector>

#include "../controller.h"
#include "../Model/Cylinder.h"
#include "method_coefficients.h"

typedef pcl::PointXYZINormal PointI;
typedef pcl::PointCloud<PointI> PointCloudI;
typedef pcl::PointCloud<pcl::PrincipalCurvatures> CurvatureCloud;
//#include <pcl/features/principal_curvatures.h>
class Controller;

class SphereFollowing
{
    std::vector<pcl::ModelCoefficients> spheres;
    std::vector<pcl::ModelCoefficients> cylinders;
    PointCloudI::Ptr treeCloud;
    std::vector<int>
    indexOfPointsNearCylinder (pcl::octree::OctreePointCloudSearch<PointI> &octree,
                               boost::shared_ptr<simpleTree::Cylinder> &cylinder,
                               float factorEnLarge = 1);
    int maxIterations;
    int iteration = 0;
    int maxDistToModel = 30;
    std::vector<float>
    distancesToModel (PointCloudI::Ptr treeCloud);
    PointCloudI::Ptr
    extractUnfittedPoints (std::vector<float> distances,
                           PointCloudI::Ptr points);
    void
    addConnectionCylinder (pcl::ModelCoefficients startSphere, float start_radius);
    boost::weak_ptr<Controller> control;

  public:
    boost::shared_ptr<Controller>
    getControl ();
    float epsilon = 0.02;
    float sphereRadiusMultiplier = 1.8f;
    float epsilon_cluster_stem = 0.02;
    float epsilon_cluster_branch = 0.008;
    float epsilon_sphere = 0.02;
  //  const float epsilon_sphere_branch = 0.013;
    int minPts_Ransac_stem = 2000;
    int minPts_Ransac_branch = std::numeric_limits<int>::max();
    int minPts_cluster_stem = 12;
    int minPts_cluster_branch = 3;
    float min_radius_sphere_stem = 0.035;
    float min_radius_sphere_branch = 0.025;
    SphereFollowing (PointCloudI::Ptr treeCloud,
                     boost::weak_ptr<Controller> control);
    SphereFollowing (PointCloudI::Ptr treeCloud,
                     boost::weak_ptr<Controller> control,
                     int maxIterations);
    SphereFollowing (PointCloudI::Ptr treeCloud,
                         boost::weak_ptr<Controller> control,
                         int maxIterations, Method_Coefficients coeff);
    virtual
    ~SphereFollowing ();
    std::vector<PointCloudI::Ptr>
    clusterPoints (PointCloudI::Ptr points_in, bool isStem);
    std::vector<PointCloudI::Ptr>
    intersectionSphereSurface (pcl::octree::OctreePointCloudSearch<PointI>& octree,
                               pcl::ModelCoefficients sphere,
                               PointCloudI::Ptr &cloud_in);
    bool
    lowestZCluster (PointCloudI cloud_in,
                    PointCloudI& cloud_out);
    void
    computeCylindersFromTree (PointCloudI::Ptr& treeCloud);
    void
    computeCylindersFromCluster (PointCloudI::Ptr& pointCluster,
                                 bool isFirstRun);
    pcl::ModelCoefficients
    fitSphereToCluster (PointCloudI::Ptr cluster,
                        float& radius, bool isStem);

    std::vector<pcl::ModelCoefficients>&
    getCylinders ()
    {
      return cylinders;
    }

};

#endif /* SPHEREFOLLOWING_H_ */
