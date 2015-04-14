/*
* Software License Agreement (BSD License)
*
* Copyright (c) 2015, Jan Hackenberg, University of Freiburg.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of Willow Garage, Inc. nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
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
    bool use_ransac_for_sphere = false;
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
