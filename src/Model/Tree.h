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
#ifndef TREE_H_
#define TREE_H_
#include <vector>

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#endif



#include "Cylinder.h"
#include "Segment.h"
#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/octree/octree.h>
#include <pcl/console/time.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/distances.h>
#include <pcl/filters/project_inliers.h>
#include <Eigen/Core>
#include <iostream>
#include <fstream>
#include "crown.h"
#include <QString>
#include "childcylinderextraction.h"
typedef pcl::PointXYZINormal PointI;
typedef pcl::PointCloud<PointI> PointCloudI;
typedef pcl::PointCloud<pcl::PrincipalCurvatures> CurvatureCloud;
class Crown;
class ChildCylinderExtraction;
namespace simpleTree
{
      

  class Tree
  {

    private:

       boost::shared_ptr<ChildCylinderExtraction> child_Cylinder_Extraction;

      static   bool
      compareChildren(boost::shared_ptr< Segment > child1, boost::shared_ptr< Segment > child2);
       float solidRadius = 0.035f;
       float octreeResolution = 0.02f;
       float minVolumeForFirstCrownBranch =  0.001f;
       float minNumberOfFirstCrownBranches = 2;
       float minSearchRadius =  0.04f;
       float minPtsForCylinderImprovement =  3;
       float radius = std::numeric_limits<float>::min();


      boost::shared_ptr<Segment> topStemSegment;
      boost::shared_ptr<Segment> rootSegment;
      boost::shared_ptr<Segment> firstCrownSegment;

      std::vector<Cylinder> cylinders;
      pcl::PointCloud<PointI>::Ptr cloud_Ptr;
      pcl::PointCloud<PointI>::Ptr
      pointsNearCylinder (pcl::octree::OctreePointCloudSearch<PointI> &octree,
                          boost::shared_ptr<Cylinder> &cylinder,
                          float factorEnLarge = 1);
      std::vector<int>
      indexOfPointsNearCylinder (pcl::octree::OctreePointCloudSearch<PointI> &octree,
                                 boost::shared_ptr<Cylinder> &cylinder,
                                 float factorEnLarge = 1);
      int SACMETHOD_TYPE;
      pcl::ModelCoefficients::Ptr
      doRANSAC (pcl::PointCloud<PointI>::Ptr cylinderPoints);
      void
      improveWithRANSACCoefficients (pcl::ModelCoefficients::Ptr coeff,
                                     boost::shared_ptr<Cylinder> cylinder);
      void
      improveWithMedianCoefficients (pcl::PointCloud<PointI>::Ptr cylinderPoints,
                                     boost::shared_ptr<Cylinder> cylinder);
      void
      projectStartAndEndPoint (pcl::PointXYZ &start,
                               pcl::PointXYZ &end,
                               pcl::ModelCoefficients::Ptr &coeff);

      pcl::PointCloud<pcl::PointXYZ>::Ptr
      crownPoints ();
      void
      improveBranchJunctions ();
      void
      improveByMedianCheck ();

      void
      closeGaps();
      void
      setBranchOrder(boost::shared_ptr<Segment> segment,float ID = 0);
      void
      setBranchID(boost::shared_ptr<Segment> segment,float ID = 0);
      void
      setSegmentID();
      void
      setIDforSegments();

    public:
      void
      improveFit ();

      void
      resetRoot();
      void
      reorderTree();
      void
      reset_crown(float h);
      void
      reset_stem();
      void
      reorderChildren(Segment segment);
      
      boost::shared_ptr<Segment>
      getLeaveSegmentWithMostVolume(std::vector<boost::shared_ptr<Segment> > leaves,boost::shared_ptr<Segment> base);
      
      float
      getVolumeToLeave(boost::shared_ptr<Segment> leave, boost::shared_ptr<Segment> base);
      
      

      
      std::vector<boost::shared_ptr<Segment> >
      getLeaveSegmentsFromSegment(boost::shared_ptr<Segment> base);
      
      std::vector<boost::shared_ptr<Segment> >
      getSegmentsBetweenLeaveAndSegment(boost::shared_ptr<Segment> leave, boost::shared_ptr<Segment> base);
      
//       std::vector<boost::shared_ptr<Segment> >
//       getChildGenerationSegments
      
      std::string
      string ();
      boost::shared_ptr<Segment>
      getFirstCrownSegment ();
      boost::shared_ptr<Segment>
      getRootSegment ();
      boost::shared_ptr<Segment>
      getStemTopSegment ();
      float
      getHeightAboveGround ();
      float
      getGrowthVolume (boost::shared_ptr<Cylinder> cylinder);
      float
      getVolumeToRoot (boost::shared_ptr<Segment> segment);
      float
      getVolumeToSegment (boost::shared_ptr<Segment> leave,
                          boost::shared_ptr<Segment> segment);
      std::vector<float>
      distancesToModel ();
      boost::shared_ptr<Crown> crown;
      std::string name;
      Tree (std::vector<pcl::ModelCoefficients> cylinders,
            pcl::PointCloud<PointI>::Ptr cloud_Ptr, std::string name, bool computeCrown = true);
      Tree ();
      virtual
      ~Tree ();
      void
      setCylinders ();
      boost::shared_ptr<Cylinder>
      getRootCylinder ();

      void
      setCylinders (std::vector<pcl::ModelCoefficients> coeff_cylinders);
      std::vector<boost::shared_ptr<Segment> >
      getSegments ();
      std::vector<boost::shared_ptr<Segment> >
      getChildGenerationSegments (boost::shared_ptr<Segment> segment);
      std::vector<boost::shared_ptr<Cylinder> >
      getCylinders (bool removeOneLengthLeaveSegments = false);

      void
      mergeCylinders (int numberIterations = 1);
      std::vector<boost::shared_ptr<Segment> >
      getLeaveSegments ();
      void
      detectStem ();
      void
      detectCrown ();
      float
      getVolume ();
      float
      getStemVolume ();
      float
      getLength ();
      float
      getHeight ();
      float
      getSolidVolume ();
      float
      getDBH ();
      float
      getBaseDiameter ();
      float
      getRootSegmentVolume ();
      std::vector<boost::shared_ptr<Cylinder> >
      getStemCylinders ();


    protected:

      void
      buildTree (boost::shared_ptr<Segment> currentSegment,
                 boost::shared_ptr<Cylinder> currentCylinder);
  };

} /* namespace simpleTree */

#endif /* TREE_H_ */
