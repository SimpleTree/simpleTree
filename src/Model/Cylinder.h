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

#ifndef CYLINDER_H_
#define CYLINDER_H_

#include <pcl/ModelCoefficients.h>
#include "Segment.h"
#include <assert.h>
#include <math.h>
#include <vector>

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/math/constants/constants.hpp>
#endif



#include <pcl/point_types.h>
#include <pcl/common/distances.h>
#include <pcl/common/common.h>
#include <pcl/filters/project_inliers.h>
typedef pcl::PointXYZINormal PointI;
namespace simpleTree
{
  class Segment;
  class Cylinder : public pcl::ModelCoefficients
  {

    private:
      boost::weak_ptr<Segment> segment;
      int depth = -1;
      float minDist = 0.00001;

    public:
      int
      getDepth ();
      void
      setDepth (int depth);
      Cylinder (pcl::ModelCoefficients);
      virtual
      ~Cylinder ();

      float
      getLength ();
      float
      getRadius ();
      float
      getVolume ();
      float
      getBoundingSphereRadius ();
      PointI
      centerPoint ();

      std::vector<float>
      getValues ();

      void
      setSegment (boost::weak_ptr<Segment> seg);
      boost::shared_ptr<Segment>
      getSegment ();

      bool
      operator == (const Cylinder& cylinder2);
      bool
      isParentOf (const Cylinder& child);
      bool
      isChildOf (const Cylinder& parent);
      std::vector<Cylinder>
      getChildren (std::vector<Cylinder>& allCylinders);
      float
      distToPoint (PointI point);
      pcl::PointXYZ
      getStart ();
      pcl::PointXYZ
      getEnd ();
      std::string
      toString ();

  };

} /* namespace simpleTree */

#endif /* CYLINDER_H_ */
