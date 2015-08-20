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
#define PCL_NO_PRECOMPILE
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
      /** \brief A weak pointer to the segment containing this cylinder
       * */
      boost::weak_ptr<Segment> segment;

      /** \brief The segment depth where the cylinder is stored in, -1 if not computed
       * */
      int depth = -1;

      /** \brief The minimum distance up to where to points are conisdered the same
       * */
      float minDist = 0.00001;


      /** \brief projects a point to the cylinder axis
       * \param p: the point to project
       * */
      PointI
      projectToAxis(const PointI& p);

      /** \brief Returns the distance along the cylinder axis, 0 if point's projection between start and end
       * \param p: the point for which the distance should be computed
       * */
      float
      getDistOnAxis(const PointI& p);

    public:
      /** \brief returns the depth
       * */
      int
      getDepth ();

      /** \brief sets the depth
       * \param depth: the depth of the cylinder
       * */
      void
      setDepth (int depth);

      /** \brief Default contrstructor
       * \param coeff: pcl ModelCoefficients of a cylinder
       * */
      Cylinder (pcl::ModelCoefficients coeff);

      /** \brief Default destructor
       * */
      virtual
      ~Cylinder ();

      /** \brief returns the length of the cylinder
       * */
      float
      getLength ();

      /** \brief returns the radius of the cylinder
       * */
      float
      getRadius ();

      /** \brief returns the volume of the cylinder
       * */
      float
      getVolume ();

      /** \brief returns the maximum distance between a point contained in the cylinder and the cylinders centerpoint
       * */
      float
      getHalfSize();

      /** \brief returns the cylinders center point
       * */
      PointI
      getCenterPoint ();

      /** \brief returns the pcl modelcoefficient values of this cylinder
       * */
      std::vector<float>
      getValues ();

      /** \brief sets a weak pointer to the segment containing this cylinder
       * */
      void
      setSegment (boost::weak_ptr<Segment> seg);

      /** \brief returns a shared pointer to the segment containing this cylinder
       * */
      boost::shared_ptr<Segment>
      getSegment ();

      /** \brief returns true if cylinders start and end points are within the defined minimum distance
       * */
      bool
      operator == (const Cylinder& cylinder2);

      /** \brief returns true if the childs start point is with the defined minimum distance to the cylinders end point
       * */
      bool
      isParentOf (const Cylinder& child);

      /** \brief returns true if the parents end point is with the defined minimum distance to the cylinders start point
       * */
      bool
      isChildOf (const Cylinder& parent);

      /** \brief returns the children of this cylinder
       * */
      std::vector<Cylinder>
      getChildren (std::vector<Cylinder>& cylinders);

      /** \brief returns the distance to a point
       * \param point: the point to which the distance is computed
       * */
      float
      distToPoint (PointI point);

      /** \brief
       * \return the cylinders start point
       * */
      pcl::PointXYZ
      getStart ();

      /** \brief
       * \return the cylinders end point
       * */
      pcl::PointXYZ
      getEnd ();

      /** \brief
       * \return A QString of form (Start.x,start.y,start.z,end.x,end.y,end.z,volume,length,radius)
       * */
      QString
      toString ();

  };

} /* namespace simpleTree */

#endif /* CYLINDER_H_ */
