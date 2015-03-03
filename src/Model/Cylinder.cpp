/*
 * Copyright (c) 2014, Jan Hackenberg jan.hackenberg@iww.uni-freiburg.de
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY Jan Hackenberg jan.hackenberg@iww.uni-freiburg.de ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Jan Hackenberg jan.hackenberg@iww.uni-freiburg.de BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include "Cylinder.h"

namespace simpleTree
{

  Cylinder::Cylinder (pcl::ModelCoefficients coef)
  {

    assert(coef.values.size () == 7);
    this->values = coef.values;

  }

  Cylinder::~Cylinder ()
  {
    // TODO Auto-generated destructor stub
  }

  int
  Cylinder::getDepth ()
  {
    return this->depth;
  }

  void
  Cylinder::setDepth (int depth)
  {
    this->depth = depth;
  }

  std::string
  Cylinder::toString ()
  {
    std::string str = "";
    std::ostringstream stream;
    pcl::PointXYZ start = getStart ();
    pcl::PointXYZ end = getEnd ();
    stream << start.x << " , " << start.y << " , " << start.z;
    stream << " , " << end.x << " , " << end.y << " , " << end.z;
    stream << " , " << getVolume ();
    stream << " , " << getRadius ();
    stream << " , " << getLength ();
    return stream.str ();
  }

  pcl::PointXYZ
  Cylinder::getStart ()
  {
    pcl::PointXYZ start (values[0], values[1], values[2]);
    return start;
  }

  pcl::PointXYZ
  Cylinder::getEnd ()
  {
    pcl::PointXYZ start (values[0] + values[3], values[1] + values[4], values[2] + values[5]);
    return start;
  }

  float
  Cylinder::getBoundingSphereRadius ()
  {
    float r = getRadius ();  //length and radius are in centimeter
    float x = getLength () / 2;
    return (sqrt (r * r + x * x));
  }

  PointI
  Cylinder::centerPoint ()
  {
    PointI center;
    center.x = values[0] + (values[3] / 2);
    center.y = values[1] + (values[4] / 2);
    center.z = values[2] + (values[5] / 2);
    return center;
  }

  float
  Cylinder::getLength ()
  {
    float length = 0;
    float x = this->values[3];
    float y = this->values[4];
    float z = this->values[5];
    length = sqrt (x * x + y * y + z * z);
    return (length);
  }

  float
  Cylinder::getRadius ()
  {
    return (this->values[6]);
  }

  float
  Cylinder::getVolume ()
  {
    const float pi = boost::math::constants::pi<float> ();
    float volume = getLength () * getRadius () * getRadius () * pi;
    return volume;
  }

  void
  Cylinder::setSegment (boost::weak_ptr<Segment> seg)
  {
    segment = seg;
  }

  std::vector<float>
  Cylinder::getValues ()
  {
    return this->values;
  }

  boost::shared_ptr<Segment>
  Cylinder::getSegment ()
  {
    return segment.lock ();
  }

  bool
  Cylinder::operator == (const Cylinder& cylinder2)
  {
    return ( (values[0] == cylinder2.values[0]) && (values[1] == cylinder2.values[1]) && (values[2] == cylinder2.values[2])
        && (values[3] == cylinder2.values[3]) && (values[4] == cylinder2.values[4]) && (values[5] == cylinder2.values[5]) && (values[6] == cylinder2.values[6]));
  }

//   bool
//   Cylinder::isChildOf (const Cylinder& parent)
//   {
//     return ( (values[0] == (parent.values[0] + parent.values[3])) && (values[1] == (parent.values[1] + parent.values[4]))
//         && (values[2] == (parent.values[2] + parent.values[5])));
//   }
// 
//   bool
//   Cylinder::isParentOf (const Cylinder& child)
//   {
//     return ( ( (values[0] + values[3]) == child.values[0]) && ( (values[1] + values[4]) == child.values[1]) && ( (values[2] + values[5]) == child.values[2]));
//   }

  bool
  Cylinder::isChildOf (const Cylinder& parent)
  {
    return (std::abs ( (values[0] - (parent.values[0] + parent.values[3])) < minDist)
        && std::abs ( (values[1] - (parent.values[1] + parent.values[4])) < minDist)
        && std::abs ( (values[2] - (parent.values[2] + parent.values[5])) < minDist));
  }

  bool
  Cylinder::isParentOf (const Cylinder& child)
  {
    return ( (std::abs ( (values[0] + values[3]) - child.values[0]) < minDist) && (std::abs ( (values[1] + values[4]) - child.values[1]) < minDist)
        && (std::abs ( (values[2] + values[5]) - child.values[2]) < minDist));
  }

  std::vector<Cylinder>
  Cylinder::getChildren (std::vector<Cylinder>& cylinders)
  {
    std::vector<Cylinder> children;
    for (std::vector<Cylinder>::iterator it = cylinders.begin (); it != cylinders.end (); it++)
    {
      Cylinder cylinder = *it;
      if (isParentOf (cylinder))
      {
        children.push_back (cylinder);
      }

    }
    return children;
  }

  float
  Cylinder::distToPoint (PointI point)
  {
    Eigen::Vector4f pt (point.x, point.y, point.z, 0);
    Eigen::Vector4f pointOnLine (values[0], values[1], values[2], 0);
    Eigen::Vector4f axis (values[3], values[4], values[5], 0);
    float dist = sqrt (pcl::sqrPointToLineDistance (pt, pointOnLine, axis));

    pcl::ModelCoefficients::Ptr coefficients_line (new pcl::ModelCoefficients);
    coefficients_line->values.push_back (values[0]);
    coefficients_line->values.push_back (values[1]);
    coefficients_line->values.push_back (values[2]);
    coefficients_line->values.push_back (values[3]);
    coefficients_line->values.push_back (values[4]);
    coefficients_line->values.push_back (values[5]);

    pcl::PointCloud<PointI>::Ptr cloud_projected (new pcl::PointCloud<PointI>);
    pcl::PointCloud<PointI>::Ptr cloud_toProject (new pcl::PointCloud<PointI>);

    cloud_toProject->push_back (point);

    // Create the filtering object
    pcl::ProjectInliers<PointI> proj;
    proj.setModelType (pcl::SACMODEL_LINE);
    proj.setInputCloud (cloud_toProject);
    proj.setModelCoefficients (coefficients_line);
    proj.filter (*cloud_projected);
    PointI point_proj = cloud_projected->points[0];

    PointI start;
    start.x = values[0];
    start.y = values[1];
    start.z = values[2];
    PointI end;
    end.x = values[0] + values[3];
    end.y = values[1] + values[4];
    end.z = values[2] + values[5];

    float distStartToEnd = pcl::euclideanDistance<PointI, PointI> (start, end);
    float distToStart = pcl::euclideanDistance<PointI, PointI> (start, point_proj);
    float distToEnd = pcl::euclideanDistance<PointI, PointI> (end, point_proj);

    float distOnAxis = 0;
    if (distStartToEnd > distToStart && distStartToEnd > distToEnd)
    {
      distOnAxis = 0;
    }
    else
    {
      distOnAxis = std::min<float> (distToEnd, distToStart);
    }
    float multiplier = 1;
    if (dist < values[6])
    {
      multiplier = -1;
    }
    float distToHull = dist - values[6];

    float distToCylinder = sqrt (distToHull * distToHull + distOnAxis * distOnAxis);
    return (distToCylinder * multiplier);

  }

} /* namespace simpleTree */
