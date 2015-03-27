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
#include "Segment.h"

namespace simpleTree
{

// Segment::Segment ( boost::weak_ptr<Segment> parentSegment)
// {
// 	std::cout<<"constructor";
//         parent = parentSegment;
// }

  
    void 
    Segment::setChildren ( std::vector< boost::shared_ptr<Segment > >children ) {
	this->children = children;
    }

  Segment::Segment ()
  {
    boost::shared_ptr<Segment> par = getParent ();
    boost::shared_ptr<Segment> nullPtr;
    par = nullPtr;
    this->segmentID = 0;
    this->branchID = 0;
    this->branchOrder = 0;
  }

  Segment::~Segment ()
  {
    // TODO Auto-generated destructor stub
  }

  std::string
  Segment::toString ()
  {
    std::string str = "";
    std::ostringstream stream;
    pcl::PointXYZ start = getStart ();
    pcl::PointXYZ end = getEnd ();
    stream << start.x << " , " << start.y << " , " << start.z;
    stream << " , " << end.x << " , " << end.y << " , " << end.z;
    stream << " , " << getVolume ();
    stream << " , " << medianRadius ();
    stream << " , " << getLength () << "\n";
    return stream.str ();
  }

  pcl::PointXYZ
  Segment::getStart ()
  {
    boost::shared_ptr<Cylinder> startCylinder = * (cylinders.begin ());
    std::vector<float> coeff = startCylinder->getValues ();
    pcl::PointXYZ start (coeff[0], coeff[1], coeff[2]);
    return start;
  }

  pcl::PointXYZ
  Segment::getEnd ()
  {
    boost::shared_ptr<Cylinder> endCylinder = (cylinders.back ());
    std::vector<float> coeff = endCylinder->getValues ();
    pcl::PointXYZ end (coeff[0] + coeff[3], coeff[1] + coeff[4], coeff[2] + coeff[5]);
    return end;
  }

  float
  Segment::getLength ()
  {
    float length = 0;
    for (std::vector<boost::shared_ptr<Cylinder> >::iterator it = cylinders.begin (); it != cylinders.end (); it++)
    {
      boost::shared_ptr<Cylinder> cylinder = *it;
      length += cylinder->getLength ();
    }
    return length;
  }

  float
  Segment::medianRadius ()
  {
    std::vector<float> radii;
    for (std::vector<boost::shared_ptr<Cylinder> >::iterator it = cylinders.begin (); it != cylinders.end (); it++)
    {
      boost::shared_ptr<Cylinder> cylinder = *it;
      radii.push_back (cylinder->getRadius ());
    }
    std::nth_element (radii.begin (), radii.begin () + (radii.size () >> 1), radii.end ());
    float median = (radii[radii.size () >> 1]);
    return (median);
  }

  void
  Segment::correctRadiusByMedianCheck ()
  {
    float median = medianRadius ();
    for (std::vector<boost::shared_ptr<Cylinder> >::iterator it = cylinders.begin (); it != cylinders.end (); it++)
    {
      boost::shared_ptr<Cylinder> cylinder = *it;
      if ( (cylinder->getRadius () < (0.8 * median)) || (cylinder->getRadius () > (1.2 * median)))
      {
        cylinder->values[6] = median;
      }
    }
  }

  float
  Segment::getDepth ()
  {
    return this->depth;
  }

  void
  Segment::setDepth (float depth)
  {
    this->depth = depth;
    for (size_t i = 0; i < getCylinders ().size (); i++)
    {
      boost::shared_ptr<Cylinder> cyl = getCylinders ()[i];
      cyl->setDepth (depth);
    }
  }

  bool
  Segment::isRoot ()
  {
    if (getParent ())
    {
      return false;
    }
    return true;
  }

  bool
  Segment::isLeave ()
  {
    if (getChildren ().size () == 0)
    {
      return true;
    }
    return false;
  }

  float
  Segment::getVolume ()
  {
    float volume = 0;
    for (std::vector<boost::shared_ptr<Cylinder> >::iterator it = cylinders.begin (); it != cylinders.end (); it++)
    {
      boost::shared_ptr<Cylinder> cylinder = *it;
      volume += cylinder->getVolume ();
    }
    return volume;
  }

  void
  Segment::correctBranchingSection ()
  {
    if (cylinders.size () > 1)
    {
      boost::shared_ptr<Cylinder> first = cylinders.at (0);
      boost::shared_ptr<Cylinder> second = cylinders.at (1);
      first->values.clear ();
      first->values.push_back (second->values[0] - second->values[3]);
      first->values.push_back (second->values[1] - second->values[4]);
      first->values.push_back (second->values[2] - second->values[5]);
      first->values.push_back (second->values[3]);
      first->values.push_back (second->values[4]);
      first->values.push_back (second->values[5]);
      first->values.push_back (second->values[6]);
    }
  }

  void
  Segment::setCylinders (std::vector<boost::shared_ptr<Cylinder> > cylinders_in)
  {
    cylinders = cylinders_in;
    setDepth (this->depth);
  }

  std::vector<boost::shared_ptr<Cylinder> >&
  Segment::getCylinders ()
  {
    return cylinders;
  }

  std::vector<boost::shared_ptr<Segment> >&
  Segment::getChildren ()
  {
    return children;
  }

  boost::shared_ptr<Segment>
  Segment::getParent ()
  {
    return parent.lock ();
  }

  void
  Segment::connectWithParent (boost::shared_ptr<Segment> parent)
  {
    this->parent = parent;
    //boost::shared_ptr<Segment> par = this->parent.lock();
    parent->getChildren ().push_back (shared_from_this ());
  }

  void
  Segment::addCylinder (Cylinder& cylinder)
  {
    cylinder.setDepth (this->depth);
    cylinders.push_back (boost::make_shared<Cylinder> (cylinder));
    cylinders.back ()->setSegment (shared_from_this ());
  }

} /* namespace simpleTree */

