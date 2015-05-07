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
#ifndef SEGMENT_H_
#define SEGMENT_H_
#include <vector>

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <qstring.h>
#endif



#include <sstream>
#include <pcl/point_types.h>
#include "Cylinder.h"
typedef pcl::PointXYZINormal PointI;
namespace simpleTree
{
  class Cylinder;
  class Segment : public boost::enable_shared_from_this<Segment>
  {

    private:
      boost::weak_ptr<Segment> parent;
      std::vector<boost::shared_ptr<Segment> > children;
      std::vector<boost::shared_ptr<Cylinder> > cylinders;
      int depth = -1;

      float
      getLength ();



    public:
      void
      setChildren(std::vector<boost::shared_ptr<Segment> > children);
      int
      segmentID;
      int
      branchID;
      int
      branchOrder;
      float
      getDepth ();
      void
      setDepth (float depth);
      Segment ();
      virtual
      ~Segment ();
      std::vector<boost::shared_ptr<Cylinder> >&
      getCylinders ();
      void
      setCylinders (std::vector<boost::shared_ptr<Cylinder> >);
      std::vector<boost::shared_ptr<Segment> >&
      getChildren ();
      boost::shared_ptr<Segment>
      getParent ();
      void
      connectWithParent (boost::shared_ptr<Segment> parent);
      void
      addCylinder (Cylinder& cylinder);
      float
      getVolume ();
      bool
      isLeave ();
      bool
      isRoot ();
      void
      correctBranchingSection ();
      void
      correctRadiusByMedianCheck ();
      QString
      toString ();
      float
      medianRadius ();
      pcl::PointXYZ
      getStart ();
      pcl::PointXYZ
      getEnd ();
  };

} /* namespace simpleTree */

#endif /* SEGMENT_H_ */
