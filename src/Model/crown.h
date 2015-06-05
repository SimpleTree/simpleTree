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
#ifndef CROWN_H
#define CROWN_H
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include "../controller.h"

#include <QMutexLocker>
#include <QMutex>

typedef pcl::PointXYZINormal PointI;
class Controller;
class Crown
{
  public:
        QMutex lock;
    float volume;
    float area;
    float crownProjectionArea;

    std::vector<pcl::Vertices> polygons;
    std::vector<pcl::Vertices> polygons_concave;
    std::vector<pcl::Vertices> proj_polygons;

    void
    reset_concave_hull(float r);

    pcl::PointCloud<pcl::PointXYZ>::Ptr proj_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr proj_hull_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr hull_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr hull_cloud_concave;
    Crown (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,boost::weak_ptr<Controller> control);
    ~Crown ();
  private:
    boost::weak_ptr<Controller> control;
           boost::shared_ptr<Controller>
               getControl ();
};

#endif // CROWN_H
