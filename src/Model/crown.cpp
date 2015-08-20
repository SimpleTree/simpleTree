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
#include "crown.h"

QMutex
Crown::lock;

Crown::Crown (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
     QMutexLocker locker(&lock);
    if(cloud != 0 && cloud->points.size()>4)
    {
  this->cloud = cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr hull_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ConvexHull<pcl::PointXYZ> hull;
  hull.setInputCloud (this->cloud);
  hull.setComputeAreaVolume (true);
  hull.reconstruct (*hull_cloud, polygons);
  volume = hull.getTotalVolume ();
  area = hull.getTotalArea ();
  this->hull_cloud = hull_cloud;
  QString str;
    str.append("The volume of the crown is ").append(
            QString::number(volume)).append(" in m^3, the crown surface area is ").append(
                    QString::number(area)).append("m^2.\n");
    QCoreApplication::processEvents();


  pcl::PointCloud<pcl::PointXYZ>::Ptr hull_cloud_concave (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ConcaveHull<pcl::PointXYZ> hull_concave;
  hull_concave.setInputCloud (this->cloud);
  hull_concave.setAlpha (0.2);
  hull_concave.reconstruct (*hull_cloud_concave, polygons_concave);

  this->hull_cloud_concave = hull_cloud_concave;

  //

  pcl::PointCloud<pcl::PointXYZ>::Ptr proj_hull_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr proj_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  for (size_t i = 0; i < cloud->size (); i++)
  {
    pcl::PointXYZ p = cloud->points[i];
    pcl::PointXYZ proj (p.x, p.y, 0);
    proj_cloud->push_back (proj);
  }

  pcl::ConvexHull<pcl::PointXYZ> hull2;
  hull2.setInputCloud (proj_cloud);
  hull2.setComputeAreaVolume (true);
  hull2.reconstruct (*proj_hull_cloud, proj_polygons);
  crownProjectionArea = hull2.getTotalArea ();
  str = "";
  str.append("The crown projection area is ").append(
                QString::number(crownProjectionArea)).append("m^2.\n");
        QCoreApplication::processEvents();
    }
}



void
Crown::reset_concave_hull(float r)
{
         QMutexLocker locker(&lock);
    if(cloud != 0 && cloud->points.size()>4)
    {
    pcl::PointCloud<pcl::PointXYZ>::Ptr hull_cloud_concave (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConcaveHull<pcl::PointXYZ> hull_concave;
    hull_concave.setInputCloud (this->cloud);
    hull_concave.setAlpha (r);
    hull_concave.reconstruct (*hull_cloud_concave, polygons_concave);

    this->hull_cloud_concave = hull_cloud_concave;
    }
}


Crown::~Crown ()
{

}
