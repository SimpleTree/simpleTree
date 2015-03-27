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

#include "exportply.h"

ExportPly::ExportPly (std::vector<boost::shared_ptr<simpleTree::Cylinder> > cylinders,
                      std::string cloudName,
                      std::string type)
{
  this->cylinders = cylinders;
  this->fileName = cloudName;
  this->type = type;
  writePly ();
}

void
ExportPly::writePly ()
{
  std::ofstream myfile;
  std::ostringstream stream;
  stream << this->path << this->fileName << "_" << this->type << ".ply";

  std::string str = stream.str ();

  char *a = new char[str.size () + 1];
  a[str.size ()] = 0;
  memcpy (a, str.c_str (), str.size ());

  myfile.open (a);

  myfile << "ply" << std::endl;
  myfile << "format ascii 1.0" << std::endl;
  myfile << "comment author : Jan Hackenberg" << std::endl;
  int vertexCount = cylinders.size () * 16;
  myfile << "element vertex " << vertexCount << std::endl;
  myfile << "property float x" << std::endl;
  myfile << "property float y" << std::endl;
  myfile << "property float z" << std::endl;
  myfile << "property uchar red  " << std::endl;
  myfile << "property uchar green" << std::endl;
  myfile << "property uchar blue" << std::endl;
  int faceCount = cylinders.size () * 16;
  myfile << "element face " << faceCount << std::endl;
  myfile << "property list uchar int vertex_index" << std::endl;
  myfile << "end_header" << std::endl;

  for (size_t i = 0; i < cylinders.size (); i++)
  {
    /**
     * Creating points;
     */
    boost::shared_ptr<simpleTree::Cylinder> cyl = cylinders[i];
    //Cylinder3D cyl = cylinders.get(i);
    float height = cyl->getLength ();
    Eigen::Vector3f zAxis (0, 0, 1);
    zAxis.normalize ();
    Eigen::Vector3f axis (cyl->values[3], cyl->values[4], cyl->values[5]);
    axis.normalize ();
    Eigen::Vector3f rotationAxis = zAxis.cross (axis);
    float rotationAngle = acos (zAxis.dot (axis));

    std::vector<Eigen::Vector3f> points;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> cloud_transformed;
    for (int j = 0; j < 8; j++)
    {
      float x = cos (boost::math::constants::pi<float> () / 180 * 45 * j) * cyl->values[6];
      float y = sin (boost::math::constants::pi<float> () / 180 * 45 * j) * cyl->values[6];
      float z = height / 2;
      Eigen::Vector3f pt (x, y, z);
      points.push_back (pt);
      pcl::PointXYZ p (x, y, z);
      cloud.push_back (p);
    }
    for (int j = 0; j < 8; j++)
    {
      float x = cos (boost::math::constants::pi<float> () / 180 * 45 * j) * cyl->values[6];
      float y = sin (boost::math::constants::pi<float> () / 180 * 45 * j) * cyl->values[6];
      float z = -height / 2;
      Eigen::Vector3f pt (x, y, z);
      points.push_back (pt);
      pcl::PointXYZ p (x, y, z);
      cloud.push_back (p);
    }
    Eigen::Affine3f transform = Eigen::Affine3f::Identity ();
    transform.translation () << (cyl->values[0] + cyl->values[3] / 2), (cyl->values[1] + cyl->values[4] / 2), (cyl->values[2] + cyl->values[5] / 2);
    transform.rotate (Eigen::AngleAxis<float> (rotationAngle, rotationAxis));
    pcl::transformPointCloud<pcl::PointXYZ> (cloud, cloud_transformed, transform);

    for (size_t j = 0; j < cloud_transformed.size (); j++)
    {
      pcl::PointXYZ p = cloud_transformed.points[j];
      myfile << std::setprecision (10) << p.x << " " << p.y << " " << p.z << " 204 0 55" << std::endl;
    }

  }
  for (size_t i = 0; i < cylinders.size (); i++)
  {
    /**
     * Writing the polygons
     */
    myfile << "3 " << ( (i * 16) + 0) << " " << ( (i * 16) + 1) << " " << ( (i * 16) + 8) << std::endl;

    myfile << "3 " << ( (i * 16) + 9) << " " << ( (i * 16) + 8) << " " << ( (i * 16) + 1) << std::endl;

    myfile << "3 " << ( (i * 16) + 1) << " " << ( (i * 16) + 2) << " " << ( (i * 16) + 9) << std::endl;

    myfile << "3 " << ( (i * 16) + 10) << " " << ( (i * 16) + 9) << " " << ( (i * 16) + 2) << std::endl;

    myfile << "3 " << ( (i * 16) + 2) << " " << ( (i * 16) + 3) << " " << ( (i * 16) + 10) << std::endl;

    myfile << "3 " << ( (i * 16) + 11) << " " << ( (i * 16) + 10) << " " << ( (i * 16) + 3) << std::endl;

    myfile << "3 " << ( (i * 16) + 3) << " " << ( (i * 16) + 4) << " " << ( (i * 16) + 11) << std::endl;

    myfile << "3 " << ( (i * 16) + 12) << " " << ( (i * 16) + 11) << " " << ( (i * 16) + 4) << std::endl;

    myfile << "3 " << ( (i * 16) + 4) << " " << ( (i * 16) + 5) << " " << ( (i * 16) + 12) << std::endl;

    myfile << "3 " << ( (i * 16) + 13) << " " << ( (i * 16) + 12) << " " << ( (i * 16) + 5) << std::endl;

    myfile << "3 " << ( (i * 16) + 5) << " " << ( (i * 16) + 6) << " " << ( (i * 16) + 13) << std::endl;

    myfile << "3 " << ( (i * 16) + 14) << " " << ( (i * 16) + 13) << " " << ( (i * 16) + 6) << std::endl;

    myfile << "3 " << ( (i * 16) + 6) << " " << ( (i * 16) + 7) << " " << ( (i * 16) + 14) << std::endl;

    myfile << "3 " << ( (i * 16) + 15) << " " << ( (i * 16) + 14) << " " << ( (i * 16) + 7) << std::endl;

    myfile << "3 " << ( (i * 16) + 7) << " " << ( (i * 16) + 0) << " " << ( (i * 16) + 15) << std::endl;

    myfile << "3 " << ( (i * 16) + 8) << " " << ( (i * 16) + 15) << " " << ( (i * 16) + 0) << std::endl;

  }
  myfile.close ();
}

ExportPly::ExportPly (const ExportPly& other)
{

}

ExportPly::~ExportPly ()
{

}
