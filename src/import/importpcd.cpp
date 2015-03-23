/*
 * Copyright (c) 2015, Jan Hackenberg
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
 * THIS SOFTWARE IS PROVIDED BY Jan Hackenberg ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Jan Hackenberg BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "importpcd.h"

ImportPCD::ImportPCD (std::string fileName,
                      boost::weak_ptr<Controller> control)
{

  this->control = control;
  this->path = fileName;
  if (fileName.length () > 4)
  {
    std::string ext = fileName.substr (fileName.length () - 3, 3);
    if (ext == "pcd")
    {
      getControl ()->getGuiPtr ()->updateProgress (0);
      QCoreApplication::processEvents ();
      cloud_intens = importPCD ();
//      getControl ()->getGuiPtr ()->updateProgress (30);
//      QCoreApplication::processEvents ();
//      computeNormals (cloud_intens);
//      getControl ()->getGuiPtr ()->updateProgress (40);
//      QCoreApplication::processEvents ();
//      principalCurvatures = computeCurvature ();
//      getControl ()->getGuiPtr ()->updateProgress (70);
//      QCoreApplication::processEvents ();
//      std::vector<float> e1;
//      std::vector<float> e2;
//      std::vector<float> e3;
//
//      EigenValueEstimator es (cloud_intens, e1, e2, e3, 0.035f);
//      getControl()->setE1(e1);
//      getControl()->setE2(e2);
//      getControl()->setE3(e3);
      getControl ()->getGuiPtr ()->updateProgress (100);
      QCoreApplication::processEvents ();

//       setRGB ();
    }
    else if (ext == "asc"||ext == "txt")
    {
      getControl ()->getGuiPtr ()->updateProgress (0);
      QCoreApplication::processEvents ();
      cloud_intens = importASC ();
//      getControl ()->getGuiPtr ()->updateProgress (30);
//      QCoreApplication::processEvents ();
//      computeNormals (cloud_intens);
//      getControl ()->getGuiPtr ()->updateProgress (40);
//      QCoreApplication::processEvents ();
//      principalCurvatures = computeCurvature ();
//      getControl ()->getGuiPtr ()->updateProgress (70);
//      QCoreApplication::processEvents ();
//      std::vector<float> e1;
//            std::vector<float> e2;
//            std::vector<float> e3;
//
//            EigenValueEstimator es (cloud_intens, e1, e2, e3, 0.035f);
//            getControl()->setE1(e1);
//            getControl()->setE2(e2);
//            getControl()->setE3(e3);
            getControl ()->getGuiPtr ()->updateProgress (100);
            QCoreApplication::processEvents ();
    }
  }
}

boost::shared_ptr<Controller>
ImportPCD::getControl ()
{
  return this->control.lock ();
}

ImportPCD::~ImportPCD ()
{
  // TODO Auto-generated destructor stub
}

// void
// ImportPCD::init ()
// {
//   path = "../data/" + fileName;
// }

// void
// ImportPCD::setRGB ()
// {
//   for (int i = 0; i < cloud_intens->points.size (); ++i)
//   {
//     cloud_intens->points[i].r = (cloud_intens->points[i].curvature * 255 * 20);
//     cloud_intens->points[i].g = 255 - (cloud_intens->points[i].curvature * 255 * 20);
//     cloud_intens->points[i].b = 0;  //cloudRGB->points[i].curvature*255;
//   }
// }

pcl::PointCloud<PointI>::Ptr
ImportPCD::importASC ()
{
//   std::cout << "importASC\n";
//   tt.tic ();
//   pcl::console::setVerbosityLevel (pcl::console::L_ALWAYS);
//   PointCloudI::Ptr temp (new PointCloudI);
// //   QFile file(QString::fromStdString(path));
//   QFile file ("/home/hackenberg/simple/data/Q6-d.asc");
//   QTextStream in(&file);
//   while(!in.atEnd()){
//     std::cout << "schleife\n";
//     QString line = in.readLine();
//     QStringList fields = line.split(" ");
//     PointI p = toPoint(fields);
//     temp->push_back(p);
//   }
//    getControl()->getGuiPtr()->writeConsole("--------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
//     QString str;
//     float f = tt.toc()/1000;
//     str.append("Imported ").append(QString::fromStdString(getControl()->getTreeID())).append(" with ").append(QString::number(temp->points.size())).append(" points in  ").append(QString::number(f)).append(" seconds.\n");
//     getControl()->getGuiPtr()->writeConsole(str);

 // std::cout << "importASC\n";
  tt.tic ();
  pcl::console::setVerbosityLevel (pcl::console::L_ALWAYS);
  PointCloudI::Ptr temp (new PointCloudI);
//   QFile file(QString::fromStdString(path));
  std::ifstream file (path.c_str ());
  std::string line;
  if (file.is_open ())
  {
    while (std::getline (file, line))
    {
      QString qLine = QString::fromStdString (line);
      //std::cout << qLine.toStdString();
      qLine.replace(QRegExp("[\\s]+"), " ");
      //std::cout << qLine.toStdString();
      if(qLine.endsWith(" "))
      {
          qLine.chop(1);
      }
      QStringList fields = qLine.split (" ");
      PointI p = toPoint (fields);
      //std::cout<< p << "\n";
      temp->push_back (p);
    }
  }

  getControl ()->getGuiPtr ()->writeConsole (
      "--------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
  QString str;
  float f = tt.toc () / 1000;
  str.append ("Imported ").append (QString::fromStdString (getControl ()->getTreeID ())).append (" with ").append (QString::number (temp->points.size ())).append (
      " points in  ").append (QString::number (f)).append (" seconds.\n");
  getControl ()->getGuiPtr ()->writeConsole (str);

  return temp;
}

PointI
ImportPCD::toPoint (QStringList fields)
{
  PointI p;
  if (fields.size () == 3)
  {
    float x, y, z;
    int i = 120;
    QString str = fields.at (0);
    x = str.toFloat ();
    str = fields.at (1);
    y = str.toFloat ();
    str = fields.at (2);
    z = str.toFloat ();
    p.x = x;
    p.y = y;
    p.z = z;
    p.intensity = i;
  }
  else if (fields.size () == 4)
  {
    float x, y, z;
    int i = 120;
    QString str = fields.at (0);
    x = str.toFloat ();
    str = fields.at (1);
    y = str.toFloat ();
    str = fields.at (2);
    z = str.toFloat ();
    str = fields.at (3);
    i = str.toInt ();
    p.x = x;
    p.y = y;
    p.z = z;
    p.intensity = i;

  }
  else if (fields.size () == 6)
  {
    float x, y, z;
    int i, r, g, b;
    QString str = fields.at (0);
    x = str.toFloat ();
    str = fields.at (1);
    y = str.toFloat ();
    str = fields.at (2);
    z = str.toFloat ();
    str = fields.at (3);
    r = str.toInt ();
    str = fields.at (4);
    g = str.toInt ();
    str = fields.at (5);
    b = str.toInt ();
    i = std::min<int> (255, ( (r + g + b) / 3));
    p.x = x;
    p.y = y;
    p.z = z;
    p.intensity = i;

  }
  return p;
}

PointCloudI::Ptr
ImportPCD::importPCD ()
{
  tt.tic ();
  pcl::console::setVerbosityLevel (pcl::console::L_ALWAYS);
  PointCloudI::Ptr temp (new PointCloudI);

  if (pcl::io::loadPCDFile<PointI> (path, *temp) == -1)
  {
    PCL_ERROR("Couldn't read pcd input file. \n");

  }
  else
  {
    getControl ()->getGuiPtr ()->writeConsole (
        "--------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
    QString str;
    float f = tt.toc () / 1000;
    str.append ("Imported ").append (QString::fromStdString (getControl ()->getTreeID ())).append (" with ").append (QString::number (temp->points.size ())).append (
        " points in  ").append (QString::number (f)).append (" seconds.\n");
    getControl ()->getGuiPtr ()->writeConsole (str);
   // std::cout << "Imported " << fileName << " with " << temp->points.size () << " points in  " << tt.toc () / 1000 << " seconds." << std::endl;

  }
  return temp;
}

void
ImportPCD::computeNormals (PointCloudI::Ptr cloud)
{
  tt.tic ();
  pcl::NormalEstimationOMP<PointI, PointI> ne (0);
  ne.setInputCloud (cloud);
  pcl::search::KdTree<PointI>::Ptr tree (new pcl::search::KdTree<PointI> ());
  ne.setSearchMethod (tree);

  ne.setKSearch (75);
  ne.compute (*cloud);
  QString str;
  float f = tt.toc () / 1000;
  str.append ("Computed normals in  ").append (QString::number (f)).append (" seconds.\n");
  getControl ()->getGuiPtr ()->writeConsole (str);
  QCoreApplication::processEvents ();
}

CurvatureCloud::Ptr
ImportPCD::computeCurvature ()
{
  tt.tic ();
  //std::cout << "computing curvatures" << "\n", tt.tic();
  pcl::PrincipalCurvaturesEstimation<PointI, PointI, pcl::PrincipalCurvatures> principalCurvaturesEstimation;
  principalCurvaturesEstimation.setInputCloud (cloud_intens);
  principalCurvaturesEstimation.setInputNormals (cloud_intens);
  pcl::search::KdTree<PointI>::Ptr tree_normal (new pcl::search::KdTree<PointI> ());
  principalCurvaturesEstimation.setSearchMethod (tree_normal);
  principalCurvaturesEstimation.setRadiusSearch (0.03);
  //principalCurvatures = new pcl::PointCloud<pcl::PrincipalCurvatures>;
  CurvatureCloud::Ptr principalCurvatures (new CurvatureCloud);
  principalCurvaturesEstimation.compute (*principalCurvatures);
//	std::cout << "output points.size (): " << principalCurvatures->points.size()
//			<< std::endl;
  // Display and retrieve the shape context descriptor vector for the 0th point.
//   pcl::PrincipalCurvatures descriptor = principalCurvatures->points[0];
  QString str;
  float f = tt.toc () / 1000;
  str.append ("Computed principal curvatures in  ").append (QString::number (f)).append (" seconds.\n");
  getControl ()->getGuiPtr ()->writeConsole (str);
  getControl ()->getGuiPtr ()->writeConsole (
      "--------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
  getControl ()->getGuiPtr ()->writeConsole ("\n");
  return principalCurvatures;
}

