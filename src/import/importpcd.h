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

#ifndef IMPORTPCD_H
#define IMPORTPCD_H
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/io/ply_io.h>
#include "../controller.h"
#include "../method/eigenvalueestimator.h"
#include <QString>
#include <qfile.h>
#include <QTextStream>

#include <iostream>
#include <fstream>
#include <string>
typedef pcl::PointXYZINormal PointI;
typedef pcl::PointCloud<PointI> PointCloudI;
typedef pcl::PointCloud<pcl::PrincipalCurvatures> CurvatureCloud;

class Controller;
class ImportPCD
{

    std::string path;
    pcl::console::TicToc tt;
    PointCloudI::Ptr cloud_intens;
    CurvatureCloud::Ptr principalCurvatures;
    void
    computeNormals (PointCloudI::Ptr points);
    CurvatureCloud::Ptr
    computeCurvature ();
//     void
//     init ();
    void
    setRGB ();
    boost::shared_ptr<Controller>
    getControl ();
    boost::weak_ptr<Controller> control;

  public:
    std::string fileName;
    ImportPCD (std::string fileName,
               boost::weak_ptr<Controller> control);
    virtual
    ~ImportPCD ();

    PointCloudI::Ptr
    importPCD ();

    PointCloudI::Ptr
    importASC ();

    PointI
    toPoint (QStringList fields);

    PointCloudI::Ptr
    getCloud ()
    {
      return cloud_intens;
    }

    CurvatureCloud::Ptr
    getCurvatures ()
    {
      return principalCurvatures;
    }
};

#endif // IMPORTPCD_H
