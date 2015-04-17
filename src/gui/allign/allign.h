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
#ifndef ALLIGN_H
#define ALLIGN_H

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>
#include "../pclviewer.h"

#include "../../../build/ui_allign_dialog.h"
#include "../../pointclouds/allignpointcloud.h"

#include <QDir>
#include <QDialog>


typedef pcl::PointXYZRGBA PointD;
typedef pcl::PointCloud<PointD> PointCloudD;
typedef pcl::PointXYZINormal PointI;
typedef pcl::PointCloud<PointI> PointCloudI;

namespace Ui {
class PCLViewer;
class AllignPointCloud;
}




class AllignPointCloudDialog
        : public QDialog
{
    Q_OBJECT
public:
    AllignPointCloudDialog(QWidget * parent = 0);
    void
    setViewer(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);
    void
    setUi(Ui::PCLViewer * ui);
    void
    setGuiPtr(boost::shared_ptr<PCLViewer> guiPtr);
    void
    init();

    void
    rotate_translate();

private:
    boost::shared_ptr<AllignPointCloud> allign_point_cloud;
    //boost::shared_ptr<QDialog> allign_dialog;

    void
    visualizeClouds(bool show_final = false);

    const float slice_height = 0.05f;

    QString name_source;
    QString name_target;


    Ui::PCLViewer *ui;

    boost::weak_ptr<PCLViewer>
    gui_ptr;
    boost::shared_ptr<PCLViewer>
    getGuiPtr();

    boost::weak_ptr<pcl::visualization::PCLVisualizer> viewer;

    boost::shared_ptr<Ui_dialog_init_allign>
    dialog;


    void
    selectFile(QString & name, QString & path);

    void
    import(boost::shared_ptr<PointCloudI> & cloud, QString & name);

    void
    saveFile(QString name, PointCloudI::Ptr cloud);

    void
    resetVisualization();

public slots:

    void
    abort();
    void
    save();
    void
    rotate_slider();
    void
    rotate_spinbox();
    void
    x_slider();
    void
    x_spinbox();
    void
    y_slider();
    void
    y_spinbox();
    void
    z_slider();
    void
    z_spinbox();
    void
    ICP();
};

#endif // ALLIGN_H
