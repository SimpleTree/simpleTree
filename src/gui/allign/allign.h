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
    /** \brief Default contrstructor
     * \param parent: The parent QT class (main UI class)
     * */
    AllignPointCloudDialog(QWidget * parent = 0);

    /** \brief connects this with the main UI class
     * */
    void
    setViewer(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);

    void
    setUi(Ui::PCLViewer * ui);
    void
    setGuiPtr(boost::shared_ptr<PCLViewer> guiPtr);

    /** \brief Lets the user choose target and source point cloud.
     * */
    void
    init();

    /** \brief Computes cloud_final from cloud_source with given x,y,z for translation and angle for rotation around z-axis
     * */
    void
    rotate_translate();

private:

    /** \brief a shared pointer to the allignPointCloud class to compute the translation and rotation of point cloud
     * */
    boost::shared_ptr<AllignPointCloud> allign_point_cloud;
    //boost::shared_ptr<QDialog> allign_dialog;

    /** \brief Visualizes cloud_source, cloud_target and cloud_final
     * \param show_final: if set to true cloud_final is visualized, otherwise only cloud_source and cloud_target are visualized
     * */
    void
    visualizeClouds(bool show_final = false);

    /** \brief The thickness of the point cloud cut
     * */
    const float slice_height = 0.05f;

    QString name_source;
    QString name_target;


    Ui::PCLViewer *ui;

    /** \brief a weak pointer to the main UI class
     * */
    boost::weak_ptr<PCLViewer>
    gui_ptr;

    /** \brief Converts the weak pointer to a shared pointer of the main UI class
     * \return a shared pointer to the main UI class
     * */
    boost::shared_ptr<PCLViewer>
    getGuiPtr();

    /** \brief a weak pointer to the main UI class
     * */
    boost::weak_ptr<pcl::visualization::PCLVisualizer> viewer;

    /** \brief A shared pointer of the UI QT class
     * */
    boost::shared_ptr<Ui_dialog_init_allign>
    dialog;

    /** \brief Let the user choose a file from a dialog and stores name and path
     * \param name: the name of the point cloud
     * \param path: the path of the point cloud
     * */
    void
    selectFile(QString & name, QString & path);

    /** \brief Imports a point cloud
     * \param cloud: the shared pointer of the point cloud where the cloud is saved to
     * \param name: the name of the stored point cloud
     * */
    void
    import(boost::shared_ptr<PointCloudI> & cloud, QString & name);

    /** \brief Opens a save dialog to store cloud_final to
     * \param name: the suggested name
     * \param cloud: the shared pointer to cloud_final
     * */
    void
    saveFile(QString name, PointCloudI::Ptr cloud);

    /** \brief removes all visualized clouds and sets the visualization to cloud_final if successful allignment and to cloud source otherwise
     * */
    void
    resetVisualization();

public slots:

    /** \brief closes the dialog
     * */
    void
    abort();

    /** \brief saves cloud_final and cloud_target
     * */
    void
    save();

    /** \brief Slot for rotation around the z-axis
     * */
    void
    rotate_slider();
    /** \brief Slot for rotation around the z-axis
     * */
    void
    rotate_spinbox();
    /** \brief Slot for x- axis translation
     * */
    void
    x_slider();
    /** \brief Slot for x- axis translation
     * */
    void
    x_spinbox();
    /** \brief Slot for y- axis translation
     * */
    void
    y_slider();
    /** \brief Slot for y- axis translation
     * */
    void
    y_spinbox();
    /** \brief Slot for z- axis translation
     * */
    void
    z_slider();
    /** \brief Slot for z- axis translation
     * */
    void
    z_spinbox();

    /** \brief Slot to call Iterative closest points.
     * */
    void
    ICP();
};

#endif // ALLIGN_H
