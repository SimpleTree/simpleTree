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
#ifndef CURVATUREDIALOG_H
#define CURVATUREDIALOG_H

#include <QDialog>
#include <QMessageBox>

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>

#include <Eigen/Eigenvalues>

#include "../../../build/ui_curvature_dialog.h"
#include "../../../src/gui/pclviewer.h"

#include "../../../src/gui/guisubclass.h"


typedef pcl::PointXYZRGBA PointD;
typedef pcl::PointCloud<PointD> PointCloudD;
typedef pcl::PointXYZINormal PointI;
typedef pcl::PointCloud<PointI> PointCloudI;

class CurvatureDialog : public QDialog, public GuiSubClass
{
    Q_OBJECT
public:

    /** \brief Default contrstructor
     * \param parent: The parent QT class (main UI class)
     * */
    explicit CurvatureDialog(QWidget *parent = 0);

    /** \brief connects this with the main UI class
     * */
    void
    setViewer(boost::shared_ptr<PCLViewer> guiPtr);

    /** \brief returns the main UI class as a shared pointer
     * \return the shared pointer of main UI class
     * */
    boost::shared_ptr<PCLViewer>
    getViewer();

    /** \brief Computes the principal components for the point cloud and connects signal with slots
     * */
    void
    init();
private:

    /** \brief Vectors which store the first, second and third principal component of all points in control stored point cloud.
     * */
    std::vector<float> e1,e2,e3;

    /** \brief Relative threshold values for the principal components
     * */
    float
    min_e1,max_e1,min_e2,max_e2,min_e3,max_e3;

    /** \brief Minimum and maximum principal component values of all computed PCA values.
     * */
    float
    min_pca1,max_pca1,min_pca2,max_pca2,min_pca3,max_pca3;

    /** \brief A shared pointer to the UI component
     * */
    boost::shared_ptr<Ui_Dialog_Eigen> dialog;

    /** \brief A weak pointer to the UI class
     * */
    boost::weak_ptr<PCLViewer> viewer;

    /** \brief A shared pointer to the used point cloud.
     * */
    boost::shared_ptr<PointCloudD> visu_cloud;


    /** \brief Clears the viewer and resets the visualization to the point cloud stored in the control class.
     * */
    void
    resetViewer();

    /** \brief Updates the visualized point cloud according to the set thresholds of the PCA values.
     * */
    void
    updateViewer();

    /** \brief Colorize a point (p) with green color.
     *  \param p: Point to set the RGBA-Value to green (102,102,0,255).
     * */
    void
    setGreen(PointD & p);

    /** \brief Colorize a point (p) with red color.
     *  \param p: Point to set the RGBA-Value to red (178,10,10,50).
     * */
    void
    setRed(PointD & p);

signals:

public slots:
    /** \brief Resets the minimum relative value of the first principal component and updates the visualization.
     * */
    void
    minPC1();

    /** \brief Resets the maximum relative value of the first principal component and updates the visualization.
     * */
    void
    maxPC1();

    /** \brief Resets the minimum relative value of the second principal component and updates the visualization.
     * */
    void
    minPC2();

    /** \brief Resets the maximum relative value of the second principal component and updates the visualization.
     * */
    void
    maxPC2();

    /** \brief Resets the minimum relative value of the third principal component and updates the visualization.
     * */
    void
    minPC3();

    /** \brief Resets the maximum relative value of the third principal component and updates the visualization.
     * */
    void
    maxPC3();

    /** \brief Closes the dialog
     * */
    void
    abort();

    /** \brief Stores the remaining cloud after threshold deletion in the control class.
     * */
    void
    save();

};

#endif // CURVATUREDIALOG_H
