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

#ifndef PCLVIEWER_H
#define PCLVIEWER_H
#include "../includes.h"
#include <iostream>
// Qt
#include <QMainWindow>
#include <QColorDialog>
#include <QMessageBox>
#include <QScrollBar>
#include <QFile>
#include <QFileInfo>
#include <QFileDialog>
#include <QWebView>
// Point Cloud Library
#define PCL_NO_PRECOMPILE
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <limits>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/visualization/point_picking_event.h>
#include <pcl/registration/icp.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include "../import/importpcd.h"
#include "../controller.h"
#include "../method/SphereFollowing.h"
#include "../Model/Tree.h"
#include "publication/quercus_petraea/quercus_folder.h"
#include "publication/pinus_massoniana/pinus_folder.h"
#include "publication/erythrophleum_fordii/ery_folder.h"
#include "publication/eucalyptus_spp/eucalypt_folder.h"
#include "publication/platanus/plane_folder.h"
#include "publication/pinus_massoniana/pine_denoise.h"
#include "publication/quercus_petraea/quercus_denoise.h"
#include "publication/erythrophleum_fordii/ery_denoise.h"
#include "../method/optimization/optimization.h"
// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>
#include<vector>
#include<iostream>
#include<utility>
#include<pcl/visualization/pcl_plotter.h>
#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <boost/thread/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

#include <boost/make_shared.hpp>
#include <boost/enable_shared_from_this.hpp>
#endif
#include "../export/writecsv.h"
#include "../export/exportply.h"
#include "../../src/gui/reference/referenceheightdialog.h"
#include "../../src/gui/cropsphere/cropspheredialog.h"
#include "../../src/gui/cropbox/cropboxdialog.h"
#include "../method/StemPointDetection.h"
#include "../method/set_coefficients.h"
#include "../method/method_coefficients.h"
#include "../gui/allign/allign.h"
#include "../pointclouds/voxelgridfilter.h"
#include "../../src/gui/curvature/curvaturedialog.h"
#include "../../src/Model/modelAdjustment/allometry.h"
#include "../../build/ui_pclviewer.h"
#include "../../build/ui_radius_dialog.h"
#include "../../build/ui_intensity_dialog.h"
#include "../../build/ui_statistical_dialog.h"
#include "../../build/ui_voxel_grid_dialog.h"
#include "../../build/ui_euclidean_dialog.h"
#include "../../build/ui_crop_box_dialog.h"
#include "../../build/ui_method_dialog.h"
#include "../../build/ui_allign_dialog.h"
#include "../../build/ui_crown_dialog.h"
#include "../../build/ui_allometry_dialog.h"
#include "../../build/ui_optimize_dialog.h"
#include "../method/optimization/optimization_stem.h"
#include "color_palette/color_factory.h"


class SetCoefficients;
class AllignPointCloudDialog;
class CurvatureDialog;
class ReferenceHeightDialog;
class CropSphereDialog;
class CropBoxDialog;
class Quercus_folder;
class Pinus_folder;
class Ery_folder;
class Eucalypt_folder;
class Plane_folder;
class Pine_denoise;
class Quercus_denoise;
class Ery_denoise;

namespace Ui {
class PCLViewer;
}

namespace simpleTree {
class Tree;
}

class Controller;
class PCLViewer;
struct callback_args {
    // structure used to pass arguments to the callback function
    boost::shared_ptr<CropBoxDialog> box_dialog_ptr;
    boost::shared_ptr<CropSphereDialog> sphere_dialog_ptr;
    PCLViewer * viewer;
};
class PCLViewer: public QMainWindow,public  boost::enable_shared_from_this<PCLViewer>  {
    Q_OBJECT
private:
    boost::shared_ptr<AllignPointCloudDialog> allign;
    boost::shared_ptr<CurvatureDialog> curvature;
    callback_args cb_args;
    boost::shared_ptr<Ui_allometry_dialog> allometry_dialog_ptr;
    boost::shared_ptr<Ui_optimize_dialog> optimize_dialog_ptr;
    boost::shared_ptr<Ui_crop_box_dialog> box_dialog_ui_ptr;
    boost::shared_ptr<Ui_method_dialog> method_dialog_ptr;
    boost::shared_ptr<SetCoefficients> coeff_ptr;
    boost::shared_ptr<simpleTree::Tree> tree_ptr;
    boost::shared_ptr<PointCloudI> allign_cloud;
    boost::shared_ptr<CropSphereDialog> crop_sphere_dialog;
    boost::shared_ptr<CropBoxDialog> crop_box_dialog;
    boost::shared_ptr<Quercus_folder> quercus_dialog;
    boost::shared_ptr<Pinus_folder> pinus_dialog;
    boost::shared_ptr<Ery_folder> ery_dialog;
    boost::shared_ptr<Eucalypt_folder> euca_dialog;
    boost::shared_ptr<Plane_folder> plane_dialog;
    boost::shared_ptr<Pine_denoise> pinus_denoise_dialog;
    boost::shared_ptr<Quercus_denoise> quercus_denoise_dialog;
    boost::shared_ptr<Ery_denoise> ery_denoise_dialog;
    boost::shared_ptr<ReferenceHeightDialog> reference_height_dialog;
    std::vector<boost::shared_ptr<PointCloudD> > intensity_clouds;
    Method_Coefficients _method_coefficients;
    pcl::console::TicToc tt;
    pcl::ModelCoefficients deleteBox;

    float minX, minY, minZ, maxX, maxY, maxZ, extension = 1;
    float a,b,fact,minRad;
    float centerX = 0,
    centerY = 0, centerZ = 0;
    float fac = 2.5;
    int point_color_max = 6;
    int tree_color_max = 4;
    int point_color;
    int tree_color = 0;
    float hull_radius = 0.1f;
    int radius_outlier_minPts = 3;
    float radius_outlier_searchradius = 0.015f;
    float intensity_outlier_minIntens = 0.0;
    float intensity_outlier_maxIntens = 255.0;
    int statistical_outlier_knn = 5;
    float statistical_outlier_stdmult = 5.0;
    float voxel_grid_size = 0.01;
    int euclidean_clustering_minsize = 100;
    float euclidean_clustering_tolerance = 0.02;
    int euclidean_clustering_clusternumber = 1;

    void
    convertPointCloud(PointCloudI::Ptr);


    void
    computeNormals(PointCloudI::Ptr cloud);
    CurvatureCloud::Ptr
    computeCurvature(PointCloudI::Ptr cloud);

    boost::weak_ptr<Controller> control;
    QString consoleString;

    boost::shared_ptr<SetCoefficients> set_coefficients;


public:

    boost::shared_ptr<pcl::visualization::PCLPlotter> plotter;

    float mean(std::vector<float> const &v);
    Ui::PCLViewer *ui;
    void
    init();

    bool reset_crown_is_active;
    bool crop_box_is_active;
    bool crop_sphere_is_active;

    void
    writeLine();
    boost::shared_ptr<Ui_dialog_init_allign> allign_dialog_ptr;

    explicit
    PCLViewer(QWidget *parent = 0);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

    float
    median(std::vector<float> values);

    void
    computeBoundingBox();
    ~PCLViewer();
    void
    connectToController(boost::shared_ptr<Controller> control);
    void
    setCloudPtr(PointCloudI::Ptr cloud, bool changeView = false);
    void
    setTreePtr(boost::shared_ptr<simpleTree::Tree> tree_ptr);
    boost::shared_ptr<Controller>
    getControl();
    void
    writeConsole(QString str);
    std::string
    selectFile();
    void
    plotAllometry();
    void
    plotIntensityHist();
    boost::shared_ptr<PointCloudI>
    extractStemBase(boost::shared_ptr<PointCloudI> tree, float height_min);

    boost::shared_ptr<PointCloudD>
    convertPointCloud(PointCloudI::Ptr  cloud, int r, int g, int b);


public slots:
    void
    allom_check();
    void
    allom_check1();
    void
    allometry_dialog();
    void
    allom_compute();
    void
    abort_crown();
    void
    abort_intensity_outlier_removal();
    void
    build_model_for_folder();
    void
    build_model();
    void
    build_model1();
    void
    compute_detectTree();
    void
    compute_final_model();
    void
    compute_quercus();
    void
    compute_plane();
    void
    compute_pinus();
    void
    compute_ery();
    void
    compute_eucalypt();
    void
    curvature_dialog();
    void
    compute_crown();
    void
    compute_ICP();
    void
    crop_box();
    void
    cropsphere();
    void
    compute_radius_outlier_removal();
    void
    compute_euclidean_clustering();
    void
    changePointColor();
    void
    computeNormals();
    void
    changeTreeColor();
    void
    compute_statistical_outlier_removal();
    void
    compute_intensity_outlier_removal();
    void
    compute_voxel_grid_downsampling();
    void
    denoise_pinus();
    void
    delete_method();
    void
    denoise_ery();
    void
    denoise_oak();
    void
    denoise_eucalypt();
    void
    exportPCDFile();
    void
    exportResults();
    void
    exportPly();
    void
    euclideanClustering();
    void
    importPCDFile();
    void
    intensityOutlierRemoval();

    void
    load_camera_position();
    void
    mergeClouds();
    const Method_Coefficients&
    getMethodCoefficients () const
    {
        return _method_coefficients;
    }
    void
    optimize_check();
    void
    optimize_check1();
    void
    plot_qstring(QString str = "", bool firstLine = false, bool secondLine = false);
    void
    plot();
    void
    reset_crown();
    void
    reset_crown_base();
    void
    radiusOutlierRemoval();
    void
    reset_stem();
    void
    reference_cloud();
    void
    save_camera_position();
    void
    setProgress(int percentage);
    void
    setTreeID(QString ID);
    void
    setTree(boost::shared_ptr<simpleTree::Tree> tree);
    void
    set_allom_fact();
    void
    set_optimize_iteration();
    void
    set_method_coeff_wdout_compute();
    void
    screenshot();
    void
    set_background();
    void
    set_crown_radius(double r);
    void
    set_method(int i);
    void
    set_voxel_grid_size(double size);
    void
    set_statistical_outlier_knn(int knn);
    void
    set_statistical_outlier_stdmult(double stdmult);
    void
    set_intensity_outlier_minIntens(double minIntens);
    void
    set_intensity_outlier_maxIntens(double maxIntens);
    void
    set_euclidean_clustering_minsize(int size);
    void
    set_euclidean_clustering_tolerance(double tolerance);
    void
    set_euclidean_clustering_clusternumber(int number);
    void
    set_radius_outlier_minPts(int minPts);
    void
    set_radius_outlier_searchradius(double searchRadius);
    void
    setPointTransparency(int value);
    void
    setTreeTransparency(int value);
    void
    setStemTransparency(int value);
    void
    setCrownTransparency(int value);
    void
    setLeaveTransparency(int value);
    void
    setPointSize(int value);
    void
    statisticalOutlierRemoval();

    void
    setMethodCoefficients (const Method_Coefficients& methodCoefficients)
    {
        _method_coefficients = methodCoefficients;
    }
    void
    setMethodCoefficientswdoutcompute (const Method_Coefficients& methodCoefficients)
    {
        _method_coefficients = methodCoefficients;
    }
    void
    updateProgress(int);


    void
    visualize_branch();
    void
    voxel_downsampling();

    void
    xNegView();
    void
    xPosView();
    void
    yNegView();
    void
    yPosView();
    void
    zNegView();
    void
    zPosView();


protected:

    PointCloudD::Ptr cloud;
    std::vector<boost::shared_ptr<simpleTree::Cylinder> > cylinders;
    std::vector<boost::shared_ptr<simpleTree::Cylinder> > stemCylinders;

    unsigned int red;
    unsigned int green;
    unsigned int blue;

private:

    void
    generateData(double *ax, double *acos, double *asin, int numPoints);
    void
    generateCloud();

};

#endif // PCLVIEWER_H

