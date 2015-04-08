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

#include "pclviewer.h"
#include <qt4/QtCore/qurl.h>



void
pp_callback ( const pcl::visualization::PointPickingEvent& event,
              void* cb_args ) {
    struct callback_args* data = ( struct callback_args * ) cb_args;
    if ( event.getPointIndex () != -1 ) {
        PCLViewer * viewer = data->viewer;
        if ( viewer->crop_sphere_is_active ) {
            boost::shared_ptr<Ui_crop_sphere_dialog> sphere_dialog_ptr = data->sphere_dialog_ptr;
            Ui_crop_sphere_dialog * dia = &*sphere_dialog_ptr;
            float x, y, z;
            event.getPoint ( x, y, z );
            dia->box_x->setValue ( x );
            dia->box_y->setValue ( y );
            dia->box_z->setValue ( z );
        } else if ( viewer->crop_box_is_active ) {
            boost::shared_ptr<Ui_crop_box_dialog> box_dialog_ptr = data->box_dialog_ptr;
            Ui_crop_box_dialog * dia = &*box_dialog_ptr;
            float x, y, z;
            event.getPoint ( x, y, z );
            dia->box_x->setValue ( x );
            dia->box_y->setValue ( y );
            dia->box_z->setValue ( z );
        } else  if (viewer->reset_crown_is_active) {

            float x, y, z;
            event.getPoint ( x, y, z );
            viewer->getControl()->getTreePtr()->reset_crown(z);
            viewer->reset_crown_is_active = false;
        } else {
            float x, y, z;
            event.getPoint ( x, y, z );
            pcl::PointXYZ p ( x,y,z );
            QString str;
            str.append ( "Selected point : " ).append ( QString::number ( x ) ).append ( " ; " ).append ( QString::number ( y ) ).append ( " ; " ).append ( QString::number ( z ) ).append (
                "\n" );
            viewer->writeConsole ( str );

        }
    } else {
      
            std::cout << "Error in PCLViewer ::no Point picked \n";
    }
}





PCLViewer::PCLViewer ( QWidget *parent ) :
QMainWindow ( parent ),
ui ( new Ui::PCLViewer ) {

    ui->setupUi ( this );
    // this->setCentralWidget(ui->scrollAreaWidgetContents);
    ui->scrollArea->setWidget ( ui->scrollAreaWidgetContents );
    this->setWindowTitle ( "SimpleTree" );
    generateCloud ();

//     qwebview = new QWebView ( ui->help_frame );
//     qwebview->setObjectName ( QString::fromUtf8 ( "qwebview" ) );
//     qwebview->setGeometry ( QRect ( 10, 10, 321, 471 ) );
//     qwebview->setProperty ( "url", QVariant ( QUrl ( QString::fromUtf8 ( "file:///home/hackenberg/simple/html/help_start/help_start.html" ) ) ) );

    // Set up the QVTK window
    viewer.reset ( new pcl::visualization::PCLVisualizer ( "Simple Tree", false ) );
    ui->qvtkWidget->SetRenderWindow ( viewer->getRenderWindow () );
    viewer->setupInteractor ( ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow () );
   // ui->qvtkWidget->setba

    ui->qvtkWidget->update ();

    plotter.reset ( new pcl::visualization::PCLPlotter ( "Plotter" ) );
    ui->qvtkWidget2->SetRenderWindow ( plotter->getRenderWindow () );
    plotter->setViewInteractor ( ui->qvtkWidget2->GetInteractor () ); //, ui->qvtkWidget2->GetRenderWindow());
    ui->qvtkWidget2->update ();

    ui->picture_frame->setStyleSheet ( "background-color:white;" );
    ui->picture_frame_3->setStyleSheet ( "background-color:white;" );

    ui->xViewNeg->setIconSize(QSize(30, 30));
    ui->xViewPos->setIconSize(QSize(30, 30));
    ui->yViewNeg->setIconSize(QSize(30, 30));
    ui->yViewPos->setIconSize(QSize(30, 30));
    ui->zViewNeg->setIconSize(QSize(30, 30));
    ui->zViewPos->setIconSize(QSize(30, 30));
    ui->point_color_button->setIconSize(QSize(30, 30));
    ui->tree_color_button->setIconSize(QSize(30, 30));
    ui->method_hackenberg->setIconSize(QSize(25, 25));
    ui->statistical_button->setIconSize(QSize(30, 30));
    ui->voxelgrid_button->setIconSize(QSize(30, 30));
    ui->intensity_button->setIconSize(QSize(30, 30));
    ui->radius_button->setIconSize(QSize(30, 30));
    ui->cluster_button->setIconSize(QSize(30, 30));
    ui->compute_normal_button->setIconSize(QSize(27, 27));
    ui->merge_cloud_button->setIconSize(QSize(35, 35));
    ui->sphere_deletion_button->setIconSize(QSize(24, 24));
    ui->box_deletion_button->setIconSize(QSize(24, 24));
    ui->reference_cloud_button->setIconSize(QSize(30, 30));
    ui->complete_folder_button->setIconSize(QSize(25, 25));
    ui->file_import_button->setIconSize(QSize(25, 25));
    ui->file_export_button->setIconSize(QSize(25, 25));
    ui->file_export_result_button->setIconSize(QSize(25, 25));
    ui->file_export_ply_button->setIconSize(QSize(25, 25));
    ui->reset_stem_button->setIconSize(QSize(25,25));
    ui->reset_crown_base_button->setIconSize(QSize(25,25));
    ui->reset_crown_button->setIconSize(QSize(25,25));
    ui->background_button->setIconSize(QSize(25,25));
    ui->screenshotbutton->setIconSize(QSize(25,25));
    viewer->setBackgroundColor(1,1,1,0);

    connect ( ui->file_import_button, SIGNAL ( clicked() ), this, SLOT ( importPCDFile() ) );
    connect ( ui->file_export_button, SIGNAL ( clicked() ), this, SLOT ( exportPCDFile() ) );
    connect ( ui->file_export_result_button, SIGNAL ( clicked() ), this, SLOT ( exportResults() ) );
    connect ( ui->file_export_ply_button, SIGNAL ( clicked() ), this, SLOT ( exportPly() ) );
    connect ( ui->point_size_slider, SIGNAL ( valueChanged ( int ) ), this, SLOT ( setPointSize ( int ) ) );
    connect ( ui->point_color_slider, SIGNAL ( valueChanged ( int ) ), this, SLOT ( setPointTransparency ( int ) ) );
    connect ( ui->stem_color_slider, SIGNAL ( valueChanged ( int ) ), this, SLOT ( setStemTransparency ( int ) ) );
    connect ( ui->tree_color_slider, SIGNAL ( valueChanged ( int ) ), this, SLOT ( setTreeTransparency ( int ) ) );
    connect ( ui->crown_color_slider, SIGNAL ( valueChanged ( int ) ), this, SLOT ( setCrownTransparency ( int ) ) );
    connect ( ui->leave_color_slider, SIGNAL ( valueChanged ( int ) ), this, SLOT ( setLeaveTransparency ( int ) ) );
    connect ( ui->xViewNeg, SIGNAL ( clicked() ), this, SLOT ( xNegView() ) );
    connect ( ui->xViewPos, SIGNAL ( clicked() ), this, SLOT ( xPosView() ) );
    connect ( ui->yViewNeg, SIGNAL ( clicked() ), this, SLOT ( yNegView() ) );
    connect ( ui->yViewPos, SIGNAL ( clicked() ), this, SLOT ( yPosView() ) );
    connect ( ui->zViewNeg, SIGNAL ( clicked() ), this, SLOT ( zNegView() ) );
    connect ( ui->zViewPos, SIGNAL ( clicked() ), this, SLOT ( zPosView() ) );
    connect ( ui->point_color_button, SIGNAL ( clicked() ), this, SLOT ( changePointColor() ) );
    connect ( ui->tree_color_button, SIGNAL ( clicked() ), this, SLOT ( changeTreeColor() ) )  ;
    connect ( ui->method_hackenberg, SIGNAL ( clicked() ), this, SLOT ( detectTree() ) );
    connect ( ui->statistical_button, SIGNAL ( clicked() ), this, SLOT ( statisticalOutlierRemoval() ) );
    connect ( ui->voxelgrid_button, SIGNAL ( clicked() ), this, SLOT ( voxel_downsampling() ) );
    connect ( ui->intensity_button, SIGNAL ( clicked() ), this, SLOT ( intensityOutlierRemoval() ) );
    connect ( ui->radius_button, SIGNAL ( clicked() ), this, SLOT ( radiusOutlierRemoval() ) );
    connect ( ui->cluster_button, SIGNAL ( clicked() ), this, SLOT ( euclideanClustering() ) );
    connect ( ui->compute_normal_button, SIGNAL ( clicked() ), this, SLOT ( computeNormals() ) );
    connect ( ui->merge_cloud_button, SIGNAL ( clicked() ), this, SLOT ( mergeClouds() ) );
    connect ( ui->sphere_deletion_button, SIGNAL ( clicked() ), this, SLOT ( cropsphere() ) );
    connect ( ui->box_deletion_button, SIGNAL ( clicked() ), this, SLOT ( crop_box() ) );
    connect ( ui->reference_cloud_button, SIGNAL ( clicked() ), this, SLOT ( reference_cloud() ) );
    connect ( ui->complete_folder_button,SIGNAL ( clicked() ),this,SLOT ( compute_complete_folder() ) );
    connect ( ui->reset_stem_button,SIGNAL(clicked()),this,SLOT(reset_stem()));
    connect ( ui->reset_crown_base_button,SIGNAL(clicked()),this,SLOT(reset_crown_base()));
    connect ( ui->reset_crown_button,SIGNAL(clicked()),this,SLOT(reset_crown()));
    connect ( ui->background_button,SIGNAL(clicked()),this,SLOT(set_background()));
    connect ( ui->screenshotbutton,SIGNAL(clicked()),this,SLOT(screenshot()));
    connect ( ui->icp_button,SIGNAL ( clicked() ),this,SLOT ( compute_ICP() ) );
    viewer->resetCamera ();
    computeBoundingBox ();
    pcl::visualization::PointCloudColorHandlerRGBAField<PointD> rgba ( cloud );
    viewer->addPointCloud<PointD> ( cloud, rgba, "cloud" );
    viewer->setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_OPACITY, 1, "cloud" );
    setPointSize ( 1 );
    setPointTransparency ( 150 );
    xNegView ();
    ui->qvtkWidget->update ();
    plot ();
    crop_sphere_is_active = false;
    sphere_dialog_ui_ptr.reset ( new Ui_crop_sphere_dialog );
    crop_box_is_active = false;
    reset_crown_is_active = false;
   // allign_clouds_is_active = false;
    box_dialog_ui_ptr.reset ( new Ui_crop_box_dialog );
    method_dialog_ptr.reset ( new Ui_method_dialog );
    cb_args.box_dialog_ptr = box_dialog_ui_ptr;
    cb_args.sphere_dialog_ptr = sphere_dialog_ui_ptr;
    cb_args.viewer = this;
    viewer->registerPointPickingCallback ( pp_callback, ( void* ) &cb_args );

}

//void
//PCLViewer::intialAllign()
//{
//     ui->qvtkWidget->update ();


//// std::cout << "initial Allign test \n";
////       pcl::SampleConsensusInitialAlignment<PointI, PointI, pcl::PointXYZ> sac_ia;
//// std::cout << "initial Allign test \n";
////       sac_ia.setMinSampleDistance (0.01);
////       sac_ia.setMaxCorrespondenceDistance(0.1);
////       sac_ia.setMaximumIterations (150);
//// std::cout << "initial Allign test \n";
////       sac_ia.setInputCloud (cloud_source);
////       sac_ia.setInputTarget (cloud_target);
//// std::cout << "initial Allign test \n"; std::cout << "initial Allign test \n";
////       sac_ia.setSourceFeatures (feature1);
////       sac_ia.setTargetFeatures (feature2);
////        std::cout << "initial Allign test \n";
////       sac_ia.align (*cloud_final);
////       boost::shared_ptr<PointCloudD> visu_source (new PointCloudD);
////       boost::shared_ptr<PointCloudD> visu_target (new PointCloudD);

////       visu_source->resize ( cloud_source->points.size () );
////       for ( size_t i = 0; i < cloud_source->points.size (); i++ ) {
////           visu_source->points[i].x = cloud_source->points[i].x;
////           visu_source->points[i].y = cloud_source->points[i].y;
////           visu_source->points[i].z = cloud_source->points[i].z;
////           float intens = cloud_source->points[i].intensity;
////           if ( intens == 0 ) {
////               intens = 180;
////           }
////           visu_source->points[i].r = 255;
////           visu_source->points[i].g = 0;
////           visu_source->points[i].b = 0;
////           visu_source->points[i].a = 255;
////       }

////       visu_target->resize ( cloud_final->points.size () );
////       for ( size_t i = 0; i < cloud_final->points.size (); i++ ) {
////           visu_target->points[i].x = cloud_final->points[i].x;
////           visu_target->points[i].y = cloud_final->points[i].y;
////           visu_target->points[i].z = cloud_final->points[i].z;
////           float intens = cloud_final->points[i].intensity;
////           if ( intens == 0 ) {
////               intens = 180;
////           }
////           visu_target->points[i].r = 0;
////           visu_target->points[i].g = 255;
////           visu_target->points[i].b = 0;
////           visu_target->points[i].a = 255;
////       }


////       viewer->removeAllPointClouds ();
////       viewer->removeAllShapes ();
////       pcl::visualization::PointCloudColorHandlerRGBAField<PointD> rgba1 ( visu_target );
////       viewer->addPointCloud<PointD> ( visu_target, rgba1, "cloud1" );
////       pcl::visualization::PointCloudColorHandlerRGBAField<PointD> rgba2 ( visu_source );
////       viewer->addPointCloud<PointD> ( visu_source, rgba2, "cloud2" );
////       xNegView ();
//       viewer->addText ( getControl ()->getTreeID (), 10, 20,20, 1, 0, 0, "tree_text" );
//       ui->qvtkWidget->update ();
//}


////void
////PCLViewer:: switch_point_for_ICP ( int i ) {
////    point = i;
////}


//boost::shared_ptr<PointCloudI>
//PCLViewer::transformToOrigin(boost::shared_ptr<PointCloudI> tree, Eigen::Vector4f center)
//{
//    Eigen::Affine3f transform = Eigen::Affine3f::Identity();

//    // Define a translation of 2.5 meters on the x axis.
//    transform.translation() << -center(0,0),-center(1,0),-center(2,0)+0.2f;



//    // Executing the transformation
//   boost::shared_ptr<PointCloudI>transformed_cloud (new PointCloudI ());
//    // You can either apply transform_1 or transform_2; they are the same
//    pcl::transformPointCloud (*tree, *transformed_cloud, transform);
//    return transformed_cloud;
//}

//boost::shared_ptr<PointCloudI>
//PCLViewer::extractStemBase(boost::shared_ptr<PointCloudI> tree, float height_min)
//{
//    boost::shared_ptr<PointCloudI> tree_base (new PointCloudI);
//    pcl::PassThrough<PointI> pass;
//      pass.setInputCloud (tree);
//      pass.setFilterFieldName ("z");
//      pass.setFilterLimits (height_min, height_min+0.05);
//      //pass.setFilterLimitsNegative (true);
//      pass.filter (*tree_base);
//      return tree_base;
//}


//void
//PCLViewer::performICP()
//{
//    int angle = allign_dialog_ptr->rotate->value();
//    int z_transform = allign_dialog_ptr->z->value();
//    cloud_target = transform(cloud_target,angle,z_transform);


//    pcl::IterativeClosestPoint<PointI, PointI> icp;
//      icp.setInputSource(cloud_source);
//      icp.setInputTarget(cloud_target);
//      icp.setMaximumIterations (20);

//      boost::shared_ptr<PointCloudI> Final (new PointCloudI);

//      icp.align(*Final);
//      cloud_target = Final;


//      if (icp.hasConverged ())
//        {
//          std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
//        }
//        else
//        {
//          PCL_ERROR ("\nICP has not converged.\n");
//        }

//    boost::shared_ptr<PointCloudD> visu_source (new PointCloudD);
//    boost::shared_ptr<PointCloudD> visu_target (new PointCloudD);

//    visu_source->resize ( cloud_source->points.size () );
//    for ( size_t i = 0; i < cloud_source->points.size (); i++ ) {
//        visu_source->points[i].x = cloud_source->points[i].x;
//        visu_source->points[i].y = cloud_source->points[i].y;
//        visu_source->points[i].z = cloud_source->points[i].z;
//        float intens = cloud_source->points[i].intensity;
//        if ( intens == 0 ) {
//            intens = 180;
//        }
//        visu_source->points[i].r = 255;
//        visu_source->points[i].g = 0;
//        visu_source->points[i].b = 0;
//        visu_source->points[i].a = 255;
//    }

//    visu_target->resize ( cloud_target->points.size () );
//    for ( size_t i = 0; i < cloud_target->points.size (); i++ ) {
//        visu_target->points[i].x = cloud_target->points[i].x;
//        visu_target->points[i].y = cloud_target->points[i].y;
//        visu_target->points[i].z = cloud_target->points[i].z;
//        float intens = cloud_target->points[i].intensity;
//        if ( intens == 0 ) {
//            intens = 180;
//        }
//        visu_target->points[i].r = 0;
//        visu_target->points[i].g = 255;
//        visu_target->points[i].b = 0;
//        visu_target->points[i].a = 255;
//    }



//    viewer->removeAllPointClouds ();
//    viewer->removeAllShapes ();
//    pcl::visualization::PointCloudColorHandlerRGBAField<PointD> rgba ( visu_target );
//    viewer->addPointCloud<PointD> ( visu_target, rgba, "cloud1" );
//    pcl::visualization::PointCloudColorHandlerRGBAField<PointD> rgba2 ( visu_source );
//    viewer->addPointCloud<PointD> ( visu_source, rgba2, "cloud2" );
//   // xNegView ();
//    viewer->addText ( getControl ()->getTreeID (), 10, 20,20, 1, 0, 0, "tree_text" );
//    ui->qvtkWidget->update ();
//}


//void
//PCLViewer::set_allign_rotate_box()
//{
//    int angle = allign_dialog_ptr->rotate_2->value()*10;
//    allign_dialog_ptr->rotate->setValue(angle);
//    allign_rotate_translate();
//}

//void
//PCLViewer::set_allign_rotate()
//{
//    float angle = allign_dialog_ptr->rotate->value();
//    allign_dialog_ptr->rotate_2->setValue(angle/10);
//    allign_rotate_translate();
//}

//void
//PCLViewer::switch_point_for_ICP(int i)
//{

//}

//void
//PCLViewer::rotateAllign()
//{

//}

//void
//PCLViewer::set_allign_z_box()
//{
//    int z = allign_dialog_ptr->z_2->value()*1000;
//    allign_dialog_ptr->z->setValue(z);
//    allign_rotate_translate();
//}

//void
//PCLViewer::set_allign_z()
//{
//    float z = allign_dialog_ptr->z->value();
//    allign_dialog_ptr->z_2->setValue(z/1000);
//    allign_rotate_translate();
//}

//void
//PCLViewer::set_allign_y()
//{
//    float y = allign_dialog_ptr->y->value();
//    allign_dialog_ptr->y_2->setValue(y/1000);
//    allign_rotate_translate();
//}

//void
//PCLViewer::set_allign_y_box()
//{
//        int y = allign_dialog_ptr->y_2->value()*1000;
//        allign_dialog_ptr->y->setValue(y);
//        allign_rotate_translate();
//}

//void
//PCLViewer::set_allign_x_box()
//{
//    int x = allign_dialog_ptr->x_2->value()*1000;
//    allign_dialog_ptr->x->setValue(x);
//    allign_rotate_translate();
//}

//void
//PCLViewer::set_allign_x()
//{
//    float x = allign_dialog_ptr->x->value();
//    allign_dialog_ptr->x_2->setValue(x/1000);
//    allign_rotate_translate();
//}



//void
//PCLViewer::allign_rotate_translate()
//{


//    boost::shared_ptr<PointCloudD> visu_source (new PointCloudD);
//    boost::shared_ptr<PointCloudD> visu_target (new PointCloudD);

//    visu_source->resize ( cloud_source->points.size () );
//    for ( size_t i = 0; i < cloud_source->points.size (); i++ ) {
//        visu_source->points[i].x = cloud_source->points[i].x;
//        visu_source->points[i].y = cloud_source->points[i].y;
//        visu_source->points[i].z = cloud_source->points[i].z;
//        float intens = cloud_source->points[i].intensity;
//        if ( intens == 0 ) {
//            intens = 180;
//        }
//        visu_source->points[i].r = 255;
//        visu_source->points[i].g = 0;
//        visu_source->points[i].b = 0;
//        visu_source->points[i].a = 255;
//    }

//    visu_target->resize ( cloud_target->points.size () );
//    for ( size_t i = 0; i < cloud_target->points.size (); i++ ) {
//        visu_target->points[i].x = cloud_target->points[i].x;
//        visu_target->points[i].y = cloud_target->points[i].y;
//        visu_target->points[i].z = cloud_target->points[i].z;
//        float intens = cloud_target->points[i].intensity;
//        if ( intens == 0 ) {
//            intens = 180;
//        }
//        visu_target->points[i].r = 0;
//        visu_target->points[i].g = 255;
//        visu_target->points[i].b = 0;
//        visu_target->points[i].a = 255;
//    }


//    int angle = allign_dialog_ptr->rotate->value();
//    int z_transform = allign_dialog_ptr->z->value();
//    visu_target = transform(visu_target,angle,z_transform);


//    viewer->removeAllPointClouds ();
//    viewer->removeAllShapes ();
//    pcl::visualization::PointCloudColorHandlerRGBAField<PointD> rgba ( visu_target );
//    viewer->addPointCloud<PointD> ( visu_target, rgba, "cloud1" );
//    pcl::visualization::PointCloudColorHandlerRGBAField<PointD> rgba2 ( visu_source );
//    viewer->addPointCloud<PointD> ( visu_source, rgba2, "cloud2" );
//   // xNegView ();
//    viewer->addText ( getControl ()->getTreeID (), 10, 20,20, 1, 0, 0, "tree_text" );
//    ui->qvtkWidget->update ();
//}

//boost::shared_ptr<PointCloudD>
//PCLViewer::transform(boost::shared_ptr<PointCloudD> tree, int angle, int z_transform)
//{
//    float theta = M_PI/3600*angle;
//    float height = z_transform/100.0f;


//    Eigen::Affine3f transform = Eigen::Affine3f::Identity();

//      // Define a translation of 2.5 meters on the x axis.
//      transform.translation() << 0, 0.0, height;

//      // The same rotation matrix as before; tetha radians arround Z axis
//      transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));



//      // Executing the transformation
//      boost::shared_ptr<PointCloudD> transformed_cloud (new PointCloudD ());
//      // You can either apply transform_1 or transform_2; they are the same
//      pcl::transformPointCloud (*tree, *transformed_cloud, transform);
//      return transformed_cloud;

//}


//boost::shared_ptr<PointCloudI>
//PCLViewer::transform(boost::shared_ptr<PointCloudI> tree, int angle, int z_transform)
//{
//    float theta = M_PI/360*angle;
//    float height = z_transform/100;


//    Eigen::Affine3f transform = Eigen::Affine3f::Identity();

//      // Define a translation of 2.5 meters on the x axis.
//      transform.translation() << 0, 0.0, height;

//      // The same rotation matrix as before; tetha radians arround Z axis
//      transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));



//      // Executing the transformation
//      boost::shared_ptr<PointCloudI> transformed_cloud (new PointCloudI ());
//      // You can either apply transform_1 or transform_2; they are the same
//      pcl::transformPointCloud (*tree, *transformed_cloud, transform);
//      return transformed_cloud;

//}

void PCLViewer::compute_ICP() {
    allign.reset(new AllignPointCloud(this));
    //boost::shared_ptr<AllignPointCloud> allign (new AllignPointCloud(this));
    allign->setViewer(viewer);
    allign->setUi(ui);
    allign->setGuiPtr(shared_from_this());
    allign->init();
 }


void
PCLViewer::reset_crown_base()
{
    if ( getControl ()->getTreePtr()  != 0 ) {
    viewer->removeAllPointClouds ();
    viewer->removeAllShapes ();
    pcl::visualization::PointCloudColorHandlerRGBAField<PointD> rgba ( this->cloud );
    viewer->addPointCloud<PointD> ( this->cloud, rgba, "cloud" );
    point_color = 0;
    xNegView ();
    viewer->addText ( getControl ()->getTreePtr()->string(), 10, 20, 20,1, 0, 0, "tree_text" );
    ui->qvtkWidget->update ();
    reset_crown_is_active = true;
    crop_box_is_active = false;
    crop_sphere_is_active = false;
    //allign_clouds_is_active = false;
    QMessageBox::information(this, tr("Simple Tree"),
                            tr("Shift Left Click on a Point to set Crown Base."), QMessageBox::Ok);
    QCoreApplication::processEvents ();
    }
ui->qvtkWidget->update ();
}

void
PCLViewer::reset_stem()
{
    if ( getControl ()->getTreePtr()  != 0 ) {
    getControl()->getTreePtr()->reset_stem();
    setTreePtr(getControl()->getTreePtr());
    QCoreApplication::processEvents ();
    }
    ui->qvtkWidget->update ();
}

void
PCLViewer::abort_crown()
{
    hull_radius = 0.1;
}

void
PCLViewer::screenshot()
{


    QString files = QFileDialog::getSaveFileName(this, tr("Save File"),
                               "../output/Screen/", tr ( "Images (*.png *.xpm *.jpg);;All Files(*)" ) );
        if(!files.endsWith(QString(".png")))
           {
            files = files.append(".png");
        }
        viewer->saveScreenshot(files.toStdString());
}

void
PCLViewer::set_background()
{
    QColorDialog dialog (QColor(255,255,255), this);
    if ( dialog.exec () )
    {
    QColor color = dialog.selectedColor();
    int r,g,b;
    color.getRgb(&r,&g,&b);
    viewer->setBackgroundColor((float)r/255.0f,(float)g/255.0f,(float)b/255.0f);
    }

}

void
PCLViewer::compute_crown()
{
    getControl()->getTreePtr()->crown->reset_concave_hull(hull_radius);
    setTreePtr(getControl()->getTreePtr());
    ui->qvtkWidget->update ();
}

void
PCLViewer::set_crown_radius(double r)
{
    hull_radius = r;
}

void
PCLViewer::reset_crown()
{
    if ( getControl ()->getTreePtr()  != 0 ) {
        //    boost::shared_ptr<QDialog>  radius_dialog (new QDialog(0,0));
        QDialog * radius_dialog = new QDialog ( this, 0 );
        Ui_Dialog_Crown rad;
        rad.setupUi ( radius_dialog );

        connect ( rad.crown_radius, SIGNAL ( valueChanged ( double ) ), this, SLOT ( set_crown_radius ( double ) ) );
//      connect (refer.box_maxIntens, SIGNAL(valueChanged(double)), this, SLOT(set_intensity_outlier_maxIntens(double)));

        connect ( rad.abort, SIGNAL ( clicked() ), this, SLOT (abort_crown() ) );
        connect ( rad.abort, SIGNAL ( clicked() ), radius_dialog, SLOT (accept() ) );
        connect ( rad.compute, SIGNAL ( clicked() ), this, SLOT ( compute_crown() ) );
        connect ( rad.compute, SIGNAL ( clicked() ), radius_dialog, SLOT ( accept() ) );
    radius_dialog->setModal(true);
        radius_dialog->show ();

    } else {
        QMessageBox::warning ( this, tr ( "Simple Tree" ), tr (
                               "Please compute a Model first " ),
                               QMessageBox::Ok );
    }
}




void PCLViewer::changeTreeColor() {
    if ( this->getControl ()->getTreePtr () != 0 ) {
        tree_color++;
        tree_color = tree_color % tree_color_max;
        switch ( tree_color ) {
        case 0: {
            if ( this->getControl ()->getTreePtr () != 0 ) {
                std::vector<boost::shared_ptr<simpleTree::Segment> > segments = tree_ptr->getSegments();
                for ( size_t i = 0; i < segments.size(); i ++ ) {
                    double r = 0.5;
                    double g = 0.5;
                    double b = 0;
                    boost::shared_ptr<simpleTree::Segment> seg = segments.at ( i );
                    for ( size_t j = 0; j < seg->getCylinders().size(); j++ ) {
                        std::stringstream ss;
                        ss << "cylinder" << i << "," << j;
                        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_COLOR, r,g,b, ss.str () );
                    }
                    ui->qvtkWidget->update ();


                    viewer->removeShape ( "cylinder_text" );
                    viewer->addText ( "Segments are colored uni colored", 10, 150, 20,0, 0.2, 0.4, "cylinder_text" );
                    ui->qvtkWidget->update ();
                }
            }

            else {
                std::cout << "Error in PCLViewer ::changeTreeColor xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\n";
            }
            break;
        }
        case 1: {
            if ( this->getControl ()->getTreePtr () != 0 ) {
                std::vector<boost::shared_ptr<simpleTree::Segment> > segments = tree_ptr->getSegments();
                for ( size_t i = 0; i < segments.size(); i ++ ) {
		  
		  

		  
		  		  double r = (rand() % 255)/255.0f;
                    double g =(rand() % 255)/255.0f;
                    double b =(rand() % 255)/255.0f;

//                     double r = 255 * ( 1024 * rand () / ( RAND_MAX + 1.0f ) );
//                     double g = 255 * ( 1024 * rand () / ( RAND_MAX + 1.0f ) );
//                     double b = 255 * ( 1024 * rand () / ( RAND_MAX + 1.0f ) );
                    boost::shared_ptr<simpleTree::Segment> seg = segments.at ( i );
                    for ( size_t j = 0; j < seg->getCylinders().size(); j++ ) {
                        std::stringstream ss;
                        ss << "cylinder" << i << "," << j;
                        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_COLOR, r,g,b, ss.str () );
                    }
                    ui->qvtkWidget->update ();


                    viewer->removeShape ( "cylinder_text" );
                    viewer->addText ( "Segments are colored differently", 10, 150, 20,0, 0.2, 0.4, "cylinder_text" );
                    ui->qvtkWidget->update ();
		    
                }
            }

            else {
                std::cout << "Error in PCLViewer ::changeTreeColor xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\n";
            }
            break;
        }
        case 2: {
            if ( this->getControl ()->getTreePtr () != 0 ) {
                std::vector<boost::shared_ptr<simpleTree::Segment> > segments = tree_ptr->getSegments();
                int max = -1;
                int min = 2;
                for ( size_t i = 0 ; i < segments.size(); i++ ) {
                    if ( segments.at ( i )->branchID > max ) {
                        max = segments.at ( i )->branchID ;
                    }
                    if ( segments.at ( i )->branchID < min ) {
                        min = segments.at ( i )->branchID ;
                    }
                }
                std::cout << "min" <<  min ;
                std::cout << "max" <<  max ;
                std::vector<double> r_vec;
                std::vector<double> g_vec;
                std::vector<double> b_vec;
                for ( int i = min ; i < max +1; i ++ ) {
		  		  double r = (rand() % 255)/255.0f;
                    double g =(rand() % 255)/255.0f;
                    double b =(rand() % 255)/255.0f;
//                     double r = 255 * ( 1024 * rand () / ( RAND_MAX + 1.0f ) );
//                     double g = 255 * ( 1024 * rand () / ( RAND_MAX + 1.0f ) );
//                     double b = 255 * ( 1024 * rand () / ( RAND_MAX + 1.0f ) );
                    r_vec.push_back ( r );
                    g_vec.push_back ( g );
                    b_vec.push_back ( b );
                }

                for ( size_t i = 0; i < segments.size(); i ++ ) {

                    boost::shared_ptr<simpleTree::Segment> seg = segments.at ( i );
                    int branchID = seg->branchID;
                    for ( size_t j = 0; j < seg->getCylinders().size(); j++ ) {
                        std::stringstream ss;
                        ss << "cylinder" << i << "," << j;
                        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_COLOR, r_vec.at ( branchID ),g_vec.at ( branchID ),b_vec.at ( branchID ), ss.str () );
                    }
                    ui->qvtkWidget->update ();


                    viewer->removeShape ( "cylinder_text" );
                    viewer->addText ( "Branches are colored differently", 10, 150,20, 0, 0.2, 0.4, "cylinder_text" );
                    ui->qvtkWidget->update ();
                }
            }

            else {
                std::cout << "Error in PCLViewer ::changeTreeColor xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\n";
            }
            break;
        }
        case 3: {
            if ( this->getControl ()->getTreePtr () != 0 ) {
                std::vector<boost::shared_ptr<simpleTree::Segment> > segments = tree_ptr->getSegments();
                int max = -1;
                int min = 2;
                for ( size_t i = 0 ; i < segments.size(); i++ ) {
                    if ( segments.at ( i )->branchOrder > max ) {
                        max = segments.at ( i )->branchOrder ;
                    }
                    if ( segments.at ( i )->branchOrder < min ) {
                        min = segments.at ( i )->branchOrder ;
                    }
                }
                std::vector<double> r_vec;
                std::vector<double> g_vec;
                std::vector<double> b_vec;
                for ( int i = min ; i < max +1 ; i ++ ) {
		  		  double r = (rand() % 255)/255.0f;
                    double g =(rand() % 255)/255.0f;
                    double b =(rand() % 255)/255.0f;
		    std::cout << r << ";" << g << ";" << b << "\n";
//                     double r = 255 * ( 1024 * rand () / ( RAND_MAX + 1.0f ) );
//                     double g = 255 * ( 1024 * rand () / ( RAND_MAX + 1.0f ) );
//                     double b = 255 * ( 1024 * rand () / ( RAND_MAX + 1.0f ) );
                    r_vec.push_back ( r );
                    g_vec.push_back ( g );
                    b_vec.push_back ( b );
                }

                for ( size_t i = 0; i < segments.size(); i ++ ) {

                    boost::shared_ptr<simpleTree::Segment> seg = segments.at ( i );
                    int branchOrder = seg->branchOrder;
                    for ( size_t j = 0; j < seg->getCylinders().size(); j++ ) {
                        std::stringstream ss;
                        ss << "cylinder" << i << "," << j;
                        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_COLOR, r_vec.at ( branchOrder ),g_vec.at ( branchOrder ),b_vec.at ( branchOrder ), ss.str () );
                    }
                    ui->qvtkWidget->update ();


                    viewer->removeShape ( "cylinder_text" );
                    viewer->addText ( "Different branch orders are colored differently", 10, 150, 20,0, 0.2, 0.4, "cylinder_text" );
                    ui->qvtkWidget->update ();
                }
            }

            else {
                std::cout << "Error in PCLViewer ::changeTreeColor xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\n";
            }
            break;
        }
        default:
            std::cout << "Error in PCLViewer ::changePointColor xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\n";
        }
    }

}


void
PCLViewer::compute_complete_folder () {
//   std::cout<<"foo"<<std::endl;
    QDir dir ( "../data" );
    std::cout << dir.absolutePath().toStdString();
    QStringList filters;
    filters << "*.pcd" << "*.asc";
    dir.setNameFilters ( filters );
    dir.setFilter ( QDir::Files );
    QStringList files = dir.entryList ();
    for ( int i = 0; i < files.size (); i++ ) {
        QString file_abs = dir.absolutePath().append ( "/" ).append ( files.at ( i ) );
        std::cout << file_abs.toStdString();
        int index = file_abs.lastIndexOf ( "/" );
        int size = file_abs.size ();
        int position = size - index - 1;
        QString file = file_abs.right ( position );
        getControl ()->setTreeID ( file.toStdString () );
        std::string file_str = file_abs.toStdString ();
        std::string abort;
        QCoreApplication::processEvents ();
        if ( file_str != abort ) {
            ImportPCD import ( file_str, control );
            getControl ()->setCloudPtr( import.getCloud () );
            plotIntensityHist ();
        }
        getControl ()->getGuiPtr ()->writeConsole (
            "--------------------------------------------------------------------------------------------------------------------------------------------------------------\n" );

        ui->qvtkWidget->update ();
        getControl ()->getGuiPtr ()->writeConsole ( "\n" );
        getControl ()->getGuiPtr ()->writeConsole (
            "--------------------------------------------------------------------------------------------------------------------------------------------------------------\n" );
        this->updateProgress ( 0 );
        QCoreApplication::processEvents ();
        computeNormals ( getControl ()->getCloudPtr () );
        getControl ()->getGuiPtr ()->updateProgress ( 40 );
        QCoreApplication::processEvents ();
        CurvatureCloud::Ptr principalCurvatures = computeCurvature ( getControl ()->getCloudPtr () );
        getControl ()->setCurvaturePtr ( principalCurvatures );
        getControl ()->getGuiPtr ()->updateProgress ( 70 );
        QCoreApplication::processEvents ();
        tt.tic ();
        std::vector<float> e1;
        std::vector<float> e2;
        std::vector<float> e3;
        std::vector<bool> isStem;

        EigenValueEstimator es ( getControl ()->getCloudPtr (), e1, e2, e3, isStem, 0.035f );
        StemPointDetection detect ( getControl ()->getCloudPtr (), isStem );
        getControl ()->setE1 ( e1 );
        getControl ()->setE2 ( e2 );
        getControl ()->setE3 ( e3 );
        getControl ()->setIsStem ( detect.getStemPtsNew () );
        getControl ()->getGuiPtr ()->updateProgress ( 100 );
        QString str;
        float f = tt.toc () / 1000;
        str.append ( "Computed PCA analysis in  " ).append ( QString::number ( f ) ).append ( " seconds.\n" );
        getControl ()->getGuiPtr ()->writeConsole (
            "--------------------------------------------------------------------------------------------------------------------------------------------------------------\n" );
        QCoreApplication::processEvents ();
        if ( getControl ()->getCloudPtr () != 0 ) {
            //if(getControl()->getCloudPtr()->points.size() != getControl()->getIsStem().size())
            //{
            pcl::console::TicToc tt;
            tt.tic ();
            QString str = "\n";
            writeConsole ( str );
            getControl ()->getGuiPtr ()->writeConsole (
                "--------------------------------------------------------------------------------------------------------------------------------------------------------------\n" );
            updateProgress ( 0 );
            QString str2 = coeff_ptr->struct_to_qstring ( method_coefficients );
            writeConsole ( str2 );
            QCoreApplication::processEvents ();
            SphereFollowing sphereFollowing ( this->getControl ()->getCloudPtr (), control, 3, method_coefficients );
            writeConsole ( str );
            updateProgress ( 50 );
            QCoreApplication::processEvents ();
            getControl ()->getGuiPtr ()->writeConsole (
                "--------------------------------------------------------------------------------------------------------------------------------------------------------------\n" );

            tt.tic ();
            writeConsole ( str );
            QCoreApplication::processEvents ();
            getControl ()->getGuiPtr ()->writeConsole (
                "--------------------------------------------------------------------------------------------------------------------------------------------------------------\n" );
            QCoreApplication::processEvents ();
            boost::shared_ptr<simpleTree::Tree> tree = boost::make_shared<simpleTree::Tree> ( sphereFollowing.getCylinders (), this->getControl ()->getCloudPtr (),
                    this->getControl ()->getTreeID (), this->control );
            float f = tt.toc () / 1000;
            str.append ( "Done tree structure in " ).append ( QString::number ( f ) ).append ( " seconds.\n" );
            writeConsole ( str );
            updateProgress ( 100 );
            getControl ()->getGuiPtr ()->writeConsole (
                "--------------------------------------------------------------------------------------------------------------------------------------------------------------\n" );
            getControl ()->setTreePtr ( tree );
            //      }else {
            //        QMessageBox::warning(this, tr("Simple Tree"),
            //            tr("Please detect stem points first"), QMessageBox::Ok);
            //      }
        }
        if ( getControl ()->getTreePtr () != 0 ) {
            WriteCSV write ( getControl ()->getTreePtr (), getControl ()->getTreeID () );
        }
        ui->qvtkWidget->update ();
        if ( getControl ()->getTreePtr () != 0 ) {
            ExportPly tree_ply ( getControl ()->getTreePtr ()->getAllCylinders (), getControl ()->getTreeID (), "tree" );
            ExportPly stem_ply ( getControl ()->getTreePtr ()->getStemCylinders (), getControl ()->getTreeID (), "stem" );
            WriteCSV write ( getControl ()->getTreePtr (), getControl ()->getTreeID () );
        }

        ui->qvtkWidget->update ();
    }

}

void
PCLViewer::set_reference_height ( double h ) {
    height = h;
}

void
PCLViewer::compute_reference () {
    PointCloudI::Ptr cloud = getControl ()->getCloudPtr ();
    Eigen::Vector4f min_pt;
    Eigen::Vector4f max_pt;
    pcl::getMinMax3D ( *cloud, min_pt, max_pt );
    float oldMinHeight = ( min_pt[2] );
    std::cout << height << "height\n";
    std::cout << oldMinHeight << "oldheight\n";
    for ( size_t i = 0; i < cloud->points.size (); i++ ) {
//      PointI p = cloud->points.at(i);
        cloud->points.at ( i ).z = cloud->points.at ( i ).z + height - oldMinHeight;
    }
    getControl ()->setCloudPtr ( cloud );
    height = 0.2;

}

void
PCLViewer::reference_cloud () {
    {
        if ( getControl ()->getCloudPtr () != 0 ) {
            //    boost::shared_ptr<QDialog>  radius_dialog (new QDialog(0,0));
            QDialog * reference_dialog = new QDialog ( this, 0 );
            Ui_Dialog_Reference refer;
            refer.setupUi ( reference_dialog );

            connect ( refer.box_height, SIGNAL ( valueChanged ( double ) ), this, SLOT ( set_reference_height ( double ) ) );
//      connect (refer.box_maxIntens, SIGNAL(valueChanged(double)), this, SLOT(set_intensity_outlier_maxIntens(double)));

            connect ( refer.abort, SIGNAL ( clicked() ), reference_dialog, SLOT ( reject() ) );
            connect ( refer.compute, SIGNAL ( clicked() ), this, SLOT ( compute_reference() ) );
            connect ( refer.compute, SIGNAL ( clicked() ), reference_dialog, SLOT ( accept() ) );
	    reference_dialog->setModal(true);
            reference_dialog->show ();

        } else {
            QMessageBox::warning ( this, tr ( "Simple Tree" ), tr ( "No Point Cloud found\n"
                                   "Please load a Point Cloud first" ),
                                   QMessageBox::Ok );
        }
    }
}

void
PCLViewer::set_crop_box_x ( double x ) {
    if ( crop_box_is_active ) {
        deleteBox.values.at ( 0 ) = x;
        viewer->removeShape ( "deleteBox" );
        viewer->addCube ( deleteBox, "deleteBox" );
        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,
                                              "deleteBox" );
        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_COLOR, 0.8, 0.2, 0, "deleteBox" );
        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_SHADING, pcl::visualization::PCL_VISUALIZER_SHADING_GOURAUD, "deleteBox" );
        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, "deleteBox" );
        viewer->removeCoordinateSystem 	 ();
        viewer->addCoordinateSystem ( 1, deleteBox.values.at ( 0 ), deleteBox.values.at ( 1 ), deleteBox.values.at ( 2 ) );
        ui->qvtkWidget->update ();
    }
}
void
PCLViewer::set_crop_box_y ( double y ) {
    if ( crop_box_is_active ) {
        deleteBox.values.at ( 1 ) = y;
        viewer->removeShape ( "deleteBox" );
        viewer->addCube ( deleteBox, "deleteBox" );
        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,
                                              "deleteBox" );
        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_COLOR, 0.8, 0.2, 0, "deleteBox" );
        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_SHADING, pcl::visualization::PCL_VISUALIZER_SHADING_GOURAUD, "deleteBox" );
        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, "deleteBox" );
        viewer->removeCoordinateSystem 	 ();
        viewer->addCoordinateSystem ( 1, deleteBox.values.at ( 0 ), deleteBox.values.at ( 1 ), deleteBox.values.at ( 2 ) );
        ui->qvtkWidget->update ();
    }
}
void

PCLViewer::set_crop_box_z ( double z ) {
    if ( crop_box_is_active ) {
        deleteBox.values.at ( 2 ) = z;
        viewer->removeShape ( "deleteBox" );
        viewer->addCube ( deleteBox, "deleteBox" );
        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,
                                              "deleteBox" );
        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_COLOR, 0.8, 0.2, 0, "deleteBox" );
        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_SHADING, pcl::visualization::PCL_VISUALIZER_SHADING_GOURAUD, "deleteBox" );
        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, "deleteBox" );
        viewer->removeCoordinateSystem 	 ();
        viewer->addCoordinateSystem ( 1, deleteBox.values.at ( 0 ), deleteBox.values.at ( 1 ), deleteBox.values.at ( 2 ) );
        ui->qvtkWidget->update ();
    }
}
void
PCLViewer::set_crop_box_x_length ( double x ) {
    if ( crop_box_is_active ) {
        deleteBox.values.at ( 7 ) = x;
        viewer->removeShape ( "deleteBox" );
        viewer->addCube ( deleteBox, "deleteBox" );
        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,
                                              "deleteBox" );
        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_COLOR, 0.8, 0.2, 0, "deleteBox" );
        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_SHADING, pcl::visualization::PCL_VISUALIZER_SHADING_GOURAUD, "deleteBox" );
        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, "deleteBox" );
        viewer->removeCoordinateSystem 	 ();
        viewer->addCoordinateSystem ( 1, deleteBox.values.at ( 0 ), deleteBox.values.at ( 1 ), deleteBox.values.at ( 2 ) );
        ui->qvtkWidget->update ();
    }
}
void
PCLViewer::set_crop_box_y_length ( double y ) {
    if ( crop_box_is_active ) {
        deleteBox.values.at ( 8 ) = y;
        viewer->removeShape ( "deleteBox" );
        viewer->addCube ( deleteBox, "deleteBox" );
        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,
                                              "deleteBox" );
        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_COLOR, 0.8, 0.2, 0, "deleteBox" );
        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_SHADING, pcl::visualization::PCL_VISUALIZER_SHADING_GOURAUD, "deleteBox" );
        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, "deleteBox" );
        viewer->removeCoordinateSystem 	 ();
        viewer->addCoordinateSystem ( 1, deleteBox.values.at ( 0 ), deleteBox.values.at ( 1 ), deleteBox.values.at ( 2 ) );
        ui->qvtkWidget->update ();
    }
}
void
PCLViewer::set_crop_box_z_length ( double z ) {
    if ( crop_box_is_active ) {
        deleteBox.values.at ( 9 ) = z;
        viewer->removeShape ( "deleteBox" );
        viewer->addCube ( deleteBox, "deleteBox" );
        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,
                                              "deleteBox" );
        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_COLOR, 0.8, 0.2, 0, "deleteBox" );
        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_SHADING, pcl::visualization::PCL_VISUALIZER_SHADING_GOURAUD, "deleteBox" );
        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, "deleteBox" );
        viewer->removeCoordinateSystem 	 ();
        viewer->addCoordinateSystem ( 1, deleteBox.values.at ( 0 ), deleteBox.values.at ( 1 ), deleteBox.values.at ( 2 ) );
        ui->qvtkWidget->update ();
    }
}

void
PCLViewer::compute_crop_box () {
    viewer->removeShape ( "deleteBox" );
    viewer->removeCoordinateSystem 	 ();

    pcl::octree::OctreePointCloudSearch<PointI> octree ( 0.02f );
    octree.setInputCloud ( getControl ()->getCloudPtr () );
    octree.addPointsFromInputCloud ();
    PointI p1;
    p1.x = deleteBox.values.at ( 0 ) - deleteBox.values.at ( 7 ) / 2;
    p1.y = deleteBox.values.at ( 1 ) - deleteBox.values.at ( 8 ) / 2;
    p1.z = deleteBox.values.at ( 2 ) - deleteBox.values.at ( 9 ) / 2;
    PointI p2;
    p2.x = deleteBox.values.at ( 0 ) + deleteBox.values.at ( 7 ) / 2;
    p2.y = deleteBox.values.at ( 1 ) + deleteBox.values.at ( 8 ) / 2;
    p2.z = deleteBox.values.at ( 2 ) + deleteBox.values.at ( 9 ) / 2;

    std::vector<int> pointIdxBoxSearch;
    PointCloudI::Ptr cloud_filtered ( new PointCloudI );
    Eigen::Vector3f min_pt ( p1.x, p1.y, p1.z );
    Eigen::Vector3f max_pt ( p2.x, p2.y, p2.z );
    octree.boxSearch ( min_pt, max_pt, pointIdxBoxSearch );
    pcl::ExtractIndices<PointI> extract;
    pcl::PointIndices::Ptr inliers ( new pcl::PointIndices () );
    inliers->indices = pointIdxBoxSearch;
    extract.setInputCloud ( getControl ()->getCloudPtr () );
    extract.setIndices ( inliers );
    extract.setNegative ( true );
    extract.filter ( *cloud_filtered );
    getControl ()->setCloudPtr ( cloud_filtered );
    viewer->removeShape ( "deleteBox" );
    viewer->removeCoordinateSystem 	 ();
    crop_box_is_active = false;

}
void
PCLViewer::abort_crop_box () {
    viewer->removeShape ( "deleteBox" );
    viewer->removeCoordinateSystem 	 ();
    crop_box_is_active = false;
}

void
PCLViewer::crop_box () {
    if ( getControl ()->getCloudPtr () != 0 ) {
        deleteBox.values.clear ();
        deleteBox.values.push_back ( centerX );
        deleteBox.values.push_back ( centerY );
        deleteBox.values.push_back ( centerZ );
        deleteBox.values.push_back ( 0 );
        deleteBox.values.push_back ( 0 );
        deleteBox.values.push_back ( 0 );
        deleteBox.values.push_back ( 0 );
        deleteBox.values.push_back ( 0.15f );
        deleteBox.values.push_back ( 0.15f );
        deleteBox.values.push_back ( 0.15f );
        viewer->addCoordinateSystem ( 1, centerX, centerY, centerZ );
        viewer->addCube ( deleteBox, "deleteBox" );
        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,
                                              "deleteBox" );
        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_COLOR, 0.8, 0.2, 0, "deleteBox" );
        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_SHADING, pcl::visualization::PCL_VISUALIZER_SHADING_GOURAUD, "deleteBox" );
        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, "deleteBox" );
        ui->qvtkWidget->update ();
//          viewer->registerPointPickingCallback (pp_callback,(void*)&viewer);

        //    boost::shared_ptr<QDialog>  radius_dialog (new QDialog(0,0));
        QDialog *box_dialog_ptr = new QDialog ( this, 0 );
        //QDialog * sphere_dialog = new QDialog(0, 0);
//                  Ui_crop_sphere_dialog sphere;

        box_dialog_ui_ptr->setupUi ( &*box_dialog_ptr );

        connect ( box_dialog_ui_ptr->box_x, SIGNAL ( valueChanged ( double ) ), this, SLOT ( set_crop_box_x ( double ) ) );
        connect ( box_dialog_ui_ptr->box_y, SIGNAL ( valueChanged ( double ) ), this, SLOT ( set_crop_box_y ( double ) ) );
        connect ( box_dialog_ui_ptr->box_z, SIGNAL ( valueChanged ( double ) ), this, SLOT ( set_crop_box_z ( double ) ) );
        connect ( box_dialog_ui_ptr->box_x_size, SIGNAL ( valueChanged ( double ) ), this, SLOT ( set_crop_box_x_length ( double ) ) );
        connect ( box_dialog_ui_ptr->box_y_size, SIGNAL ( valueChanged ( double ) ), this, SLOT ( set_crop_box_y_length ( double ) ) );
        connect ( box_dialog_ui_ptr->box_z_size, SIGNAL ( valueChanged ( double ) ), this, SLOT ( set_crop_box_z_length ( double ) ) );
        connect ( box_dialog_ui_ptr->abort, SIGNAL ( clicked() ), this, SLOT ( abort_crop_box() ) );
        connect ( box_dialog_ui_ptr->abort, SIGNAL ( clicked() ), &*box_dialog_ptr, SLOT ( reject() ) );
        connect ( box_dialog_ui_ptr->compute, SIGNAL ( clicked() ), this, SLOT ( compute_crop_box() ) );
        connect ( box_dialog_ui_ptr->compute, SIGNAL ( clicked() ), &*box_dialog_ptr, SLOT ( accept() ) );
        //      viewer->registerPointPickingCallback (pp_callback,(void*)this);
        // viewer->registerPointPickingCallback (pp_callback, (void*) &*sphere_dialog_ui_ptr);
        crop_box_is_active = true;
	box_dialog_ptr->setModal(false);
        box_dialog_ptr->show ();

    } else {
        QMessageBox::warning ( this, tr ( "Simple Tree" ), tr ( "No Point Cloud found\n"
                               "Please load a Point Cloud first" ),
                               QMessageBox::Ok );
    }

}

void
PCLViewer::set_crop_sphere_x ( double x ) {
    if ( crop_sphere_is_active ) {
        deleteSphere.values.at ( 0 ) = x;
        viewer->removeShape ( "deleteSphere" );
        viewer->addSphere ( deleteSphere, "deleteSphere" );
        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,
                                              "deleteSphere" );
        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_COLOR, 0.8, 0.2, 0, "deleteSphere" );
        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_SHADING, pcl::visualization::PCL_VISUALIZER_SHADING_GOURAUD, "deleteSphere" );
        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, "deleteSphere" );
        viewer->removeCoordinateSystem 	 ();
        viewer->addCoordinateSystem ( 1, deleteSphere.values.at ( 0 ), deleteSphere.values.at ( 1 ), deleteSphere.values.at ( 2 ) );
        ui->qvtkWidget->update ();
    }
}
void
PCLViewer::set_crop_sphere_y ( double y ) {
    if ( crop_sphere_is_active ) {
        deleteSphere.values.at ( 1 ) = y;
        viewer->removeShape ( "deleteSphere" );
        viewer->addSphere ( deleteSphere, "deleteSphere" );
        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,
                                              "deleteSphere" );
        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_COLOR, 0.8, 0.2, 0, "deleteSphere" );
        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_SHADING, pcl::visualization::PCL_VISUALIZER_SHADING_GOURAUD, "deleteSphere" );
        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, "deleteSphere" );
        viewer->removeCoordinateSystem 	 ();
        viewer->addCoordinateSystem ( 1, deleteSphere.values.at ( 0 ), deleteSphere.values.at ( 1 ), deleteSphere.values.at ( 2 ) );
        ui->qvtkWidget->update ();
    }
}
void

PCLViewer::set_crop_sphere_z ( double z ) {
    if ( crop_sphere_is_active ) {
        deleteSphere.values.at ( 2 ) = z;
        viewer->removeShape ( "deleteSphere" );
        viewer->addSphere ( deleteSphere, "deleteSphere" );
        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,
                                              "deleteSphere" );
        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_COLOR, 0.8, 0.2, 0, "deleteSphere" );
        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_SHADING, pcl::visualization::PCL_VISUALIZER_SHADING_GOURAUD, "deleteSphere" );
        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, "deleteSphere" );
        viewer->removeCoordinateSystem 	 ();
        viewer->addCoordinateSystem ( 1, deleteSphere.values.at ( 0 ), deleteSphere.values.at ( 1 ), deleteSphere.values.at ( 2 ) );
        ui->qvtkWidget->update ();
    }
}
void
PCLViewer::set_crop_sphere_r ( double r ) {
    if ( crop_sphere_is_active ) {
        deleteSphere.values.at ( 3 ) = r;
        viewer->removeShape ( "deleteSphere" );
        viewer->addSphere ( deleteSphere, "deleteSphere" );
        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,
                                              "deleteSphere" );
        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_COLOR, 0.8, 0.2, 0, "deleteSphere" );
        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_SHADING, pcl::visualization::PCL_VISUALIZER_SHADING_GOURAUD, "deleteSphere" );
        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, "deleteSphere" );
        viewer->removeCoordinateSystem 	 ();
        viewer->addCoordinateSystem ( 1, deleteSphere.values.at ( 0 ), deleteSphere.values.at ( 1 ), deleteSphere.values.at ( 2 ) );
        ui->qvtkWidget->update ();
    }
}

void
PCLViewer::compute_crop_sphere () {
    viewer->removeShape ( "deleteSphere" );
    viewer->removeCoordinateSystem 	 ();

    pcl::octree::OctreePointCloudSearch<PointI> octree ( 0.02f );
    octree.setInputCloud ( getControl ()->getCloudPtr () );
    octree.addPointsFromInputCloud ();
    PointI p;
    p.x = deleteSphere.values.at ( 0 );
    p.y = deleteSphere.values.at ( 1 );
    p.z = deleteSphere.values.at ( 2 );
    float r = deleteSphere.values.at ( 3 );
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    PointCloudI::Ptr cloud_filtered ( new PointCloudI );
    octree.radiusSearch ( p, r, pointIdxRadiusSearch, pointRadiusSquaredDistance );
    pcl::ExtractIndices<PointI> extract;
    pcl::PointIndices::Ptr inliers ( new pcl::PointIndices () );
    inliers->indices = pointIdxRadiusSearch;
    extract.setInputCloud ( getControl ()->getCloudPtr () );
    extract.setIndices ( inliers );
    extract.setNegative ( true );
    extract.filter ( *cloud_filtered );
    getControl ()->setCloudPtr ( cloud_filtered );
    viewer->removeShape ( "deleteSphere" );
    viewer->removeCoordinateSystem 	 ();
    crop_sphere_is_active = false;

}
void
PCLViewer::abort_crop_sphere () {
    viewer->removeShape ( "deleteSphere" );
    viewer->removeCoordinateSystem 	 ();
    crop_sphere_is_active = false;
}

void
PCLViewer::cropsphere () {
    if ( getControl ()->getCloudPtr () != 0 ) {
        deleteSphere.values.clear ();
        deleteSphere.values.push_back ( centerX );
        deleteSphere.values.push_back ( centerY );
        deleteSphere.values.push_back ( centerZ );
        deleteSphere.values.push_back ( 0.5f );
        viewer->addCoordinateSystem ( 1, centerX, centerY, centerZ );
        viewer->addSphere ( deleteSphere, "deleteSphere" );
        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,
                                              "deleteSphere" );
        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_COLOR, 0.8, 0.2, 0, "deleteSphere" );
        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_SHADING, pcl::visualization::PCL_VISUALIZER_SHADING_GOURAUD, "deleteSphere" );
        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, "deleteSphere" );
        ui->qvtkWidget->update ();
//		      viewer->registerPointPickingCallback (pp_callback,(void*)&viewer);

        //		boost::shared_ptr<QDialog>  radius_dialog (new QDialog(0,0));
        QDialog *sphere_dialog_ptr = new QDialog ( this, 0 );
        //QDialog * sphere_dialog = new QDialog(0, 0);
//		              Ui_crop_sphere_dialog sphere;

        sphere_dialog_ui_ptr->setupUi ( &*sphere_dialog_ptr );

        connect ( sphere_dialog_ui_ptr->box_x, SIGNAL ( valueChanged ( double ) ), this, SLOT ( set_crop_sphere_x ( double ) ) );
        connect ( sphere_dialog_ui_ptr->box_y, SIGNAL ( valueChanged ( double ) ), this, SLOT ( set_crop_sphere_y ( double ) ) );
        connect ( sphere_dialog_ui_ptr->box_z, SIGNAL ( valueChanged ( double ) ), this, SLOT ( set_crop_sphere_z ( double ) ) );
        connect ( sphere_dialog_ui_ptr->box_r, SIGNAL ( valueChanged ( double ) ), this, SLOT ( set_crop_sphere_r ( double ) ) );
        connect ( sphere_dialog_ui_ptr->abort, SIGNAL ( clicked() ), this, SLOT ( abort_crop_sphere() ) );
        connect ( sphere_dialog_ui_ptr->abort, SIGNAL ( clicked() ), &*sphere_dialog_ptr, SLOT ( reject() ) );
        connect ( sphere_dialog_ui_ptr->compute, SIGNAL ( clicked() ), this, SLOT ( compute_crop_sphere() ) );
        connect ( sphere_dialog_ui_ptr->compute, SIGNAL ( clicked() ), &*sphere_dialog_ptr, SLOT ( accept() ) );
        //      viewer->registerPointPickingCallback (pp_callback,(void*)this);

        crop_sphere_is_active = true;
	sphere_dialog_ptr->setModal(false);
        sphere_dialog_ptr->show ();

    } else {
        QMessageBox::warning ( this, tr ( "Simple Tree" ), tr ( "No Point Cloud found\n"
                               "Please load a Point Cloud first" ),
                               QMessageBox::Ok );
    }

}

void
PCLViewer::set_intensity_outlier_minIntens ( double minIntens ) {
    float intens = minIntens;
    this->intensity_outlier_minIntens = intens;
    for(int i = 0; i < 255 ; i++)
    {
            QString cloud_name = "cloud";
                       cloud_name.append(QString::number(i));
        float val = 0;
        if(i>=intensity_outlier_minIntens&&i<=intensity_outlier_maxIntens)
        {
            val = 1;
        }
        viewer->setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_OPACITY, val, cloud_name.toStdString() );

    }
    ui->qvtkWidget->update ();
}

void
PCLViewer::set_intensity_outlier_maxIntens ( double maxIntens ) {
    float intens = maxIntens;
    this->intensity_outlier_maxIntens = intens;

    for(int i = 0; i < 255 ; i++)
    {
           QString cloud_name = "cloud";
           cloud_name.append(QString::number(i));
        float val = 0;
        if(i>=intensity_outlier_minIntens&&i<=intensity_outlier_maxIntens)
        {
            val = 1;
        }
        viewer->setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_OPACITY, val, cloud_name.toStdString() );

    }
    ui->qvtkWidget->update ();
}

void
PCLViewer::compute_intensity_outlier_removal () {
    writeConsole ( "\n" );
    getControl ()->getGuiPtr ()->writeConsole (
        "--------------------------------------------------------------------------------------------------------------------------------------------------------------\n" );

    QString str = "";
    str.append ( "Intensity outlier removal starts witn minimum Intensity = " ).append ( QString::number ( intensity_outlier_minIntens ) ).append (
        " and maximum Intensity = " ).append ( QString::number ( intensity_outlier_maxIntens ) ).append ( "\n" );
    writeConsole ( str );
    PointCloudI::Ptr cloud_filtered ( new PointCloudI );

    pcl::ConditionAnd<PointI>::Ptr intens_cond ( new pcl::ConditionAnd<PointI> () );
    intens_cond->addComparison (
        pcl::FieldComparison<PointI>::ConstPtr ( new pcl::FieldComparison<PointI> ( "intensity", pcl::ComparisonOps::GT, intensity_outlier_minIntens ) ) );
    intens_cond->addComparison (
        pcl::FieldComparison<PointI>::ConstPtr ( new pcl::FieldComparison<PointI> ( "intensity", pcl::ComparisonOps::LT, intensity_outlier_maxIntens ) ) );
    // build the filter
//		    pcl::ConditionalRemoval<PointI> condrem (intens_cond);
    pcl::ConditionalRemoval<PointI> condrem;
    condrem.setCondition ( intens_cond );
    condrem.setInputCloud ( getControl ()->getCloudPtr () );
    //condrem.setKeepOrganized(true);
    // apply filter
    condrem.filter ( *cloud_filtered );

    int size_before = getControl ()->getCloudPtr ()->points.size ();
    int size_after = cloud_filtered->points.size ();
    float a = size_before;
    float b = size_after;
    float percentage = ( 100.0f * b ) / ( a );
    getControl ()->setCloudPtr ( cloud_filtered );
    writeConsole (
        QString ( "Outlier removal done, " ).append ( QString::number ( size_after ) ).append ( " points left, size reduced to " ).append (
            QString::number ( percentage ).append ( " percent of original cloud.\n" ) ) );
    getControl ()->getGuiPtr ()->writeConsole (
        "--------------------------------------------------------------------------------------------------------------------------------------------------------------\n" );
    intensity_outlier_minIntens = 0.0;
    intensity_outlier_maxIntens = 255.0;
}

void
PCLViewer::abort_intensity_outlier_removal()
{
    intensity_clouds.clear();
    viewer->removeAllPointClouds();
    getControl()->setCloudPtr(getControl()->getCloudPtr());
}

void
PCLViewer::intensityOutlierRemoval () {
    if ( getControl ()->getCloudPtr () != 0 ) {
        viewer->removeAllPointClouds ();
        viewer->removeAllShapes ();




        intensity_clouds.clear();
        for(int i = 0 ; i < 256 ; i++)
        {
            boost::shared_ptr<PointCloudD> cloud (new PointCloudD);
            intensity_clouds.push_back(cloud);
        }

        for(size_t i = 0 ; i < getControl()->getCloudPtr()->points.size(); i++)
        {
            PointI pI = getControl()->getCloudPtr()->points.at(i);
            pcl::PointXYZRGBA pD;
            pD.x = pI.x;
            pD.y = pI.y;
            pD.z = pI.z;
            pD.r = 255 - pI.intensity;
            pD.g = pI.intensity;
            pD.b = 255 - pI.intensity;
            pD.a = 255;
            intensity_clouds.at(pI.intensity)->points.push_back(pD);
        }

        for(int i = 0 ; i < 255 ; i++)
        {
        QString cloud_name = "cloud";
            pcl::visualization::PointCloudColorHandlerRGBAField<PointD> rgba ( intensity_clouds.at(i) );
            viewer->addPointCloud<PointD> ( intensity_clouds.at(i), rgba, cloud_name.append(QString::number(i)).toStdString());
        }
        point_color = 0;
        xNegView ();

        //		boost::shared_ptr<QDialog>  radius_dialog (new QDialog(0,0));
        QDialog * intensity_dialog = new QDialog ( this, 0 );
        Ui_Dialog_Intensity intens;
        intens.setupUi ( intensity_dialog );

        connect ( intens.box_minIntens, SIGNAL ( valueChanged ( double ) ), this, SLOT ( set_intensity_outlier_minIntens ( double ) ) );
        connect ( intens.box_maxIntens, SIGNAL ( valueChanged ( double ) ), this, SLOT ( set_intensity_outlier_maxIntens ( double ) ) );
        connect ( intens.abort, SIGNAL ( clicked() ), this, SLOT (abort_intensity_outlier_removal()) );
        connect ( intens.abort, SIGNAL ( clicked() ), intensity_dialog, SLOT ( reject() ) );

        connect ( intens.compute, SIGNAL ( clicked() ), this, SLOT ( compute_intensity_outlier_removal() ) );
        connect ( intens.compute, SIGNAL ( clicked() ), intensity_dialog, SLOT ( accept() ) );
        connect ( intensity_dialog, SIGNAL (closeEvent()), this, SLOT (abort_intensity_outlier_removal()));
        intensity_dialog->setModal(true);
        intensity_dialog->show ();

    } else {
        QMessageBox::warning ( this, tr ( "Simple Tree" ), tr ( "No Point Cloud found\n"
                               "Please load a Point Cloud first" ),
                               QMessageBox::Ok );
    }
}
void
PCLViewer::set_radius_outlier_minPts ( int minPts ) {
    this->radius_outlier_minPts = minPts;
}

void
PCLViewer::set_radius_outlier_searchradius ( double searchRadius ) {
    float radius = searchRadius;
    this->radius_outlier_searchradius = radius;
}
void
PCLViewer::compute_radius_outlier_removal () {
    writeConsole ( "\n" );
    getControl ()->getGuiPtr ()->writeConsole (
        "--------------------------------------------------------------------------------------------------------------------------------------------------------------\n" );
    QString str = "";
    str.append ( "Radius outlier removal starts.\n All points who have less than " ).append ( QString::number ( radius_outlier_minPts ) ).append (
        " points in a radius of  " ).append ( QString::number ( radius_outlier_searchradius ) ).append ( "m are deleted.\n" );
    writeConsole ( str );

    PointCloudI::Ptr cloud_filtered ( new PointCloudI );
    pcl::RadiusOutlierRemoval<PointI> outrem;
    // build the filter
    outrem.setInputCloud ( getControl ()->getCloudPtr () );
    outrem.setRadiusSearch ( radius_outlier_searchradius );
    outrem.setMinNeighborsInRadius ( radius_outlier_minPts );
    // apply filter
    outrem.filter ( *cloud_filtered );

    int size_before = getControl ()->getCloudPtr ()->points.size ();
    int size_after = cloud_filtered->points.size ();
    float a = size_before;
    float b = size_after;
    float percentage = ( 100.0f * b ) / ( a );
    getControl ()->setCloudPtr ( cloud_filtered );
    writeConsole (
        QString ( "Outlier removal done, " ).append ( QString::number ( size_after ) ).append ( " points left, size reduced to " ).append (
            QString::number ( percentage ).append ( " percent of original cloud.\n" ) ) );
    getControl ()->getGuiPtr ()->writeConsole (
        "--------------------------------------------------------------------------------------------------------------------------------------------------------------\n" );
    radius_outlier_minPts = 3;
    radius_outlier_searchradius = 0.015f;

}

void
PCLViewer::radiusOutlierRemoval () {
    if ( getControl ()->getCloudPtr () != 0 ) {
//		boost::shared_ptr<QDialog>  radius_dialog (new QDialog(0,0));
        QDialog * radius_dialog = new QDialog ( this, 0 );
        Ui_Dialog_Radius rad;
        rad.setupUi ( radius_dialog );

        connect ( rad.box_minPts, SIGNAL ( valueChanged ( int ) ), this, SLOT ( set_radius_outlier_minPts ( int ) ) );
        connect ( rad.box_radius, SIGNAL ( valueChanged ( double ) ), this, SLOT ( set_radius_outlier_searchradius ( double ) ) );

        connect ( rad.abort, SIGNAL ( clicked() ), radius_dialog, SLOT ( reject() ) );
        connect ( rad.compute, SIGNAL ( clicked() ), this, SLOT ( compute_radius_outlier_removal() ) );
        connect ( rad.compute, SIGNAL ( clicked() ), radius_dialog, SLOT ( accept() ) );
	radius_dialog->setModal(true);
        radius_dialog->show ();

    } else {
        QMessageBox::warning ( this, tr ( "Simple Tree" ), tr ( "No Point Cloud found\n"
                               "Please load a Point Cloud first" ),
                               QMessageBox::Ok );
    }
}

void
PCLViewer::set_statistical_outlier_knn ( int knn ) {
    statistical_outlier_knn = knn;
}
void
PCLViewer::set_statistical_outlier_stdmult ( double stdmult ) {
    float fac = stdmult;
    statistical_outlier_stdmult = fac;
}
void
PCLViewer::compute_statistical_outlier_removal () {
    writeConsole ( "\n" );
    getControl ()->getGuiPtr ()->writeConsole (
        "--------------------------------------------------------------------------------------------------------------------------------------------------------------\n" );
    QString str = "";

    str.append ( "Statistical outlier removal starts.\n All points whith a mean distance larger than " ).append ( QString::number ( statistical_outlier_stdmult ) ).append (
        " times the average distance to its  " ).append ( QString::number ( statistical_outlier_knn ) ).append ( " nearest neighbors are deleted.\n" );
    writeConsole ( str );

    PointCloudI::Ptr cloud_filtered ( new PointCloudI );
    pcl::StatisticalOutlierRemoval<PointI> sor;
    sor.setInputCloud ( getControl ()->getCloudPtr () );
    sor.setMeanK ( statistical_outlier_knn );
    sor.setStddevMulThresh ( statistical_outlier_stdmult );
    sor.filter ( *cloud_filtered );

    int size_before = getControl ()->getCloudPtr ()->points.size ();
    int size_after = cloud_filtered->points.size ();
    float a = size_before;
    float b = size_after;
    float percentage = ( 100.0f * b ) / ( a );
    getControl ()->setCloudPtr ( cloud_filtered );
    writeConsole (
        QString ( "Outlier removal done, " ).append ( QString::number ( size_after ) ).append ( " points left, size reduced to " ).append (
            QString::number ( percentage ).append ( " percent of original cloud.\n" ) ) );
    getControl ()->getGuiPtr ()->writeConsole (
        "--------------------------------------------------------------------------------------------------------------------------------------------------------------\n" );
    statistical_outlier_knn = 5;
    statistical_outlier_stdmult = 5.0;
}

void
PCLViewer::statisticalOutlierRemoval () {
    if ( getControl ()->getCloudPtr () != 0 ) {
        //		boost::shared_ptr<QDialog>  radius_dialog (new QDialog(0,0));
        QDialog * statistical_dialog = new QDialog ( this, 0 );
        //TODO solve pointer issue
        QWebView *  qwebview =new QWebView ( statistical_dialog );
        qwebview->setObjectName ( QString::fromUtf8 ( "qwebview" ) );
        qwebview->setGeometry ( QRect ( 20, 140, 431, 192 ) );
        qwebview->setProperty ( "url", QVariant ( QUrl::fromLocalFile ( QFileInfo ( "../html/tip_stat.html" ).absoluteFilePath() ) ) );
        QWebView *  qwebview2 =new QWebView ( statistical_dialog );
        qwebview2->setObjectName ( QString::fromUtf8 ( "qwebview2" ) );
        qwebview2->setGeometry ( QRect ( 450, 20, 361, 311 ) );
        qwebview2->setProperty ( "url", QVariant ( QUrl::fromLocalFile ( QFileInfo ( "../html/cite_stat.html" ).absoluteFilePath() ) ) );
        Ui_Dialog_Statistical stat;
        stat.setupUi ( statistical_dialog );

        connect ( stat.box_knn, SIGNAL ( valueChanged ( int ) ), this, SLOT ( set_statistical_outlier_knn ( int ) ) );
        connect ( stat.box_stdmult, SIGNAL ( valueChanged ( double ) ), this, SLOT ( set_statistical_outlier_stdmult ( double ) ) );

        connect ( stat.abort, SIGNAL ( clicked() ), statistical_dialog, SLOT ( reject() ) );
        connect ( stat.compute, SIGNAL ( clicked() ), this, SLOT ( compute_statistical_outlier_removal() ) );
        connect ( stat.compute, SIGNAL ( clicked() ), statistical_dialog, SLOT ( accept() ) );
	statistical_dialog->setModal(true);
        statistical_dialog->show ();

    } else {
        QMessageBox::warning ( this, tr ( "Simple Tree" ), tr ( "No Point Cloud found\n"
                               "Please load a Point Cloud first" ),
                               QMessageBox::Ok );
    }
//	if (getControl()->getCloudPtr() != 0) {
////		writeConsole("\n");
////		getControl()->getGuiPtr()->writeConsole(
////				"--------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
////		QString str = "";
////
////								 str.append("Statistical outlier removal starts.\n All points whith a mean distance larger than ").
////										 append(QString::number(statistical_outlier_stdmult = 5.0))
////										 .append(" times the average distance to its  ").append(QString::number(statistical_outlier_knn))
////										 .append(" nearest neighbors are deleted.\n");
////						writeConsole(str);
////
////		PointCloudI::Ptr cloud_filtered(new PointCloudI);
////		pcl::StatisticalOutlierRemoval<PointI> sor;
////		sor.setInputCloud(getControl()->getCloudPtr());
////		sor.setMeanK(5);
////		sor.setStddevMulThresh(4.0);
////		sor.filter(*cloud_filtered);
////
////		int size_before = getControl()->getCloudPtr()->points.size();
////		int size_after = cloud_filtered->points.size();
////		float a = size_before;
////		float b = size_after;
////		float percentage = (100.0f * b) / (a);
////		getControl()->setCloudPtr(cloud_filtered);
////		writeConsole(
////				QString("Outlier removal done, ").append(
////						QString::number(size_after)).append(
////						" points left, size reduced to ").append(
////						QString::number(percentage).append(
////								" percent of original cloud.\n")));
////		getControl()->getGuiPtr()->writeConsole(
////				"--------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
//	} else {
//		QMessageBox::warning(this, tr("Simple Tree"),
//				tr("No Point Cloud found\n"
//						"Please load a Point Cloud first"), QMessageBox::Ok);
//	}
}
void
PCLViewer::set_voxel_grid_size ( double size ) {
    float sizeF = size;
    voxel_grid_size = sizeF;
}

void
PCLViewer::compute_voxel_grid_downsampling () {
    writeConsole ( "\n" );
    getControl ()->getGuiPtr ()->writeConsole (
        "--------------------------------------------------------------------------------------------------------------------------------------------------------------\n" );

    PointCloudI::Ptr cloud_filtered ( new PointCloudI );
    pcl::VoxelGrid<PointI> sor;
    QString str = "";

    str.append ( "Voxel grid filtering starts.\n, all points inside a voxel with " ).append ( QString::number ( voxel_grid_size ) ).append (
        " side length are merged to one point.\n" );
    writeConsole ( str );

    sor.setInputCloud ( getControl ()->getCloudPtr () );
    sor.setLeafSize ( voxel_grid_size, voxel_grid_size, voxel_grid_size );
    sor.filter ( *cloud_filtered );

    int size_before = getControl ()->getCloudPtr ()->points.size ();
    int size_after = cloud_filtered->points.size ();
    float a = size_before;
    float b = size_after;
    float percentage = ( 100.0f * b ) / ( a );
    getControl ()->setCloudPtr ( cloud_filtered );
    writeConsole (
        QString ( "Downsampling done, " ).append ( QString::number ( size_after ) ).append ( " points left, size reduced to " ).append (
            QString::number ( percentage ).append ( " percent of original cloud.\n" ) ) );

    getControl ()->getGuiPtr ()->writeConsole (
        "--------------------------------------------------------------------------------------------------------------------------------------------------------------\n" );
    voxel_grid_size = 0.01;
}

void
PCLViewer::voxel_downsampling ()

{
    if ( getControl ()->getCloudPtr () != 0 ) {
        //		boost::shared_ptr<QDialog>  radius_dialog (new QDialog(0,0));
        QDialog * voxel_dialog = new QDialog ( this, 0 );
        Ui_Dialog_Grid_Downsampling down;
        down.setupUi ( voxel_dialog );

        connect ( down.box_gridsize, SIGNAL ( valueChanged ( double ) ), this, SLOT ( set_voxel_grid_size ( double ) ) );
        connect ( down.abort, SIGNAL ( clicked() ), voxel_dialog, SLOT ( reject() ) );
        connect ( down.compute, SIGNAL ( clicked() ), this, SLOT ( compute_voxel_grid_downsampling() ) );
        connect ( down.compute, SIGNAL ( clicked() ), voxel_dialog, SLOT ( accept() ) );
        voxel_dialog->setModal(true);
        voxel_dialog->show ();
	

    } else {
        QMessageBox::warning ( this, tr ( "Simple Tree" ), tr ( "No Point Cloud found\n"
                               "Please load a Point Cloud first" ),
                               QMessageBox::Ok );
    }
}
void
PCLViewer::set_euclidean_clustering_minsize ( int size ) {
    euclidean_clustering_minsize = size;
}
void
PCLViewer::set_euclidean_clustering_tolerance ( double tolerance ) {
    float tol = tolerance;
    euclidean_clustering_tolerance = tol;
}
void
PCLViewer::set_euclidean_clustering_clusternumber ( int number ) {
    euclidean_clustering_clusternumber = number;
}
void
PCLViewer::compute_euclidean_clustering () {
    writeConsole ( "\n" );
    getControl ()->getGuiPtr ()->writeConsole (
        "--------------------------------------------------------------------------------------------------------------------------------------------------------------\n" );
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::search::KdTree<PointI>::Ptr tree ( new pcl::search::KdTree<PointI> );
    tree->setInputCloud ( getControl ()->getCloudPtr () );
    pcl::EuclideanClusterExtraction<PointI> ec;
    ec.setClusterTolerance ( euclidean_clustering_tolerance ); // 2cm
    ec.setMinClusterSize ( euclidean_clustering_minsize );
    ec.setSearchMethod ( tree );
    ec.setInputCloud ( getControl ()->getCloudPtr () );
    ec.extract ( cluster_indices );
    PointCloudI::Ptr cloud_filtered ( new PointCloudI );
    if ( cluster_indices.size () > 0 ) {
        int i = std::min<int> ( euclidean_clustering_clusternumber, cluster_indices.size () );
        for ( int j = 0; j < i; j++ ) {
            pcl::PointIndices largestCluster = cluster_indices.at ( j );
            for ( std::vector<int>::const_iterator pit = largestCluster.indices.begin (); pit != largestCluster.indices.end (); pit++ )
                cloud_filtered->points.push_back ( getControl ()->getCloudPtr ()->points[*pit] ); //*
        }
    }

    int size_before = getControl ()->getCloudPtr ()->points.size ();
    int size_after = cloud_filtered->points.size ();
    cloud_filtered->width = size_after;
    cloud_filtered->height = 1;
    float a = size_before;
    float b = size_after;
    float percentage = ( 100.0f * b ) / ( a );
    getControl ()->setCloudPtr ( cloud_filtered );
    writeConsole (
        QString ( "Clustering done, in the " ).append ( QString::number ( euclidean_clustering_clusternumber ) ).append ( " largest clusters " ).append (
            QString::number ( size_after ) ).append ( " points left, size reduced to " ).append (
            QString::number ( percentage ).append ( " percent of original cloud.\n" ) ) );
    getControl ()->getGuiPtr ()->writeConsole (
        "--------------------------------------------------------------------------------------------------------------------------------------------------------------\n" );
    euclidean_clustering_minsize = 100;
    euclidean_clustering_tolerance = 0.02;
    euclidean_clustering_clusternumber = 1;
}

void
PCLViewer::euclideanClustering () {

    if ( getControl ()->getCloudPtr () != 0 ) {
        QDialog * cluster_dialog = new QDialog ( this, 0 );
        Ui_Dialog_Euclidean_Clustering cluster;
        cluster.setupUi ( cluster_dialog );

        connect ( cluster.box_minsize, SIGNAL ( valueChanged ( int ) ), this, SLOT ( set_euclidean_clustering_minsize ( int ) ) );
        connect ( cluster.box_numbercluster, SIGNAL ( valueChanged ( int ) ), this, SLOT ( set_euclidean_clustering_clusternumber ( int ) ) );
        connect ( cluster.box_tolerance, SIGNAL ( valueChanged ( double ) ), this, SLOT ( set_euclidean_clustering_tolerance ( double ) ) );

        connect ( cluster.abort, SIGNAL ( clicked() ), cluster_dialog, SLOT ( reject() ) );
        connect ( cluster.compute, SIGNAL ( clicked() ), this, SLOT ( compute_euclidean_clustering() ) );
        connect ( cluster.compute, SIGNAL ( clicked() ), cluster_dialog, SLOT ( accept() ) );
	cluster_dialog->setModal(true);
        cluster_dialog->show ();
    } else {
        QMessageBox::warning ( this, tr ( "Simple Tree" ), tr ( "No Point Cloud found\n"
                               "Please load a Point Cloud first" ),
                               QMessageBox::Ok );
    }
}

void
PCLViewer::changePointColor () {
    if ( this->getControl ()->getCloudPtr () != 0 ) {
        point_color++;
        point_color = point_color % point_color_max;
        switch ( point_color ) {
        case 0: {
            if ( this->getControl ()->getCloudPtr () != 0 ) {
                PointCloudI::Ptr cloud2 = this->getControl ()->getCloudPtr ();
                if ( cloud2->points.size () == this->cloud->points.size () ) {
                    for ( size_t i = 0; i < cloud2->points.size (); i++ ) {
                        float intens = cloud2->points[i].intensity;
                        if ( intens == 0 ) {
                            intens = 180;
                        }
                        cloud2->points[i].intensity = intens;
                        this->cloud->points[i].r = 255 - intens;
                        this->cloud->points[i].g = intens;
                        this->cloud->points[i].b = 255 - intens;
                        this->cloud->points[i].a = 255;
                    }
                } else {
                    std::cout << "Error in PCLViewer ::changePointColor xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\n";
                }
                viewer->updatePointCloud ( cloud, "cloud" );
                viewer->removeShape ( "point_text" );
                viewer->addText ( "Point Color by Intensity", 10, 120,20 ,0, 0.2, 0.4, "point_text" );
                ui->qvtkWidget->update ();
            } else {
                QMessageBox::warning ( this, tr ( "Simple Tree" ), tr ( "No Point Cloud found\n"
                                       "Please load a Point Cloud first" ),
                                       QMessageBox::Ok );
            }
            break;
        }
        case 1: {
            if ( this->getControl ()->getCurvaturePtr () != 0 ) {
                CurvatureCloud::Ptr cloud2 = this->getControl ()->getCurvaturePtr ();
                if ( cloud2->points.size () == this->cloud->points.size () ) {
                    float maxX = -10000000, maxY = -10000000, maxZ = -10000000;
                    for ( size_t i = 0; i < cloud2->points.size (); i++ ) {
                        if ( cloud2->points[i].principal_curvature_x > maxX ) {
                            maxX = cloud2->points[i].principal_curvature_x;
                        }
                        if ( cloud2->points[i].principal_curvature_y > maxY ) {
                            maxY = cloud2->points[i].principal_curvature_y;
                        }
                        if ( cloud2->points[i].principal_curvature_z > maxZ ) {
                            maxZ = cloud2->points[i].principal_curvature_z;
                        }
                    }
                    for ( size_t i = 0; i < cloud2->points.size (); i++ ) {
                        this->cloud->points[i].r = 255 * ( cloud2->points[i].principal_curvature_x / maxX );
                        this->cloud->points[i].g = 255 * ( cloud2->points[i].principal_curvature_y / maxY );
                        this->cloud->points[i].b = 255 * ( cloud2->points[i].principal_curvature_z / maxZ );
                        this->cloud->points[i].a = 255;
                    }
                } else {
                    std::cout << "Error in PCLViewer ::changePointColor xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\n";
                }
                viewer->removeShape ( "point_text" );
                viewer->addText ( "Point Color by Principal Curvature direction", 10, 120, 20,0, 0.2, 0.4, "point_text" );
                viewer->updatePointCloud ( cloud, "cloud" );
                ui->qvtkWidget->update ();
            } else {
                QMessageBox::warning ( this, tr ( "Simple Tree" ), tr ( "No Principal Curvatures found\n"
                                       "Please load a Point Cloud first" ),
                                       QMessageBox::Ok );
            }
            break;
        }
        case 2: {
            if ( this->getControl ()->getCurvaturePtr () != 0 ) {
                CurvatureCloud::Ptr cloud2 = this->getControl ()->getCurvaturePtr ();
                if ( cloud2->points.size () == this->cloud->points.size () ) {
                    float maxI = -10000000, maxJ = -10000000;
                    for ( size_t i = 0; i < cloud2->points.size (); i++ ) {
                        if ( cloud2->points[i].pc1 > maxI ) {
                            maxI = cloud2->points[i].pc1;
                        }
                        if ( cloud2->points[i].pc2 > maxJ ) {
                            maxJ = cloud2->points[i].pc2;
                        }
                    }
                    for ( size_t i = 0; i < cloud2->points.size (); i++ ) {
                        this->cloud->points[i].r = 255 * ( cloud2->points[i].pc1 / maxI );
                        this->cloud->points[i].g = 255 * ( cloud2->points[i].pc2 / maxJ );
                        this->cloud->points[i].b = 122;
                        this->cloud->points[i].a = 255;
                    }
                } else {
                    std::cout << "Error in PCLViewer ::changePointColor xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\n";
                }
                viewer->removeShape ( "point_text" );
                viewer->addText ( "Point Color by Principal Curvature magnitude", 10, 120, 20,0, 0.2, 0.4, "point_text" );
                viewer->updatePointCloud ( cloud, "cloud" );
                ui->qvtkWidget->update ();
            } else {
                QMessageBox::warning ( this, tr ( "Simple Tree" ), tr ( "No Principal Curvatures found\n"
                                       "Please load a Point Cloud first" ),
                                       QMessageBox::Ok );
            }
            break;
        }
        case 3: {
            if ( this->getControl ()->getCloudPtr () != 0 ) {
                PointCloudI::Ptr cloud2 = this->getControl ()->getCloudPtr ();
                if ( cloud2->points.size () == this->cloud->points.size () ) {
                    float maxX = -10000000;
                    for ( size_t i = 0; i < cloud2->points.size (); i++ ) {
                        if ( cloud2->points[i].curvature > maxX ) {
                            maxX = cloud2->points[i].curvature;
                        }
                    }
                    for ( size_t i = 0; i < cloud2->points.size (); i++ ) {
                        this->cloud->points[i].r = 255 * ( cloud2->points[i].curvature / maxX );
                        this->cloud->points[i].g = 255 * ( cloud2->points[i].curvature / maxX );
                        this->cloud->points[i].b = 122;
                        this->cloud->points[i].a = 255;
                    }
                } else {
                    std::cout << "Error in PCLViewer ::changePointColor xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\n";
                }
                viewer->removeShape ( "point_text" );
                viewer->addText ( "Point Color by Curvature", 10, 120,20, 0, 0.2, 0.4, "point_text" );
                viewer->updatePointCloud ( cloud, "cloud" );
                ui->qvtkWidget->update ();
            } else {
                QMessageBox::warning ( this, tr ( "Simple Tree" ), tr ( "No Point Cloud found\n"
                                       "Please load a Point Cloud first" ),
                                       QMessageBox::Ok );
            }
            break;
        }
        case 4: {
            if ( this->getControl ()->getCloudPtr () != 0 ) {
                PointCloudI::Ptr cloud2 = this->getControl ()->getCloudPtr ();
                std::vector<float> e1 = this->getControl ()->getE1 ();
                std::vector<float> e2 = this->getControl ()->getE2 ();
                std::vector<float> e3 = this->getControl ()->getE3 ();
                if ( cloud2->points.size () == e1.size () && cloud2->points.size () == e3.size () && cloud2->points.size () == e2.size () ) {
                    float maxE1 = std::numeric_limits<float>::min ();
                    float maxE2 = std::numeric_limits<float>::min ();
                    float maxE3 = std::numeric_limits<float>::min ();
                    for ( size_t i = 0; i < cloud2->points.size (); i++ ) {
                        if ( e1[i] > maxE1 ) {
                            maxE1 = e1[i];
                        }
                        if ( e2[i] > maxE2 ) {
                            maxE2 = e2[i];
                        }
                        if ( e3[i] > maxE3 ) {
                            maxE3 = e3[i];
                        }

                    }
                    for ( size_t i = 0; i < cloud2->points.size (); i++ ) {
                        this->cloud->points[i].r = 255 * ( e1[i] / maxE1 );
                        this->cloud->points[i].g = 255 * ( e2[i] / maxE2 );
                        this->cloud->points[i].b = 255 * ( e3[i] / maxE3 );
                        this->cloud->points[i].a = 255;
                    }
                } else {
                    std::cout << "Error in PCLViewer ::changePointColor xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\n";
                }
                viewer->removeShape ( "point_text" );
                viewer->addText ( "Point Color by eigen-values of PCA", 10, 120,20, 0, 0.2, 0.4, "point_text" );
                viewer->updatePointCloud ( cloud, "cloud" );
                ui->qvtkWidget->update ();
            } else {
                QMessageBox::warning ( this, tr ( "Simple Tree" ), tr ( "No Point Cloud found\n"
                                       "Please load a Point Cloud first" ),
                                       QMessageBox::Ok );
            }
            break;
        }
        case 5: {
            if ( this->getControl ()->getCloudPtr () != 0 ) {
                PointCloudI::Ptr cloud2 = this->getControl ()->getCloudPtr ();
                std::vector<bool> isStem = this->getControl ()->getIsStem ();
                if ( isStem.size () == cloud2->points.size () ) {
                    for ( size_t i = 0; i < cloud2->points.size (); i++ ) {
                        if ( isStem.at ( i ) ) {
                            this->cloud->points[i].r = 255;
                            this->cloud->points[i].g = 0;
                            this->cloud->points[i].b = 0;
                            this->cloud->points[i].a = 255;
                        } else {
                            this->cloud->points[i].r = 0;
                            this->cloud->points[i].g = 0;
                            this->cloud->points[i].b = 255;
                            this->cloud->points[i].a = 255;
                        }
                    }
                }

                else {
                    std::cout << "Error in PCLViewer ::changePointColor xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\n";
                }
                viewer->removeShape ( "point_text" );
                viewer->addText ( "Point Color by stem detection", 10, 120,20, 0, 0.2, 0.4, "point_text" );
                viewer->updatePointCloud ( cloud, "cloud" );
                ui->qvtkWidget->update ();
            } else {
                QMessageBox::warning ( this, tr ( "Simple Tree" ), tr ( "No Point Cloud found\n"
                                       "Please load a Point Cloud first" ),
                                       QMessageBox::Ok );
            }
            break;
        }
        default:
            std::cout << "Error in PCLViewer ::changePointColor xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\n";
        }
    }
}

void
PCLViewer::exportPCDFile () {
    if ( getControl ()->getCloudPtr () != 0 ) {
        QFileDialog dialog ( this, tr ( "Save File" ), "../data/", tr ( "Point Cloud File(*pcd)" ) );
        dialog.setAcceptMode ( QFileDialog::AcceptSave );
        dialog.setOptions ( ( QFileDialog::DontUseNativeDialog ) );
        dialog.setViewMode ( QFileDialog::Detail );
        QString file;
        QStringList files;
        if ( dialog.exec () )
            files = dialog.selectedFiles ();
        file = files.at ( 0 );
        if ( !file.endsWith ( ".pcd" ) ) {
            file.append ( ".pcd" );
        }

        pcl::io::savePCDFileASCII<PointI> ( file.toStdString (), * ( getControl ()->getCloudPtr () ) );
    }
}

void
PCLViewer::updateProgress ( int i ) {
    if ( i == 0 ) {
        ui->progress_bar->reset ();
    }
    ui->progress_bar->setValue ( i );
    ui->qvtkWidget->update ();
}

void
PCLViewer::exportPly () {
    if ( getControl ()->getTreePtr () != 0 ) {
        ExportPly tree_ply ( getControl ()->getTreePtr ()->getAllCylinders (), getControl ()->getTreeID (), "tree" );
        ExportPly stem_ply ( getControl ()->getTreePtr ()->getStemCylinders (), getControl ()->getTreeID (), "stem" );
        WriteCSV write ( getControl ()->getTreePtr (), getControl ()->getTreeID () );
    } else {
        QMessageBox::warning ( this, tr ( "Simple Tree" ), tr ( "No Tree Model computed\n" ), QMessageBox::Ok );
    }
    ui->qvtkWidget->update ();
}

void
PCLViewer::exportResults () {
    if ( getControl ()->getTreePtr () != 0 ) {
        WriteCSV write ( getControl ()->getTreePtr (), getControl ()->getTreeID () );
    } else {
        QMessageBox::warning ( this, tr ( "Simple Tree" ), tr ( "No Tree Model computed\n" ), QMessageBox::Ok );
    }
    ui->qvtkWidget->update ();
}

void
PCLViewer::generateCloud () {

    cloud.reset ( new PointCloudD );
    uint8_t r ( 255 ), g ( 15 ), b ( 15 );
    for ( float z ( 0.2 ); z <= 5.0; z += 0.02 ) {
        for ( float angle ( 0.0 ); angle <= 360.0; angle += 1.0 ) {
            PointD basic_point;
            basic_point.x = 0.8 * cos ( pcl::deg2rad ( angle ) );
            basic_point.y = sin ( pcl::deg2rad ( angle ) );
            basic_point.z = z;
            basic_point.r = r;
            basic_point.g = g;
            basic_point.b = b;
            basic_point.a = 255;
            cloud->points.push_back ( basic_point );
        }
        if ( z < 2.6 ) {
            r -= 1;
            g += 1;
        } else {
            g -= 1;
            b += 1;
        }
    }
}

void
PCLViewer::plotIntensityHist () {
    std::vector<double> histData;
    PointCloudI::Ptr cloud = getControl ()->getCloudPtr ();
    int min = std::numeric_limits<int>::max ();
    int max = std::numeric_limits<int>::min ();
    for ( size_t i = 0; i < cloud->points.size (); i++ ) {
        double d = cloud->points[i].intensity;
        histData.push_back ( d );
        if ( d < min ) {
            min = d;
        }
        if ( d > max ) {
            max = d;
        }
    }
    histData.push_back ( 0 );
    histData.push_back ( 255 );

    std::cout << min << ";" << max << "\n";
    plotter->clearPlots ();
    plotter->setTitle ( "Intensity histogram" );
    plotter->setXTitle ( "Intensity (0-255)" );
    plotter->setYTitle ( "" );
    plotter->setShowLegend ( false );
    plotter->addHistogramData ( histData, 255 );
    plotter->setXRange ( 0, 255 );
    plotter->setYRange ( 0, cloud->points.size () / 50 );
    ui->qvtkWidget2->update ();
}

void
PCLViewer::plotAllometry () {

    std::vector<boost::shared_ptr<simpleTree::Cylinder> > cylinders = getControl ()->getTreePtr ()->getAllCylinders ();
    double diameters[cylinders.size ()];
    double volumes[cylinders.size ()];
    double max_volume = std::numeric_limits<double>::min ();
    double max_diameter = std::numeric_limits<double>::min ();
    for ( size_t i = 0; i < cylinders.size (); i++ ) {
        boost::shared_ptr<simpleTree::Cylinder> cylinder = cylinders.at ( i );
        double diameter = cylinder->getRadius () * 200;
        if ( diameter > max_diameter ) {
            max_diameter = diameter;
        }
        diameters[i] = diameter;
        double volume = getControl ()->getTreePtr ()->getGrowthVolume ( cylinder );
        volumes[i] = volume;
        if ( volume > max_volume ) {
            max_volume = volume;
        }
    }

    plotter->clearPlots ();
    plotter->setTitle ( "Allometric Model" );
    plotter->setXTitle ( "Diameter (cm)" );
    plotter->setYTitle ( "Volume (l)" );
    plotter->setShowLegend ( false );
    plotter->addPlotData ( diameters, volumes, cylinders.size (), "Volume (l)", vtkChart::POINTS );
    plotter->setXRange ( 0, max_diameter );
    plotter->setYRange ( 0, max_volume );
    ui->qvtkWidget2->update ();
}

void
PCLViewer::generateData ( double* ax,
                          double* acos,
                          double* asin,
int numPoints ) {
    double inc = 7.5 / ( numPoints - 1 );
    for ( int i = 0; i < numPoints; ++i ) {
        ax[i] = i * inc;
        acos[i] = cos ( i * inc );
        asin[i] = sin ( i * inc );
    }
}

void
PCLViewer::plot () {
    plotter->setShowLegend ( true );

    //generating point correspondances
    int numPoints = 69;
    double ax[100], acos[100], asin[100];
    generateData ( ax, acos, asin, numPoints );
    std::vector<double> vec;
    for ( int i = 0; i < 1000; i++ ) {
        vec.push_back ( 0 );
    }
    //adding plot data
    plotter->addHistogramData ( vec, 10 );
//    plotter->addPlotData ( ax, acos, numPoints, "cos" );
//    plotter->addPlotData ( ax, asin, numPoints, "sin" );

    //display for 2 seconds
    plotter->spin ();
//   plotter->clearPlots ();
    ui->qvtkWidget2->update ();
}

void
PCLViewer::writeConsole ( QString qstr ) {
    consoleString.append ( qstr );
    ui->console->setText ( consoleString );
//     ui->console->ensureCursorVisible();
    ui->console->verticalScrollBar ()->setValue ( ui->console->verticalScrollBar ()->maximum () );
    ui->qvtkWidget->update ();
}

boost::shared_ptr<Controller>
PCLViewer::getControl () {
    return control.lock ();
}

void
PCLViewer::connectToController ( boost::shared_ptr<Controller> control ) {
    this->control = control;
}

//void
//PCLViewer::setCrownVisibility ()
//{
//  if (getControl ()->getTreePtr () != 0)
//  {
//    if (isVisibleCrown)
//    {
//      viewer->removePolygonMesh ("crown");
//      isVisibleCrown = false;
//    }
//    else
//    {
//      pcl::PointCloud<pcl::PointXYZ>::Ptr hullPoints = getControl ()->getTreePtr ()->crown->hull_cloud;
//      std::vector<pcl::Vertices> polygons = getControl ()->getTreePtr ()->crown->polygons;
//      viewer->addPolygonMesh<pcl::PointXYZ> (hullPoints, polygons, "crown");
//      isVisibleCrown = true;
//    }
//  }
//}
//
//void
//PCLViewer::setLeaveVisibility ()
//{
//  if (getControl ()->getTreePtr () != 0)
//  {
//    if (isVisibleLeaves)
//    {
//      viewer->removePolygonMesh ("leaves");
//      isVisibleLeaves = false;
//    }
//    else
//    {
//      pcl::PointCloud<pcl::PointXYZ>::Ptr hullPoints_concave = getControl ()->getTreePtr ()->crown->hull_cloud_concave;
//      std::vector<pcl::Vertices> polygons_concave = getControl ()->getTreePtr ()->crown->polygons_concave;
//      viewer->addPolygonMesh<pcl::PointXYZ> (hullPoints_concave, polygons_concave, "leaves");
//      isVisibleLeaves = true;
//    }
//  }
//}

void
PCLViewer::setTreePtr ( boost::shared_ptr<simpleTree::Tree> tree_ptr ) {

    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
    this->tree_ptr = tree_ptr;
    setCloudPtr(getControl()->getCloudPtr());
    cylinders = tree_ptr->getAllCylinders();
    tree_color = 0;
    std::vector<boost::shared_ptr<simpleTree::Segment> > segments = tree_ptr->getSegments();
    for ( size_t i = 0; i < segments.size(); i ++ ) {
        boost::shared_ptr<simpleTree::Segment> seg = segments.at ( i );
        for ( size_t j = 0; j < seg->getCylinders().size(); j++ ) {
            std::stringstream ss;
            ss << "cylinder" << i << "," << j;
            pcl::ModelCoefficients coeff;
            coeff.values.push_back ( seg->getCylinders() [j]->values[0] );
            coeff.values.push_back ( seg->getCylinders() [j]->values[1] );
            coeff.values.push_back ( seg->getCylinders() [j]->values[2] );
            coeff.values.push_back ( seg->getCylinders() [j]->values[3] );
            coeff.values.push_back ( seg->getCylinders() [j]->values[4] );
            coeff.values.push_back ( seg->getCylinders() [j]->values[5] );
            coeff.values.push_back ( seg->getCylinders() [j]->values[6] );
            viewer->addCylinder ( coeff, ss.str () );
            viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,
                                                  ss.str () );
            viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 0.5, 0, ss.str () );
            viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_SHADING, pcl::visualization::PCL_VISUALIZER_SHADING_GOURAUD, ss.str () );
            viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, ss.str () );
        }
    }

    stemCylinders = tree_ptr->getStemCylinders ();
    for ( size_t i = 0; i < stemCylinders.size (); i++ ) {
        std::stringstream ss;
        ss << "stemCylinder" << i;
        pcl::ModelCoefficients coeff;
        coeff.values.push_back ( stemCylinders[i]->values[0] );
        coeff.values.push_back ( stemCylinders[i]->values[1] );
        coeff.values.push_back ( stemCylinders[i]->values[2] );
        coeff.values.push_back ( stemCylinders[i]->values[3] );
        coeff.values.push_back ( stemCylinders[i]->values[4] );
        coeff.values.push_back ( stemCylinders[i]->values[5] );
        coeff.values.push_back ( stemCylinders[i]->values[6] );
        viewer->addCylinder ( coeff, ss.str () );
        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,
                                              ss.str () );
    if(coeff.values[2]<=1.3f&&(coeff.values[2]+coeff.values[5])>1.3f)
	{
      viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 1.0, 0.5, ss.str () );
	}
	else
	{
	  viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.5, 0, ss.str () );
	}
        
        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_SHADING, pcl::visualization::PCL_VISUALIZER_SHADING_GOURAUD, ss.str () );
        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, ss.str () );
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr hullPoints = tree_ptr->crown->hull_cloud;
    std::vector<pcl::Vertices> polygons = tree_ptr->crown->polygons;
    viewer->addPolygonMesh<pcl::PointXYZ> ( hullPoints, polygons, "crown" );
    viewer->setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_COLOR, 0.2, 0.45, 0, "crown" );
//     viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_SHADING, pcl::visualization::PCL_VISUALIZER_SHADING_GOURAUD, "crown");
//     viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "crown");

    pcl::PointCloud<pcl::PointXYZ>::Ptr hullPoints_concave = tree_ptr->crown->hull_cloud_concave;
    std::vector<pcl::Vertices> polygons_concave = tree_ptr->crown->polygons_concave;
    viewer->addPolygonMesh<pcl::PointXYZ> ( hullPoints_concave, polygons_concave, "leaves" );
    viewer->setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_COLOR, 0.2, 0.45, 0, "leaves" );
//     viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_SHADING, pcl::visualization::PCL_VISUALIZER_SHADING_GOURAUD, "leaves");
//     viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5,  "leaves");


    plotAllometry ();
    ui->qvtkWidget->update ();
    ui->qvtkWidget2->update ();
    ui->tree_color_slider->setValue ( 1 ); //dummi to fire signal changed.
    ui->stem_color_slider->setValue ( 1 );
    ui->point_color_slider->setValue ( 1 );
    ui->crown_color_slider->setValue ( 1 );
    ui->leave_color_slider->setValue ( 1 );
    ui->tree_color_slider->setValue ( 0 );
    ui->stem_color_slider->setValue ( 0 );
    ui->point_color_slider->setValue ( 240 );
    ui->crown_color_slider->setValue ( 225 );
    ui->leave_color_slider->setValue ( 240 );
        viewer->addText ( tree_ptr->string (), 10, 20, 20,1, 0, 0, "tree_text" );
    ui->qvtkWidget->update ();
}

void
PCLViewer::computeNormals ( PointCloudI::Ptr cloud ) {
    tt.tic ();
    pcl::NormalEstimationOMP<PointI, PointI> ne ( 0 );
    ne.setInputCloud ( cloud );
    pcl::search::KdTree<PointI>::Ptr tree ( new pcl::search::KdTree<PointI> () );
    ne.setSearchMethod ( tree );

    ne.setKSearch ( 75 );
    ne.compute ( *cloud );
    QString str;
    float f = tt.toc () / 1000;
    str.append ( "Computed normals in  " ).append ( QString::number ( f ) ).append ( " seconds.\n" );
    getControl ()->getGuiPtr ()->writeConsole ( str );
    QCoreApplication::processEvents ();
}

CurvatureCloud::Ptr
PCLViewer::computeCurvature ( PointCloudI::Ptr cloud ) {
    tt.tic ();
    //std::cout << "computing curvatures" << "\n", tt.tic();
    pcl::PrincipalCurvaturesEstimation<PointI, PointI, pcl::PrincipalCurvatures> principalCurvaturesEstimation;
    principalCurvaturesEstimation.setInputCloud ( cloud );
    principalCurvaturesEstimation.setInputNormals ( cloud );
    pcl::search::KdTree<PointI>::Ptr tree_normal ( new pcl::search::KdTree<PointI> () );
    principalCurvaturesEstimation.setSearchMethod ( tree_normal );
    principalCurvaturesEstimation.setRadiusSearch ( 0.03 );
    //principalCurvatures = new pcl::PointCloud<pcl::PrincipalCurvatures>;
    CurvatureCloud::Ptr principalCurvatures ( new CurvatureCloud );
    principalCurvaturesEstimation.compute ( *principalCurvatures );
//	std::cout << "output points.size (): " << principalCurvatures->points.size()
//			<< std::endl;
    // Display and retrieve the shape context descriptor vector for the 0th point.
//   pcl::PrincipalCurvatures descriptor = principalCurvatures->points[0];
    QString str;
    float f = tt.toc () / 1000;
    str.append ( "Computed principal curvatures in  " ).append ( QString::number ( f ) ).append ( " seconds.\n" );
    getControl ()->getGuiPtr ()->writeConsole ( str );
//  getControl ()->getGuiPtr ()->writeConsole (
//      "--------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
    getControl ()->getGuiPtr ()->writeConsole ( "\n" );
    return principalCurvatures;
}

void
PCLViewer::computeNormals () {
    getControl ()->getGuiPtr ()->writeConsole ( "\n" );
    getControl ()->getGuiPtr ()->writeConsole (
        "--------------------------------------------------------------------------------------------------------------------------------------------------------------\n" );
    this->updateProgress ( 0 );
    QCoreApplication::processEvents ();
    computeNormals ( getControl ()->getCloudPtr () );
    getControl ()->getGuiPtr ()->updateProgress ( 40 );
    QCoreApplication::processEvents ();
    //CurvatureCloud::Ptr principalCurvatures = computeCurvature ( getControl ()->getCloudPtr () );
    //getControl ()->setCurvaturePtr ( principalCurvatures );
    getControl ()->getGuiPtr ()->updateProgress ( 70 );
    QCoreApplication::processEvents ();
    tt.tic ();
    std::vector<float> e1;
    std::vector<float> e2;
    std::vector<float> e3;
    std::vector<bool> isStem;

    EigenValueEstimator es ( getControl ()->getCloudPtr (), e1, e2, e3, isStem, 0.035f );
    StemPointDetection detect ( getControl ()->getCloudPtr (), isStem );
    getControl ()->setE1 ( e1 );
    getControl ()->setE2 ( e2 );
    getControl ()->setE3 ( e3 );
    getControl ()->setIsStem ( detect.getStemPtsNew () );
    getControl ()->getGuiPtr ()->updateProgress ( 100 );
    QString str;
    float f = tt.toc () / 1000;
    str.append ( "Computed PCA analysis in  " ).append ( QString::number ( f ) ).append ( " seconds.\n" );
    getControl ()->getGuiPtr ()->writeConsole (
        "--------------------------------------------------------------------------------------------------------------------------------------------------------------\n" );
    QCoreApplication::processEvents ();

}

void
PCLViewer::setCrownTransparency ( int value ) {
    float val = 1 - ( value / 255.0f );
    viewer->setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_OPACITY, val, "crown" );
    ui->qvtkWidget->update ();
}

void
PCLViewer::setLeaveTransparency ( int value ) {
    float val = 1 - ( value / 255.0f );
    viewer->setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_OPACITY, val, "leaves" );
    ui->qvtkWidget->update ();
}

void
PCLViewer::setStemTransparency ( int value ) {
    float val = 1 - ( value / 255.0f );
    for ( size_t i = 0; i < stemCylinders.size (); i++ ) {
        std::stringstream ss;
        ss << "stemCylinder" << i;
        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_OPACITY, val, ss.str () );
    }
    ui->qvtkWidget->update ();
}

void
PCLViewer::setTreeTransparency ( int value ) {
    float val = 1 - ( value / 255.0f );
    std::vector<boost::shared_ptr<simpleTree::Segment> > segments = tree_ptr->getSegments();
    for ( size_t i = 0; i < segments.size(); i ++ ) {
        boost::shared_ptr<simpleTree::Segment> seg = segments.at ( i );
        for ( size_t j = 0; j < seg->getCylinders().size(); j++ ) {
            std::stringstream ss;
            ss << "cylinder" << i << "," << j;
            viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_OPACITY, val, ss.str () );
        }
    }
    ui->qvtkWidget->update ();
}

void
PCLViewer::setCloudPtr ( PointCloudI::Ptr cloud_old , bool changeView) {
    convertPointCloud ( cloud_old );
    viewer->removeAllPointClouds ();
    viewer->removeAllShapes ();
    pcl::visualization::PointCloudColorHandlerRGBAField<PointD> rgba ( this->cloud );
    viewer->addPointCloud<PointD> ( this->cloud, rgba, "cloud" );
    point_color = 0;
    if(changeView){
        xNegView ();
    }
    if(getControl()->getTreePtr()!=0)
    {
        viewer->addText ( getControl ()->getTreePtr()->string(), 10, 20,20, 1, 0, 0, "tree_text" );
    }   else {
      viewer->addText ( getControl ()->getTreeID (), 10, 20,20, 1, 0, 0, "tree_text" );
    }
    ui->qvtkWidget->update ();

}

std::string
PCLViewer::selectFile () {
    QFileDialog dialog ( this, tr ( "OpenFile" ), "../data/", tr ( "Point Cloud File(*pcd);;ASCII - File(*.asc);;txt - File(*.txt);;All Files(*)" ) );
    dialog.setOptions ( ( QFileDialog::DontUseNativeDialog ) );
    dialog.setViewMode ( QFileDialog::Detail );
    QStringList files;
    if ( dialog.exec () )
        files = dialog.selectedFiles ();
    QString file_abs;
    if ( files.size () > 0 ) {
        file_abs = files.at ( 0 );
        int index = file_abs.lastIndexOf ( "/" );
        int size = file_abs.size ();
        int position = size - index - 1;
        QString file = file_abs.right ( position );
        getControl ()->setTreeID ( file.toStdString () );
    }
    return file_abs.toStdString ();
}

void
PCLViewer::mergeClouds () {
    getControl ()->getGuiPtr ()->writeConsole (
        "--------------------------------------------------------------------------------------------------------------------------------------------------------------\n" );

    PointCloudI::Ptr cloudA ( new PointCloudI );
    PointCloudI::Ptr cloudB ( new PointCloudI );
    int sizeA = 0, sizeB = 0, sizeAB = 0;
    std::string file = selectFile ();
    std::string abort;
    QCoreApplication::processEvents ();
    if ( file != abort ) {
        ImportPCD import ( file, control );
        cloudA = import.getCloud ();
        sizeA = cloudA->points.size ();
        //plotIntensityHist();
    }
    file = selectFile ();

    QCoreApplication::processEvents ();
    if ( file != abort ) {
        ImportPCD import ( file, control );
        cloudB = import.getCloud ();
        sizeB = cloudB->points.size ();
        //plotIntensityHist();
    }
    *cloudB += *cloudA;
    PointCloudI::Ptr cloud_filtered ( new PointCloudI );
    pcl::VoxelGrid<PointI> sor;

    sor.setInputCloud ( cloudB );
    sor.setLeafSize ( 0.003, 0.003, 0.003 );
    sor.filter ( *cloud_filtered );

//		int size_before = getControl()->getCloudPtr()->points.size();
//		int size_after = cloud_filtered->points.size();
//		float a = size_before;
//		float b = size_after;
//		float percentage = (100.0f * b) / (a);
    getControl ()->setCloudPtr ( cloud_filtered,true );
    sizeAB = cloud_filtered->points.size ();
    QString str;
    str.append ( "Merged cloud A with " ).append ( QString::number ( sizeA ) ).append ( " points and cloud B with " ).append ( QString::number ( sizeB ) ).append (
        " points\n" ).append ( " The resulting point cloud has " ).append ( QString::number ( sizeAB ) ).append (
        " points\n A grid voxel downsampling was performed to remove duplicates.\n" );

    getControl ()->getGuiPtr ()->writeConsole ( str );
    getControl ()->getGuiPtr ()->writeConsole (
        "--------------------------------------------------------------------------------------------------------------------------------------------------------------\n" );

    ui->qvtkWidget->update ();
}

void
PCLViewer::importPCDFile () {

    std::string file = selectFile ();
    std::string abort;
    QCoreApplication::processEvents ();
    if ( file != abort ) {
        ImportPCD import ( file, control );
        getControl ()->setCloudPtr ( import.getCloud (),true );
        plotIntensityHist ();
    }
    getControl ()->getGuiPtr ()->writeConsole (
        "--------------------------------------------------------------------------------------------------------------------------------------------------------------\n" );

    ui->qvtkWidget->update ();
}

void
PCLViewer::set_method ( int i ) {
    QString method = method_dialog_ptr->comboBox->currentText ();
    coeff_ptr->set_coefficients_for_method ( method );
    method_dialog_ptr->name->setText ( method_coefficients.name );
    method_dialog_ptr->sphere_radius_multiplier->setValue ( method_coefficients.sphere_radius_multiplier );
    method_dialog_ptr->epsilon_cluster_stem->setValue ( method_coefficients.epsilon_cluster_stem );
    method_dialog_ptr->epsilon_cluster_branch->setValue ( method_coefficients.epsilon_cluster_branch );
    method_dialog_ptr->epsilon_sphere->setValue ( method_coefficients.epsilon_sphere );
    method_dialog_ptr->minPts_ransac_stem->setValue ( method_coefficients.minPts_ransac_stem );
    method_dialog_ptr->minPts_ransac_branch->setValue ( method_coefficients.minPts_ransac_branch );
    method_dialog_ptr->minPts_cluster_stem->setValue ( method_coefficients.minPts_cluster_stem );
    method_dialog_ptr->minPts_cluster_branch->setValue ( method_coefficients.minPts_cluster_branch );
    method_dialog_ptr->min_radius_sphere_stem->setValue ( method_coefficients.min_radius_sphere_stem );
    method_dialog_ptr->min_radius_sphere_branch->setValue ( method_coefficients.min_radius_sphere_branch );

}
void
PCLViewer::set_method_coeff () {
    method_coefficients.name = method_dialog_ptr->name->displayText ();
    method_coefficients.sphere_radius_multiplier = method_dialog_ptr->sphere_radius_multiplier->value ();
    method_coefficients.epsilon_cluster_stem = method_dialog_ptr->epsilon_cluster_stem->value ();
    method_coefficients.epsilon_cluster_branch = method_dialog_ptr->epsilon_cluster_branch->value ();
    method_coefficients.epsilon_sphere = method_dialog_ptr->epsilon_sphere->value ();
    method_coefficients.minPts_ransac_stem = method_dialog_ptr->minPts_ransac_stem->value ();
    method_coefficients.minPts_ransac_branch = method_dialog_ptr->minPts_ransac_branch->value ();
    method_coefficients.minPts_cluster_stem = method_dialog_ptr->minPts_cluster_stem->value ();
    method_coefficients.minPts_cluster_branch = method_dialog_ptr->minPts_cluster_branch->value ();
    method_coefficients.min_radius_sphere_stem = method_dialog_ptr->min_radius_sphere_stem->value ();
    method_coefficients.min_radius_sphere_branch = method_dialog_ptr->min_radius_sphere_branch->value ();

    coeff_ptr->struct_to_settings ( method_coefficients );
    coeff_ptr->settings->sync ();
    if ( getControl ()->getCloudPtr () != 0 ) {
        //if(getControl()->getCloudPtr()->points.size() != getControl()->getIsStem().size())
        //{
        pcl::console::TicToc tt;
        tt.tic ();
        QString str = "\n";
        writeConsole ( str );
        getControl ()->getGuiPtr ()->writeConsole (
            "--------------------------------------------------------------------------------------------------------------------------------------------------------------\n" );
        updateProgress ( 0 );
        QString str2 = coeff_ptr->struct_to_qstring ( method_coefficients );
        writeConsole ( str2 );
        SphereFollowing sphereFollowing ( this->getControl ()->getCloudPtr (), control, 1, method_coefficients );
        writeConsole ( str );
        updateProgress ( 50 );
        getControl ()->getGuiPtr ()->writeConsole (
            "--------------------------------------------------------------------------------------------------------------------------------------------------------------\n" );

        tt.tic ();
        writeConsole ( str );
        getControl ()->getGuiPtr ()->writeConsole (
            "--------------------------------------------------------------------------------------------------------------------------------------------------------------\n" );
        boost::shared_ptr<simpleTree::Tree> tree = boost::make_shared<simpleTree::Tree> ( sphereFollowing.getCylinders (), this->getControl ()->getCloudPtr (),
                this->getControl ()->getTreeID (), this->control );
        float f = tt.toc () / 1000;
        str.append ( "Done tree structure in " ).append ( QString::number ( f ) ).append ( " seconds.\n" );
        writeConsole ( str );
        updateProgress ( 100 );
        getControl ()->getGuiPtr ()->writeConsole (
            "--------------------------------------------------------------------------------------------------------------------------------------------------------------\n" );
        getControl ()->setTreePtr ( tree );
        //			}else {
        //				QMessageBox::warning(this, tr("Simple Tree"),
        //						tr("Please detect stem points first"), QMessageBox::Ok);
        //			}
    } else {
        QMessageBox::warning ( this, tr ( "Simple Tree" ), tr ( "No Point Cloud found\n"
                               "Please load a Point Cloud first" ),
                               QMessageBox::Ok );
    }

}

void
PCLViewer::delete_method () {
    QString method = method_dialog_ptr->comboBox->currentText ();
    std::cout << method.toStdString () << std::endl;
    coeff_ptr->settings->remove ( method );
    coeff_ptr->settings->sync ();
}
void
PCLViewer::compute_detectTree () {
    if ( getControl ()->getCloudPtr () != 0 ) {
        //if(getControl()->getCloudPtr()->points.size() != getControl()->getIsStem().size())
        //{
        pcl::console::TicToc tt;
        tt.tic ();
        QString str = "\n";
        writeConsole ( str );
        getControl ()->getGuiPtr ()->writeConsole (
            "--------------------------------------------------------------------------------------------------------------------------------------------------------------\n" );
        updateProgress ( 0 );
        QString str2 = coeff_ptr->struct_to_qstring ( method_coefficients );
        writeConsole ( str2 );
        QCoreApplication::processEvents ();
        SphereFollowing sphereFollowing ( this->getControl ()->getCloudPtr (), control, 1 , method_coefficients );
        writeConsole ( str );
        updateProgress ( 50 );
        QCoreApplication::processEvents ();
        getControl ()->getGuiPtr ()->writeConsole (
            "--------------------------------------------------------------------------------------------------------------------------------------------------------------\n" );

        tt.tic ();
        writeConsole ( str );
        QCoreApplication::processEvents ();
        getControl ()->getGuiPtr ()->writeConsole (
            "--------------------------------------------------------------------------------------------------------------------------------------------------------------\n" );
        QCoreApplication::processEvents ();
        boost::shared_ptr<simpleTree::Tree> tree = boost::make_shared<simpleTree::Tree> ( sphereFollowing.getCylinders (), this->getControl ()->getCloudPtr (),
                this->getControl ()->getTreeID (), this->control );
        float f = tt.toc () / 1000;
        str.append ( "Done tree structure in " ).append ( QString::number ( f ) ).append ( " seconds.\n" );
        writeConsole ( str );
        updateProgress ( 100 );
        getControl ()->getGuiPtr ()->writeConsole (
            "--------------------------------------------------------------------------------------------------------------------------------------------------------------\n" );
        getControl ()->setTreePtr ( tree );
//			}else {
//				QMessageBox::warning(this, tr("Simple Tree"),
//						tr("Please detect stem points first"), QMessageBox::Ok);
//			}
    } else {
        QMessageBox::warning ( this, tr ( "Simple Tree" ), tr ( "No Point Cloud found\n"
                               "Please load a Point Cloud first" ),
                               QMessageBox::Ok );
    }
}

void
PCLViewer::detectTree () {
    QString name;
    QDialog * method_dialog = new QDialog ( this, 0 );

    method_dialog_ptr->setupUi ( method_dialog );
    QStringList list = coeff_ptr->get_method_names ();
    for ( int i = 0; i < list.size (); i++ ) {
        QString name = list.at ( i );
        method_dialog_ptr->comboBox->addItem ( name );
    }
    method_dialog_ptr->name->setText ( method_coefficients.name );
    method_dialog_ptr->sphere_radius_multiplier->setValue ( method_coefficients.sphere_radius_multiplier );
    method_dialog_ptr->epsilon_cluster_stem->setValue ( method_coefficients.epsilon_cluster_stem );
    method_dialog_ptr->epsilon_cluster_branch->setValue ( method_coefficients.epsilon_cluster_branch );
    method_dialog_ptr->epsilon_sphere->setValue ( method_coefficients.epsilon_sphere );
    method_dialog_ptr->minPts_ransac_stem->setValue ( method_coefficients.minPts_ransac_stem );
    method_dialog_ptr->minPts_ransac_branch->setValue ( method_coefficients.minPts_ransac_branch );
    method_dialog_ptr->minPts_cluster_stem->setValue ( method_coefficients.minPts_cluster_stem );
    method_dialog_ptr->minPts_cluster_branch->setValue ( method_coefficients.minPts_cluster_branch );
    method_dialog_ptr->min_radius_sphere_stem->setValue ( method_coefficients.min_radius_sphere_stem );
    method_dialog_ptr->min_radius_sphere_branch->setValue ( method_coefficients.min_radius_sphere_branch );

    connect ( method_dialog_ptr->comboBox, SIGNAL ( currentIndexChanged ( int ) ), this, SLOT ( set_method ( int ) ) );
    connect ( method_dialog_ptr->abort, SIGNAL ( clicked() ), method_dialog, SLOT ( reject() ) );
    connect ( method_dialog_ptr->compute, SIGNAL ( clicked() ), this, SLOT ( set_method_coeff() ) );
    //connect(method_dialog_ptr->compute, SIGNAL(clicked()), this, SLOT(compute_detectTree()));
    connect ( method_dialog_ptr->delete_2, SIGNAL ( clicked() ), this, SLOT ( delete_method() ) );
    connect ( method_dialog_ptr->delete_2, SIGNAL ( clicked() ), method_dialog, SLOT ( reject() ) );
    connect ( method_dialog_ptr->compute, SIGNAL ( clicked() ), method_dialog, SLOT ( accept() ) );
   // method_dialog->setParent(this);
    method_dialog->setModal(true);
    method_dialog->show ();
//           connect(rad.box_minPts, SIGNAL(valueChanged(int)), this,
//               SLOT(set_radius_outlier_minPts(int)));
//           connect(rad.box_radius, SIGNAL(valueChanged(double)), this,
//               SLOT(set_radius_outlier_searchradius(double)));
//
//           connect(rad.abort, SIGNAL(clicked()), method_dialog, SLOT(reject()));
//           connect(rad.compute, SIGNAL(clicked()), this,
//               SLOT(compute_radius_outlier_removal()));
//           connect(rad.compute, SIGNAL(clicked()), method_dialog, SLOT(accept()));

}

void
PCLViewer::xNegView () {
    float pos_x = ( centerX - fac * - ( extension ) );
    viewer->setCameraPosition ( pos_x, centerY, centerZ, centerX, centerY, centerZ, 0, 0, 1 );
    ui->qvtkWidget->update ();
}

void
PCLViewer::xPosView () {
    float pos_x = ( centerX - fac * extension );
    viewer->setCameraPosition ( pos_x, centerY, centerZ, centerX, centerY, centerZ, 0, 0, 1 );
    ui->qvtkWidget->update ();
}

void
PCLViewer::yNegView () {
    float pos_y = ( centerY - fac * -extension );
    viewer->setCameraPosition ( centerX, pos_y, centerZ, centerX, centerY, centerZ, 0, 0, 1 );
    ui->qvtkWidget->update ();
}

void
PCLViewer::yPosView () {
    float pos_y = ( centerY - fac * extension );
    viewer->setCameraPosition ( centerX, pos_y, centerZ, centerX, centerY, centerZ, 0, 0, 1 );
    ui->qvtkWidget->update ();
}

void
PCLViewer::zNegView () {
    float pos_z = ( centerZ - fac * -extension );
    viewer->setCameraPosition ( centerX, centerY, pos_z, centerX, centerY, centerZ, 1, 0, 0 );
    ui->qvtkWidget->update ();
}

void
PCLViewer::zPosView () {
    float pos_z = ( centerZ - fac * extension );
    viewer->setCameraPosition ( centerX, centerY, pos_z, centerX, centerY, centerZ, 1, 0, 0 );
    ui->qvtkWidget->update ();
}

boost::shared_ptr<PointCloudD>
PCLViewer::convertPointCloud ( PointCloudI::Ptr & cloud , int r, int g, int b)
{
    boost::shared_ptr<PointCloudD> visu_cloud (new PointCloudD);
    visu_cloud->resize ( cloud->points.size () );
    for ( size_t i = 0; i < visu_cloud->points.size (); i++ ) {
        visu_cloud->points[i].x = cloud->points[i].x;
        visu_cloud->points[i].y = cloud->points[i].y;
        visu_cloud->points[i].z = cloud->points[i].z;
        visu_cloud->points[i].r = r;
        visu_cloud->points[i].g = g;
        visu_cloud->points[i].b = b;
        visu_cloud->points[i].a = 128;
    }
    return visu_cloud;
}


void
PCLViewer::convertPointCloud ( PointCloudI::Ptr cloud2 ) {

    this->cloud.reset ( new PointCloudD );
    std::cout << cloud2->points.size();
    this->cloud->resize ( cloud2->points.size () );
    for ( size_t i = 0; i < cloud2->points.size (); i++ ) {
        this->cloud->points[i].x = cloud2->points[i].x;
        this->cloud->points[i].y = cloud2->points[i].y;
        this->cloud->points[i].z = cloud2->points[i].z;
        float intens = cloud2->points[i].intensity;
        if ( intens == 0 ) {
            intens = 180;
        }
        cloud2->points[i].intensity = intens;
        this->cloud->points[i].r = 255 - intens;
        this->cloud->points[i].g = intens;
        this->cloud->points[i].b = 255 - intens;
        this->cloud->points[i].a = 255;
    }
    computeBoundingBox ();
}

void
PCLViewer::computeBoundingBox () {
    minX = 1000000;
    minY = 1000000;
    minZ = 1000000;
    maxX = -1000000;
    maxY = -1000000;
    maxZ = -1000000;
    for ( size_t i = 0; i < cloud->points.size (); i++ ) {
        PointD p = cloud->points[i];
        if ( p.x < minX ) {
            minX = p.x;
        }
        if ( p.y < minY ) {
            minY = p.y;
        }
        if ( p.z < minZ ) {
            minZ = p.z;
        }
        if ( p.x > maxX ) {
            maxX = p.x;
        }
        if ( p.y > maxY ) {
            maxY = p.z;
        }
        if ( p.z > maxZ ) {
            maxZ = p.z;
        }
        centerX = ( minX + maxX ) / 2;
        centerY = ( minY + maxY ) / 2;
        centerZ = ( minZ + maxZ ) / 2;
        extension = std::max<float> ( maxX - minX, maxY - minY );
        extension = std::max<float> ( extension, maxZ - minZ );
    }
}

void
PCLViewer::setPointTransparency ( int value ) {
    float val = 1 - ( value / 255.0f );
    viewer->setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_OPACITY, val, "cloud" );
    ui->qvtkWidget->update ();
}

void
PCLViewer::setPointSize ( int value ) {
    viewer->setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, value, "cloud" );
    ui->qvtkWidget->update ();
}

PCLViewer::~PCLViewer () {
    delete ui;
}

void
PCLViewer::init () {
    coeff_ptr.reset ( new SetCoefficients ( shared_from_this () ) );
    coeff_ptr->set_coefficients_for_method ( "Prunus_avium_Breisach_Germany" );

}

