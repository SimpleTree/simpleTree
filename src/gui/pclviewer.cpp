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
            boost::shared_ptr<CropSphereDialog> dialog = data->sphere_dialog_ptr;
            float x, y, z;
            event.getPoint ( x, y, z );
            dialog->dialog->box_x->setValue ( x );
            dialog->dialog->box_y->setValue ( y );
            dialog->dialog->box_z->setValue ( z );

        } else if ( viewer->crop_box_is_active ) {
            boost::shared_ptr<CropBoxDialog> dialog = data->box_dialog_ptr;
            float x, y, z;
            event.getPoint ( x, y, z );
            dialog->dialog->box_x->setValue ( x );
            dialog->dialog->box_y->setValue ( y );
            dialog->dialog->box_z->setValue ( z );

        } else if (viewer->reset_crown_is_active) {
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
    ui->scrollArea->setWidget ( ui->scrollAreaWidgetContents );

    this->setWindowTitle ( "SimpleTree" );
    generateCloud ();

    viewer.reset ( new pcl::visualization::PCLVisualizer ( "Simple Tree", false ) );
    ui->qvtkWidget->SetRenderWindow ( viewer->getRenderWindow () );
    viewer->setupInteractor ( ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow () );
    ui->qvtkWidget->update ();
    plotter.reset ( new pcl::visualization::PCLPlotter ( "Plotter" ) );
    ui->qvtkWidget2->SetRenderWindow ( plotter->getRenderWindow () );
    plotter->setViewInteractor ( ui->qvtkWidget2->GetInteractor () );
    ui->qvtkWidget2->update ();

    ui->australia_button->setIconSize(QSize(35, 35));
    ui->allometry_button->setIconSize(QSize(35, 35));
    ui->box_deletion_button->setIconSize(QSize(24, 24));
    ui->background_button->setIconSize(QSize(25,25));
    ui->cluster_button->setIconSize(QSize(30, 30));
    ui->compute_normal_button->setIconSize(QSize(27, 27));
    ui->complete_folder_button->setIconSize(QSize(25, 25));
    ui->ery_denoise_button->setIconSize(QSize(35, 35));
    ui->ery_button->setIconSize(QSize(35, 35));
    ui->euca_denoise_button->setIconSize(QSize(35, 35));
    ui->file_import_button->setIconSize(QSize(25, 25));
    ui->file_export_button->setIconSize(QSize(25, 25));
    ui->file_export_result_button->setIconSize(QSize(25, 25));
    ui->file_export_ply_button->setIconSize(QSize(25, 25));
    ui->intensity_button->setIconSize(QSize(30, 30));
    ui->merge_cloud_button->setIconSize(QSize(35, 35));
    ui->method_hackenberg->setIconSize(QSize(25, 25));
    ui->oak_denoise_button->setIconSize(QSize(35, 35));
    ui->pine_denoise_button->setIconSize(QSize(35, 35));
    ui->pine_button_fast->setIconSize(QSize(35, 35));
    ui->point_color_button->setIconSize(QSize(30, 30));
    ui->picture_frame->setStyleSheet ( "background-color:white;" );
    ui->picture_frame_3->setStyleSheet ( "background-color:white;" );
    ui->quercus_fast->setIconSize(QSize(35, 35));
    ui->radius_button->setIconSize(QSize(30, 30));
    ui->reference_cloud_button->setIconSize(QSize(30, 30));
    ui->reset_stem_button->setIconSize(QSize(25,25));
    ui->reset_crown_base_button->setIconSize(QSize(25,25));
    ui->reset_crown_button->setIconSize(QSize(25,25));
    ui->screenshotbutton->setIconSize(QSize(25,25));
    ui->statistical_button->setIconSize(QSize(30, 30));
    ui->sphere_deletion_button->setIconSize(QSize(24, 24));
    ui->tree_color_button->setIconSize(QSize(30, 30));
    ui->voxelgrid_button->setIconSize(QSize(30, 30));
    ui->xViewNeg->setIconSize(QSize(30, 30));
    ui->xViewPos->setIconSize(QSize(30, 30));
    ui->yViewNeg->setIconSize(QSize(30, 30));
    ui->yViewPos->setIconSize(QSize(30, 30));
    ui->zViewNeg->setIconSize(QSize(30, 30));
    ui->zViewPos->setIconSize(QSize(30, 30));
    ui->load_camera_button->setIconSize(QSize(30, 30));
    ui->save_camera_button->setIconSize(QSize(30, 30));

    viewer->setBackgroundColor(1,1,1,0);

    connect (ui->allometry_button, SIGNAL(clicked()),this, SLOT(allometry_dialog()));
    connect ( ui->australia_button, SIGNAL(clicked()),this, SLOT(compute_eucalypt()));
    connect ( ui->background_button,SIGNAL(clicked()),this,SLOT(set_background()));
    connect ( ui->box_deletion_button, SIGNAL ( clicked() ), this, SLOT ( crop_box() ) );
    connect ( ui->cluster_button, SIGNAL ( clicked() ), this, SLOT ( euclideanClustering() ) );
    connect ( ui->complete_folder_button,SIGNAL ( clicked() ),this,SLOT ( build_model1() ) );
    connect ( ui->compute_normal_button, SIGNAL ( clicked() ), this, SLOT ( computeNormals() ) );
    connect ( ui->crown_color_slider, SIGNAL ( valueChanged ( int ) ), this, SLOT ( setCrownTransparency ( int ) ) );
    connect ( ui->eigen_value_button, SIGNAL (clicked()), this, SLOT(curvature_dialog()));
    connect ( ui->ery_denoise_button, SIGNAL(clicked()),this, SLOT(denoise_ery()));
    connect ( ui->ery_button, SIGNAL(clicked()),this, SLOT(compute_ery()));
    connect ( ui->euca_denoise_button, SIGNAL(clicked()),this, SLOT(denoise_eucalypt()));
    connect ( ui->file_import_button, SIGNAL ( clicked() ), this, SLOT ( importPCDFile() ) );
    connect ( ui->file_export_button, SIGNAL ( clicked() ), this, SLOT ( exportPCDFile() ) );
    connect ( ui->file_export_result_button, SIGNAL ( clicked() ), this, SLOT ( exportResults() ) );
    connect ( ui->file_export_ply_button, SIGNAL ( clicked() ), this, SLOT ( exportPly() ) );
    connect ( ui->icp_button,SIGNAL ( clicked() ),this,SLOT ( compute_ICP() ) );
    connect ( ui->intensity_button, SIGNAL ( clicked() ), this, SLOT ( intensityOutlierRemoval() ) );
    connect ( ui->leave_color_slider, SIGNAL ( valueChanged ( int ) ), this, SLOT ( setLeaveTransparency ( int ) ) );
    connect ( ui->merge_cloud_button, SIGNAL ( clicked() ), this, SLOT ( mergeClouds() ) );
    connect ( ui->method_hackenberg, SIGNAL ( clicked() ), this, SLOT ( build_model() ) );
    connect ( ui->oak_denoise_button, SIGNAL(clicked()),this, SLOT(denoise_oak()));
    connect ( ui->pine_denoise_button, SIGNAL(clicked()),this, SLOT(denoise_pinus()));
    connect ( ui->pine_button_fast, SIGNAL(clicked()),this, SLOT(compute_pinus()));
    connect ( ui->point_color_button, SIGNAL ( clicked() ), this, SLOT ( changePointColor() ) );
    connect ( ui->point_size_slider, SIGNAL ( valueChanged ( int ) ), this, SLOT ( setPointSize ( int ) ) );
    connect ( ui->point_color_slider, SIGNAL ( valueChanged ( int ) ), this, SLOT ( setPointTransparency ( int ) ) );
    connect ( ui->quercus_fast, SIGNAL(clicked()),this, SLOT(compute_quercus()));
    connect ( ui->radius_button, SIGNAL ( clicked() ), this, SLOT ( radiusOutlierRemoval() ) );
    connect ( ui->reference_cloud_button, SIGNAL ( clicked() ), this, SLOT ( reference_cloud() ) );
    connect ( ui->reset_stem_button,SIGNAL(clicked()),this,SLOT(reset_stem()));
    connect ( ui->reset_crown_base_button,SIGNAL(clicked()),this,SLOT(reset_crown_base()));
    connect ( ui->reset_crown_button,SIGNAL(clicked()),this,SLOT(reset_crown()));
    connect ( ui->screenshotbutton,SIGNAL(clicked()),this,SLOT(screenshot()));
    connect ( ui->sphere_deletion_button, SIGNAL ( clicked() ), this, SLOT ( cropsphere() ) );
    connect ( ui->statistical_button, SIGNAL ( clicked() ), this, SLOT ( statisticalOutlierRemoval() ) );
    connect ( ui->stem_color_slider, SIGNAL ( valueChanged ( int ) ), this, SLOT ( setStemTransparency ( int ) ) );
    connect ( ui->tree_color_button, SIGNAL ( clicked() ), this, SLOT ( changeTreeColor() ) )  ;
    connect ( ui->tree_color_slider, SIGNAL ( valueChanged ( int ) ), this, SLOT ( setTreeTransparency ( int ) ) );
    connect ( ui->voxelgrid_button, SIGNAL ( clicked() ), this, SLOT ( voxel_downsampling() ) );
    connect ( ui->xViewNeg, SIGNAL ( clicked() ), this, SLOT ( xNegView() ) );
    connect ( ui->xViewPos, SIGNAL ( clicked() ), this, SLOT ( xPosView() ) );
    connect ( ui->yViewNeg, SIGNAL ( clicked() ), this, SLOT ( yNegView() ) );
    connect ( ui->yViewPos, SIGNAL ( clicked() ), this, SLOT ( yPosView() ) );
    connect ( ui->zViewNeg, SIGNAL ( clicked() ), this, SLOT ( zNegView() ) );
    connect ( ui->zViewPos, SIGNAL ( clicked() ), this, SLOT ( zPosView() ) );
    connect ( ui->load_camera_button, SIGNAL ( clicked() ), this, SLOT ( load_camera_position()  ) );
    connect ( ui->save_camera_button, SIGNAL ( clicked() ), this, SLOT ( save_camera_position()));

    //connect (ui->branch_button, SIGNAL ( clicked() ), this, SLOT ( visualize_branch() ) );


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
    crop_box_is_active = false;
    reset_crown_is_active = false;
    box_dialog_ui_ptr.reset ( new Ui_crop_box_dialog );
    method_dialog_ptr.reset ( new Ui_method_dialog );
    cb_args.box_dialog_ptr = crop_box_dialog;
    cb_args.sphere_dialog_ptr = crop_sphere_dialog;
    cb_args.viewer = this;
    viewer->registerPointPickingCallback ( pp_callback, ( void* ) &cb_args );

}

PCLViewer::~PCLViewer () {
    delete ui;
}


void
PCLViewer::abort_crown()
{
    hull_radius = 0.1;
}
void
PCLViewer::abort_intensity_outlier_removal()
{
    intensity_clouds.clear();
    viewer->removeAllPointClouds();
    getControl()->setCloudPtr(getControl()->getCloudPtr());
}

void
PCLViewer::allom_compute()
{

    float a = allometry_dialog_ptr->a->value();
    float b = allometry_dialog_ptr->b->value();
    float fac = allometry_dialog_ptr->fact->value();
    float minRad = allometry_dialog_ptr->minRad->value();


    simpleTree::Allometry allom;
    allom.setTree(getControl()->getTreePtr());
    allom.setCoefficients(a,b);
    allom.setFac(fac);
    allom.setMinRad(minRad);
    allom.improveTree();
    getControl()->setTreePtr(getControl()->getTreePtr());

}

void
PCLViewer::allometry_dialog()
{
    if (getControl ()->getCloudPtr() !=0) {
        if(getControl()->getTreePtr() != 0)
        {

            QDialog * allometry_dialog=new QDialog(this,0);
            allometry_dialog_ptr.reset(new Ui_allometry_dialog);
            allometry_dialog_ptr->setupUi(allometry_dialog);
            connect ( allometry_dialog_ptr->abort, SIGNAL ( clicked() ),allometry_dialog , SLOT ( reject() ) );
            connect ( allometry_dialog_ptr->compute, SIGNAL ( clicked() ),this , SLOT ( allom_compute() ) );
            connect ( allometry_dialog_ptr->compute, SIGNAL ( clicked() ),allometry_dialog , SLOT ( accept() ) );
            allometry_dialog->setModal(true);
            allometry_dialog->show ();
        } else {
            QMessageBox::warning ( this, tr ( "Simple Tree" ), tr ( "No Model computed" ),
                                   QMessageBox::Ok );
        }
    } else {
        QMessageBox::warning ( this, tr ( "Simple Tree" ), tr ( "No Point Cloud found\n"
                                                                "Please load a Point Cloud first" ),
                               QMessageBox::Ok );

    }
}
void
PCLViewer::allom_check()
{
    QDialog * allometry_dialog=new QDialog(this,0);
    allometry_dialog_ptr.reset(new Ui_allometry_dialog);
    allometry_dialog_ptr->setupUi(allometry_dialog);
    allometry_dialog_ptr->a->setValue ( _method_coefficients.a );
    allometry_dialog_ptr->b->setValue ( _method_coefficients.b );
    allometry_dialog_ptr->fact->setValue ( _method_coefficients.fact );
    allometry_dialog_ptr->minRad->setValue ( _method_coefficients.minRad );


    connect ( allometry_dialog_ptr->compute, SIGNAL ( clicked() ), this, SLOT ( set_method_coeff_wdout_compute() ) );
    connect ( allometry_dialog_ptr->compute, SIGNAL ( clicked() ), this, SLOT ( optimize_check() ) );
    connect ( allometry_dialog_ptr->compute, SIGNAL ( clicked() ), allometry_dialog, SLOT ( accept() ) );



    connect ( allometry_dialog_ptr->abort, SIGNAL ( clicked() ), this, SLOT ( optimize_check() ) );
    connect ( allometry_dialog_ptr->abort, SIGNAL ( clicked() ), this, SLOT ( set_allom_fact() ) );
    connect ( allometry_dialog_ptr->abort, SIGNAL ( clicked() ), allometry_dialog, SLOT ( reject() ) );

    allometry_dialog->setModal(true);
    allometry_dialog->show ();
}

void
PCLViewer::allom_check1()
{
    QDialog * allometry_dialog=new QDialog(this,0);
    allometry_dialog_ptr.reset(new Ui_allometry_dialog);
    allometry_dialog_ptr->setupUi(allometry_dialog);
    allometry_dialog_ptr->a->setValue ( _method_coefficients.a );
    allometry_dialog_ptr->b->setValue ( _method_coefficients.b );
    allometry_dialog_ptr->fact->setValue ( _method_coefficients.fact );
    allometry_dialog_ptr->minRad->setValue ( _method_coefficients.minRad );

    //connect for allometry and optimize
    connect ( allometry_dialog_ptr->compute, SIGNAL ( clicked() ), this, SLOT ( set_method_coeff_wdout_compute() ) );
    connect ( allometry_dialog_ptr->compute, SIGNAL ( clicked() ), this, SLOT ( optimize_check1() ) );
    connect ( allometry_dialog_ptr->compute, SIGNAL ( clicked() ), allometry_dialog, SLOT ( accept() ) );
    connect ( allometry_dialog_ptr->abort, SIGNAL ( clicked() ), this, SLOT ( optimize_check1() ) );
    connect ( allometry_dialog_ptr->abort, SIGNAL ( clicked() ), this, SLOT ( set_allom_fact() ) );
    connect ( allometry_dialog_ptr->abort, SIGNAL ( clicked() ), allometry_dialog, SLOT ( reject() ) );

    allometry_dialog->setModal(true);
    allometry_dialog->show ();
}

void
PCLViewer::build_model1 () {
    QString name;

    QDialog * method_dialog = new QDialog ( this, 0 );

    method_dialog_ptr->setupUi ( method_dialog );
    QStringList list = coeff_ptr->get_method_names ();
    for ( int i = 0; i < list.size (); i++ ) {
        QString name = list.at ( i );
        method_dialog_ptr->comboBox->addItem ( name );
    }
    method_dialog_ptr->name->setText ( _method_coefficients.name );
    method_dialog_ptr->sphere_radius_multiplier->setValue ( _method_coefficients.sphere_radius_multiplier );
    method_dialog_ptr->epsilon_cluster_stem->setValue ( _method_coefficients.epsilon_cluster_stem );
    method_dialog_ptr->epsilon_cluster_branch->setValue ( _method_coefficients.epsilon_cluster_branch );
    method_dialog_ptr->epsilon_sphere->setValue ( _method_coefficients.epsilon_sphere );
    method_dialog_ptr->minPts_ransac_stem->setValue ( _method_coefficients.minPts_ransac_stem );
    method_dialog_ptr->minPts_ransac_branch->setValue ( _method_coefficients.minPts_ransac_branch );
    method_dialog_ptr->minPts_cluster_stem->setValue ( _method_coefficients.minPts_cluster_stem );
    method_dialog_ptr->minPts_cluster_branch->setValue ( _method_coefficients.minPts_cluster_branch );
    method_dialog_ptr->min_radius_sphere_stem->setValue ( _method_coefficients.min_radius_sphere_stem );
    method_dialog_ptr->min_radius_sphere_branch->setValue ( _method_coefficients.min_radius_sphere_branch );

    connect ( method_dialog_ptr->comboBox, SIGNAL ( currentIndexChanged ( int ) ), this, SLOT ( set_method ( int ) ) );

    connect ( method_dialog_ptr->compute, SIGNAL ( clicked() ), this, SLOT ( set_method_coeff_wdout_compute() ) );
    connect ( method_dialog_ptr->compute, SIGNAL ( clicked() ), method_dialog, SLOT ( accept() ) );
    connect (method_dialog_ptr ->compute,SIGNAL  ( clicked() ),this, SLOT(allom_check1()));

    connect ( method_dialog_ptr->abort, SIGNAL ( clicked() ), method_dialog, SLOT ( reject() ) );

    connect ( method_dialog_ptr->delete_2, SIGNAL ( clicked() ), this, SLOT ( delete_method() ) );
    connect ( method_dialog_ptr->delete_2, SIGNAL ( clicked() ), method_dialog, SLOT ( reject() ) );


    method_dialog->setModal(true);
    method_dialog->show ();
}


void
PCLViewer::build_model () {
    QString name;

    QDialog * method_dialog = new QDialog ( this, 0 );

    method_dialog_ptr->setupUi ( method_dialog );
    QStringList list = coeff_ptr->get_method_names ();
    for ( int i = 0; i < list.size (); i++ ) {
        QString name = list.at ( i );
        method_dialog_ptr->comboBox->addItem ( name );
    }
    if (getControl ()->getCloudPtr() !=0) {
        method_dialog_ptr->name->setText ( _method_coefficients.name );
        method_dialog_ptr->sphere_radius_multiplier->setValue ( _method_coefficients.sphere_radius_multiplier );
        method_dialog_ptr->epsilon_cluster_stem->setValue ( _method_coefficients.epsilon_cluster_stem );
        method_dialog_ptr->epsilon_cluster_branch->setValue ( _method_coefficients.epsilon_cluster_branch );
        method_dialog_ptr->epsilon_sphere->setValue ( _method_coefficients.epsilon_sphere );
        method_dialog_ptr->minPts_ransac_stem->setValue ( _method_coefficients.minPts_ransac_stem );
        method_dialog_ptr->minPts_ransac_branch->setValue ( _method_coefficients.minPts_ransac_branch );
        method_dialog_ptr->minPts_cluster_stem->setValue ( _method_coefficients.minPts_cluster_stem );
        method_dialog_ptr->minPts_cluster_branch->setValue ( _method_coefficients.minPts_cluster_branch );
        method_dialog_ptr->min_radius_sphere_stem->setValue ( _method_coefficients.min_radius_sphere_stem );
        method_dialog_ptr->min_radius_sphere_branch->setValue ( _method_coefficients.min_radius_sphere_branch );

        connect ( method_dialog_ptr->comboBox, SIGNAL ( currentIndexChanged ( int ) ), this, SLOT ( set_method ( int ) ) );
        connect ( method_dialog_ptr->compute, SIGNAL ( clicked() ), this, SLOT ( set_method_coeff_wdout_compute() ) );
        connect ( method_dialog_ptr->compute, SIGNAL ( clicked() ), method_dialog, SLOT ( accept() ) );
        connect (method_dialog_ptr->compute,SIGNAL(clicked()),this,SLOT(allom_check()));

        connect ( method_dialog_ptr->abort, SIGNAL ( clicked() ), method_dialog, SLOT ( reject() ) );

        connect ( method_dialog_ptr->delete_2, SIGNAL ( clicked() ), this, SLOT ( delete_method() ) );
        connect ( method_dialog_ptr->delete_2, SIGNAL ( clicked() ), method_dialog, SLOT ( reject() ) );


        method_dialog->setModal(true);
        method_dialog->show ();
    } else {
        QMessageBox::warning ( this, tr ( "Simple Tree" ), tr ( "No Point Cloud found\n"
                                                                "Please load a Point Cloud first" ),
                               QMessageBox::Ok );

    }

}


void
PCLViewer::build_model_for_folder () {
    pcl::console::TicToc tt,tt2;
    tt.tic();
    tt2.tic();
    QString path;
    QString result;
    QDir dir = QFileDialog::getExistingDirectory(this, tr("Open Directory"),
                                                 "../data/",
                                                 QFileDialog::ShowDirsOnly
                                                 | QFileDialog::DontResolveSymlinks);
    QFileInfo fi(dir, path);
    std::cout << dir.absolutePath().toStdString();
    QStringList filters;
    filters << "*.pcd" ;
    dir.setNameFilters ( filters );
    dir.setFilter ( QDir::Files );
    QStringList files = dir.entryList ();
    path = dir.absolutePath();
    Method_Coefficients user_coefficients = _method_coefficients;
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
        QString str ("Imported ");
        str.append(QString::fromStdString(getControl()->getTreeID())).append(" with ");
        str.append(QString::number(getControl()->getCloudPtr()->points.size())).append(" points in ").append(QString::number(tt.toc()/1000)).append(" seconds.");
        plot_qstring(str, true, true);
        this->updateProgress ( 0 );
        QCoreApplication::processEvents ();
        tt.tic();
        computeNormals ( getControl ()->getCloudPtr () );
        str = "Computed normals in ";
        str.append(QString::number(tt.toc()/1000)).append(" seconds.");
        plot_qstring(str, true, false);
        QCoreApplication::processEvents ();
        tt.tic ();
        std::vector<float> e1;
        std::vector<float> e2;
        std::vector<float> e3;
        std::vector<bool> isStem;
        updateProgress(30);
        QCoreApplication::processEvents ();
        EigenValueEstimator es ( getControl ()->getCloudPtr (), e1, e2, e3, isStem, 0.035f );

        str = "";
        str.append ( "Computed PCA analysis in  " ).append ( QString::number ( tt.toc () / 1000 ) ).append ( " seconds.\n" );
        plot_qstring(str,false,false);
        tt.tic();
        updateProgress(80);
        QCoreApplication::processEvents ();
        StemPointDetection detect ( getControl ()->getCloudPtr (), isStem );
        str = "";
        str.append ( "Detected stem points in  " ).append ( QString::number ( tt.toc () / 1000 ) ).append ( " seconds.\n" );
        plot_qstring(str,false,true);
        updateProgress(100);
        QCoreApplication::processEvents ();

        getControl ()->setE1 ( e1 );
        getControl ()->setE2 ( e2 );
        getControl ()->setE3 ( e3 );
        getControl ()->setIsStem ( detect.getStemPtsNew () );

        tt.tic();
        str = "";
        str.append ( "Starting optimization, parameters are :.\n" );
        plot_qstring(str,true,false);
        updateProgress(0);
        plot_qstring(user_coefficients.toQString(),false,false);
        updateProgress(1);
        QCoreApplication::processEvents ();
        boost::shared_ptr<Optimization> optimize (new Optimization(_method_coefficients.max_iterations,_method_coefficients.seeds_per_voxel,
                                                                   _method_coefficients.min_dist));
        optimize->setCloudPtr(getControl()->getCloudPtr());
        optimize->setIsStem(getControl()->getIsStem());
        optimize->setTreeID(files.at(i).toStdString());
        optimize->setCoefficients(user_coefficients);
        connect (&(*optimize), SIGNAL(emit_progress(int) ), this, SLOT(updateProgress(int) ),Qt::DirectConnection );
        optimize->optimize();
        _method_coefficients = optimize->getCoefficients();
        str = "";
        str.append("Optimization took ").append(QString::number(tt.toc()/1000)).append(" seconds. New parameter are :\n").append(_method_coefficients.toQString());
        plot_qstring(str,false,true);
        updateProgress(100);
        QCoreApplication::processEvents ();
        writeConsole(_method_coefficients.toQString());
        if ( getControl ()->getCloudPtr () != 0 ) {
            tt.tic ();
            updateProgress ( 0 );
            QCoreApplication::processEvents ();

            SphereFollowing sphereFollowing ( this->getControl ()->getCloudPtr (), getControl()->getIsStem(), 1, _method_coefficients );
            str = "";
            str.append("Final Speherefollowing took ").append(QString::number(tt.toc()/1000)).append(" seconds. \n ");
            plot_qstring(str,true,false);
            updateProgress(50);
            QCoreApplication::processEvents ();
            tt.tic ();
            boost::shared_ptr<simpleTree::Tree> tree = boost::make_shared<simpleTree::Tree> ( sphereFollowing.getCylinders (), this->getControl ()->getCloudPtr (),
                                                                                              this->getControl ()->getTreeID (), true );
            str = "";
            str.append("Building the model took ").append(QString::number(tt.toc()/1000)).append(" seconds. \n ");
            plot_qstring(str,false,false);
            updateProgress(90);

            tt.tic();
            simpleTree::Allometry allom;
            allom.setTree(tree);
            allom.setCoefficients(_method_coefficients.a,_method_coefficients.b);
            allom.setFac(_method_coefficients.fact);
            allom.setMinRad(_method_coefficients.minRad);
            allom.improveTree();
            str = "";
            str.append("Allometry did take ").append(QString::number(tt.toc()/1000)).append(" seconds. \n ");
            plot_qstring(str,false,true);
            updateProgress(100);

            getControl ()->setTreePtr ( tree );
            getControl()->setTreeID(getControl()->getTreeID().append("_test_friday."));
            if ( getControl ()->getTreePtr () != 0 ) {
                ExportPly tree_ply ( getControl ()->getTreePtr ()->getCylinders (), getControl ()->getTreeID (), "tree" );
                ExportPly stem_ply ( getControl ()->getTreePtr ()->getStemCylinders (), getControl ()->getTreeID (), "stem" );
                WriteCSV write ( getControl ()->getTreePtr (), getControl ()->getTreeID () );
            }

            ui->qvtkWidget->update ();

        }
        result.append(QString::number(1)).append(",").append(files.at(i)).append(",")
                .append(QString::number(getControl ()->getTreePtr ()->getVolume())).append(",")
                .append(QString::number(getControl ()->getTreePtr ()->getDBH())).append(",")
                .append(QString::number(getControl ()->getTreePtr ()->getBaseDiameter())).append("\n");

        ui->qvtkWidget->update ();
    }
    QString str = "";
    str.append("The complete Folder was run in ").append(QString::number(tt2.toc()/1000)).append(" seconds. \n ");
    plot_qstring(str,true, false);
    plot_qstring(result, false,true);

}
boost::shared_ptr<PointCloudD>
PCLViewer::convertPointCloud ( PointCloudI::Ptr  cloud , int r, int g, int b)
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
    minX = std::numeric_limits<float>::max();
    minY = std::numeric_limits<float>::max();
    minZ = std::numeric_limits<float>::max();
    maxX = std::numeric_limits<float>::min();
    maxY = std::numeric_limits<float>::min();
    maxZ = std::numeric_limits<float>::min();
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
                }
                viewer->updatePointCloud ( cloud, "cloud" );
                viewer->removeShape ( "point_text" );
                viewer->addText ( "Intensity", 10, 120,20 ,0, 0.2, 0.4, "point_text" );
                ui->qvtkWidget->update ();
            } else {
                QMessageBox::warning ( this, tr ( "Simple Tree" ), tr ( "No Point Cloud found\n"
                                                                        "Please load a Point Cloud first" ),
                                       QMessageBox::Ok );
            }
            break;
        }
        case 1: {
            if ( this->getControl ()->getE1().size() >= 0 ) {
                std::vector<float> eigen_values = this->getControl ()->getE1();
                if ( eigen_values.size() == this->cloud->points.size () ) {
                    std::vector<float>::iterator max_it = std::max_element(eigen_values.begin(),eigen_values.end());
                    float max = *max_it;
                    std::vector<float>::iterator min_it = std::min_element(eigen_values.begin(),eigen_values.end());
                    float min = *min_it;
                    float dif = max-min;

                    for ( size_t i = 0; i < eigen_values.size (); i++ ) {
                        float ratio = eigen_values[i]/dif;
                        this->cloud->points[i].r = 255 *ratio-255*min/dif;
                        this->cloud->points[i].g = 255 - (255 *ratio-255*min/dif);
                        this->cloud->points[i].b = 0;
                        this->cloud->points[i].a = 255;
                    }
                }
                viewer->removeShape ( "point_text" );
                viewer->addText ( "First principal component", 10, 120, 20,0, 0.2, 0.4, "point_text" );
                viewer->updatePointCloud ( cloud, "cloud" );
                ui->qvtkWidget->update ();
            } else {
                QMessageBox::warning ( this, tr ( "Simple Tree" ), tr ( "Compute PCA first"),
                                       QMessageBox::Ok );
            }
            break;
        }
        case 2: {
            if ( this->getControl ()->getE2().size() >= 0 ) {
                std::vector<float> eigen_values = this->getControl ()->getE2();
                if ( eigen_values.size() == this->cloud->points.size () ) {
                    std::vector<float>::iterator max_it = std::max_element(eigen_values.begin(),eigen_values.end());
                    float max = *max_it;
                    std::vector<float>::iterator min_it = std::min_element(eigen_values.begin(),eigen_values.end());
                    float min = *min_it;
                    float dif = max-min;

                    for ( size_t i = 0; i < eigen_values.size (); i++ ) {
                        float ratio = eigen_values[i]/dif;
                        this->cloud->points[i].r = 255 *ratio-255*min/dif;
                        this->cloud->points[i].g = 255 - (255 *ratio-255*min/dif);
                        this->cloud->points[i].b = 0;
                        this->cloud->points[i].a = 255;
                    }
                }
                viewer->removeShape ( "point_text" );
                viewer->addText ( "Second principal component", 10, 120, 20,0, 0.2, 0.4, "point_text" );
                viewer->updatePointCloud ( cloud, "cloud" );
                ui->qvtkWidget->update ();
            } else {
                QMessageBox::warning ( this, tr ( "Simple Tree" ), tr ( "Compute PCA first"),
                                       QMessageBox::Ok );
            }
            break;
        }
        case 3: {
            if ( this->getControl ()->getE3().size() >= 0 ) {
                std::vector<float> eigen_values = this->getControl ()->getE3();
                if ( eigen_values.size() == this->cloud->points.size () ) {
                    std::vector<float>::iterator max_it = std::max_element(eigen_values.begin(),eigen_values.end());
                    float max = *max_it;
                    std::vector<float>::iterator min_it = std::min_element(eigen_values.begin(),eigen_values.end());
                    float min = *min_it;
                    float dif = max-min;

                    for ( size_t i = 0; i < eigen_values.size (); i++ ) {
                        float ratio = eigen_values[i]/dif;
                        this->cloud->points[i].r = 255 *ratio-255*min/dif;
                        this->cloud->points[i].g = 255 - (255 *ratio-255*min/dif);
                        this->cloud->points[i].b = 0;
                        this->cloud->points[i].a = 255;
                    }
                }
                viewer->removeShape ( "point_text" );
                viewer->addText ( "Third principal component", 10, 120, 20,0, 0.2, 0.4, "point_text" );
                viewer->updatePointCloud ( cloud, "cloud" );
                ui->qvtkWidget->update ();
            } else {
                QMessageBox::warning ( this, tr ( "Simple Tree" ), tr ( "Compute PCA first"),
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
                }
                viewer->removeShape ( "point_text" );
                viewer->addText ( "Surface planarity", 10, 120,20, 0, 0.2, 0.4, "point_text" );
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

                viewer->removeShape ( "point_text" );
                viewer->addText ( "Main branching structure", 10, 120,20, 0, 0.2, 0.4, "point_text" );
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
            ;
        }
    }
}
void PCLViewer::changeTreeColor() {
    if ( this->getControl ()->getTreePtr () != 0 && this->getControl()->getCloudPtr() !=0) {
        tree_color++;
        tree_color = tree_color % tree_color_max;
        switch ( tree_color ) {
        case 0: {
            if ( this->getControl ()->getTreePtr () != 0 ) {
                Color_Factory color_factory;

                std::vector<boost::shared_ptr<simpleTree::Segment> > segments = tree_ptr->getSegments();
                for ( size_t i = 0; i < segments.size(); i ++ ) {
                    boost::shared_ptr<simpleTree::Segment> seg = segments.at ( i );
                    for ( size_t j = 0; j < seg->getCylinders().size(); j++ ) {
                        std::stringstream ss;
                        ss << "cylinder" << i << "," << j;
                        boost::shared_ptr<simpleTree::Cylinder> cylinder = seg->getCylinders().at(j);
                        Color color = color_factory.get_color_from_radius(cylinder->getRadius());
                        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_COLOR, color.r/255.0f,color.g/255.0f,color.b/255.0f, ss.str () );
                    }
                    ui->qvtkWidget->update ();


                    viewer->removeShape ( "cylinder_text" );
                    viewer->addText ( "Cylinders are colored by diameter class", 10, 150, 20,0, 0.2, 0.4, "cylinder_text" );
                    ui->qvtkWidget->update ();
                }
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
                std::vector<double> r_vec;
                std::vector<double> g_vec;
                std::vector<double> b_vec;
                for ( int i = min ; i < max +1; i ++ ) {
                    double r = (rand() % 255)/255.0f;
                    double g =(rand() % 255)/255.0f;
                    double b =(rand() % 255)/255.0f;
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
            break;
        }
        default:
            ;
        }
    }

}
void PCLViewer::compute_ICP() {
    allign.reset(new AllignPointCloudDialog(this));
    allign->setViewer(viewer);
    allign->setUi(ui);
    allign->setGuiPtr(shared_from_this());
    allign->init();
}
void
PCLViewer::compute_crown()
{
    getControl()->getTreePtr()->crown->reset_concave_hull(hull_radius);
    setTreePtr(getControl()->getTreePtr());
    ui->qvtkWidget->update ();
}
void
PCLViewer::curvature_dialog()
{
    curvature.reset(new CurvatureDialog(this));
    curvature->setViewer(shared_from_this());
    curvature->init();
}
void
PCLViewer::compute_eucalypt()
{
    euca_dialog.reset(new Eucalypt_folder(this));
    connect(&*euca_dialog, SIGNAL(emitQString(QString, bool, bool)), this, SLOT(plot_qstring(QString, bool, bool)));
    connect(&*euca_dialog, SIGNAL(emitTreeID(QString)), &*getControl(), SLOT(setTreeID(QString)));
    connect(&*euca_dialog, SIGNAL(emitTree(boost::shared_ptr<simpleTree::Tree>)), &*getControl(), SLOT(setTreePtr(boost::shared_ptr<simpleTree::Tree>)));
    connect(&*euca_dialog, SIGNAL(emitCloud(PointCloudI::Ptr, bool)), &*getControl(), SLOT(setCloudPtr (PointCloudI::Ptr , bool)));
    connect(&*euca_dialog, SIGNAL(emit_progress(int)), this, SLOT(updateProgress(int)));
    connect(&*euca_dialog, SIGNAL(emitIsStem(std::vector<bool>)), &*getControl(), SLOT(setIsStem (std::vector<bool>)));
    connect(&*euca_dialog, SIGNAL(emitE1(std::vector<float>)), &*getControl(), SLOT(setE1 (std::vector<float>)));
    connect(&*euca_dialog, SIGNAL(emitE2(std::vector<float>)), &*getControl(), SLOT(setE2 (std::vector<float>)));
    connect(&*euca_dialog, SIGNAL(emitE3(std::vector<float>)), &*getControl(), SLOT(setE3 (std::vector<float>)));
    euca_dialog->compute();
}
void
PCLViewer::compute_quercus()
{
    quercus_dialog.reset(new Quercus_folder(this));
    connect(&*quercus_dialog, SIGNAL(emitQString(QString, bool, bool)), this, SLOT(plot_qstring(QString, bool, bool)));
    connect(&*quercus_dialog, SIGNAL(emitTreeID(QString)), &*getControl(), SLOT(setTreeID(QString)));
    connect(&*quercus_dialog, SIGNAL(emitTree(boost::shared_ptr<simpleTree::Tree>)), &*getControl(), SLOT(setTreePtr(boost::shared_ptr<simpleTree::Tree>)));
    connect(&*quercus_dialog, SIGNAL(emitCloud(PointCloudI::Ptr, bool)), &*getControl(), SLOT(setCloudPtr (PointCloudI::Ptr , bool)));
    connect(&*quercus_dialog, SIGNAL(emit_progress(int)), this, SLOT(updateProgress(int)));
    connect(&*quercus_dialog, SIGNAL(emitIsStem(std::vector<bool>)), &*getControl(), SLOT(setIsStem (std::vector<bool>)));
    connect(&*quercus_dialog, SIGNAL(emitE1(std::vector<float>)), &*getControl(), SLOT(setE1 (std::vector<float>)));
    connect(&*quercus_dialog, SIGNAL(emitE2(std::vector<float>)), &*getControl(), SLOT(setE2 (std::vector<float>)));
    connect(&*quercus_dialog, SIGNAL(emitE3(std::vector<float>)), &*getControl(), SLOT(setE3 (std::vector<float>)));
    quercus_dialog->compute();
}
void
PCLViewer::compute_pinus()
{
    pinus_dialog.reset(new Pinus_folder(this));
    connect(&*pinus_dialog, SIGNAL(emitQString(QString, bool, bool)), this, SLOT(plot_qstring(QString, bool, bool)));
    connect(&*pinus_dialog, SIGNAL(emitTreeID(QString)), &*getControl(), SLOT(setTreeID(QString)));
    connect(&*pinus_dialog, SIGNAL(emitTree(boost::shared_ptr<simpleTree::Tree>)), &*getControl(), SLOT(setTreePtr(boost::shared_ptr<simpleTree::Tree>)));
    connect(&*pinus_dialog, SIGNAL(emitCloud(PointCloudI::Ptr, bool)), &*getControl(), SLOT(setCloudPtr (PointCloudI::Ptr , bool)));
    connect(&*pinus_dialog, SIGNAL(emit_progress(int)), this, SLOT(updateProgress(int)));
    connect(&*pinus_dialog, SIGNAL(emitIsStem(std::vector<bool>)), &*getControl(), SLOT(setIsStem (std::vector<bool>)));
    connect(&*pinus_dialog, SIGNAL(emitE1(std::vector<float>)), &*getControl(), SLOT(setE1 (std::vector<float>)));
    connect(&*pinus_dialog, SIGNAL(emitE2(std::vector<float>)), &*getControl(), SLOT(setE2 (std::vector<float>)));
    connect(&*pinus_dialog, SIGNAL(emitE3(std::vector<float>)), &*getControl(), SLOT(setE3 (std::vector<float>)));
    pinus_dialog->compute();
}
void
PCLViewer::compute_ery()
{
    ery_dialog.reset(new Ery_folder(this));
    connect(&*ery_dialog, SIGNAL(emitQString(QString, bool, bool)), this, SLOT(plot_qstring(QString, bool, bool)));
    connect(&*ery_dialog, SIGNAL(emitTreeID(QString)), &*getControl(), SLOT(setTreeID(QString)));
    connect(&*ery_dialog, SIGNAL(emitTree(boost::shared_ptr<simpleTree::Tree>)), &*getControl(), SLOT(setTreePtr(boost::shared_ptr<simpleTree::Tree>)));
    connect(&*ery_dialog, SIGNAL(emitCloud(PointCloudI::Ptr, bool)), &*getControl(), SLOT(setCloudPtr (PointCloudI::Ptr , bool)));
    connect(&*ery_dialog, SIGNAL(emit_progress(int)), this, SLOT(updateProgress(int)));
    connect(&*ery_dialog, SIGNAL(emitIsStem(std::vector<bool>)), &*getControl(), SLOT(setIsStem (std::vector<bool>)));
    connect(&*ery_dialog, SIGNAL(emitE1(std::vector<float>)), &*getControl(), SLOT(setE1 (std::vector<float>)));
    connect(&*ery_dialog, SIGNAL(emitE2(std::vector<float>)), &*getControl(), SLOT(setE2 (std::vector<float>)));
    connect(&*ery_dialog, SIGNAL(emitE3(std::vector<float>)), &*getControl(), SLOT(setE3 (std::vector<float>)));
    ery_dialog->compute();
}
void
PCLViewer::compute_plane()
{
    plane_dialog.reset(new Plane_folder(this));
    connect(&*plane_dialog, SIGNAL(emitQString(QString, bool, bool)), this, SLOT(plot_qstring(QString, bool, bool)));
    connect(&*plane_dialog, SIGNAL(emitTreeID(QString)), &*getControl(), SLOT(setTreeID(QString)));
    connect(&*plane_dialog, SIGNAL(emitTree(boost::shared_ptr<simpleTree::Tree>)), &*getControl(), SLOT(setTreePtr(boost::shared_ptr<simpleTree::Tree>)));
    connect(&*plane_dialog, SIGNAL(emitCloud(PointCloudI::Ptr, bool)), &*getControl(), SLOT(setCloudPtr (PointCloudI::Ptr , bool)));
    connect(&*plane_dialog, SIGNAL(emit_progress(int)), this, SLOT(updateProgress(int)));
    connect(&*plane_dialog, SIGNAL(emitIsStem(std::vector<bool>)), &*getControl(), SLOT(setIsStem (std::vector<bool>)));
    connect(&*plane_dialog, SIGNAL(emitE1(std::vector<float>)), &*getControl(), SLOT(setE1 (std::vector<float>)));
    connect(&*plane_dialog, SIGNAL(emitE2(std::vector<float>)), &*getControl(), SLOT(setE2 (std::vector<float>)));
    connect(&*plane_dialog, SIGNAL(emitE3(std::vector<float>)), &*getControl(), SLOT(setE3 (std::vector<float>)));
    plane_dialog->compute();
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

    pcl::ConditionalRemoval<PointI> condrem;
    condrem.setCondition ( intens_cond );
    condrem.setInputCloud ( getControl ()->getCloudPtr () );
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
PCLViewer::crop_box () {
    if ( getControl ()->getCloudPtr () != 0 ) {
        crop_box_dialog.reset(new CropBoxDialog);
        crop_box_dialog->setViewer(shared_from_this());
        crop_box_dialog->init();
        cb_args.box_dialog_ptr = crop_box_dialog;

    } else {
        QMessageBox::warning ( this, tr ( "Simple Tree" ), tr ( "No Point Cloud found\n"
                                                                "Please load a Point Cloud first" ),
                               QMessageBox::Ok );
    }

}
void
PCLViewer::cropsphere () {
    if ( getControl ()->getCloudPtr () != 0 ) {
        crop_sphere_dialog.reset(new CropSphereDialog);
        crop_sphere_dialog->setViewer(shared_from_this());
        crop_sphere_dialog->init();
        cb_args.sphere_dialog_ptr = crop_sphere_dialog;
    } else {
        QMessageBox::warning ( this, tr ( "Simple Tree" ), tr ( "No Point Cloud found\n"
                                                                "Please load a Point Cloud first" ),
                               QMessageBox::Ok );
    }

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

    VoxelGridFilter voxel(voxel_grid_size,1);
    voxel.setInput(getControl()->getCloudPtr());
    voxel.voxel_grid_filter();
    cloud_filtered = voxel.getOutput();
    {
        int size_before = getControl ()->getCloudPtr ()->points.size ();
        int size_after = cloud_filtered->points.size ();
        float a = size_before;
        float b = size_after;
        float percentage = ( 100.0f * b ) / ( a );
        writeConsole ( QString ( "Downsampling done, " ).append ( QString::number ( size_after ) ).append ( " points left, size reduced to " ).append (
                           QString::number ( percentage ).append ( " percent of original cloud.\n" ) ) );
    }

    getControl ()->setCloudPtr ( cloud_filtered );
    getControl ()->getGuiPtr ()->writeConsole (
                "--------------------------------------------------------------------------------------------------------------------------------------------------------------\n" );
    voxel_grid_size = 0.01;
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
PCLViewer::connectToController ( boost::shared_ptr<Controller> control ) {
    this->control = control;
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
    pcl::PrincipalCurvaturesEstimation<PointI, PointI, pcl::PrincipalCurvatures> principalCurvaturesEstimation;
    principalCurvaturesEstimation.setInputCloud ( cloud );
    principalCurvaturesEstimation.setInputNormals ( cloud );
    pcl::search::KdTree<PointI>::Ptr tree_normal ( new pcl::search::KdTree<PointI> () );
    principalCurvaturesEstimation.setSearchMethod ( tree_normal );
    principalCurvaturesEstimation.setRadiusSearch ( 0.03 );
    CurvatureCloud::Ptr principalCurvatures ( new CurvatureCloud );
    principalCurvaturesEstimation.compute ( *principalCurvatures );
    QString str;
    float f = tt.toc () / 1000;
    str.append ( "Computed principal curvatures in  " ).append ( QString::number ( f ) ).append ( " seconds.\n" );
    getControl ()->getGuiPtr ()->writeConsole ( str );
    getControl ()->getGuiPtr ()->writeConsole ( "\n" );
    return principalCurvatures;
}

void
PCLViewer::computeNormals () {
    if(getControl()->getCloudPtr()!=0)
    {
        getControl ()->getGuiPtr ()->writeConsole ( "\n" );
        getControl ()->getGuiPtr ()->writeConsole (
                    "--------------------------------------------------------------------------------------------------------------------------------------------------------------\n" );
        this->updateProgress ( 0 );
        QCoreApplication::processEvents ();
        computeNormals ( getControl ()->getCloudPtr () );
        getControl ()->getGuiPtr ()->updateProgress ( 40 );
        QCoreApplication::processEvents ();
        getControl ()->getGuiPtr ()->updateProgress ( 30 );
        QCoreApplication::processEvents ();
        tt.tic ();
        std::vector<float> e1;
        std::vector<float> e2;
        std::vector<float> e3;
        std::vector<bool> isStem;

        EigenValueEstimator es ( getControl ()->getCloudPtr (), e1, e2, e3, isStem, 0.035f );
        getControl ()->setE1 ( e1 );
        getControl ()->setE2 ( e2 );
        getControl ()->setE3 ( e3 );
        getControl()->getGuiPtr()->writeConsole(es.result);
        getControl ()->getGuiPtr ()->updateProgress ( 70 );
        QCoreApplication::processEvents ();

        StemPointDetection detect ( getControl ()->getCloudPtr (), isStem );
        getControl ()->setIsStem (detect.getStemPtsNew());
        getControl ()->getGuiPtr ()->writeConsole(detect.result);
        getControl ()->getGuiPtr ()->updateProgress (100);
        getControl ()->getGuiPtr ()->writeConsole (
                    "--------------------------------------------------------------------------------------------------------------------------------------------------------------\n" );
        QCoreApplication::processEvents ();
    }
}
void
PCLViewer::compute_detectTree () {
    if ( getControl ()->getCloudPtr () != 0 ) {
        pcl::console::TicToc tt;
        tt.tic ();
        QString str = "\n";
        writeConsole ( str );
        getControl ()->getGuiPtr ()->writeConsole (
                    "--------------------------------------------------------------------------------------------------------------------------------------------------------------\n" );
        updateProgress ( 0 );
        QString str2 = coeff_ptr->struct_to_qstring ( _method_coefficients );
        writeConsole ( str2 );
        QCoreApplication::processEvents ();
        SphereFollowing sphereFollowing ( this->getControl ()->getCloudPtr (), getControl()->getIsStem(), 1 , _method_coefficients );
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
                                                                                          this->getControl ()->getTreeID (), true );
        float f = tt.toc () / 1000;
        str.append ( "Done tree structure in " ).append ( QString::number ( f ) ).append ( " seconds.\n" );
        writeConsole ( str );
        updateProgress ( 100 );
        getControl ()->getGuiPtr ()->writeConsole (
                    "--------------------------------------------------------------------------------------------------------------------------------------------------------------\n" );
        getControl ()->setTreePtr ( tree );

    } else {
        QMessageBox::warning ( this, tr ( "Simple Tree" ), tr ( "No Point Cloud found\n"
                                                                "Please load a Point Cloud first" ),
                               QMessageBox::Ok );
    }
}

void
PCLViewer::compute_final_model()
{

    pcl::console::TicToc tt;
    tt.tic ();

    if(_method_coefficients.max_iterations>0)
    {
        boost::shared_ptr<Optimization> optimize (new Optimization(_method_coefficients.max_iterations,_method_coefficients.seeds_per_voxel,
                                                                   _method_coefficients.min_dist));

        plot_qstring("Opimization of parameters : ",true,false);
        writeConsole(_method_coefficients.toQString());
        optimize->setCloudPtr(getControl()->getCloudPtr());
        optimize->setIsStem(getControl()->getIsStem());
        optimize->setTreeID(getControl ()->getTreeID());
        optimize->setCoefficients(_method_coefficients);
        connect(&*optimize, SIGNAL(emit_progress(int)), this, SLOT(updateProgress(int)));
        optimize->optimize();
        _method_coefficients = optimize->getCoefficients();
        plot_qstring(QString("Opimization of parameters : \n ").append(QString::number((tt.toc()/100))).append(" seconds."));
        plot_qstring("to : \n ");
        plot_qstring(_method_coefficients.toQString(), false, true);
    }else{
        plot_qstring("No optimization desired. \n", true, true);
    }


    updateProgress ( 0 );
    QString str2 = coeff_ptr->struct_to_qstring ( _method_coefficients );
    plot_qstring ( str2 ,true,true);
    SphereFollowing sphereFollowing ( this->getControl ()->getCloudPtr (), getControl()->getIsStem(), 3, _method_coefficients, 10, 1 );

    updateProgress ( 50 );

    tt.tic ();
    boost::shared_ptr<simpleTree::Tree> tree = boost::make_shared<simpleTree::Tree> ( sphereFollowing.getCylinders (), this->getControl ()->getCloudPtr (),
                                                                                      this->getControl ()->getTreeID (), true);
    getControl ()->setTreePtr ( tree );
    float f = tt.toc () / 1000;
    QString str;
    str.append ( "Done tree structure in " ).append ( QString::number ( f ) ).append ( " seconds.\n" );
    plot_qstring( str, true, true );
    updateProgress ( 100 );


    if(_method_coefficients.fact!=100000)
    {
        plot_qstring ( "Allometry improvement done" , true, true);
        simpleTree::Allometry allom;
        allom.setTree(tree);
        allom.setCoefficients(_method_coefficients.a,_method_coefficients.b);
        allom.setFac(_method_coefficients.fact);
        allom.setMinRad(_method_coefficients.minRad);
        allom.improveTree();
        getControl ()->setTreePtr ( tree );
    } else {
        writeConsole("No Allometry improvement desired. \n");
    }
}

void
PCLViewer::denoise_pinus()
{
    pinus_denoise_dialog.reset(new Pine_denoise(shared_from_this(), this));
    connect(&*pinus_denoise_dialog, SIGNAL(emitQString(QString, bool, bool)), this, SLOT(plot_qstring(QString, bool, bool)));
    connect(&*pinus_denoise_dialog, SIGNAL(emitTreeID(QString)), &*getControl(), SLOT(setTreeID(QString)));
    connect(&*pinus_denoise_dialog, SIGNAL(emitCloud(PointCloudI::Ptr, bool)), &*getControl(), SLOT(setCloudPtr (PointCloudI::Ptr , bool)));
    connect(&*pinus_denoise_dialog, SIGNAL(emit_progress(int)), this, SLOT(updateProgress(int)));
    pinus_denoise_dialog->compute();
}
void
PCLViewer::denoise_oak()
{
    quercus_denoise_dialog.reset(new Quercus_denoise(shared_from_this(), this));
    connect(&*quercus_denoise_dialog, SIGNAL(emitQString(QString, bool, bool)), this, SLOT(plot_qstring(QString, bool, bool)));
    connect(&*quercus_denoise_dialog, SIGNAL(emitTreeID(QString)), &*getControl(), SLOT(setTreeID(QString)));
    connect(&*quercus_denoise_dialog, SIGNAL(emitCloud(PointCloudI::Ptr, bool)), &*getControl(), SLOT(setCloudPtr (PointCloudI::Ptr , bool)));
    connect(&*quercus_denoise_dialog, SIGNAL(emit_progress(int)), this, SLOT(updateProgress(int)));
    quercus_denoise_dialog->compute();
}
void
PCLViewer::denoise_ery()
{

    ery_denoise_dialog.reset(new Ery_denoise(shared_from_this(), this));
    connect(&*ery_denoise_dialog, SIGNAL(emitQString(QString, bool, bool)), this, SLOT(plot_qstring(QString, bool, bool)));
    connect(&*ery_denoise_dialog, SIGNAL(emitTreeID(QString)), &*getControl(), SLOT(setTreeID(QString)));
    connect(&*ery_denoise_dialog, SIGNAL(emitCloud(PointCloudI::Ptr, bool)), &*getControl(), SLOT(setCloudPtr (PointCloudI::Ptr , bool)));
    connect(&*ery_denoise_dialog, SIGNAL(emit_progress(int)), this, SLOT(updateProgress(int)));
    ery_denoise_dialog->compute();

}
void
PCLViewer::denoise_eucalypt()
{
    pcl::console::TicToc tt2;
    tt2.tic();
    tt.tic ();
    QString error;
    QDir dir ( "../data/australia" );
    std::cout << dir.absolutePath().toStdString();
    QStringList filters;
    filters << "*.txt" ;
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


        {

            std::vector<bool> isStem;
            std::vector<float> e1;
            std::vector<float> e2;
            std::vector<float> e3;
            EigenValueEstimator es ( getControl ()->getCloudPtr (), e1, e2, e3, isStem, 0.035f );
            getControl ()->setE1 ( e1 );
            getControl ()->setE2 ( e2 );
            getControl ()->setE3 ( e3 );


            float min_e1 = *(std::min_element(e1.begin(),e1.end()));
            float max_e1 = *(std::max_element(e1.begin(),e1.end()));
            float min_e2 = *(std::min_element(e2.begin(),e2.end()));
            float max_e2 = *(std::max_element(e2.begin(),e2.end()));
            float min_e3 = *(std::min_element(e3.begin(),e3.end()));
            float max_e3 = *(std::max_element(e3.begin(),e3.end()));




            float min1b = 0;
            float max1b = 60;
            float min2b = 0;
            float max2b = 100;
            float min3b = 0;
            float max3b = 100;

            float min1 = min_e1 + (max_e1 - min_e1)/100*min1b;
            float max1 = min_e1 + (max_e1 - min_e1)/100*max1b;
            float min2 = min_e2 + (max_e2 - min_e2)/100*min2b;
            float max2 = min_e2 + (max_e2 - min_e2)/100*max2b;
            float min3 = min_e3 + (max_e3 - min_e3)/100*min3b;
            float max3 = min_e3 + (max_e3 - min_e3)/100*max3b;

            boost::shared_ptr<PointCloudI> new_cloud (new PointCloudI);
            std::vector<float> e1_new;
            std::vector<float> e2_new;
            std::vector<float> e3_new;

            for(size_t i = 0; i < getControl()->getCloudPtr()->points.size(); i++)
            {
                float pc1,pc2,pc3;
                pc1 = e1.at(i);
                pc2 = e2.at(i);
                pc3 = e3.at(i);

                if(pc1>=min1&&pc1<=max1&&pc2>=min2&&pc2<=max2&&pc3>=min3&&pc3<=max3)
                {
                    new_cloud->push_back(getControl()->getCloudPtr()->points.at(i));
                    e1_new.push_back(e1.at(i));
                    e2_new.push_back(e2.at(i));
                    e3_new.push_back(e3.at(i));

                }
            }
            float size_before = getControl()->getCloudPtr()->points.size();
            float size_after  = new_cloud->points.size();
            float perc = 0;
            if(size_before!=0)
            {
                perc = size_after/size_before*100;
            }
            getControl()->setCloudPtr(new_cloud);
            writeConsole(QString("\n"));
            writeLine();
            QString str;
            str.append(QString("By curvature thresholds clouod size was reduced to ")).append(QString::number(perc)).append(QString(" percent.\n"));
            str.append(QString("The new cloud has ")).append(QString::number(new_cloud->points.size())).append(QString(" points.\n"));
            writeConsole(str);
        }


        {
            QString str = "";

            str.append ( "Statistical outlier removal starts.\n All points whith a mean distance larger than " ).append ( QString::number ( 0.5 ) ).append (
                        " times the average distance to its  " ).append ( QString::number ( 30 ) ).append ( " nearest neighbors are deleted.\n" );
            writeConsole ( str );

            PointCloudI::Ptr cloud_filtered ( new PointCloudI );
            pcl::StatisticalOutlierRemoval<PointI> sor;
            sor.setInputCloud ( getControl ()->getCloudPtr () );
            sor.setMeanK ( 30 );
            sor.setStddevMulThresh ( 0.5 );
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
        }


        {
            euclidean_clustering_minsize = 100;
            euclidean_clustering_tolerance = 0.1;
            euclidean_clustering_clusternumber = 1;
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
                if(euclidean_clustering_clusternumber == 1)
                {
                    pcl::PointIndices largestCluster = cluster_indices.at(0);
                    size_t largest_cluster_size = largestCluster.indices.size();
                    float center_z = std::numeric_limits<float>::max();
                    size_t j = 0;
                    while(j<cluster_indices.size()&&cluster_indices.at(j).indices.size()>largest_cluster_size/3)
                    {
                        PointCloudI::Ptr tempCloud (new PointCloudI);
                        largestCluster= cluster_indices.at ( j );
                        for ( std::vector<int>::const_iterator pit = largestCluster.indices.begin (); pit != largestCluster.indices.end (); pit++ )
                            tempCloud->points.push_back ( getControl ()->getCloudPtr ()->points[*pit] ); //*
                        Eigen::Vector4f xyz_centroid;
                        pcl::compute3DCentroid<PointI> (*tempCloud, xyz_centroid);
                        float z  = xyz_centroid[2];
                        if(z<center_z)
                        {
                            center_z = z;
                            cloud_filtered = tempCloud;
                        }
                        j++;

                    }
                }
                else
                {
                    for ( int j = 0; j < i; j++ ) {
                        pcl::PointIndices largestCluster;
                        largestCluster= cluster_indices.at ( j );
                        for ( std::vector<int>::const_iterator pit = largestCluster.indices.begin (); pit != largestCluster.indices.end (); pit++ )
                            cloud_filtered->points.push_back ( getControl ()->getCloudPtr ()->points[*pit] ); //*
                    }
                }
            }

            int size_after = cloud_filtered->points.size ();
            cloud_filtered->width = size_after;
            cloud_filtered->height = 1;


            getControl ()->setCloudPtr ( cloud_filtered );
        }

        QString path = dir.absolutePath().append ( "/automatic/" ).append( files.at ( i )).append(".pcd");
        pcl::io::savePCDFileASCII (path.toStdString(), *getControl()->getCloudPtr());


        ui->qvtkWidget->update ();
    }
    writeConsole(error);
    QString timestr("running complete folder took ");
    timestr.append(QString::number(tt2.toc()/1000)).append(QString(" seconds.\n"));
    writeConsole(timestr);

}
void
PCLViewer::delete_method () {
    QString method = method_dialog_ptr->comboBox->currentText ();
    std::cout << method.toStdString () << std::endl;
    coeff_ptr->settings->remove ( method );
    coeff_ptr->settings->sync ();
}
void
PCLViewer::exportPCDFile () {
    if ( getControl ()->getCloudPtr () != 0 ) {
        QString dir("../data/");
        dir.append(QString::fromStdString(getControl()->getTreeID()));
        QString file = QFileDialog::getSaveFileName(this, tr("Save File"),
                                                    dir, tr ( "Clouds (*.pcd);;All Files(*)" ) );
        if ( !file.endsWith ( ".pcd" ) ) {
            file.append ( ".pcd" );
        }

        pcl::io::savePCDFileASCII<PointI> ( file.toStdString (), * ( getControl ()->getCloudPtr () ) );
    }
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
PCLViewer::exportPly () {
    if ( getControl ()->getTreePtr () != 0 ) {
        ExportPly tree_ply ( getControl ()->getTreePtr ()->getCylinders (), getControl ()->getTreeID (), "tree" );
        ExportPly stem_ply ( getControl ()->getTreePtr ()->getStemCylinders (), getControl ()->getTreeID (), "stem" );
        WriteCSV write ( getControl ()->getTreePtr (), getControl ()->getTreeID () );
    } else {
        QMessageBox::warning ( this, tr ( "Simple Tree" ), tr ( "No Tree Model computed\n" ), QMessageBox::Ok );
    }
    ui->qvtkWidget->update ();
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

boost::shared_ptr<Controller>
PCLViewer::getControl () {
    return control.lock ();
}
void
PCLViewer::init () {
    coeff_ptr.reset ( new SetCoefficients ( shared_from_this () ) );
    coeff_ptr->set_coefficients_for_method ( "Prunus_avium_Breisach_Germany" );

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
PCLViewer::load_camera_position()
{
    QString path = QFileDialog::getOpenFileName(this, tr("Open File"),
                                                    "../config/position1.cam",
                                                    tr("Camera File (*.cam)"));
    std::string camera_path = path.toStdString();
    if(!this->viewer->loadCameraParameters(camera_path))
    {
        QMessageBox::warning ( this, tr ( "Simple Tree" ), tr ( "The camera file did not exist." ),
                               QMessageBox::Ok );
    }


}

void
PCLViewer::importPCDFile () {

    std::string file = selectFile ();
    std::string abort;
    QCoreApplication::processEvents ();
    if ( file != abort ) {
        ImportPCD import (QString::fromStdString( file ) );
        connect(&import,SIGNAL(emitProgress(int)), this, SLOT(setProgress(int)));
        connect(&import,SIGNAL(emitQString(QString, bool, bool)), this, SLOT(plot_qstring(QString, bool, bool)));
        connect(&import,SIGNAL(emitCloud(PointCloudI::Ptr, bool)), &(*getControl()), SLOT(setCloudPtr(PointCloudI::Ptr, bool)) );
        import.compute();
        plotIntensityHist ();
    }
    ui->qvtkWidget->update ();
}

float
PCLViewer::median(std::vector<float> values)
{
    float median;
    size_t size = values.size();

    sort(values.begin(), values.end());

    if (size  % 2 == 0)
    {
        median = (values[size / 2 - 1] + values[size / 2]) / 2;
    }
    else
    {
        median = values[size / 2];
    }

    return median;
}
float
PCLViewer::mean(std::vector<float> const &v)
{
    std::vector<float>::size_type taille = v.size();
    float sum = 0;
    for(std::vector<float>::const_iterator i = v.begin(); i != v.end(); ++i)
        sum += *i;
    return sum/taille;
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

    }
    file = selectFile ();

    QCoreApplication::processEvents ();
    if ( file != abort ) {
        ImportPCD import ( file, control );
        cloudB = import.getCloud ();
        sizeB = cloudB->points.size ();

    }
    *cloudB += *cloudA;
    PointCloudI::Ptr cloud_filtered ( new PointCloudI );
    pcl::VoxelGrid<PointI> sor;

    sor.setInputCloud ( cloudB );
    sor.setLeafSize ( 0.003, 0.003, 0.003 );
    sor.filter ( *cloud_filtered );
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
PCLViewer::optimize_check()
{
    QDialog * optimize_dialog=new QDialog(this,0);
    optimize_dialog_ptr.reset(new Ui_optimize_dialog);
    optimize_dialog_ptr->setupUi(optimize_dialog);
    optimize_dialog_ptr->min_dist->setValue ( _method_coefficients.min_dist );
    optimize_dialog_ptr->max_iterations->setValue ( _method_coefficients.max_iterations );
    optimize_dialog_ptr->seeds_per_voxel->setValue ( _method_coefficients.seeds_per_voxel );

    connect ( optimize_dialog_ptr->compute, SIGNAL ( clicked() ), this, SLOT ( set_method_coeff_wdout_compute() ) );
    connect ( optimize_dialog_ptr->compute, SIGNAL ( clicked() ), optimize_dialog, SLOT ( accept() ) );
    connect ( optimize_dialog_ptr->compute, SIGNAL ( clicked() ), this, SLOT ( compute_final_model() ) );

    connect ( optimize_dialog_ptr->abort, SIGNAL ( clicked() ), optimize_dialog, SLOT ( reject() ) );
    connect ( optimize_dialog_ptr->abort, SIGNAL ( clicked() ), this, SLOT ( set_optimize_iteration() ) );
    connect ( optimize_dialog_ptr->abort, SIGNAL ( clicked() ), this, SLOT ( compute_final_model() ) );

    optimize_dialog->setModal(true);
    optimize_dialog->show ();

}
void
PCLViewer::optimize_check1()
{
    QDialog * optimize_dialog=new QDialog(this,0);
    optimize_dialog_ptr.reset(new Ui_optimize_dialog);    //  Ui_allometry_dialog allomet;
    optimize_dialog_ptr->setupUi(optimize_dialog);
    optimize_dialog_ptr->min_dist->setValue ( _method_coefficients.min_dist );
    optimize_dialog_ptr->max_iterations->setValue ( _method_coefficients.max_iterations );
    optimize_dialog_ptr->seeds_per_voxel->setValue ( _method_coefficients.seeds_per_voxel );

    connect ( optimize_dialog_ptr->compute, SIGNAL ( clicked() ), this, SLOT ( set_method_coeff_wdout_compute() ) );

    connect ( optimize_dialog_ptr->compute, SIGNAL ( clicked() ), this, SLOT ( build_model_for_folder() ) );
    connect ( optimize_dialog_ptr->compute, SIGNAL ( clicked() ), optimize_dialog, SLOT ( accept() ) );

    connect ( optimize_dialog_ptr->abort, SIGNAL ( clicked() ), this, SLOT ( set_optimize_iteration() ) );
    connect ( optimize_dialog_ptr->abort, SIGNAL ( clicked() ), this, SLOT ( build_model_for_folder() ) );
    connect ( optimize_dialog_ptr->abort, SIGNAL ( clicked() ), optimize_dialog, SLOT ( reject() ) );

    optimize_dialog->setModal(true);
    optimize_dialog->show ();

}
void
PCLViewer::plot_qstring(QString str, bool firstLine, bool secondLine)
{
    if(firstLine)
    {
        writeConsole (
                    "--------------------------------------------------------------------------------------------------------------------------------------------------------------\n" );
    }
    if(str != QString(""))
    {
        writeConsole(str);
        writeConsole("\n");
    }
    if(secondLine)
    {
        writeConsole (
                    "--------------------------------------------------------------------------------------------------------------------------------------------------------------\n" );
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

    std::vector<boost::shared_ptr<simpleTree::Cylinder> > cylinders = getControl ()->getTreePtr ()->getCylinders ();
    double * diameters = new double[cylinders.size ()];
    double * volumes = new double[cylinders.size ()];
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
    delete[] diameters;
    delete[] volumes;
}

void
PCLViewer::plot () {
    plotter->setShowLegend ( true );
    int numPoints = 69;
    double ax[100], acos[100], asin[100];
    generateData ( ax, acos, asin, numPoints );
    std::vector<double> vec;
    for ( int i = 0; i < 1000; i++ ) {
        vec.push_back ( 0 );
    }
    plotter->addHistogramData ( vec, 10 );

    plotter->spin ();
    ui->qvtkWidget2->update ();
}
void
PCLViewer::reference_cloud () {
    {
        if ( getControl ()->getCloudPtr () != 0 ) {

            reference_height_dialog.reset(new ReferenceHeightDialog(this));
            reference_height_dialog->setViewer(shared_from_this());
            reference_height_dialog->init();

        } else {
            QMessageBox::warning ( this, tr ( "Simple Tree" ), tr ( "No Point Cloud found\n"
                                                                    "Please load a Point Cloud first" ),
                                   QMessageBox::Ok );
        }
    }
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
PCLViewer::reset_crown()
{
    if ( getControl ()->getTreePtr()  != 0 ) {
        QDialog * radius_dialog = new QDialog ( this, 0 );
        Ui_Dialog_Crown rad;
        rad.setupUi ( radius_dialog );

        connect ( rad.crown_radius, SIGNAL ( valueChanged ( double ) ), this, SLOT ( set_crown_radius ( double ) ) );

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
void
PCLViewer::radiusOutlierRemoval () {
    if ( getControl ()->getCloudPtr () != 0 ) {

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
PCLViewer::save_camera_position()
{
    QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"),
                               "../config/position1.cam",
                               tr("Camera File (*.cam)"));
//    qDebug() << fileName << "\n";
    std::string camera_path = fileName.toStdString();
    this->viewer->saveCameraParameters(camera_path);
}

void
PCLViewer::setTree(boost::shared_ptr<simpleTree::Tree> tree)
{
    getControl()->setTreePtr(tree);
}
void
PCLViewer::setTreeID(QString ID)
{
    getControl()->setTreeID(ID.toStdString());
}
void
PCLViewer::screenshot()
{

    QString dir("../output/Screen/");
    dir.append(QString::fromStdString(getControl()->getTreeID()));
    QString files = QFileDialog::getSaveFileName(this, tr("Save File"),
                                                 dir, tr ( "Images (*.png *.xpm *.jpg);;All Files(*)" ) );
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
PCLViewer::set_crown_radius(double r)
{
    hull_radius = r;
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
PCLViewer::set_radius_outlier_minPts ( int minPts ) {
    this->radius_outlier_minPts = minPts;
}
void
PCLViewer::set_radius_outlier_searchradius ( double searchRadius ) {
    float radius = searchRadius;
    this->radius_outlier_searchradius = radius;
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
PCLViewer::statisticalOutlierRemoval () {
    if ( getControl ()->getCloudPtr () != 0 ) {
        QDialog * statistical_dialog = new QDialog ( this, 0 );
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

}
void
PCLViewer::set_voxel_grid_size ( double size ) {
    float sizeF = size;
    voxel_grid_size = sizeF;
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
PCLViewer::setTreePtr ( boost::shared_ptr<simpleTree::Tree> tree_ptr ) {

    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
    this->tree_ptr = tree_ptr;
    setCloudPtr(getControl()->getCloudPtr());
    cylinders = tree_ptr->getCylinders();
    tree_color = 0;
    Color_Factory color_factory;


    std::vector<boost::shared_ptr<simpleTree::Segment> > segments = tree_ptr->getSegments();
    for ( size_t i = 0; i < segments.size(); i ++ ) {
        boost::shared_ptr<simpleTree::Segment> seg = segments.at ( i );
        for ( size_t j = 0; j < seg->getCylinders().size(); j++ ) {
            std::stringstream ss;
            ss << "cylinder" << i << "," << j;
            pcl::ModelCoefficients coeff;
            boost::shared_ptr<simpleTree::Cylinder> cylinder = seg->getCylinders().at(j);
            Color color = color_factory.get_color_from_radius(cylinder->getRadius());
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
            viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_COLOR, color.r/255.0f,color.g/255.0f,color.b/255.0f, ss.str () );
            viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_SHADING, pcl::visualization::PCL_VISUALIZER_SHADING_PHONG, ss.str () );
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

        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_SHADING, pcl::visualization::PCL_VISUALIZER_SHADING_PHONG, ss.str () );
        viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, ss.str () );
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr hullPoints = tree_ptr->crown->hull_cloud;
    std::vector<pcl::Vertices> polygons = tree_ptr->crown->polygons;
    viewer->addPolygonMesh<pcl::PointXYZ> ( hullPoints, polygons, "crown" );
    viewer->setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_COLOR, 0.2, 0.45, 0, "crown" );

    pcl::PointCloud<pcl::PointXYZ>::Ptr hullPoints_concave = tree_ptr->crown->hull_cloud_concave;
    std::vector<pcl::Vertices> polygons_concave = tree_ptr->crown->polygons_concave;
    viewer->addPolygonMesh<pcl::PointXYZ> ( hullPoints_concave, polygons_concave, "leaves" );
    viewer->setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_COLOR, 0.2, 0.45, 0, "leaves" );


    plotAllometry ();
    ui->qvtkWidget->update ();
    ui->qvtkWidget2->update ();
    ui->tree_color_slider->setValue ( 1 );
    ui->stem_color_slider->setValue ( 1 );
    ui->point_color_slider->setValue ( 1 );
    ui->crown_color_slider->setValue ( 1 );
    ui->leave_color_slider->setValue ( 1 );
    ui->tree_color_slider->setValue ( 0 );
    ui->stem_color_slider->setValue ( 255 );
    ui->point_color_slider->setValue ( 253 );
    ui->crown_color_slider->setValue ( 255 );
    ui->leave_color_slider->setValue ( 250 );
    viewer->addText ( tree_ptr->string (), 10, 20, 20,1, 0, 0, "tree_text" );
    ui->qvtkWidget->update ();
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
    if(stemCylinders.size()!=0)
    {
        float val = 1 - ( value / 255.0f );
        for ( size_t i = 0; i < stemCylinders.size (); i++ ) {
            std::stringstream ss;
            ss << "stemCylinder" << i;
            viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_OPACITY, val, ss.str () );
        }
        ui->qvtkWidget->update ();
    }
}

void
PCLViewer::setTreeTransparency ( int value ) {
    if(tree_ptr!=0)
    {
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

    QCoreApplication::processEvents();
    ui->qvtkWidget->update ();

}
void
PCLViewer::set_method ( int i ) {
    QString method = method_dialog_ptr->comboBox->currentText ();
    coeff_ptr->set_coefficients_for_method ( method );
    method_dialog_ptr->name->setText ( _method_coefficients.name );
    method_dialog_ptr->sphere_radius_multiplier->setValue ( _method_coefficients.sphere_radius_multiplier );
    method_dialog_ptr->epsilon_cluster_stem->setValue ( _method_coefficients.epsilon_cluster_stem );
    method_dialog_ptr->epsilon_cluster_branch->setValue ( _method_coefficients.epsilon_cluster_branch );
    method_dialog_ptr->epsilon_sphere->setValue ( _method_coefficients.epsilon_sphere );
    method_dialog_ptr->minPts_ransac_stem->setValue ( _method_coefficients.minPts_ransac_stem );
    method_dialog_ptr->minPts_ransac_branch->setValue ( _method_coefficients.minPts_ransac_branch );
    method_dialog_ptr->minPts_cluster_stem->setValue ( _method_coefficients.minPts_cluster_stem );
    method_dialog_ptr->minPts_cluster_branch->setValue ( _method_coefficients.minPts_cluster_branch );
    method_dialog_ptr->min_radius_sphere_stem->setValue ( _method_coefficients.min_radius_sphere_stem );
    method_dialog_ptr->min_radius_sphere_branch->setValue ( _method_coefficients.min_radius_sphere_branch );

}

void
PCLViewer::set_method_coeff_wdout_compute () {
    if (method_dialog_ptr!=0){
        _method_coefficients.name = method_dialog_ptr->name->displayText ();
        _method_coefficients.sphere_radius_multiplier = method_dialog_ptr->sphere_radius_multiplier->value ();
        _method_coefficients.epsilon_cluster_stem = method_dialog_ptr->epsilon_cluster_stem->value ();
        _method_coefficients.epsilon_cluster_branch = method_dialog_ptr->epsilon_cluster_branch->value ();
        _method_coefficients.epsilon_sphere = method_dialog_ptr->epsilon_sphere->value ();
        _method_coefficients.minPts_ransac_stem = method_dialog_ptr->minPts_ransac_stem->value ();
        _method_coefficients.minPts_ransac_branch = method_dialog_ptr->minPts_ransac_branch->value ();
        _method_coefficients.minPts_cluster_stem = method_dialog_ptr->minPts_cluster_stem->value ();
        _method_coefficients.minPts_cluster_branch = method_dialog_ptr->minPts_cluster_branch->value ();
        _method_coefficients.min_radius_sphere_stem = method_dialog_ptr->min_radius_sphere_stem->value ();
        _method_coefficients.min_radius_sphere_branch = method_dialog_ptr->min_radius_sphere_branch->value ();
    }
    if (allometry_dialog_ptr!=0){
        _method_coefficients.a = allometry_dialog_ptr->a->value ();
        _method_coefficients.b = allometry_dialog_ptr->b->value ();
        _method_coefficients.fact = allometry_dialog_ptr->fact->value ();
        _method_coefficients.minRad = allometry_dialog_ptr->minRad->value ();
    }
    if (optimize_dialog_ptr!=0){

        _method_coefficients.min_dist = optimize_dialog_ptr->min_dist->value ();
        _method_coefficients.max_iterations = optimize_dialog_ptr->max_iterations->value ();
        _method_coefficients.seeds_per_voxel = optimize_dialog_ptr->seeds_per_voxel->value ();
    }
    coeff_ptr->struct_to_settings ( _method_coefficients );
    coeff_ptr->settings->sync ();

}
void
PCLViewer::set_optimize_iteration()
{
    _method_coefficients.max_iterations=0;

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
void
PCLViewer::set_allom_fact()
{
    _method_coefficients.fact=100000;
    allometry_dialog_ptr->fact->setValue ( _method_coefficients.fact );
}
void
PCLViewer::setProgress(int percentage = 0)
{
    updateProgress (percentage);
    QCoreApplication::processEvents ();
}

void
PCLViewer::visualize_branch()
{
    int branchID = 0;
    float x1 = 0.28738;
    float x2 = 0.387285;
    float x3 = 0.396254;
    float y1 = -1.94153;
    float y2 = 0.430485;
    float y3 = -2.39448;
    float z1 = 4.38007;
    float z2 = 2.75629;
    float z3 = 4.1701;

    float min_dist = std::numeric_limits<float>::max();

    std::vector<boost::shared_ptr<simpleTree::Segment> > segments = getControl()->getTreePtr()->getSegments();
    //    for(std::vector<boost::shared_ptr<simpleTree::Segment> >::iterator it = segments.begin(); it != segments.end(); ++ it)
    //    {
    //        boost::shared_ptr<simpleTree::Segment> seg = *it;
    //        pcl::PointXYZ p = seg->getEnd ();
    //        QString id  = QString::fromStdString(getControl()->getTreeID());
    //        float dist = std::numeric_limits<float>::max();
    //        if(id.contains("2012"))
    //        {
    //            dist = std::sqrt( (x1-p.x)*(x1-p.x)+(y1-p.y)*(y1-p.y)+(z1-p.z)*(z1-p.z) );
    //            if(dist < min_dist)
    //            {
    //                min_dist = dist;
    //                branchID = seg->branchID;
    //            }
    //        } else if (id.contains("2013"))
    //        {
    //            dist = std::sqrt( (x2-p.x)*(x2-p.x)+(y2-p.y)*(y2-p.y)+(z2-p.z)*(z2-p.z) );
    //            if(dist < min_dist)
    //            {
    //                min_dist = dist;
    //                branchID = seg->branchID;
    //            }
    //        } else if (id.contains("2014"))
    //        {
    //            dist = std::sqrt( (x3-p.x)*(x3-p.x)+(y3-p.y)*(y3-p.y)+(z3-p.z)*(z3-p.z) );
    //            if(dist < min_dist)
    //            {
    //                min_dist = dist;
    //                branchID = seg->branchID;
    //            }
    //        }


    //    }
    for(std::vector<boost::shared_ptr<simpleTree::Segment> >::iterator it = segments.begin(); it != segments.end(); ++ it)
    {
        boost::shared_ptr<simpleTree::Segment> seg = *it;
        pcl::PointXYZ p = seg->getEnd ();
        QString id  = QString::fromStdString(getControl()->getTreeID());
        float dist = std::numeric_limits<float>::max();

        dist = std::sqrt( (x2-p.x)*(x2-p.x)+(y2-p.y)*(y2-p.y)+(z2-p.z)*(z2-p.z) );
        if(dist < min_dist)
        {
            min_dist = dist;
            branchID = seg->branchID;
        }


    }
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
    this->tree_ptr = tree_ptr;
    setCloudPtr(getControl()->getCloudPtr());
    cylinders = tree_ptr->getCylinders();
    tree_color = 0;
    Color_Factory color_factory;


    //std::vector<boost::shared_ptr<simpleTree::Segment> > segments = tree_ptr->getSegments();
    for ( size_t i = 0; i < segments.size(); i ++ ) {
        boost::shared_ptr<simpleTree::Segment> seg = segments.at ( i );
        if(seg->branchID == branchID)
        {
            for ( size_t j = 0; j < seg->getCylinders().size(); j++ ) {
                std::stringstream ss;
                ss << "cylinder" << i << "," << j;
                pcl::ModelCoefficients coeff;
                boost::shared_ptr<simpleTree::Cylinder> cylinder = seg->getCylinders().at(j);
                Color color = color_factory.get_color_from_radius(cylinder->getRadius());
                //Color color = color_factory.get_color_from_branch_order(cylinder->getSegment()->branchOrder);
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
                viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_COLOR, color.r/255.0f,color.g/255.0f,color.b/255.0f, ss.str () );
                viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_SHADING, pcl::visualization::PCL_VISUALIZER_SHADING_GOURAUD, ss.str () );
                viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, ss.str () );
            }
        } /*else
        {
            for ( size_t j = 0; j < seg->getCylinders().size(); j++ ) {




                std::stringstream ss;
                ss << "cylinder" << i << "," << j;
                pcl::ModelCoefficients coeff;
                boost::shared_ptr<simpleTree::Cylinder> cylinder = seg->getCylinders().at(j);
                Color color = color_factory.get_color_from_radius(cylinder->getRadius());
                coeff.values.push_back ( seg->getCylinders() [j]->values[0] );
                coeff.values.push_back ( seg->getCylinders() [j]->values[1] );
                coeff.values.push_back ( seg->getCylinders() [j]->values[2] );
                coeff.values.push_back ( seg->getCylinders() [j]->values[3] );
                coeff.values.push_back ( seg->getCylinders() [j]->values[4] );
                coeff.values.push_back ( seg->getCylinders() [j]->values[5] );
                coeff.values.push_back ( seg->getCylinders() [j]->values[6] );
                viewer->addCylinder ( coeff, ss.str () );
                viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
                                                      ss.str () );
            }

        }*/
    }


}

void
PCLViewer::voxel_downsampling ()

{
    if ( getControl ()->getCloudPtr () != 0 ) {
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
PCLViewer::updateProgress ( int i ) {
    if ( i == 0 ) {
        ui->progress_bar->reset ();
    }
    ui->progress_bar->setValue ( i );
    QApplication::processEvents();
    ui->qvtkWidget->update ();
    QApplication::processEvents();
}
void
PCLViewer::writeLine()
{
    writeConsole( "--------------------------------------------------------------------------------------------------------------------------------------------------------------\n");
}

void
PCLViewer::writeConsole ( QString qstr ) {
    consoleString.append ( qstr );
    ui->console->setText ( consoleString );
    ui->console->ensureCursorVisible();
    ui->console->verticalScrollBar ()->setValue ( ui->console->verticalScrollBar ()->maximum () );
    ui->qvtkWidget->update ();
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



