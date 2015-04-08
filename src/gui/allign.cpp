#include "allign.h"

AllignPointCloudDialog::AllignPointCloudDialog(QObject *parent) : QObject(parent)
{
}

void
AllignPointCloudDialog::setViewer(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
    this->viewer = viewer;
}

void
AllignPointCloudDialog::setUi(Ui::PCLViewer * ui)
{
    this->ui = ui;
}

boost::shared_ptr<PCLViewer>
AllignPointCloudDialog::getGuiPtr()
{
    return gui_ptr.lock();
}

void
AllignPointCloudDialog::saveFile(QString name, PointCloudI::Ptr cloud)
{
    if (cloud != 0 ) {
        QString dir ("../data/");
        dir.append(name);
        QString saveName = QFileDialog::getSaveFileName(&*getGuiPtr(), tr("Save File"),dir, tr ( "Point Cloud File(*pcd)" ));
        pcl::io::savePCDFileASCII<PointI> ( saveName.toStdString (), * cloud );
    }
}

void
AllignPointCloudDialog::resetVisualization()
{
    getGuiPtr()->viewer->removeShape("tree_text1");
    getGuiPtr()->viewer->removeShape("tree_text2");
    getGuiPtr()->getControl()->setTreeID(name_source.toStdString());
    *getGuiPtr()->getControl()->getCloudPtr() = *cloud_target;
    getGuiPtr()->setCloudPtr(getGuiPtr()->getControl()->getCloudPtr());
}

void
AllignPointCloudDialog::abort()
{
    //cloud_source.reset(new PointCloudI);
    cloud_source.reset(new PointCloudI);
    cloud_final.reset(new PointCloudI);
    resetVisualization();
    allign_dialog->close();

}

void
AllignPointCloudDialog::save()
{
    saveFile(name_source, cloud_target);
    saveFile(name_target, cloud_final);

   // cloud_source.reset(new PointCloudI);
    cloud_source.reset(new PointCloudI);
    cloud_final.reset(new PointCloudI);

    resetVisualization();
    allign_dialog->close();
}

void
AllignPointCloudDialog::init()
{
    import(cloud_target,name_target);
    Eigen::Vector4f base_source;
    stemBase(cloud_target,base_source);
    cloud_target = transformToOrigin(cloud_target, base_source);


    import(cloud_source,name_source);
    Eigen::Vector4f base_target;
    stemBase(cloud_source,base_target);
    cloud_source = transformToOrigin(cloud_source, base_target);

    cloud_final.reset(new PointCloudI);
    *cloud_final = *cloud_source;

    getGuiPtr()->getControl()->setCloudPtr(cloud_target);
    getGuiPtr()->computeBoundingBox();
    visualizeClouds();

    allign_dialog.reset( new QDialog ( &*getGuiPtr(), 0 ));
    allign_dialog_ptr.reset ( new Ui_dialog_init_allign );
    allign_dialog_ptr->setupUi ( &*allign_dialog );

    connect ( allign_dialog_ptr->rotate, SIGNAL(sliderReleased()), this, SLOT (rotate_slider()));
    connect ( allign_dialog_ptr->rotate_2, SIGNAL(valueChanged(double)), this, SLOT (rotate_spinbox()));
    connect ( allign_dialog_ptr->x_2, SIGNAL(valueChanged(double)), this, SLOT (x_spinbox()));
    connect ( allign_dialog_ptr->x, SIGNAL(sliderReleased()), this, SLOT (x_slider()));
    connect ( allign_dialog_ptr->y_2, SIGNAL(valueChanged(double)), this, SLOT (y_spinbox()));
    connect ( allign_dialog_ptr->y, SIGNAL(sliderReleased()), this, SLOT (y_slider()));
    connect ( allign_dialog_ptr->z_2, SIGNAL(valueChanged(double)), this, SLOT (z_spinbox()));
    connect ( allign_dialog_ptr->z, SIGNAL(sliderReleased()), this, SLOT (z_slider()));
    connect ( allign_dialog_ptr->pushButton, SIGNAL(clicked()), this, SLOT (ICP()));
    connect ( allign_dialog_ptr->save_button, SIGNAL(clicked()), this, SLOT (save()));
    connect ( allign_dialog_ptr->abort_button, SIGNAL(clicked()), this, SLOT (abort()));
    allign_dialog->setModal(false);
    allign_dialog->show();
}

void
AllignPointCloudDialog::setGuiPtr(boost::shared_ptr<PCLViewer> guiPtr)
{
    this->gui_ptr = guiPtr;
}

void
AllignPointCloudDialog::stemBase(boost::shared_ptr<PointCloudI> cloud,Eigen::Vector4f & base)
{
    PointI p1;
    PointI p2;
    pcl::getMinMax3D<PointI>(*cloud,p1,p2);
    float minHeight = p1.z;
    boost::shared_ptr<PointCloudI> cloud_base (new PointCloudI);
    extractBaseCloud(cloud, cloud_base, minHeight);
    pcl::compute3DCentroid(*cloud_base, base);
}

void
AllignPointCloudDialog::import(boost::shared_ptr<PointCloudI> & cloud, QString & name)
{
    cloud.reset(new PointCloudI);
    QString path;
    selectFile(name, path);
    ImportPCD import ( path.toStdString(), getGuiPtr()->getControl() );
    cloud = import.getCloud();
    getGuiPtr()->writeConsole(name);
    getGuiPtr()->writeConsole("\n");
}

void
AllignPointCloudDialog::selectFile (QString & name, QString & path) {
    QFileDialog dialog ( &*getGuiPtr(), tr ( "Open File" ), "../data/", tr ( "Point Cloud File(*pcd);;ASCII - File(*.asc);;txt - File(*.txt);;All Files(*)" ) );
    dialog.setOptions ( ( QFileDialog::DontUseNativeDialog ) );
    dialog.setViewMode ( QFileDialog::Detail );
    QStringList files;
    if ( dialog.exec () )
    {
        files = dialog.selectedFiles ();
    }
    if ( files.size () > 0 ) {
        path = files.at ( 0 );
        int index = path.lastIndexOf (QDir::separator() );
        int size = path.size ();
        int position = size - index - 1;
        name = path.right ( position );
    }
}

void
AllignPointCloudDialog::extractBaseCloud(boost::shared_ptr<PointCloudI> cloud, boost::shared_ptr<PointCloudI> base, float height)
{
    pcl::PassThrough<PointI> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (height, height+slice_height);
    pass.filter (*base);
}

boost::shared_ptr<PointCloudI>
AllignPointCloudDialog::transformToOrigin(boost::shared_ptr<PointCloudI> cloud,Eigen::Vector4f base)
{
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << -base(0,0),-base(1,0),-base(2,0)+0.2f;
    boost::shared_ptr<PointCloudI>transformed_cloud (new PointCloudI ());
    pcl::transformPointCloud (*cloud, *transformed_cloud, transform);
    return transformed_cloud;
}


void
AllignPointCloudDialog::visualizeClouds(bool show_final)
{
    getGuiPtr()->viewer->removeAllPointClouds ();
    getGuiPtr()->viewer->removeAllShapes ();

    if(show_final)
    {
        boost::shared_ptr<PointCloudD> visu_source (new PointCloudD);
        visu_source = getGuiPtr()->convertPointCloud(cloud_target,51,102,51);
        pcl::visualization::PointCloudColorHandlerRGBAField<PointD> rgba1 ( visu_source );
        getGuiPtr()->viewer->addPointCloud<PointD> ( visu_source, rgba1, "cloud1" );
        getGuiPtr()->viewer->setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud1" );
        getGuiPtr()->viewer->addText(name_source.toStdString(),10,50,20,0.2,0.4,0.2,"tree_text1");

        boost::shared_ptr<PointCloudD> visu_target (new PointCloudD);
        visu_target = getGuiPtr()->convertPointCloud(cloud_source,128,0,0);
        pcl::visualization::PointCloudColorHandlerRGBAField<PointD> rgba2 ( visu_target );
        getGuiPtr()->viewer->addPointCloud<PointD> ( visu_target, rgba2, "cloud2" );
        getGuiPtr()->viewer->setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud2" );

        boost::shared_ptr<PointCloudD> visu_final (new PointCloudD);
        visu_final = getGuiPtr()->convertPointCloud(cloud_final,255,204,54);
        pcl::visualization::PointCloudColorHandlerRGBAField<PointD> rgba3 ( visu_final );
        getGuiPtr()->viewer->addPointCloud<PointD> ( visu_final, rgba3, "cloud3" );
        getGuiPtr()->viewer->setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud3" );
            getGuiPtr()->viewer->addText( name_target.toStdString(), 10, 20,20, 1, 0.8, 0.2, "tree_text2" );
    } else {
        boost::shared_ptr<PointCloudD> visu_source (new PointCloudD);
        visu_source = getGuiPtr()->convertPointCloud(cloud_target,51,102,51);
        pcl::visualization::PointCloudColorHandlerRGBAField<PointD> rgba1 ( visu_source );
        getGuiPtr()->viewer->addPointCloud<PointD> ( visu_source, rgba1, "cloud1" );
        getGuiPtr()->viewer->setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud1" );
        getGuiPtr()->viewer->addText(name_source.toStdString(),10,50,20,0.2,0.4,0.2,"tree_text1");

        boost::shared_ptr<PointCloudD> visu_target (new PointCloudD);
        visu_target = getGuiPtr()->convertPointCloud(cloud_source,128,0,0);
        pcl::visualization::PointCloudColorHandlerRGBAField<PointD> rgba2 ( visu_target );
        getGuiPtr()->viewer->addPointCloud<PointD> ( visu_target, rgba2, "cloud2" );
        getGuiPtr()->viewer->setPointCloudRenderingProperties ( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud2" );
        getGuiPtr()->viewer->addText( name_target.toStdString(), 10, 20,20, 0.5, 0, 0, "tree_text2" );


        getGuiPtr()->xNegView ();
    }
    getGuiPtr()->ui->qvtkWidget->update ();
}



boost::shared_ptr<PointCloudI>
AllignPointCloudDialog::extractStemBase(boost::shared_ptr<PointCloudI> tree, float height_min)
{
    boost::shared_ptr<PointCloudI> tree_base (new PointCloudI);
    pcl::PassThrough<PointI> pass;
    pass.setInputCloud (tree);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (height_min, height_min+0.05);
    pass.filter (*tree_base);
    return tree_base;
}

PointCloudI::Ptr
AllignPointCloudDialog::downsampleCloud(PointCloudI::Ptr cloud)
{
    boost::shared_ptr<PointCloudI> downsampled (new PointCloudI);
    pcl::VoxelGrid<PointI> sor;
    sor.setInputCloud ( cloud );
    sor.setLeafSize ( 0.02, 0.02, 0.02 );
    sor.filter ( *downsampled );
    return downsampled;
}

void
AllignPointCloudDialog::ICP()
{
    *cloud_source = *cloud_final;

    boost::shared_ptr<PointCloudI> downsampled_target (new PointCloudI);
    boost::shared_ptr<PointCloudI> downsampled_source (new PointCloudI);

    downsampled_source = downsampleCloud(cloud_target);
    downsampled_target = downsampleCloud(cloud_source);




    pcl::IterativeClosestPoint<PointI, PointI> icp;
    icp.setInputSource(downsampled_target);
    icp.setInputTarget(downsampled_source);
    icp.setMaximumIterations (150);
    icp.setMaxCorrespondenceDistance(0.06);
    icp.setTransformationEpsilon(1e-8);
    icp.setEuclideanFitnessEpsilon(1e-5);

    icp.align(*cloud_final);


    if (icp.hasConverged ())
    {

        QString str("\nICP has converged, score is ");
        str.append(QString::number(icp.getFitnessScore ())).append("\n");
        Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity ();
        transformation_matrix *= icp.getFinalTransformation();
        std::stringstream stream;
        stream << transformation_matrix;
        str.append(QString::fromStdString(stream.str()));
        getGuiPtr()->writeConsole(QString(str));
        pcl::transformPointCloud(*cloud_source,*cloud_final, transformation_matrix);
    }
    else
    {
        PCL_ERROR ("\nICP has not converged.\n");
    }

    visualizeClouds(true);
}



void
AllignPointCloudDialog::rotate_spinbox()
{
    int angle = allign_dialog_ptr->rotate_2->value();
    allign_dialog_ptr->rotate->setValue(angle*10);
    rotate_translate();
}

void
AllignPointCloudDialog::rotate_slider()
{
    float angle = allign_dialog_ptr->rotate->value();
    allign_dialog_ptr->rotate_2->setValue(angle/10.0f);
    rotate_translate();
}




void
AllignPointCloudDialog::z_spinbox()
{
    int z = allign_dialog_ptr->z_2->value()*1000;
    allign_dialog_ptr->z->setValue(z);
    rotate_translate();
}

void
AllignPointCloudDialog::z_slider()
{
    float z = allign_dialog_ptr->z->value();
    allign_dialog_ptr->z_2->setValue(z/1000.0f);
    rotate_translate();
}

void
AllignPointCloudDialog::y_spinbox()
{
    int y = allign_dialog_ptr->y_2->value()*1000;
    allign_dialog_ptr->y->setValue(y);
    rotate_translate();
}

void
AllignPointCloudDialog::y_slider()
{
    int y = allign_dialog_ptr->y->value();
    allign_dialog_ptr->y_2->setValue(y/1000.0f);
    rotate_translate();
}

void
AllignPointCloudDialog::x_spinbox()
{
    int x = allign_dialog_ptr->x_2->value()*1000;
    allign_dialog_ptr->x->setValue(x);
    rotate_translate();
}

void
AllignPointCloudDialog::x_slider()
{
    float x = allign_dialog_ptr->x->value();
    allign_dialog_ptr->x_2->setValue(x/1000.0f);
    rotate_translate();
}



void
AllignPointCloudDialog::rotate_translate()
{
    int angle = allign_dialog_ptr->rotate->value();
    int x = allign_dialog_ptr->x->value();
    int y = allign_dialog_ptr->y->value();
    int z = allign_dialog_ptr->z->value();

    cloud_final.reset(new PointCloudI);
    cloud_final = transform(cloud_source,angle,x,y,z);
    visualizeClouds(true);

    getGuiPtr()->ui->qvtkWidget->update ();
}

boost::shared_ptr<PointCloudD>
AllignPointCloudDialog::transform(boost::shared_ptr<PointCloudD> & tree, int angle, int x, int y,  int z)
{
    float theta = 2*M_PI/3600.0f*angle;
    float dx = x/100.0f;
    float dy = y/100.0f;
    float dz = z/100.0f;

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << dx, dy,dz;
    transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
    boost::shared_ptr<PointCloudD> transformed_cloud (new PointCloudD ());
    pcl::transformPointCloud (*tree, *transformed_cloud, transform);
    return transformed_cloud;

}


boost::shared_ptr<PointCloudI>
AllignPointCloudDialog::transform(boost::shared_ptr<PointCloudI> & tree, int angle, int x, int y, int z)
{
    float theta = 2*M_PI/3600.0f*angle;
    float dx = x/100.0f;
    float dy = y/100.0f;
    float dz = z/100.0f;

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << dx, dy,dz;
    transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
    boost::shared_ptr<PointCloudI> transformed_cloud (new PointCloudI ());
    pcl::transformPointCloud (*tree, *transformed_cloud, transform);
    return transformed_cloud;

}

