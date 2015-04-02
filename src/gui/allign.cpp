#include "allign.h"

AllignPointCloud::AllignPointCloud()
{
}

void
AllignPointCloud::setViewer(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
    this->viewer = viewer;
}

void
AllignPointCloud::setUi(Ui::PCLViewer * ui)
{
    this->ui = ui;
}

boost::shared_ptr<PCLViewer>
AllignPointCloud::getGuiPtr()
{
    return gui_ptr.lock();
}


void
AllignPointCloud::init()
{
    import(cloud_source,name_source);
    Eigen::Vector4f base_source;
    stemBase(cloud_source,base_source);
    cloud_source = transformToOrigin(cloud_source, base_source);
    //boost::shared_ptr<PointCloudD> visu_source (new PointCloudD);

    import(cloud_target,name_target);
    Eigen::Vector4f base_target;
    stemBase(cloud_target,base_target);
    cloud_target = transformToOrigin(cloud_target, base_target);
    //boost::shared_ptr<PointCloudD> visu_target (new PointCloudD);

    getGuiPtr()->getControl()->setCloudPtr(cloud_source);
    getGuiPtr()->computeBoundingBox();
    visualizeClouds();
    std::cout << " 111 " << std::endl;

    QDialog * allign_dialog = new QDialog ( &*getGuiPtr(), 0 );
std::cout << " 2111 " << std::endl;
    allign_dialog_ptr.reset ( new Ui_dialog_init_allign );
    std::cout << " 3111 " << std::endl;
    allign_dialog_ptr->setupUi ( allign_dialog );
    std::cout << " 4111 " << std::endl;
//    connect ( allign_dialog_ptr->init_button, SIGNAL (clicked()), this, SLOT (intialAllign()));
//    connect ( allign_dialog_ptr->rotate, SIGNAL(sliderReleased()), this, SLOT (set_allign_rotate()));
//    connect ( allign_dialog_ptr->rotate_2, SIGNAL(valueChanged()), this, SLOT (set_allign_rotate_box()));
//    connect ( allign_dialog_ptr->z, SIGNAL(sliderReleased()), this, SLOT (set_allign_z()));
//    connect ( allign_dialog_ptr->z_2, SIGNAL(valueChanged()), this, SLOT (set_allign_z_box()));
//    connect ( allign_dialog_ptr->y, SIGNAL(sliderReleased()), this, SLOT (set_allign_y()));
//    connect ( allign_dialog_ptr->y_2, SIGNAL(valueChanged()), this, SLOT (set_allign_y_box()));
//    connect ( allign_dialog_ptr->x, SIGNAL(sliderReleased()), this, SLOT (set_allign_x()));
//    connect ( allign_dialog_ptr->x_2, SIGNAL(valueChanged()), this, SLOT (set_allign_x_box()));
//    connect ( allign_dialog_ptr->pushButton, SIGNAL(clicked()), this, SLOT (performICP()));
    allign_dialog->setModal(false);
    allign_dialog->show();
}

void
AllignPointCloud::setGuiPtr(boost::shared_ptr<PCLViewer> guiPtr)
{
    this->gui_ptr = guiPtr;
}

void
AllignPointCloud::stemBase(boost::shared_ptr<PointCloudI> cloud,Eigen::Vector4f & base)
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
AllignPointCloud::import(boost::shared_ptr<PointCloudI> & cloud, QString & name)
{
    cloud.reset(new PointCloudI);
    QString path;
    selectFile(name, path);
    ImportPCD import ( path.toStdString(), getGuiPtr()->getControl() );
    cloud = import.getCloud();
    getGuiPtr()->writeConsole(name);

}

void
AllignPointCloud::selectFile (QString & name, QString & path) {
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
AllignPointCloud::extractBaseCloud(boost::shared_ptr<PointCloudI> cloud, boost::shared_ptr<PointCloudI> base, float height)
{
      pcl::PassThrough<PointI> pass;
      pass.setInputCloud (cloud);
      pass.setFilterFieldName ("z");
      pass.setFilterLimits (height, height+slice_height);
      pass.filter (*base);

}

boost::shared_ptr<PointCloudI>
AllignPointCloud::transformToOrigin(boost::shared_ptr<PointCloudI> cloud,Eigen::Vector4f base)
{
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << -base(0,0),-base(1,0),-base(2,0)+0.2f;
    boost::shared_ptr<PointCloudI>transformed_cloud (new PointCloudI ());
    pcl::transformPointCloud (*cloud, *transformed_cloud, transform);
    return transformed_cloud;
}

void
AllignPointCloud::dialog()
{
//    cloud_source.reset(new PointCloudI);
//    cloud_target.reset(new PointCloudI);
//    cloud_final.reset(new PointCloudI);

//    std::string file_target = "/home/hackenberg/simpleTree/data/2-16-(-2)_2013-2014.pcd";
//    std::string file_source = "/home/hackenberg/simpleTree/data/2-16-(-2)_2014-2015.pcd";

//    ImportPCD import_source ( file_source, control );
//    ImportPCD import_target ( file_target, control );
//    cloud_source = import_source.getCloud();
//    cloud_target = import_target.getCloud();

//    float min_height_source = 10000;
//    float min_height_target = 10000;
//    for ( size_t i = 0; i < cloud_target->points.size (); i++ ) {
//        if(cloud_target->points.at(i).z<min_height_target)
//        {
//            min_height_target = cloud_target->points.at(i).z;
//        }
//    }

//    for ( size_t i = 0; i < cloud_source->points.size (); i++ ) {
//        if(cloud_source->points.at(i).z<min_height_source)
//        {
//            min_height_source = cloud_source->points.at(i).z;
//        }
//    }
//    boost::shared_ptr<PointCloudI> base_source = extractStemBase(cloud_source,min_height_source);
//    boost::shared_ptr<PointCloudI> base_target = extractStemBase(cloud_target,min_height_target);
//        Eigen::Vector4f root_source;
//        Eigen::Vector4f root_target;
//        pcl::compute3DCentroid(*base_source, root_source);
//        pcl::compute3DCentroid(*base_target, root_target);
//        cloud_source = transformToOrigin(cloud_source, root_source);
//        cloud_target = transformToOrigin(cloud_target, root_target);
//        std::cout << root_source <<"\n ";
//        std::cout << root_target <<"\n ";






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


//    getGuiPtr()->viewer->removeAllPointClouds ();
//    getGuiPtr()->viewer->removeAllShapes ();
//    pcl::visualization::PointCloudColorHandlerRGBAField<PointD> rgba ( visu_target );
//    getGuiPtr()->viewer->addPointCloud<PointD> ( visu_target, rgba, "cloud1" );
//    pcl::visualization::PointCloudColorHandlerRGBAField<PointD> rgba2 ( visu_source );
//    getGuiPtr()->viewer->addPointCloud<PointD> ( visu_source, rgba2, "cloud2" );
//    getGuiPtr()->xNegView ();
//    getGuiPtr()->viewer->addText (getGuiPtr()-> getControl ()->getTreeID (), 10, 20,20, 1, 0, 0, "tree_text" );
//    getGuiPtr()->ui->qvtkWidget->update ();


//    std::cout << "visualization (PCLViewer::getointCloud) cloud size " << this->cloud->points.size() << "\n";
//    computeBoundingBox ();

//    QDialog * allign_dialog = new QDialog (&*getGuiPtr(), 0 );
//    allign_dialog_ptr.reset ( new Ui_dialog_init_allign );
//    // Ui_dialog_init_allign allign;
//    allign_dialog_ptr->setupUi ( allign_dialog );

//  //  connect ( allign_dialog_ptr->spinBox, SIGNAL ( valueChanged ( int ) ), this, SLOT ( switch_point_for_ICP ( int ) ) );
//    connect ( allign_dialog_ptr->init_button, SIGNAL (clicked()), this, SLOT (intialAllign()));
//    connect ( allign_dialog_ptr->rotate, SIGNAL(sliderReleased()), this, SLOT (set_allign_rotate()));
//    connect ( allign_dialog_ptr->rotate_2, SIGNAL(valueChanged()), this, SLOT (set_allign_rotate_box()));
//    //connect ( allign_dialog_ptr->rotate2, SIGNAL(sliderReleased()), this, SLOT (rotateAllign()));
//    connect ( allign_dialog_ptr->z, SIGNAL(sliderReleased()), this, SLOT (set_allign_z()));
//    connect ( allign_dialog_ptr->z_2, SIGNAL(valueChanged()), this, SLOT (set_allign_z_box()));
//    connect ( allign_dialog_ptr->y, SIGNAL(sliderReleased()), this, SLOT (set_allign_y()));
//    connect ( allign_dialog_ptr->y_2, SIGNAL(valueChanged()), this, SLOT (set_allign_y_box()));
//    connect ( allign_dialog_ptr->x, SIGNAL(sliderReleased()), this, SLOT (set_allign_x()));
//    connect ( allign_dialog_ptr->x_2, SIGNAL(valueChanged()), this, SLOT (set_allign_x_box()));
//    connect ( allign_dialog_ptr->pushButton, SIGNAL(clicked()), this, SLOT (performICP()));
//    allign_dialog->setModal(false);
//    allign_dialog->show();

}

void
AllignPointCloud::visualizeClouds()
{
    ui->qvtkWidget->update ();



    getGuiPtr()->viewer->removeAllPointClouds ();
    getGuiPtr()->viewer->removeAllShapes ();

    boost::shared_ptr<PointCloudD> visu_source (new PointCloudD);
    visu_source = getGuiPtr()->convertPointCloud(cloud_source,128,128,0);
    pcl::visualization::PointCloudColorHandlerRGBAField<PointD> rgba1 ( visu_source );
    getGuiPtr()->viewer->addPointCloud<PointD> ( visu_source, rgba1, "cloud1" );
    getGuiPtr()->viewer->addText(name_source.toStdString(),10,20,128,128,0,"tree_text1");

    boost::shared_ptr<PointCloudD> visu_target (new PointCloudD);
    visu_target = getGuiPtr()->convertPointCloud(cloud_target,128,0,0);
    pcl::visualization::PointCloudColorHandlerRGBAField<PointD> rgba2 ( visu_target );
    getGuiPtr()->viewer->addPointCloud<PointD> ( visu_target, rgba2, "cloud2" );
    getGuiPtr()->viewer->addText ( getGuiPtr()->getControl ()->getTreeID (), 10, 20,20, 128, 0, 0, "tree_text2" );
    getGuiPtr()->xNegView ();
    getGuiPtr()->ui->qvtkWidget->update ();
}



boost::shared_ptr<PointCloudI>
AllignPointCloud::extractStemBase(boost::shared_ptr<PointCloudI> tree, float height_min)
{
    boost::shared_ptr<PointCloudI> tree_base (new PointCloudI);
    pcl::PassThrough<PointI> pass;
      pass.setInputCloud (tree);
      pass.setFilterFieldName ("z");
      pass.setFilterLimits (height_min, height_min+0.05);
      //pass.setFilterLimitsNegative (true);
      pass.filter (*tree_base);
      return tree_base;
}


void
AllignPointCloud::performICP()
{
    int angle = allign_dialog_ptr->rotate->value();
    int z_transform = allign_dialog_ptr->z->value();
    cloud_target = transform(cloud_target,angle,z_transform);


    pcl::IterativeClosestPoint<PointI, PointI> icp;
      icp.setInputSource(cloud_source);
      icp.setInputTarget(cloud_target);
      icp.setMaximumIterations (20);

      boost::shared_ptr<PointCloudI> Final (new PointCloudI);

      icp.align(*Final);
      cloud_target = Final;


      if (icp.hasConverged ())
        {
          std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
        }
        else
        {
          PCL_ERROR ("\nICP has not converged.\n");
        }

    boost::shared_ptr<PointCloudD> visu_source (new PointCloudD);
    boost::shared_ptr<PointCloudD> visu_target (new PointCloudD);

    visu_source->resize ( cloud_source->points.size () );
    for ( size_t i = 0; i < cloud_source->points.size (); i++ ) {
        visu_source->points[i].x = cloud_source->points[i].x;
        visu_source->points[i].y = cloud_source->points[i].y;
        visu_source->points[i].z = cloud_source->points[i].z;
        float intens = cloud_source->points[i].intensity;
        if ( intens == 0 ) {
            intens = 180;
        }
        visu_source->points[i].r = 255;
        visu_source->points[i].g = 0;
        visu_source->points[i].b = 0;
        visu_source->points[i].a = 255;
    }

    visu_target->resize ( cloud_target->points.size () );
    for ( size_t i = 0; i < cloud_target->points.size (); i++ ) {
        visu_target->points[i].x = cloud_target->points[i].x;
        visu_target->points[i].y = cloud_target->points[i].y;
        visu_target->points[i].z = cloud_target->points[i].z;
        float intens = cloud_target->points[i].intensity;
        if ( intens == 0 ) {
            intens = 180;
        }
        visu_target->points[i].r = 0;
        visu_target->points[i].g = 255;
        visu_target->points[i].b = 0;
        visu_target->points[i].a = 255;
    }



    getGuiPtr()->viewer->removeAllPointClouds ();
    getGuiPtr()->viewer->removeAllShapes ();
    pcl::visualization::PointCloudColorHandlerRGBAField<PointD> rgba ( visu_target );
    getGuiPtr()->viewer->addPointCloud<PointD> ( visu_target, rgba, "cloud1" );
    pcl::visualization::PointCloudColorHandlerRGBAField<PointD> rgba2 ( visu_source );
    getGuiPtr()->viewer->addPointCloud<PointD> ( visu_source, rgba2, "cloud2" );
   // xNegView ();
    getGuiPtr()->viewer->addText (getGuiPtr()-> getControl ()->getTreeID (), 10, 20,20, 1, 0, 0, "tree_text" );
    getGuiPtr()->ui->qvtkWidget->update ();
}


void
AllignPointCloud::set_allign_rotate_box()
{
    int angle = allign_dialog_ptr->rotate_2->value()*10;
    allign_dialog_ptr->rotate->setValue(angle);
    allign_rotate_translate();
}

void
AllignPointCloud::set_allign_rotate()
{
    float angle = allign_dialog_ptr->rotate->value();
    allign_dialog_ptr->rotate_2->setValue(angle/10);
    allign_rotate_translate();
}


void
AllignPointCloud::rotateAllign()
{

}

void
AllignPointCloud::set_allign_z_box()
{
    int z = allign_dialog_ptr->z_2->value()*1000;
    allign_dialog_ptr->z->setValue(z);
    allign_rotate_translate();
}

void
AllignPointCloud::set_allign_z()
{
    float z = allign_dialog_ptr->z->value();
    allign_dialog_ptr->z_2->setValue(z/1000);
    allign_rotate_translate();
}

void
AllignPointCloud::set_allign_y()
{
    float y = allign_dialog_ptr->y->value();
    allign_dialog_ptr->y_2->setValue(y/1000);
    allign_rotate_translate();
}

void
AllignPointCloud::set_allign_y_box()
{
        int y = allign_dialog_ptr->y_2->value()*1000;
        allign_dialog_ptr->y->setValue(y);
        allign_rotate_translate();
}

void
AllignPointCloud::set_allign_x_box()
{
    int x = allign_dialog_ptr->x_2->value()*1000;
    allign_dialog_ptr->x->setValue(x);
    allign_rotate_translate();
}

void
AllignPointCloud::set_allign_x()
{
    float x = allign_dialog_ptr->x->value();
    allign_dialog_ptr->x_2->setValue(x/1000);
    allign_rotate_translate();
}



void
AllignPointCloud::allign_rotate_translate()
{


    boost::shared_ptr<PointCloudD> visu_source (new PointCloudD);
    boost::shared_ptr<PointCloudD> visu_target (new PointCloudD);

    visu_source->resize ( cloud_source->points.size () );
    for ( size_t i = 0; i < cloud_source->points.size (); i++ ) {
        visu_source->points[i].x = cloud_source->points[i].x;
        visu_source->points[i].y = cloud_source->points[i].y;
        visu_source->points[i].z = cloud_source->points[i].z;
        float intens = cloud_source->points[i].intensity;
        if ( intens == 0 ) {
            intens = 180;
        }
        visu_source->points[i].r = 255;
        visu_source->points[i].g = 0;
        visu_source->points[i].b = 0;
        visu_source->points[i].a = 255;
    }

    visu_target->resize ( cloud_target->points.size () );
    for ( size_t i = 0; i < cloud_target->points.size (); i++ ) {
        visu_target->points[i].x = cloud_target->points[i].x;
        visu_target->points[i].y = cloud_target->points[i].y;
        visu_target->points[i].z = cloud_target->points[i].z;
        float intens = cloud_target->points[i].intensity;
        if ( intens == 0 ) {
            intens = 180;
        }
        visu_target->points[i].r = 0;
        visu_target->points[i].g = 255;
        visu_target->points[i].b = 0;
        visu_target->points[i].a = 255;
    }


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
    ui->qvtkWidget->update ();
}

boost::shared_ptr<PointCloudD>
AllignPointCloud::transform(boost::shared_ptr<PointCloudD> tree, int angle, int z_transform)
{
    float theta = M_PI/3600*angle;
    float height = z_transform/100.0f;


    Eigen::Affine3f transform = Eigen::Affine3f::Identity();

      // Define a translation of 2.5 meters on the x axis.
      transform.translation() << 0, 0.0, height;

      // The same rotation matrix as before; tetha radians arround Z axis
      transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));



      // Executing the transformation
      boost::shared_ptr<PointCloudD> transformed_cloud (new PointCloudD ());
      // You can either apply transform_1 or transform_2; they are the same
      pcl::transformPointCloud (*tree, *transformed_cloud, transform);
      return transformed_cloud;

}


boost::shared_ptr<PointCloudI>
AllignPointCloud::transform(boost::shared_ptr<PointCloudI> tree, int angle, int z_transform)
{
    float theta = M_PI/360*angle;
    float height = z_transform/100;


    Eigen::Affine3f transform = Eigen::Affine3f::Identity();

      // Define a translation of 2.5 meters on the x axis.
      transform.translation() << 0, 0.0, height;

      // The same rotation matrix as before; tetha radians arround Z axis
      transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));



      // Executing the transformation
      boost::shared_ptr<PointCloudI> transformed_cloud (new PointCloudI ());
      // You can either apply transform_1 or transform_2; they are the same
      pcl::transformPointCloud (*tree, *transformed_cloud, transform);
      return transformed_cloud;

}

