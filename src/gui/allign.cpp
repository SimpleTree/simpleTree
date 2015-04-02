#include "allign.h"

AllignPointCloud::AllignPointCloud()
{
}



void
AllignPointCloud::intialAllign()
{
     gui_ptr->ui->qvtkWidget->update ();

       viewer->addText ( getControl ()->getTreeID (), 10, 20,20, 1, 0, 0, "tree_text" );
       ui->qvtkWidget->update ();
}




boost::shared_ptr<PointCloudI>
AllignPointCloud::transformToOrigin(boost::shared_ptr<PointCloudI> tree, Eigen::Vector4f center)
{
    float minHeight;
    boost::shared_ptr<PointCloudI> stemBase = extractStemBase(tree, minHeight);
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();

    // Define a translation of 2.5 meters on the x axis.
    transform.translation() << -center(0,0),-center(1,0),-center(2,0)+0.2f;



    // Executing the transformation
   boost::shared_ptr<PointCloudI>transformed_cloud (new PointCloudI ());
    // You can either apply transform_1 or transform_2; they are the same
    pcl::transformPointCloud (*tree, *transformed_cloud, transform);
    return transformed_cloud;
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
      icp.setInputCloud(cloud_source);
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



    viewer->removeAllPointClouds ();
    viewer->removeAllShapes ();
    pcl::visualization::PointCloudColorHandlerRGBAField<PointD> rgba ( visu_target );
    viewer->addPointCloud<PointD> ( visu_target, rgba, "cloud1" );
    pcl::visualization::PointCloudColorHandlerRGBAField<PointD> rgba2 ( visu_source );
    viewer->addPointCloud<PointD> ( visu_source, rgba2, "cloud2" );
   // xNegView ();
    viewer->addText ( getControl ()->getTreeID (), 10, 20,20, 1, 0, 0, "tree_text" );
    ui->qvtkWidget->update ();
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
AllignPointCloud::switch_point_for_ICP(int i)
{

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


    int angle = allign_dialog_ptr->rotate->value();
    int z_transform = allign_dialog_ptr->z->value();
    visu_target = transform(visu_target,angle,z_transform);


    viewer->removeAllPointClouds ();
    viewer->removeAllShapes ();
    pcl::visualization::PointCloudColorHandlerRGBAField<PointD> rgba ( visu_target );
    viewer->addPointCloud<PointD> ( visu_target, rgba, "cloud1" );
    pcl::visualization::PointCloudColorHandlerRGBAField<PointD> rgba2 ( visu_source );
    viewer->addPointCloud<PointD> ( visu_source, rgba2, "cloud2" );
   // xNegView ();
    viewer->addText ( getControl ()->getTreeID (), 10, 20,20, 1, 0, 0, "tree_text" );
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

