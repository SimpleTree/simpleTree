#include "cropspheredialog.h"

CropSphereDialog::CropSphereDialog(QWidget *parent) :
    QDialog(parent)
{
}

void
CropSphereDialog::setViewer(boost::shared_ptr<PCLViewer> guiPtr)
{
    this->viewer = guiPtr;
}

boost::shared_ptr<PCLViewer>
CropSphereDialog::getViewer()
{
    return this->viewer.lock();
}

void
CropSphereDialog::addSphere()
{
    getViewer()->viewer->removeShape ( "deleteSphere" );
    getViewer()->viewer->addSphere ( deleteSphere, "deleteSphere" );
    getViewer()->viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,
                                          "deleteSphere" );
    getViewer()->viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_COLOR, 0.8, 0.2, 0, "deleteSphere" );
    getViewer()->viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_SHADING, pcl::visualization::PCL_VISUALIZER_SHADING_GOURAUD, "deleteSphere" );
    getViewer()->viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, "deleteSphere" );
    getViewer()->viewer->removeCoordinateSystem 	 ();
    getViewer()->viewer->addCoordinateSystem ( 1, deleteSphere.values.at ( 0 ), deleteSphere.values.at ( 1 ), deleteSphere.values.at ( 2 ) );
    getViewer()->ui->qvtkWidget->update ();
}


void
CropSphereDialog::set_crop_sphere_x ( double x ) {
    if (getViewer()-> crop_sphere_is_active ) {
        deleteSphere.values.at ( 0 ) = x;
        addSphere();
    }
}
void
CropSphereDialog::set_crop_sphere_y ( double y ) {
    if ( getViewer()->crop_sphere_is_active ) {
        deleteSphere.values.at ( 1 ) = y;
        addSphere();
    }
}
void

CropSphereDialog::set_crop_sphere_z ( double z ) {
    if (getViewer()-> crop_sphere_is_active ) {
        deleteSphere.values.at ( 2 ) = z;
        addSphere();
    }
}
void
CropSphereDialog::set_crop_sphere_r ( double r ) {
    if ( getViewer()->crop_sphere_is_active ) {
        deleteSphere.values.at ( 3 ) = r;
        addSphere();
    }
}

void
CropSphereDialog::compute_crop_sphere () {
    getViewer()->viewer->removeShape ( "deleteSphere" );
    getViewer()->viewer->removeCoordinateSystem 	 ();

    pcl::octree::OctreePointCloudSearch<PointI> octree ( 0.02f );
    octree.setInputCloud (getViewer()-> getControl ()->getCloudPtr () );
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
    extract.setInputCloud (getViewer()-> getControl ()->getCloudPtr () );
    extract.setIndices ( inliers );
    extract.setNegative ( true );
    extract.filter ( *cloud_filtered );
    getViewer()->getControl ()->setCloudPtr ( cloud_filtered );
    getViewer()->viewer->removeShape ( "deleteSphere" );
    getViewer()->viewer->removeCoordinateSystem 	 ();
    getViewer()->crop_sphere_is_active = false;

}

void
CropSphereDialog::abort_crop_sphere () {
    getViewer()->viewer->removeShape ( "deleteSphere" );
    getViewer()->viewer->removeCoordinateSystem 	 ();
    getViewer()->crop_sphere_is_active = false;
}

void
CropSphereDialog::init () {
    if ( getViewer()->getControl ()->getCloudPtr () != 0 ) {
        deleteSphere.values.clear ();
        deleteSphere.values.push_back ( centerX );
        deleteSphere.values.push_back ( centerY );
        deleteSphere.values.push_back ( centerZ );
        deleteSphere.values.push_back ( r );
        addSphere();

        dialog.reset(new Ui_crop_sphere_dialog);
        dialog->setupUi ( this );

        connect ( dialog->box_x, SIGNAL ( valueChanged ( double ) ), this, SLOT ( set_crop_sphere_x ( double ) ) );
        connect ( dialog->box_y, SIGNAL ( valueChanged ( double ) ), this, SLOT ( set_crop_sphere_y ( double ) ) );
        connect ( dialog->box_z, SIGNAL ( valueChanged ( double ) ), this, SLOT ( set_crop_sphere_z ( double ) ) );
        connect ( dialog->box_r, SIGNAL ( valueChanged ( double ) ), this, SLOT ( set_crop_sphere_r ( double ) ) );
        connect ( dialog->abort, SIGNAL ( clicked() ), this, SLOT ( abort_crop_sphere() ) );
        connect ( dialog->abort, SIGNAL ( clicked() ), this, SLOT ( reject() ) );
        connect ( dialog->compute, SIGNAL ( clicked() ), this, SLOT ( compute_crop_sphere() ) );
        connect ( dialog->compute, SIGNAL ( clicked() ), this, SLOT ( accept() ) );
        getViewer()->crop_sphere_is_active = true;
        this->setModal(false);
        this->setWindowFlags(Qt::WindowStaysOnTopHint);
        this->show ();

    } else {
        QMessageBox::warning ( this, tr ( "Simple Tree" ), tr ( "No Point Cloud found\n"
                               "Please load a Point Cloud first" ),
                               QMessageBox::Ok );
    }

}

