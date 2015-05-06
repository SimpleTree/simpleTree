#include "cropboxdialog.h"

CropBoxDialog::CropBoxDialog(QWidget *parent) :
    QDialog(parent)
{
}

void
CropBoxDialog::setViewer(boost::shared_ptr<PCLViewer> guiPtr)
{
    this->viewer = guiPtr;
}

boost::shared_ptr<PCLViewer>
CropBoxDialog::getViewer()
{
    return this->viewer.lock();
}

void
CropBoxDialog::addCube()
{
    getViewer()->viewer->removeShape ( "deleteBox" );
    getViewer()->viewer->addCube ( deleteBox, "deleteBox" );
    getViewer()->viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,
                                          "deleteBox" );
    getViewer()->viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_COLOR, 0.8, 0.2, 0, "deleteBox" );
    getViewer()->viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_SHADING, pcl::visualization::PCL_VISUALIZER_SHADING_GOURAUD, "deleteBox" );
    getViewer()->viewer->setShapeRenderingProperties ( pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, "deleteBox" );
    getViewer()->viewer->removeCoordinateSystem 	 ();
    getViewer()->viewer->addCoordinateSystem ( 1, deleteBox.values.at ( 0 ), deleteBox.values.at ( 1 ), deleteBox.values.at ( 2 ) );
    getViewer()->ui->qvtkWidget->update ();
}

void
CropBoxDialog::set_crop_box_x ( double x ) {
    if ( getViewer()->crop_box_is_active ) {
        deleteBox.values.at ( 0 ) = x;
        addCube();
    }
}
void
CropBoxDialog::set_crop_box_y ( double y ) {
    if ( getViewer()->crop_box_is_active ) {
        deleteBox.values.at ( 1 ) = y;
        addCube();
    }
}
void

CropBoxDialog::set_crop_box_z ( double z ) {
    if (getViewer()-> crop_box_is_active ) {
        deleteBox.values.at ( 2 ) = z;
        addCube();
    }
}
void
CropBoxDialog::set_crop_box_x_length ( double x ) {
    if ( getViewer()->crop_box_is_active ) {
        deleteBox.values.at ( 7 ) = x;
        addCube();
    }
}
void
CropBoxDialog::set_crop_box_y_length ( double y ) {
    if ( getViewer()->crop_box_is_active ) {
        deleteBox.values.at ( 8 ) = y;
        addCube();
    }
}
void
CropBoxDialog::set_crop_box_z_length ( double z ) {
    if ( getViewer()->crop_box_is_active ) {
        deleteBox.values.at ( 9 ) = z;
        addCube();
    }
}

void
CropBoxDialog::compute_crop_box () {
    getViewer()->viewer->removeShape ( "deleteBox" );
    getViewer()->viewer->removeCoordinateSystem 	 ();

    pcl::octree::OctreePointCloudSearch<PointI> octree ( 0.02f );
    octree.setInputCloud ( getViewer()->getControl ()->getCloudPtr () );
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
    extract.setInputCloud ( getViewer()->getControl ()->getCloudPtr () );
    extract.setIndices ( inliers );
    extract.setNegative ( true );
    extract.filter ( *cloud_filtered );
    getViewer()->getControl ()->setCloudPtr ( cloud_filtered );
    getViewer()->viewer->removeShape ( "deleteBox" );
    getViewer()->viewer->removeCoordinateSystem 	 ();
    getViewer()->crop_box_is_active = false;
}

void
CropBoxDialog::abort_crop_box () {
    getViewer()->viewer->removeShape ( "deleteBox" );
    getViewer()->viewer->removeCoordinateSystem 	 ();
    getViewer()->crop_box_is_active = false;
}

void
CropBoxDialog::init () {
    if ( getViewer()->getControl ()->getCloudPtr () != 0 ) {
        deleteBox.values.clear ();
        deleteBox.values.push_back ( centerX );
        deleteBox.values.push_back ( centerY );
        deleteBox.values.push_back ( centerZ );
        deleteBox.values.push_back ( 0 );
        deleteBox.values.push_back ( 0 );
        deleteBox.values.push_back ( 0 );
        deleteBox.values.push_back ( 0 );
        deleteBox.values.push_back ( 0.35f );
        deleteBox.values.push_back ( 0.35f );
        deleteBox.values.push_back ( 0.35f );
        addCube();

        dialog.reset(new Ui_crop_box_dialog);
        dialog->setupUi ( this );

        connect ( dialog->box_x, SIGNAL ( valueChanged ( double ) ), this, SLOT ( set_crop_box_x ( double ) ) );
        connect ( dialog->box_y, SIGNAL ( valueChanged ( double ) ), this, SLOT ( set_crop_box_y ( double ) ) );
        connect ( dialog->box_z, SIGNAL ( valueChanged ( double ) ), this, SLOT ( set_crop_box_z ( double ) ) );
        connect ( dialog->box_x_size, SIGNAL ( valueChanged ( double ) ), this, SLOT ( set_crop_box_x_length ( double ) ) );
        connect ( dialog->box_y_size, SIGNAL ( valueChanged ( double ) ), this, SLOT ( set_crop_box_y_length ( double ) ) );
        connect ( dialog->box_z_size, SIGNAL ( valueChanged ( double ) ), this, SLOT ( set_crop_box_z_length ( double ) ) );
        connect ( dialog->abort, SIGNAL ( clicked() ), this, SLOT ( abort_crop_box() ) );
        connect ( dialog->abort, SIGNAL ( clicked() ), this, SLOT ( reject() ) );
        connect ( dialog->compute, SIGNAL ( clicked() ), this, SLOT ( compute_crop_box() ) );
        connect ( dialog->compute, SIGNAL ( clicked() ), this, SLOT ( accept() ) );
        getViewer()->crop_box_is_active = true;
        this->setModal(false);
        this->setWindowFlags(Qt::WindowStaysOnTopHint);
        this->show ();


    } else {
        QMessageBox::warning ( this, tr ( "Simple Tree" ), tr ( "No Point Cloud found\n"
                               "Please load a Point Cloud first" ),
                               QMessageBox::Ok );
    }

}

