#include "referenceheightdialog.h"

ReferenceHeightDialog::ReferenceHeightDialog(QWidget *parent) :
    QDialog(parent)
{
}

void
ReferenceHeightDialog::init()
{
    if (getViewer()-> getControl ()->getCloudPtr () != 0 ) {
        Ui_Dialog_Reference refer;
        refer.setupUi(this);
        connect ( refer.box_height, SIGNAL ( valueChanged ( double ) ), this, SLOT ( set_reference_height ( double ) ) );
        connect ( refer.abort, SIGNAL ( clicked() ), this, SLOT ( reject() ) );
        connect ( refer.compute, SIGNAL ( clicked() ), this, SLOT ( compute_reference() ) );
        connect ( refer.compute, SIGNAL ( clicked() ), this, SLOT ( accept() ) );
        this->setModal(true);
        this->show ();

    } else {
        QMessageBox::warning ( this, tr ( "Simple Tree" ), tr ( "No Point Cloud found\n"
                               "Please load a Point Cloud first" ),
                               QMessageBox::Ok );
    }
}

boost::shared_ptr<PCLViewer>
ReferenceHeightDialog::getViewer()
{
    return this->viewer.lock();
}


void
ReferenceHeightDialog::setViewer(boost::shared_ptr<PCLViewer> guiPtr)
{
    this->viewer = guiPtr;
}




void
ReferenceHeightDialog::set_reference_height ( double h ) {
    height = h;
}

void
ReferenceHeightDialog::compute_reference () {
    PointCloudI::Ptr cloud = getViewer()-> getControl ()->getCloudPtr ();
    Eigen::Vector4f min_pt;
    Eigen::Vector4f max_pt;
    pcl::getMinMax3D ( *cloud, min_pt, max_pt );
    float oldMinHeight = ( min_pt[2] );
    for ( size_t i = 0; i < cloud->points.size (); i++ ) {
        cloud->points.at ( i ).z = cloud->points.at ( i ).z + height - oldMinHeight;
    }
    getViewer()-> getControl ()->setCloudPtr ( cloud );
    height = 0.2;

}


