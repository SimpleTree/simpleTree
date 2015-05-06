#ifndef CROPSPHEREDIALOG_H
#define CROPSPHEREDIALOG_H

#include <QDialog>

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>
#include "../pclviewer.h"

#include "../../../build/ui_crop_sphere_dialog.h"
#include "../../pointclouds/allignpointcloud.h"

#include "../../../src/gui/guisubclass.h"

#include <QDir>
#include <QDialog>
class CropSphereDialog : public QDialog, public GuiSubClass
{
    Q_OBJECT
public:
    explicit CropSphereDialog(QWidget *parent = 0);

    void
    setViewer(boost::shared_ptr<PCLViewer> guiPtr);

    boost::shared_ptr<PCLViewer>
    getViewer();

    void
    init();

    boost::shared_ptr<Ui_crop_sphere_dialog> dialog;
private:

    boost::weak_ptr<PCLViewer> viewer;
    float centerX = 0;
    float centerY = 0;
    float centerZ = 0;
    float r = 0.5f;

    pcl::ModelCoefficients deleteSphere;

    void
    addSphere();






signals:




public slots:
    void
    set_crop_sphere_x ( double x );

    void
    set_crop_sphere_y ( double y ) ;

    void
    set_crop_sphere_z ( double z ) ;

    void
    set_crop_sphere_r ( double r ) ;

    void
    compute_crop_sphere () ;

    void
    abort_crop_sphere () ;

//    void
//    cropsphere () ;

};

#endif // CROPSPHEREDIALOG_H
