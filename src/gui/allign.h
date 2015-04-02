#ifndef ALLIGN_H
#define ALLIGN_H

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>
#include "pclviewer.h"

#include "../../build/ui_allign_dialog.h"

#include <QDir>


typedef pcl::PointXYZRGBA PointD;
typedef pcl::PointCloud<PointD> PointCloudD;
typedef pcl::PointXYZINormal PointI;
typedef pcl::PointCloud<PointI> PointCloudI;

namespace Ui {
class PCLViewer;
class AllignPointCloud;
}




class AllignPointCloud : public QObject
{
    Q_OBJECT
public:
    AllignPointCloud();
    void
    setViewer(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);
    void
    setUi(Ui::PCLViewer * ui);
    void
    setGuiPtr(boost::shared_ptr<PCLViewer> guiPtr);
    void
    dialog();
    void
    init();

private:
    void
    visualizeClouds();

    const float slice_height = 0.05f;

    QString name_source;
    QString name_target;

    boost::shared_ptr<PointCloudI> cloud_source;
    boost::shared_ptr<PointCloudI> cloud_target;
    boost::shared_ptr<PointCloudI> cloud_final;

    Ui::PCLViewer *ui;

    boost::weak_ptr<PCLViewer>
    gui_ptr;
    boost::shared_ptr<PCLViewer>
    getGuiPtr();

    boost::weak_ptr<pcl::visualization::PCLVisualizer> viewer;

    boost::shared_ptr<Ui_dialog_init_allign>
    allign_dialog_ptr;


    void
    selectFile(QString & name, QString & path);
    void
    import(boost::shared_ptr<PointCloudI> & cloud, QString & name);
    void
    stemBase(boost::shared_ptr<PointCloudI> cloud,Eigen::Vector4f & base);
    boost::shared_ptr<PointCloudI>
    extractStemBase(boost::shared_ptr<PointCloudI> tree, float height_min);
    void
    extractBaseCloud(boost::shared_ptr<PointCloudI> cloud, boost::shared_ptr<PointCloudI> base, float height);
    boost::shared_ptr<PointCloudI>
    transformToOrigin(boost::shared_ptr<PointCloudI> cloud,Eigen::Vector4f base);
    boost::shared_ptr<PointCloudI>
    transform(boost::shared_ptr<PointCloudI> tree, int angle, int z_transform);
    boost::shared_ptr<PointCloudD>
    transform(boost::shared_ptr<PointCloudD> tree, int angle, int z_transform);


public slots:
    void
    allign_rotate_translate();
    void
    set_allign_rotate();
    void
    set_allign_rotate_box();
    void
    set_allign_x();
    void
    set_allign_x_box();
    void
    set_allign_y();
    void
    set_allign_y_box();
    void
    set_allign_z();
    void
    set_allign_z_box();
    void
    performICP();
    void
    rotateAllign();
    //    void
    //    intialAllign();
};

#endif // ALLIGN_H
