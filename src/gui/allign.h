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
    explicit AllignPointCloud(QObject * parent = 0);
    void
    setViewer(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);
    void
    setUi(Ui::PCLViewer * ui);
    void
    setGuiPtr(boost::shared_ptr<PCLViewer> guiPtr);
    void
    init();

    void
    rotate_translate();

private:
    boost::shared_ptr<QDialog> allign_dialog;

    void
    visualizeClouds(bool show_final = false);

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
    transform(boost::shared_ptr<PointCloudI> & tree, int angle, int x, int y ,int z);
    boost::shared_ptr<PointCloudD>
    transform(boost::shared_ptr<PointCloudD> & tree, int angle, int x, int y, int z);


    PointCloudI::Ptr
    downsampleCloud(PointCloudI::Ptr cloud);

    void
    saveFile(QString name, PointCloudI::Ptr cloud);

    void
    resetVisualization();

public slots:

    void
    abort();
    void
    save();
    void
    rotate_slider();
    void
    rotate_spinbox();
    void
    x_slider();
    void
    x_spinbox();
    void
    y_slider();
    void
    y_spinbox();
    void
    z_slider();
    void
    z_spinbox();
    void
    ICP();
//    void
//    rotateAllign();
    //    void
    //    intialAllign();
};

#endif // ALLIGN_H
