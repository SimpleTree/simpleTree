#ifndef CURVATUREDIALOG_H
#define CURVATUREDIALOG_H

#include <QDialog>
#include <QMessageBox>

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/centroid.h>

#include <Eigen/Eigenvalues>

#include "../../../build/ui_curvature_dialog.h"
#include "../../../src/gui/pclviewer.h"



typedef pcl::PointXYZRGBA PointD;
typedef pcl::PointCloud<PointD> PointCloudD;
typedef pcl::PointXYZINormal PointI;
typedef pcl::PointCloud<PointI> PointCloudI;

class CurvatureDialog : public QDialog
{
    Q_OBJECT
public:
    explicit CurvatureDialog(QWidget *parent = 0);
    void
    setViewer(boost::shared_ptr<PCLViewer> guiPtr);
    boost::shared_ptr<PCLViewer>
    getViewer();
    void
    init();
private:

    std::vector<float> e1,e2,e3;
    float
    min_e1,max_e1,min_e2,max_e2,min_e3,max_e3;
    float
    min_pca1,max_pca1,min_pca2,max_pca2,min_pca3,max_pca3;
    boost::shared_ptr<Ui_Dialog_Eigen> dialog;
    boost::weak_ptr<PCLViewer> viewer;
    boost::shared_ptr<PointCloudD> visu_cloud;
    void
    resetViewer();
    void
    updateViewer();
    void
    setGreen(PointD & p);
    void
    setRed(PointD & p);

signals:

public slots:
    void
    minPC1();
    void
    maxPC1();
    void
    minPC2();
    void
    maxPC2();
    void
    minPC3();
    void
    maxPC3();
    void
    abort();
    void
    save();

};

#endif // CURVATUREDIALOG_H
