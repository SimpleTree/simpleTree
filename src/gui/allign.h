#ifndef ALLIGN_H
#define ALLIGN_H

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include "pclviewer.h"

namespace Ui {
class AllignPointCloud;

}

class AllignPointCloud : public QObject
{

    Q_OBJECT
public:
    AllignPointCloud();

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
    void
    intialAllign();
 private:
    boost::shared_ptr<PCLViewer> gui_ptr;


};

#endif // ALLIGN_H
