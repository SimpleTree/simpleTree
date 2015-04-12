#ifndef VOXEL_GRID_DIALOG_H
#define VOXEL_GRID_DIALOG_H

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include "pclviewer.h"

class Voxel_Grid_Dialog
{
private:
    boost::weak_ptr<PCLViewer> pclviewer;
public:
    Voxel_Grid_Dialog();

    boost::shared_ptr<PCLViewer>
    getViewer();
};

#endif // VOXEL_GRID_DIALOG_H
