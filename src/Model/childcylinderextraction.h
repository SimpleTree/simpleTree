#ifndef CHILDCYLINDEREXTRACTION_H
#define CHILDCYLINDEREXTRACTION_H

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "Tree.h"
#include "Cylinder.h"

namespace simpleTree {
class ChildCylinderExtraction
{
private:
    boost::shared_ptr<Tree> tree;
    std::vector<boost::shared_ptr<Cylinder> > cylinders;
    boost::shared_ptr<pcl::KdTreeFLANN<PointI> > kdtree;
    int knn = 10;

    pcl::PointCloud<PointI>
    generateCloud();


public:
    ChildCylinderExtraction(boost::shared_ptr<Tree> tree);
};

}
#endif // CHILDCYLINDEREXTRACTION_H
