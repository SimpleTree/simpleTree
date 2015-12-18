#ifndef CHILDCYLINDEREXTRACTION_H
#define CHILDCYLINDEREXTRACTION_H

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <QString>


#include "Tree.h"
#include "Cylinder.h"

typedef pcl::PointXYZINormal PointI;
typedef pcl::PointCloud<PointI> PointCloudI;

namespace simpleTree {
class ChildCylinderExtraction
{
private:
    //boost::shared_ptr<Tree> tree;
    std::vector<Cylinder > cylinders;
    boost::shared_ptr<PointCloudI> cloud_cylinder_centers;

    boost::shared_ptr<pcl::KdTreeFLANN<PointI> > kdtree;
    //int knn = 100;
    std::vector<int> pointIdxKnnSearch;
    std::vector<float> pointIdxKnnSquaredDistance;
    pcl::PointCloud<PointI>
    generateCloud();

    void
    fillKdTree();

    float radius = std::numeric_limits<float>::min();

    std::vector<bool> _indices;







public:
    ChildCylinderExtraction(std::vector<Cylinder>  & cylinders);

    std::vector<Cylinder>
    getChildren (boost::shared_ptr<Cylinder>  cylinder);
};

}
#endif // CHILDCYLINDEREXTRACTION_H
