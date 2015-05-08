#include "childcylinderextraction.h"
namespace simpleTree {
ChildCylinderExtraction::ChildCylinderExtraction(std::vector<Cylinder > & cylinders)
{
//    this->tree = tree;
    this->cylinders = cylinders;
    fillKdTree();
}

void
ChildCylinderExtraction::fillKdTree()
{
    cloud_cylinder_centers.reset(new PointCloudI);
    kdtree.reset(new pcl::KdTreeFLANN<PointI>);
    for(size_t i = 0; i < cylinders.size(); i++)
    {
        Cylinder cylinder = cylinders.at(i);
        if(cylinder.getHalfSize()>radius)
        {
            radius = cylinder.getHalfSize();
        }
        PointI center = cylinder.getCenterPoint();
        cloud_cylinder_centers->push_back( center);
    }
    radius *= 2;
    radius += 0.05;
    kdtree->setInputCloud(cloud_cylinder_centers);
}

std::vector<Cylinder>
ChildCylinderExtraction::getChildren(boost::shared_ptr<Cylinder>  cylinder)
{
    {
      std::vector<Cylinder > children;
      std::vector<Cylinder > knnNeighbors;
      PointI searchPoint = cylinder->getCenterPoint();

      kdtree->radiusSearch(searchPoint, radius, pointIdxKnnSearch,pointIdxKnnSquaredDistance);
      for(size_t i = 0; i < pointIdxKnnSearch.size(); i++)
      {
          knnNeighbors.push_back(cylinders.at(pointIdxKnnSearch[i]));
      }
      for(size_t i = 0; i <  pointIdxKnnSearch.size(); i++)
      {
          Cylinder candidate = knnNeighbors.at(i);
          if(cylinder->isParentOf(candidate))
          {
              children.push_back(candidate);
          }
      }
      return children;
    }
}

}


