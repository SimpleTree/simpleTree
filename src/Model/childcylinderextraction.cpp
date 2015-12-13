#include "childcylinderextraction.h"
namespace simpleTree {
ChildCylinderExtraction::ChildCylinderExtraction(std::vector<Cylinder > & cylinders)
{
//    this->tree = tree;
    this->cylinders = cylinders;
    _indices.resize(this->cylinders.size(),false);

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

 //   std::cout << "finished building kdtree" << std::endl;
}

std::vector<Cylinder>
ChildCylinderExtraction::getChildren(boost::shared_ptr<Cylinder>  cylinder)
{
    {
       // std::cout << cylinder->toString().toStdString();

      std::vector<Cylinder > children;
      std::vector<Cylinder > knnNeighbors;
      PointI searchPoint = cylinder->getCenterPoint();

//
      //std::cout << searchPoint << "search Point" << std::endl;
      //std::cout << radius << " radius " << std::endl;

      pointIdxKnnSearch.clear();
      pointIdxKnnSquaredDistance.clear();
//      kdtree->radiusSearch(searchPoint, 0.0001f, pointIdxKnnSearch,pointIdxKnnSquaredDistance);
//      kdtree->
      kdtree->radiusSearch(searchPoint, radius, pointIdxKnnSearch,pointIdxKnnSquaredDistance);
      //std::cout << " number neighbors " << pointIdxKnnSearch.size() << std::endl;
      //std::cout << "radius search finished" << std::endl;
      std::vector<int> indices_neighbors;
      for(size_t i = 0; i < pointIdxKnnSearch.size(); i++)
      {
         // std::cout << "i : " << i << std::endl;
          if(!_indices.at(pointIdxKnnSearch.at(i)))
          {
          knnNeighbors.push_back(cylinders.at(pointIdxKnnSearch[i]));
          indices_neighbors.push_back(pointIdxKnnSearch[i]);
          }
      }
     // std::cout << "radius sepointIdxarch finished" << std::endl;
      for(size_t i = 0; i <  knnNeighbors.size(); i++)
      {


          Cylinder candidate = knnNeighbors.at(i);
          //std::cout << candidate.toString().toStdString() << std::endl;
          if(cylinder->isParentOf(candidate))
          {
              if(!_indices.at(indices_neighbors.at(i)))
              {
                children.push_back(candidate);
                _indices.at(indices_neighbors.at(i)) = true;
              }


          }
      }
      return children;
    }
}

}


