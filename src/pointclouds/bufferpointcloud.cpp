#include "bufferpointcloud.h"
#include "voxelgridfilter.h"
#include <pcl/octree/octree_search.h>

BufferPointCloud::BufferPointCloud(boost::shared_ptr<PointCloudI> source, boost::shared_ptr<PointCloudI> target, float buffer_range)
{
    this->source = source;
    this->target = target;
    this->buffer_range = buffer_range;
    this->downsample_range = buffer_range/4;

    VoxelGridFilter filter (VoxelGridFilter(this->downsample_range));
    filter.setInput(source);
    filter.voxel_grid_filter();

    this->down_sampled = filter.getOutput();
    buffer();
    std::cout << "-------" << std::endl;
    std::cout << this->source->points.size() << std::endl;
    std::cout << this->target->points.size() << std::endl;
    std::cout << buffered->points.size() << std::endl;
}

void
BufferPointCloud::buffer()
{
    buffered.reset(new PointCloudI);
    pcl::octree::OctreePointCloudSearch<PointI> octree (0.02f);

    octree.setInputCloud (target);
    octree.addPointsFromInputCloud ();

    for(size_t i = 0; i < down_sampled->points.size(); i++)
    {
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        PointI searchPoint = down_sampled->points.at(i);
        octree.radiusSearch (searchPoint, buffer_range, pointIdxRadiusSearch, pointRadiusSquaredDistance);
        for(size_t j = 0; j < pointIdxRadiusSearch.size(); j++)
        {
            int index = pointIdxRadiusSearch.at(j);
            buffered->push_back(target->points.at(index));
            //octree.deleteVoxelAtPoint(index);
            this->pointIdxRadiusSearch.push_back(index);
        }

    }
}
