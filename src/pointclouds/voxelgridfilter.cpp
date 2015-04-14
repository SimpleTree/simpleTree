#include "voxelgridfilter.h"

VoxelGridFilter::VoxelGridFilter(float cell_size , float split_size)
{
    this -> cell_size = cell_size;
    this -> split_size = split_size;
}


boost::shared_ptr<PointCloudI>
VoxelGridFilter::extractSubCloud(PointI min, PointI max)
{
    pcl::octree::OctreePointCloudSearch<PointI> octree ( 0.02f );
    octree.setInputCloud ( input );
    octree.addPointsFromInputCloud ();
    boost::shared_ptr<PointCloudI> sub_cloud (new PointCloudI);
    std::vector<int> pointIdxBoxSearch;
    Eigen::Vector3f min_pt ( min.x, min.y, min.z );
    Eigen::Vector3f max_pt ( max.x, max.y, max.z );
    octree.boxSearch ( min_pt, max_pt, pointIdxBoxSearch );

    pcl::ExtractIndices<PointI> extract;
    pcl::PointIndices::Ptr inliers ( new pcl::PointIndices () );
    inliers->indices = pointIdxBoxSearch;
    extract.setInputCloud ( input );
    extract.setIndices ( inliers );
    extract.setNegative ( false );
    extract.filter ( *sub_cloud );
    return sub_cloud;
}

boost::shared_ptr<PointCloudI>
VoxelGridFilter::getOutput()
{
    return this->output;
}

boost::shared_ptr<PointCloudI>
VoxelGridFilter::down_sample(boost::shared_ptr<PointCloudI> cloud)
{
        boost::shared_ptr<PointCloudI> downsampled_cloud (new PointCloudI);
    pcl::VoxelGrid<PointI> sor;

    sor.setInputCloud ( cloud );
    sor.setLeafSize ( cell_size, cell_size, cell_size );
    sor.filter ( *downsampled_cloud );
    return downsampled_cloud;
}

void
VoxelGridFilter::voxel_grid_filter ()
{
    output.reset(new PointCloudI);
    PointI p1;
    PointI p2;
    pcl::getMinMax3D<PointI>(*input,p1,p2);

    float xMin = p1.x;
    float xMax = p2.x;

    float yMin = p1.y;
    float yMax = p2.y;

    float zMin = p1.z;
    float zMax = p2.z;


    float xCurrent= xMin;
    while(xCurrent < xMax)
    {
         float yCurrent = yMin;
        while(yCurrent < yMax)
        {
                float zCurrent = zMin;
            while(zCurrent < zMax)
            {
                PointI min;
                PointI max;
                min.x = xCurrent;
                min.y = yCurrent;
                min.z = zCurrent;
                max.x = xCurrent + split_size;
                max.y = yCurrent + split_size;
                max.z = zCurrent + split_size;
                boost::shared_ptr<PointCloudI> sub_cloud = extractSubCloud(min,max);
                boost::shared_ptr<PointCloudI> sub_downsampled = down_sample(sub_cloud);
                * output += * sub_downsampled;
                zCurrent += split_size;
            }
            yCurrent += split_size;
        }
        xCurrent += split_size;
    }
    output_size = output->points.size();

}




void
VoxelGridFilter::setInput(boost::shared_ptr<PointCloudI> input_cloud)
{
    this->input_size = input_cloud->points.size();
    this->input = input_cloud;
}
