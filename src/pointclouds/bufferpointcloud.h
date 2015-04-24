#ifndef BUFFERPOINTCLOUD_H
#define BUFFERPOINTCLOUD_H

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

#include <pcl/common/common.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZRGBA PointD;
typedef pcl::PointCloud<PointD> PointCloudD;
typedef pcl::PointXYZINormal PointI;
typedef pcl::PointCloud<PointI> PointCloudI;




class BufferPointCloud
{
public:
    BufferPointCloud(boost::shared_ptr<PointCloudI> source, boost::shared_ptr<PointCloudI> target, float buffer_range);
    boost::shared_ptr<PointCloudI>
    getOutput()
    {
        return buffered;
    }
    std::vector<int>
    getOutPutIndices()
    {
        return pointIdxRadiusSearch;
    }

private:
    std::vector<int> pointIdxRadiusSearch;
    boost::shared_ptr<PointCloudI> source;
    boost::shared_ptr<PointCloudI> down_sampled;
    boost::shared_ptr<PointCloudI> target;
    boost::shared_ptr<PointCloudI> buffered;
    float buffer_range;
    float downsample_range;
    void
    buffer();
};

#endif // BUFFERPOINTCLOUD_H
