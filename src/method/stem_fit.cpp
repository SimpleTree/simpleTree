#include "stem_fit.h"

Stem_fit::Stem_fit(PointCloudI::Ptr treeCloud, float min_height, float bin_width)
{
    this->_bin_width = bin_width;
    this->_min_height = min_height;
    this->_cloud = treeCloud;
    compute();
}

void
Stem_fit::compute()
{
    float lower_height = 0.0f;
    float upper_height = lower_height + _bin_width;
    while(lower_height < _min_height)
    {
        _temp_cloud.reset(new PointCloudI);
        pcl::PassThrough<PointI> pass;
        pass.setInputCloud (_cloud);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (lower_height, upper_height);
        pass.filter (*_temp_cloud);
        pcl::ModelCoefficients cylinder = fit_circle(_temp_cloud, lower_height,upper_height);



        lower_height += _bin_width;
        upper_height += _bin_width;
    }
    _upper_cloud.reset(new PointCloudI);

}

pcl::ModelCoefficients
Stem_fit::fit_circle(PointCloudI::Ptr cloud, float lower_height, float upper_height)
{
    pcl::PointCloud<PointI>::Ptr cloud_2d (new pcl::PointCloud<PointI>);
    cloud_2d->points.resize(cloud->points.size());
    for(size_t i = 0; i < cloud->points.size(); i++)
    {
        float x = cloud->points.at(i).x;
        float y = cloud->points.at(i).y;
        float z = (lower_height+upper_height)/2;


        PointI p;
        p.x =x;
        p.y =y;
        p.z = 0;
        cloud_2d->points.at(i) = p;
    }


    pcl::NormalEstimation<PointI, PointI> ne;
      ne.setInputCloud (cloud_2d);
      pcl::search::KdTree<PointI>::Ptr tree (new pcl::search::KdTree<PointI> ());
      ne.setSearchMethod (tree);
      ne.setKSearch(20);
      ne.compute (*cloud_2d);




    pcl::SACSegmentationFromNormals<PointI, PointI> seg;
    pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients_circle3d(
            new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CIRCLE3D);
    seg.setMethodType(pcl::SAC_MLESAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.1);
    seg.setInputCloud(cloud_2d);
    seg.setInputNormals(cloud_2d);

    // Obtain the cylinder inliers and coefficients
    seg.segment(*inliers_cylinder, *coefficients_circle3d);
    std::cout << "foo" << std::endl;
    std::cout << *coefficients_circle3d << std::endl;
    circles.push_back(*coefficients_circle3d);



}
