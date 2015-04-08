#ifndef ALLIGNPOINTCLOUD_H
#define ALLIGNPOINTCLOUD_H


#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <QDialog>

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

typedef pcl::PointXYZRGBA PointD;
typedef pcl::PointCloud<PointD> PointCloudD;
typedef pcl::PointXYZINormal PointI;
typedef pcl::PointCloud<PointI> PointCloudI;

class AllignPointCloud
{
public:
    AllignPointCloud();

private:
    QString result_str;

    boost::weak_ptr<QDialog> allign_dialog;

    boost::shared_ptr<QDialog>
    getDialog();

    void
    initialAllign();


    const float slice_thickness = 0.05f;

    boost::shared_ptr<PointCloudI> cloud_target;
    boost::shared_ptr<PointCloudI> cloud_source;
    boost::shared_ptr<PointCloudI> cloud_final;

    void
    setInputSource(PointCloudI::Ptr);

    void
    setInputTarget(PointCloudI::Ptr);

    void
    import(boost::shared_ptr<PointCloudI> & cloud, QString & name);

    void
    extractLargestCluster(boost::shared_ptr<PointCloudI> cloud_in, boost::shared_ptr<PointCloudI> cloud_out, float distance = 0.03f);

    void
    getRootPoint(boost::shared_ptr<PointCloudI> cloud,Eigen::Vector4f & base, float minHeight);

    void
    extractZSlice(boost::shared_ptr<PointCloudI> tree, boost::shared_ptr<PointCloudI> cloud, float slice_thickness);

    void
    extractBaseCloud(boost::shared_ptr<PointCloudI> cloud, boost::shared_ptr<PointCloudI> base, float height);


    template <typename PointT>
    boost::shared_ptr<pcl::PointCloud<PointT> >
    transform(boost::shared_ptr<pcl::PointCloud<PointT> > & cloud,Eigen::Vector4f dest);

    template <typename PointT>
    boost::shared_ptr<pcl::PointCloud<PointT> >
    transform(boost::shared_ptr<pcl::PointCloud<PointT> > & cloud, int angle, int x, int y ,int z);


    PointCloudI::Ptr
    downsampleCloud(PointCloudI::Ptr cloud, float leaf_size = 0.02f);


    void
    resetVisualization();

    void
    ICP();

};

#endif // ALLIGNPOINTCLOUD_H
