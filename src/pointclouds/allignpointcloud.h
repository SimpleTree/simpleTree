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

    void
    setInputSource(PointCloudI::Ptr);

    PointCloudI::Ptr
    getSource();

    PointCloudI::Ptr
    getTarget();

    PointCloudI::Ptr
    getFinal();

    void
    setInputTarget(PointCloudI::Ptr);

    void
    setInputFinal(PointCloudI::Ptr);

    void
    initialAllign();

    void
    ICP();

    QString
    result_str;

    template <typename PointT>
    boost::shared_ptr<pcl::PointCloud<PointT> >
    transform(boost::shared_ptr<pcl::PointCloud<PointT> >  tree, int angleInDegree, int x, int y, int z)
    {
        float angle = angleInDegree;
        float theta = 2*M_PI/3600.0f*angle;
        float dx = x/100.0f;
        float dy = y/100.0f;
        float dz = z/100.0f;

        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.translation() << dx, dy,dz;
        transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
        boost::shared_ptr<pcl::PointCloud<PointT> > transformed_cloud (new pcl::PointCloud<PointT>() );
        pcl::transformPointCloud (*tree, *transformed_cloud, transform);
        return transformed_cloud;
    }

    template <typename PointT>
    boost::shared_ptr<pcl::PointCloud<PointT> >
    transform(boost::shared_ptr<pcl::PointCloud<PointT> >  cloud,Eigen::Vector4f dest)
    {
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.translation() << -dest(0,0),-dest(1,0),-dest(2,0)+0.2f;
        boost::shared_ptr<pcl::PointCloud<PointT> >transformed_cloud (new PointCloudI ());
        pcl::transformPointCloud (*cloud, *transformed_cloud, transform);
        return transformed_cloud;
    }





private:


    boost::weak_ptr<QDialog> allign_dialog;

    boost::shared_ptr<QDialog>
    getDialog();



    const float slice_thickness = 0.05f;

    boost::shared_ptr<PointCloudI> cloud_target;
    boost::shared_ptr<PointCloudI> cloud_source;
    boost::shared_ptr<PointCloudI> cloud_final;



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





    PointCloudI::Ptr
    downsampleCloud(PointCloudI::Ptr cloud, float leaf_size = 0.02f);




};

#endif // ALLIGNPOINTCLOUD_H
