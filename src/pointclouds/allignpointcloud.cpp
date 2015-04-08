#include "allignpointcloud.h"

AllignPointCloud::AllignPointCloud()
{

}


void
AllignPointCloud::getRootPoint(boost::shared_ptr<PointCloudI> cloud,Eigen::Vector4f & base, float minHeight)
{
    PointI p1;
    PointI p2;
    pcl::getMinMax3D<PointI>(*cloud,p1,p2);
    minHeight = p1.z;
    boost::shared_ptr<PointCloudI> z_slice (new PointCloudI);
    extractZSlice(cloud, z_slice, minHeight);

    boost::shared_ptr<PointCloudI> cluster (new PointCloudI);
    extractLargestCluster(z_slice,cluster);

    pcl::compute3DCentroid(*cluster, base);
}

void
AllignPointCloud::initialAllign()
{
    float minHeight_source = std::numeric_limits<float>::max();
    Eigen::Vector4f root_source;
    getRootPoint(cloud_source,root_source,minHeight_source);
    transform(cloud_source,root_source);

    float minHeight_target = std::numeric_limits<float>::max();
    Eigen::Vector4f root_target;
    getRootPoint(cloud_target,root_target,minHeight_target);
    transform(cloud_target,root_target);





}


void
AllignPointCloud::extractLargestCluster(boost::shared_ptr<PointCloudI> cloud_in, boost::shared_ptr<PointCloudI>  cloud_out, float distance)
{
    std::vector<pcl::PointIndices> cluster_indices;
    cloud_out.reset(new PointCloudI);

    pcl::search::KdTree<PointI>::Ptr tree ( new pcl::search::KdTree<PointI> );
    tree->setInputCloud ( cloud_in );

    pcl::EuclideanClusterExtraction<PointI> ec;
    ec.setClusterTolerance ( distance );
    ec.setMinClusterSize ( 1 );
    ec.setSearchMethod ( tree );
    ec.setInputCloud ( cloud_in );
    ec.extract ( cluster_indices );

    if ( cluster_indices.size () > 0 ) {
        pcl::PointIndices largestCluster = cluster_indices.at ( 0 );
        for ( std::vector<int>::const_iterator pit = largestCluster.indices.begin (); pit != largestCluster.indices.end (); pit++ )
            cloud_out->points.push_back ( cloud_in->points[*pit] );
    }

}


void
AllignPointCloud::setInputSource(boost::shared_ptr<PointCloudI> cloud_source)
{
    this->cloud_source = cloud_source;
}

void
AllignPointCloud::setInputTarget(boost::shared_ptr<PointCloudI> cloud_target)
{
    this->cloud_target = cloud_target;
}

boost::shared_ptr<QDialog>
AllignPointCloud::getDialog()
{
    return this->allign_dialog.lock();
}

template <typename PointT>
boost::shared_ptr<pcl::PointCloud<PointT> >
transform(boost::shared_ptr<pcl::PointCloud<PointT> > & cloud,Eigen::Vector4f dest)
{
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << -dest(0,0),-dest(1,0),-dest(2,0)+0.2f;
    boost::shared_ptr<pcl::PointCloud<PointT> >transformed_cloud (new PointCloudI ());
    pcl::transformPointCloud (*cloud, *transformed_cloud, transform);
    return transformed_cloud;
}


void
AllignPointCloud::extractZSlice(boost::shared_ptr<PointCloudI> cloud, boost::shared_ptr<PointCloudI> base, float height)
{
    pcl::PassThrough<PointI> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (height, height+slice_thickness);
    pass.filter (*base);
}


template <typename PointT>
boost::shared_ptr<pcl::PointCloud<PointT> >
AllignPointCloud::transform(boost::shared_ptr<pcl::PointCloud<PointT> > & tree, int angleInDegree, int x, int y, int z)
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



PointCloudI::Ptr
AllignPointCloud::downsampleCloud(PointCloudI::Ptr cloud, float leaf_size)
{
    boost::shared_ptr<PointCloudI> downsampled (new PointCloudI);
    pcl::VoxelGrid<PointI> sor;
    sor.setInputCloud ( cloud );
    sor.setLeafSize ( leaf_size, leaf_size, leaf_size );
    sor.filter ( *downsampled );
    return downsampled;
}

void
AllignPointCloud::ICP()
{
    *cloud_source = *cloud_final;

    boost::shared_ptr<PointCloudI> downsampled_source (new PointCloudI);
    boost::shared_ptr<PointCloudI> downsampled_target (new PointCloudI);

    downsampled_target = downsampleCloud(cloud_target);
    downsampled_source = downsampleCloud(cloud_source);

    pcl::IterativeClosestPoint<PointI, PointI> icp;
    icp.setInputSource(downsampled_source);
    icp.setInputTarget(downsampled_target);
    icp.setMaximumIterations (150);
    icp.setMaxCorrespondenceDistance(0.06);
    icp.setTransformationEpsilon(1e-8);
    icp.setEuclideanFitnessEpsilon(1e-5);
    icp.align(*cloud_final);

    if (icp.hasConverged ())
    {

        QString str("\nICP has converged, score is ");
        str.append(QString::number(icp.getFitnessScore ())).append("\n The transformation Matrix : \n");
        Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity ();
        transformation_matrix *= icp.getFinalTransformation();
        std::stringstream stream;
        stream << transformation_matrix << "\n";
        str.append(QString::fromStdString(stream.str()));
        result_str = str;
        pcl::transformPointCloud(*cloud_source,*cloud_final, transformation_matrix);
    }
    else
    {
        result_str = QString("\n ICP has not converged. \n");
    }
}


