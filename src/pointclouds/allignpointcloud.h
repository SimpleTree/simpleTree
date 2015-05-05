/*
* Software License Agreement (BSD License)
*
* Copyright (c) 2015, Jan Hackenberg, University of Freiburg.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of Willow Garage, Inc. nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
*/
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
    /** \brief empty constructor
     * */
    AllignPointCloud();

    /** \brief sets the source point cloud
     * \param the source_cloud
     * */
    void
    setInputSource(PointCloudI::Ptr);

    /** \brief returns a shared pointer to the source_cloud
     * */
    PointCloudI::Ptr
    getSource();

    /** \brief returns a shared pointer to the target_cloud
     * */
    PointCloudI::Ptr
    getTarget();

    /** \brief returns a shared pointer to the final_cloud
     * */
    PointCloudI::Ptr
    getFinal();

    /** \brief sets the target point cloud
     * \param the target_cloud
     * */
    void
    setInputTarget(PointCloudI::Ptr);

    /** \brief sets the final point cloud
     * \param the final_cloud
     * */
    void
    setInputFinal(PointCloudI::Ptr);

    /** \brief Transform both cloud_source's and cloud_target's estimated root point to the coordinate system center
     * */
    void
    initialAllign();

    /** \brief Computes the Iterative Closest point algorithm. Initial allignment is necesarry
     * */
    void
    ICP();

    QString
    result_str;

    /** \brief Computes a transformation with given x,y,z and a rotation around angle angleInDegree for a given cloud tree
     * \param tree: the input point cloud
     * \param angleInDeGree: the rotation angle in DegreeMeasureConcept
     * \param x: the x-translation
     * \param y: the y-translation
     * \param z: the z-translation
     * \return: the transformed point cloud
     * */
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
    /** \brief Computes a transformation with given x,y,z of an Vector4f point for a given cloud
     * \param cloud: the input point cloud
     * \param dest: The Vector4f storing x,y,z translation info
     * \return: the transformed point cloud
     * */
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

    /** \brief a weak pointer to the dialog
     * */
    boost::weak_ptr<QDialog> allign_dialog;

    /** \brief Converts the weak pointer to a shared pointer of the dialog
     * \return the shared pointer of the dialog class
     * */
    boost::shared_ptr<QDialog>
    getDialog();


    /** \brief the thickness of the point cloud slice to compute the intial allignment from
     * */
    const float slice_thickness = 0.05f;

    /** \brief a shared pointer of the target cloud
     * */
    boost::shared_ptr<PointCloudI> cloud_target;

    /** \brief a shared pointer of the source cloud
     * */
    boost::shared_ptr<PointCloudI> cloud_source;

    /** \brief a shared pointer of the final cloud
     * */
    boost::shared_ptr<PointCloudI> cloud_final;


    /** \brief a shared pointer of the target cloud
     * */
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
