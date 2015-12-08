#ifndef STEM_FIT_H
#define STEM_FIT_H

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <boost/smart_ptr/shared_ptr.hpp>
#include <boost/smart_ptr/weak_ptr.hpp>
#endif



#include <pcl/console/time.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree_search.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include "../method/set_coefficients.h"
#include <iostream>
#include <limits>
#include <vector>

#include "../controller.h"
#include "../Model/Cylinder.h"
#include "method_coefficients.h"

typedef pcl::PointXYZINormal PointI;
typedef pcl::PointCloud<PointI> PointCloudI;

class Stem_fit
{

private:
     std::vector<pcl::ModelCoefficients> cylinders;
     std::vector<pcl::ModelCoefficients> circles;
     PointCloudI::Ptr _upper_cloud;
     PointCloudI::Ptr _temp_cloud;
     PointCloudI::Ptr _cloud;
     float _min_height;
     float _bin_width;

     void
     fit_circle(PointCloudI::Ptr cloud, float lower_height, float upper_height);





public:
    Stem_fit(PointCloudI::Ptr treeCloud, float min_height, float bin_width);

    void
    compute();

    std::vector<pcl::ModelCoefficients>&
    getCylinders ()
    {
      return cylinders;
    }
};

#endif // STEM_FIT_H
