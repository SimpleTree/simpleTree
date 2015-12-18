#ifndef OPTIMIZATION_STEM_H
#define OPTIMIZATION_STEM_H

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <boost/smart_ptr/shared_ptr.hpp>
#include <boost/smart_ptr/weak_ptr.hpp>
#endif


#include <QThreadPool>
#include <QMutexLocker>
#include <QMutex>
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
#include "../../method/set_coefficients.h"
#include "workerstemfit.h"
#include <iostream>
#include <limits>
#include <vector>

#include "../../Model/Cylinder.h"
#include "../method_coefficients.h"

typedef pcl::PointXYZINormal PointI;
typedef pcl::PointCloud<PointI> PointCloudI;


class optimization_stem : public QThreadPool,  public boost::enable_shared_from_this<optimization_stem>
{
private:
    int numberThreads = 1;
    float _min_height;
    float _bin_width;
    float _epsilon;
    float _best_dist = 100;
    float _best_binwidth = 1;
    float _best_epsilon = 0.1;
    PointCloudI::Ptr _cloud;

    void
    reduce_cloud();

public:
    optimization_stem(float min_height, float bin_width, float epsilon, PointCloudI::Ptr cloud);
    ~optimization_stem();


    void
    updateCoeff(float dist, float bin_width, float epsilon);

    void
    optimize();

    float
    get_bin_width()
    {
        return _best_binwidth;
    }

    float
    get_epsilon ()
    {
        return _best_epsilon;
    }

};

#endif // OPTIMIZATION_STEM_H
