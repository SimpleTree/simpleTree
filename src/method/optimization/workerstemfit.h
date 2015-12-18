#ifndef WORKERSTEMFIT_H
#define WORKERSTEMFIT_H

#include <QRunnable>

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
    #include <QReadWriteLock>
#include "optimization_stem.h"
#include "../../Model/Cylinder.h"

#include "../method_coefficients.h"
#include "../stem_fit.h"

class optimization_stem;
class WorkerStemFit: public QRunnable
{

private:
    float
    _min_height;

    float
    _bin_width;

    float
    _epsilon;

    PointCloudI::Ptr _cloud;

    boost::weak_ptr<optimization_stem> _optim;

    float
    median(std::vector<float>  &values);

    float
    squared_mean(std::vector<float>  &v);

    std::vector<float>
    compute_distances(std::vector<pcl::ModelCoefficients>  & cylinders);

    std::vector<float>
    reduce_distances(std::vector<float>   & distances_in, std::vector<pcl::ModelCoefficients>  & cylinders);

    std::vector<int>
    indexOfPointsNearCylinder ( pcl::octree::OctreePointCloudSearch<PointI>& octree,
                                      pcl::ModelCoefficients& cylinder) ;
public:
    WorkerStemFit(PointCloudI::Ptr cloud, float min_height,  float bin_width, float epsilon);
    ~WorkerStemFit();

    void
    run();

    boost::shared_ptr<optimization_stem>
    get_optimization(){
        return _optim.lock();
    }

    void
    set_optimization(boost::shared_ptr<optimization_stem> optim)
    {
        std::cout << "a " << optim << std::endl;
        this->_optim = optim;
        std::cout << "b" << std::endl;
    }

signals:


public slots:

};

#endif // WORKERSTEMFIT_H
