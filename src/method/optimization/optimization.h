#ifndef OPTIMIZATION_H
#define OPTIMIZATION_H

#include <QThreadPool>
#include <QMutexLocker>
#include <QMutex>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/enable_shared_from_this.hpp>

#include "../method_coefficients.h"
#include "workerspherefollowing.h"
#include "../../controller.h"
#include "../../Model/Tree.h"
#include <random>
#include <pcl/console/time.h>
#include <chrono>


class Optimization : public QThreadPool,  public boost::enable_shared_from_this<Optimization>
{
    Q_OBJECT
private :
    Method_Coefficients coefficients_start, coefficients_end, coefficients_sd;




    float a,b,fac;
    float min_rad = 0.0025;
    float start_dist;
    float end_dist;
    float current_dist;
    float dist_update = 0.1f;
    pcl::console::TicToc tt;


    float min_dist = 0.0001;
    int max_iterations = 6;

    int seeds_per_voxel = 81;

    float current_iteration = 0;

    int numberThreads;
    int _max_seeds = 1;
    int _current_seeds = 0;

    QMutex lock;





    Method_Coefficients
    generate_coefficients();

    std::vector<Method_Coefficients>
    neighboring_coefficients(int i);

    std::vector<Method_Coefficients>
    generate_seeds(Method_Coefficients const coeff);

    PointCloudI::Ptr _cloud_ptr;

    std::string treeID;

    std::vector<bool> isStem;

    void
    make_coefficients_positive(Method_Coefficients & coeff);

    void
    update_sd();
public:

    Method_Coefficients
    getCoefficients()
    {
        return coefficients_end;
    }


    float
    get_current_dist();


    void
    updateCoeff(Method_Coefficients & coeff, float dist, float min_radius);

    explicit Optimization(QObject *parent = 0);

     Optimization(int iterations, int seeds, float min_dist, QObject *parent = 0);

    void
    setCoefficients(Method_Coefficients coeffcients);

    void
    optimize();

    float getFac() const;
    void setFac(float value);

    float getB() const;
    void setB(float value);

    float getA() const;
    void setA(float value);

    void
    setMinRad(float min_rad)
    {
        this->min_rad = min_rad;
    }

    float
    getMinRad()
    {
        return min_rad;
    }


    void
    setCloudPtr(PointCloudI::Ptr cloud_ptr)
    {
        this->_cloud_ptr = cloud_ptr;
    }

    void
    setTreeID(std::string ID)
    {
        this->treeID = ID;
    }

    void
    setIsStem(std::vector<bool> isStem)
    {
        this->isStem = isStem;
    }

signals:
    /**
     * @brief emitProgress The progress emitted in a percentage number
     * @param progress The progress in percentage
     */
    void
    emit_progress(int progress);

public slots:

};

#endif // OPTIMIZATION_H
