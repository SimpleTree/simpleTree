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
    float start_dist;
    float end_dist;
    float current_dist;
    float dist_update = 0.1f;
    pcl::console::TicToc tt;

//    float epsilon_cluster_branch_update = 0.01;
//    float epsilon_cluster_stem_update = 0.01;
//    float epsilon_sphere_update = 0.005;
//    float min_pts_ransac_stem_update = 100;

    float min_dist = 0.0001;
    int max_iterations = 8;

    int seeds_per_voxel = 81;

    float current_iteration = 0;

    int numberThreads;

    QMutex lock;


       Method_Coefficients
       generate_coefficients();

       std::vector<Method_Coefficients>
       neighboring_coefficients();

       std::vector<Method_Coefficients>
       generate_seeds(Method_Coefficients const coeff);


        boost::shared_ptr<Controller> controller;


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
    setControl(boost::shared_ptr<Controller> control);


    void
     updateCoeff(Method_Coefficients & coeff, float dist);



    explicit Optimization(QObject *parent = 0);

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

signals:

public slots:

};

#endif // OPTIMIZATION_H
