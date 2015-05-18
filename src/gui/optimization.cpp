#include "optimization.h"

Optimization::Optimization(QObject *parent) :
    QThreadPool(parent)
{
    numberThreads = QThread::idealThreadCount();

}


void
Optimization::setCoefficients(Method_Coefficients coeffcients)
{
    this->coefficients_old = coeffcients;
    this->coefficients_new = coeffcients;
}

std::vector<float>
generateCoefficients(float coeff)
{

}

void
Optimization::updateCoeff(Method_Coefficients coeff, float dist)
{
    lock.lockForRead();
    lock.lockForWrite();

    if(dist < current_thresh)
    {
        coefficients_new = coeff;
        current_thresh = dist;
    }
    lock.unlock();
}

void
Optimization::setControl(boost::shared_ptr<Controller> control)
{
    this->controller = control;
}

void
Optimization::optimize()
{
    while (max_iterations>current_iteration&&min_adjust<current_adjust&&min_thresh<current_thresh) {
        std::cout << "next iteration starts" << current_iteration << std::endl;

        float old_coeff = coefficients_old.epsilon_cluster_branch;
        std::vector<float> coefficients = generateCoefficients(old_coeff);

        for(int i = 0; i < coefficients.size(); i++)
        {
            float coef = coefficients.at(i);
            coefficients_new.epsilon_cluster_branch = coeff;
            WorkerSphereFollowing * worker = new WorkerSphereFollowing();
            worker->control = this->controller;
            worker->setOptimize(shared_from_this());
            this->start(worker);
        }



        method_coefficients.epsilon_cluster_branch += mod_vec.at(k);
        method_coefficients.epsilon_cluster_stem += mod_vec.at(k);
        method_coefficients.epsilon_sphere += mod_vec.at(k);
        method_coefficients.minPts_ransac_stem = 200;
        method_coefficients.minPts_ransac_branch = 1111200;
        method_coefficients.minPts_cluster_stem = 3;
        method_coefficients.minPts_cluster_branch = 5;
        method_coefficients.min_radius_sphere_stem += mod_vec.at(k);
        method_coefficients.min_radius_sphere_branch += mod_vec.at(k);


    }

}
