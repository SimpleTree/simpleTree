#include "optimization.h"

Optimization::Optimization(QObject *parent) :
    QThreadPool(parent)
{
    numberThreads = QThread::idealThreadCount()-1;
    std::cout << "Optimization Constructor : number of threads :" << numberThreads << std::endl;
    this->setMaxThreadCount(std::max<int>(numberThreads,1));

}




void
Optimization::setCoefficients(Method_Coefficients coeffcients)
{
    this->coefficients_start = coeffcients;
    this->coefficients_end = coeffcients;
}


void
Optimization::updateCoeff(Method_Coefficients & coeff, float dist)
{
    QMutexLocker locker(&lock);

    if(dist < current_dist)
    {
        std::cout << "update mit " << dist << std::endl;
        coefficients_end = coeff;
        current_dist = dist;
    }
}

void
Optimization::make_coefficients_positive(Method_Coefficients & coeff)
{
    if(coeff.epsilon_cluster_branch<0.01)
    {
        coeff.epsilon_cluster_branch = 0.01;
    }

    if(coeff.epsilon_cluster_stem<0.02)
    {
        coeff.epsilon_cluster_stem = 0.02;
    }

    if(coeff.epsilon_sphere<0.005)
    {
        coeff.epsilon_sphere = 0.005;
    }

    if(coeff.epsilon_sphere>0.03)
    {
        coeff.epsilon_sphere = 0.03;
    }

    if(coeff.minPts_ransac_stem<100)
    {
        coeff.minPts_ransac_stem = 100;
    }
}

std::vector<Method_Coefficients>
Optimization::generate_seeds(Method_Coefficients const coeff)
{








    std::vector<Method_Coefficients> vec;
    for(int i = 0; i < seeds_per_voxel; i++)
    {
            Method_Coefficients temp = coeff;
        unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
      std::default_random_engine generator (seed);
      std::normal_distribution<float> distribution_epsilon_cluster_branch (0,epsilon_cluster_branch_update/2);
      std::normal_distribution<float> distribution_epsilon_cluster_stem (0,epsilon_cluster_stem_update/2);
      std::normal_distribution<float> distribution_epsilon_sphere (0,epsilon_sphere_update/2);
      std::normal_distribution<float> distribution_min_Pts_Ransac_Stem (0,min_pts_ransac_stem_update/2);


        float update = distribution_epsilon_cluster_branch(generator);
        temp.epsilon_cluster_branch += update;


        update = distribution_epsilon_cluster_stem(generator);
        temp.epsilon_cluster_stem += update;

        update = distribution_epsilon_sphere(generator);
        temp.epsilon_sphere += update;

        int update_2 = distribution_min_Pts_Ransac_Stem(generator);
        temp.minPts_ransac_stem += update_2;

        vec.push_back(temp);
    }
    return vec;
}


std::vector<Method_Coefficients>
Optimization::neighboring_coefficients()
{
    std::vector<Method_Coefficients> vec;
    for(int i = -1 ; i < 2 ; i++ )
    {
        for (int j = -1 ; j < 2 ; j++)
        {
            for(int k = -1; k < 2; k++)
            {
                for(int l = -1; l < 2; l++)
                {
                    Method_Coefficients coeff = coefficients_start;
                    coeff.epsilon_cluster_branch += i*epsilon_cluster_branch_update;
                    coeff.epsilon_cluster_stem   += j*epsilon_cluster_stem_update;
                    coeff.epsilon_sphere         += k*epsilon_sphere_update;
                    coeff.minPts_ransac_stem     += l*min_pts_ransac_stem_update;
                    vec.push_back(coeff);
                }
            }
        }
    }
    return vec;
}

void
Optimization::setControl(boost::shared_ptr<Controller> control)
{
    this->controller = control;
}

float
Optimization::get_current_dist()
{
    return current_dist;
}

void
Optimization::optimize()
{
    tt.tic();
    current_iteration = 0;
    start_dist = 0.1;
    current_dist = start_dist;
    while (max_iterations>current_iteration&&min_dist<start_dist) {


        std::cout << "next iteration starts " << current_iteration << std::endl;
        std::vector<Method_Coefficients> neighbors = neighboring_coefficients();
        for(std::vector<Method_Coefficients>::iterator it = neighbors.begin(); it < neighbors.end(); it++)
        {
            Method_Coefficients coeff = *it;
            make_coefficients_positive(coeff);

             {
                 WorkerSphereFollowing * worker = new WorkerSphereFollowing();
                 worker->set_control(this->controller);
                 worker->setOptimize(shared_from_this());
                 worker->set_coefficients(coeff);
                 this->start(worker);
             }


        }
        std::cout << "Halve threads started " << std::endl;
        this->waitForDone();
        std::vector<Method_Coefficients> seeds = generate_seeds(this->coefficients_end);
        for(std::vector<Method_Coefficients>::iterator kit = seeds.begin(); kit < seeds.end(); kit++)
        {
            Method_Coefficients seed = *kit;
            make_coefficients_positive(seed);
            WorkerSphereFollowing * worker = new WorkerSphereFollowing();
            worker->set_control(this->controller);
            worker->setOptimize(shared_from_this());
            worker->set_coefficients(seed);
            this->start(worker);
        }
        current_iteration ++;
        std::cout << "All threads started " << std::endl;
        this->waitForDone();
        end_dist = current_dist;

        std::cout << "All threads ended"  << std::endl;
            std::cout << "dist improve : start" << start_dist  << std::endl;
        start_dist = end_dist;


        std::cout << "             : end  " << end_dist  << std::endl;




        epsilon_cluster_branch_update = std::abs(coefficients_start.epsilon_cluster_branch-coefficients_end.epsilon_cluster_branch);
        epsilon_cluster_stem_update = std::abs(coefficients_start.epsilon_cluster_stem-coefficients_end.epsilon_cluster_stem);
        epsilon_sphere_update = std::abs(coefficients_start.epsilon_sphere-coefficients_end.epsilon_sphere);
        min_pts_ransac_stem_update = std::abs(coefficients_start.minPts_ransac_stem-coefficients_end.minPts_ransac_stem);


        epsilon_cluster_branch_update = std::max<float>(epsilon_cluster_branch_update,0.002f);
        epsilon_cluster_stem_update = std::max<float>(epsilon_cluster_stem_update,0.002f);
        epsilon_sphere_update = std::max<float>(epsilon_sphere_update,0.001f);
        min_pts_ransac_stem_update = std::max<int>(min_pts_ransac_stem_update,20);


        std::cout << "from" << std::endl;
        std::cout << coefficients_start.toQString().toStdString() << std::endl;

        coefficients_start = coefficients_end;
        std::cout << "too" << std::endl;
        std::cout << coefficients_end.toQString().toStdString() << std::endl;

        std::cout << "took " << tt.toc()/1000 << " seconds" << std::endl;



    }



}
