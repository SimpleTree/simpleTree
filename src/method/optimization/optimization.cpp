#include "optimization.h"

Optimization::Optimization(  QObject *parent) :
    QThreadPool(parent)
{
    numberThreads = QThread::idealThreadCount()-1;
    this->setMaxThreadCount(std::max<int>(numberThreads,1));



    coefficients_sd.sphere_radius_multiplier = 0.3f;
    coefficients_sd.epsilon_cluster_branch = 0.01f;
    coefficients_sd.epsilon_cluster_stem = 0.01f;
    coefficients_sd.epsilon_sphere = 0.005f;
    coefficients_sd.minPts_ransac_stem = 100;
    coefficients_sd.minPts_ransac_branch = 1;
    coefficients_sd.minPts_cluster_stem = 1;
    coefficients_sd.minPts_cluster_branch = 1;
    coefficients_sd.min_radius_sphere_stem = 0.0055f;
    coefficients_sd.min_radius_sphere_branch = 0.005f;
}


Optimization::Optimization(int iterations, int seeds, float min_dist, QObject *parent)
{
    numberThreads = QThread::idealThreadCount()-1;
    this->setMaxThreadCount(std::max<int>(numberThreads,1));

    this->max_iterations = iterations;
    this->seeds_per_voxel = seeds;
    if(max_iterations*seeds_per_voxel>1)
    {
        _max_seeds = max_iterations*seeds_per_voxel;
    }
    this->min_dist = min_dist;



    coefficients_sd.sphere_radius_multiplier = 0.3f;
    coefficients_sd.epsilon_cluster_branch = 0.01f;
    coefficients_sd.epsilon_cluster_stem = 0.01f;
    coefficients_sd.epsilon_sphere = 0.005f;
    coefficients_sd.minPts_ransac_stem = 100;
    coefficients_sd.minPts_ransac_branch = 1;
    coefficients_sd.minPts_cluster_stem = 1;
    coefficients_sd.minPts_cluster_branch = 1;
    coefficients_sd.min_radius_sphere_stem = 0.0055f;
    coefficients_sd.min_radius_sphere_branch = 0.005f;
}


void
Optimization::update_sd()
{
    coefficients_sd.sphere_radius_multiplier = std::abs(coefficients_start.sphere_radius_multiplier-coefficients_end.sphere_radius_multiplier);
    coefficients_sd.epsilon_cluster_branch = std::abs(coefficients_start.epsilon_cluster_branch-coefficients_end.epsilon_cluster_branch);
    coefficients_sd.epsilon_cluster_stem = std::abs(coefficients_start.epsilon_cluster_stem-coefficients_end.epsilon_cluster_stem);
    coefficients_sd.epsilon_sphere = std::abs(coefficients_start.epsilon_sphere-coefficients_end.epsilon_sphere);
    coefficients_sd.minPts_ransac_stem = std::abs(coefficients_start.minPts_ransac_stem-coefficients_end.minPts_ransac_stem);
    coefficients_sd.minPts_cluster_stem = std::abs(coefficients_start.minPts_cluster_stem-coefficients_end.minPts_cluster_stem);
    coefficients_sd.minPts_cluster_branch = std::abs(coefficients_start.minPts_cluster_branch-coefficients_end.minPts_cluster_branch);
    coefficients_sd.min_radius_sphere_stem = std::abs(coefficients_start.min_radius_sphere_stem-coefficients_end.min_radius_sphere_stem);
    coefficients_sd.min_radius_sphere_branch = std::abs(coefficients_start.min_radius_sphere_branch-coefficients_end.min_radius_sphere_branch);

    coefficients_sd.sphere_radius_multiplier = std::max<float>(coefficients_sd.sphere_radius_multiplier,0.02f);
    coefficients_sd.epsilon_cluster_branch = std::max<float>(coefficients_sd.epsilon_cluster_branch,0.02f);
    coefficients_sd.epsilon_cluster_stem = std::max<float>(coefficients_sd.epsilon_cluster_stem,0.02f);
    coefficients_sd.epsilon_sphere = std::max<float>(coefficients_sd.epsilon_sphere,0.02f);
    coefficients_sd.minPts_ransac_stem = std::max<int>(coefficients_sd.minPts_ransac_stem,50);
    coefficients_sd.minPts_cluster_stem = std::max<int>(coefficients_sd.minPts_cluster_stem,10);
    coefficients_sd.minPts_cluster_branch = std::max<int>(coefficients_sd.minPts_cluster_branch,10);
    coefficients_sd.min_radius_sphere_stem = std::max<float>(coefficients_sd.min_radius_sphere_stem,0.02f);
    coefficients_sd.min_radius_sphere_branch = std::max<float>(coefficients_sd.min_radius_sphere_branch,0.02f);
}




void
Optimization::setCoefficients(Method_Coefficients coeffcients)
{
    this->coefficients_start = coeffcients;
    this->coefficients_end = coeffcients;
}


void
Optimization::updateCoeff(Method_Coefficients & coeff, float dist, float minRadius)
{
    QMutexLocker locker(&lock);
    _current_seeds++;
    int i = 0;
    if(_max_seeds>0)
    {
        i = (_current_seeds*100)/_max_seeds;
    }
    if(std::abs(dist) < std::abs(current_dist))
    {
        coefficients_end = coeff;
        current_dist = dist;
        coefficients_end.minRad = minRadius;
        this->min_rad = minRadius;
    }
    if(i!=0)
    {
        emit emit_progress(i);
        QCoreApplication::processEvents();
    }

}

void
Optimization::make_coefficients_positive(Method_Coefficients & coeff)
{

    if(coeff.sphere_radius_multiplier<1.5)
    {
        coeff.sphere_radius_multiplier = 1.5;
    }

    if(coeff.epsilon_cluster_branch<0.01)
    {
        coeff.epsilon_cluster_branch = 0.01;
    }

    if(coeff.epsilon_cluster_stem<0.02)
    {
        coeff.epsilon_cluster_stem = 0.02;
    }

    if(coeff.epsilon_sphere<0.01)
    {
        coeff.epsilon_sphere = 0.01;
    }

    if(coeff.epsilon_sphere>0.03)
    {
        coeff.epsilon_sphere = 0.03;
    }

    if(coeff.minPts_ransac_stem<100)
    {
        coeff.minPts_ransac_stem = 100;
    }

    if(coeff.minPts_cluster_stem<3)
    {
        coeff.minPts_cluster_stem = 3;
    }

    if(coeff.minPts_cluster_branch<3)
    {
        coeff.minPts_cluster_branch = 3;
    }

    if(coeff.min_radius_sphere_branch<0.03)
    {
        coeff.min_radius_sphere_branch = 0.03;
    }

    if(coeff.min_radius_sphere_stem<0.07)
    {
        coeff.min_radius_sphere_stem = 0.07;
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
        std::normal_distribution<float> distribution_sphere_radius_multiplier (0,coefficients_sd.sphere_radius_multiplier/2);

        std::normal_distribution<float> distribution_epsilon_cluster_branch (0,coefficients_sd.epsilon_cluster_branch/2);
        std::normal_distribution<float> distribution_epsilon_cluster_stem (0,coefficients_sd.epsilon_cluster_stem/2);
        std::normal_distribution<float> distribution_epsilon_sphere (0,coefficients_sd.epsilon_sphere/2);
        std::normal_distribution<float> distribution_min_Pts_Ransac_Stem (0,coefficients_sd.minPts_ransac_stem/2);

        std::normal_distribution<float> distribution_minPts_cluster_stem (0,coefficients_sd.minPts_cluster_stem/2);
        std::normal_distribution<float> distribution_minPts_cluster_branch (0,coefficients_sd.minPts_cluster_branch/2);
        std::normal_distribution<float> distribution_min_radius_sphere_stem (0,coefficients_sd.min_radius_sphere_stem/2);
        std::normal_distribution<float> distribution_min_radius_sphere_branch (0,coefficients_sd.min_radius_sphere_branch/2);


        float update = distribution_epsilon_cluster_branch(generator);
        temp.epsilon_cluster_branch += update;

        update = distribution_sphere_radius_multiplier(generator);
        temp.sphere_radius_multiplier += update;


        update = distribution_epsilon_cluster_stem(generator);
        temp.epsilon_cluster_stem += update;

        update = distribution_epsilon_sphere(generator);
        temp.epsilon_sphere += update;

        int update_2 = distribution_min_Pts_Ransac_Stem(generator);
        temp.minPts_ransac_stem += update_2;

        update = distribution_min_radius_sphere_stem(generator);
        temp.min_radius_sphere_stem +=update;

        update = distribution_min_radius_sphere_branch(generator);
        temp.min_radius_sphere_branch += update;

        vec.push_back(temp);
    }
    return vec;
}



float Optimization::getFac() const
{
    return fac;
}

void Optimization::setFac(float value)
{
    fac = value;
}

float Optimization::getB() const
{
    return b;
}

void Optimization::setB(float value)
{
    b = value;
}

float Optimization::getA() const
{
    return a;
}

void Optimization::setA(float value)
{
    a = value;
}
std::vector<Method_Coefficients>
Optimization::neighboring_coefficients(int i)
{
    std::vector<Method_Coefficients> vec;
    coefficients_sd = coefficients_end;

    switch (i) {
    case 0:
    {
        float radius = coefficients_sd.sphere_radius_multiplier;
        coefficients_sd.sphere_radius_multiplier = radius * 0.5f;
        vec.push_back(coefficients_sd);
        coefficients_sd.sphere_radius_multiplier = radius * 0.7f;
        vec.push_back(coefficients_sd);
        coefficients_sd.sphere_radius_multiplier = radius * 0.85f;
        vec.push_back(coefficients_sd);
        coefficients_sd.sphere_radius_multiplier = radius * 1;
        vec.push_back(coefficients_sd);
        coefficients_sd.sphere_radius_multiplier = radius * 1.15f;
        vec.push_back(coefficients_sd);
        coefficients_sd.sphere_radius_multiplier = radius * 1.3f;
        vec.push_back(coefficients_sd);
        coefficients_sd.sphere_radius_multiplier = radius * 1.5f;
        vec.push_back(coefficients_sd);
        break;
    }
    case 1:
    {
        float epsilon_cluster_branch = coefficients_sd.epsilon_cluster_branch;
        coefficients_sd.epsilon_cluster_branch = epsilon_cluster_branch * 0.5f;
        vec.push_back(coefficients_sd);
        coefficients_sd.epsilon_cluster_branch = epsilon_cluster_branch * 0.7f;
        vec.push_back(coefficients_sd);
        coefficients_sd.epsilon_cluster_branch = epsilon_cluster_branch * 0.85f;
        vec.push_back(coefficients_sd);
        coefficients_sd.epsilon_cluster_branch = epsilon_cluster_branch * 1;
        vec.push_back(coefficients_sd);
        coefficients_sd.epsilon_cluster_branch = epsilon_cluster_branch * 1.15f;
        vec.push_back(coefficients_sd);
        coefficients_sd.epsilon_cluster_branch = epsilon_cluster_branch * 1.3f;
        vec.push_back(coefficients_sd);
        coefficients_sd.epsilon_cluster_branch = epsilon_cluster_branch * 1.5f;
        vec.push_back(coefficients_sd);
        break;
    }
    case 2:
    {
        float epsilon_cluster_stem = coefficients_sd.epsilon_cluster_stem;
        coefficients_sd.epsilon_cluster_stem = epsilon_cluster_stem * 0.5f;
        vec.push_back(coefficients_sd);
        coefficients_sd.epsilon_cluster_stem = epsilon_cluster_stem * 0.7f;
        vec.push_back(coefficients_sd);
        coefficients_sd.epsilon_cluster_stem = epsilon_cluster_stem * 0.85f;
        vec.push_back(coefficients_sd);
        coefficients_sd.epsilon_cluster_stem = epsilon_cluster_stem * 1;
        vec.push_back(coefficients_sd);
        coefficients_sd.epsilon_cluster_stem = epsilon_cluster_stem * 1.15f;
        vec.push_back(coefficients_sd);
        coefficients_sd.epsilon_cluster_stem = epsilon_cluster_stem * 1.3f;
        vec.push_back(coefficients_sd);
        coefficients_sd.epsilon_cluster_stem = epsilon_cluster_stem * 1.5f;
        vec.push_back(coefficients_sd);
        break;
    }
    case 3:
    {
        float epsilon_sphere = coefficients_sd.epsilon_sphere;
        coefficients_sd.epsilon_sphere = epsilon_sphere * 0.5f;
        vec.push_back(coefficients_sd);
        coefficients_sd.epsilon_sphere = epsilon_sphere * 0.7f;
        vec.push_back(coefficients_sd);
        coefficients_sd.epsilon_sphere = epsilon_sphere * 0.85f;
        vec.push_back(coefficients_sd);
        coefficients_sd.epsilon_sphere = epsilon_sphere * 1;
        vec.push_back(coefficients_sd);
        coefficients_sd.epsilon_sphere = epsilon_sphere * 1.15f;
        vec.push_back(coefficients_sd);
        coefficients_sd.epsilon_sphere = epsilon_sphere * 1.3f;
        vec.push_back(coefficients_sd);
        coefficients_sd.epsilon_sphere = epsilon_sphere * 1.5f;
        vec.push_back(coefficients_sd);
        break;
    }
    case 4:
    {
        float minPts_ransac_stem = coefficients_sd.minPts_ransac_stem;
        coefficients_sd.minPts_ransac_stem = minPts_ransac_stem * 0.5f;
        vec.push_back(coefficients_sd);
        coefficients_sd.minPts_ransac_stem = minPts_ransac_stem * 0.7f;
        vec.push_back(coefficients_sd);
        coefficients_sd.minPts_ransac_stem = minPts_ransac_stem * 0.85f;
        vec.push_back(coefficients_sd);
        coefficients_sd.minPts_ransac_stem = minPts_ransac_stem * 1;
        vec.push_back(coefficients_sd);
        coefficients_sd.minPts_ransac_stem = minPts_ransac_stem * 1.15f;
        vec.push_back(coefficients_sd);
        coefficients_sd.minPts_ransac_stem = minPts_ransac_stem * 1.3f;
        vec.push_back(coefficients_sd);
        coefficients_sd.minPts_ransac_stem = minPts_ransac_stem * 1.5f;
        vec.push_back(coefficients_sd);
        break;
    }
    case 5:
    {
        float min_radius_sphere_stem = coefficients_sd.min_radius_sphere_stem;
        coefficients_sd.min_radius_sphere_stem = min_radius_sphere_stem * 0.5f;
        vec.push_back(coefficients_sd);
        coefficients_sd.min_radius_sphere_stem = min_radius_sphere_stem * 0.7f;
        vec.push_back(coefficients_sd);
        coefficients_sd.min_radius_sphere_stem = min_radius_sphere_stem * 0.85f;
        vec.push_back(coefficients_sd);
        coefficients_sd.min_radius_sphere_stem = min_radius_sphere_stem * 1;
        vec.push_back(coefficients_sd);
        coefficients_sd.min_radius_sphere_stem = min_radius_sphere_stem * 1.15f;
        vec.push_back(coefficients_sd);
        coefficients_sd.min_radius_sphere_stem = min_radius_sphere_stem * 1.3f;
        vec.push_back(coefficients_sd);
        coefficients_sd.min_radius_sphere_stem = min_radius_sphere_stem * 1.5f;
        vec.push_back(coefficients_sd);
        break;
    }
    case 6:
    {
        float min_radius_sphere_branch = coefficients_sd.min_radius_sphere_branch;
        coefficients_sd.min_radius_sphere_branch = min_radius_sphere_branch * 0.5f;
        vec.push_back(coefficients_sd);
        coefficients_sd.min_radius_sphere_branch = min_radius_sphere_branch * 0.7f;
        vec.push_back(coefficients_sd);
        coefficients_sd.min_radius_sphere_branch = min_radius_sphere_branch * 0.85f;
        vec.push_back(coefficients_sd);
        coefficients_sd.min_radius_sphere_branch = min_radius_sphere_branch * 1;
        vec.push_back(coefficients_sd);
        coefficients_sd.min_radius_sphere_branch = min_radius_sphere_branch * 1.15f;
        vec.push_back(coefficients_sd);
        coefficients_sd.min_radius_sphere_branch = min_radius_sphere_branch * 1.3f;
        vec.push_back(coefficients_sd);
        coefficients_sd.min_radius_sphere_branch = min_radius_sphere_branch * 1.5f;
        vec.push_back(coefficients_sd);
        break;
    }
    default:
        break;
    }


    return vec;
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
    _current_seeds = 0;
    while (max_iterations>current_iteration&&min_dist<dist_update) {
        //emit emit_progress(0);

        std::vector<Method_Coefficients> seeds = generate_seeds(this->coefficients_end);
        for(std::vector<Method_Coefficients>::iterator kit = seeds.begin(); kit < seeds.end(); kit++)
        {
            Method_Coefficients seed = *kit;
            make_coefficients_positive(seed);
            WorkerSphereFollowing * worker = new WorkerSphereFollowing(1);
            // worker->set_control(this->controller);
            worker->setOptimize(shared_from_this());
            worker->set_coefficients(seed);
            worker->setA(coefficients_end.a);
            worker->setB(coefficients_end.b);
            worker->setFac(coefficients_end.fact);
            worker->setMinRad(coefficients_end.minRad);
            worker->setCloudPtr(_cloud_ptr);
            worker->setTreeID(treeID);
            worker->setIsStem(isStem);
            this->start(worker);
        }
        current_iteration ++;
        this->waitForDone();
        end_dist = current_dist;
        start_dist = end_dist;

        update_sd();
        coefficients_start = coefficients_end;


    }
    std::vector<float>
            minRadii;
    minRadii.push_back(0.015);
    minRadii.push_back(0.02);
    minRadii.push_back(0.025);
    minRadii.push_back(0.03);
    minRadii.push_back(0.035);
    minRadii.push_back(0.04);
    minRadii.push_back(0.045);

    if(!qFuzzyCompare(coefficients_start.minRad,0.0025f))
    {
        for(std::vector<float>::iterator f_it = minRadii.begin(); f_it < minRadii.end(); f_it++)
        {
            float radius = *f_it;
            WorkerSphereFollowing * worker = new WorkerSphereFollowing(1);
            // worker->set_control(this->controller);

            worker->setOptimize(shared_from_this());
            worker->set_coefficients(coefficients_start);
            worker->setA(coefficients_end.a);
            worker->setB(coefficients_end.b);
            worker->setFac(coefficients_end.fact);
            worker->setMinRad(radius);
            worker->setCloudPtr(_cloud_ptr);
            worker->setTreeID(treeID);
            worker->setIsStem(isStem);
            this->start(worker);
        }
    }
    this->waitForDone();
    end_dist = current_dist;
    start_dist = end_dist;
    update_sd();
    coefficients_start = coefficients_end;



}
