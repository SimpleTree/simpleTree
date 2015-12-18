#include "optimization_stem.h"

optimization_stem::optimization_stem(float min_height, float bin_width, float epsilon, PointCloudI::Ptr cloud)
{
    numberThreads = QThread::idealThreadCount()-1;
    this->setMaxThreadCount(std::max<int>(numberThreads,1));
    this->_min_height = min_height;
    this->_bin_width = bin_width;
    this->_epsilon = epsilon;
    this->_cloud = cloud;
}

optimization_stem::~optimization_stem()
{

}

void
optimization_stem::optimize()
{



    PointCloudI::Ptr temp_cloud (new PointCloudI);
    pcl::PassThrough<PointI> pass;
    pass.setInputCloud (_cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0, _min_height + 5);
    pass.filter (*temp_cloud);
    _cloud = temp_cloud;

    for(int i = 0; i < 10; i++)
    {
        for(int j = 0; j < 10; j++)
        {
            float bin_width = 0.5 + 0.5*i;
            float epsilon = 0.03 + 0.02*j;
            std::cout << "1" << std::endl;
            WorkerStemFit * worker = new WorkerStemFit(_cloud, _min_height, bin_width,epsilon);
            std::cout << "2" << std::endl;
            worker->set_optimization(shared_from_this());
            std::cout << "2.5" << std::endl;

            this->start(worker);
            std::cout << "3" << std::endl;

        }
    }
    this->waitForDone();





}

void
optimization_stem::updateCoeff(float dist, float bin_width, float epsilon)
{
    if(dist<_best_dist)
    {
        _best_dist = dist;
        _best_binwidth = bin_width;
        _best_epsilon = epsilon;
    }
}
