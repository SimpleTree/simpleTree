#ifndef WORKERSPHEREFOLLOWING_H
#define WORKERSPHEREFOLLOWING_H

#include <QRunnable>

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
    #include <QReadWriteLock>

#include "../../Model/modelAdjustment/allometry.h"
#include "down_hill_simplex.h"

#include "../method_coefficients.h"
#include "../SphereFollowing.h"


//class Optimization;
class Optimization;

class WorkerSphereFollowing : public QRunnable
{
private:
    float a,b,fac;
    float minRad = 0.0025;
    int i;



    std::vector<float>
    getAbsDistances(std::vector<float>);

    PointCloudI::Ptr _cloud_ptr;

    std::string treeID;

    std::vector<bool> isStem;

public:
    boost::shared_ptr<Optimization> optimize;

    explicit WorkerSphereFollowing(int i ,QObject *parent = 0);

    Method_Coefficients coefficients;
    float meanDist;

    void
    run();

    void
    setOptimize(boost::shared_ptr<Optimization> optimize);



    void
    set_coefficients(Method_Coefficients coefficients);




    void
    setMinRad(float minRad)
    {
    	this->minRad = minRad;
    }

    float getA() const;
    void setA(float value);
    float getB() const;
    void setB(float value);
    float getFac() const;
    void setFac(float value);

    float
    median(std::vector<float> values);

    float
    mean(std::vector<float> const &v);


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

};

#endif // WORKERSPHEREFOLLOWING_H
