#ifndef WORKERSPHEREFOLLOWING_H
#define WORKERSPHEREFOLLOWING_H

#include <QRunnable>

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
    #include <QReadWriteLock>

#include "../../Model/modelAdjustment/allometry.h"

#include "../method_coefficients.h"
#include "optimization.h"

class Optimization;


class WorkerSphereFollowing : public QRunnable
{
private:
    float a,b,fac;

public:
    boost::shared_ptr<Controller> control;
    boost::shared_ptr<Optimization> optimize;

    explicit WorkerSphereFollowing(QObject *parent = 0);

    Method_Coefficients coefficients;
    float meanDist;

    void
    run();

    void
    setOptimize(boost::shared_ptr<Optimization> optimize);

    void
    set_control(boost::shared_ptr<Controller> control);

    void
    set_coefficients(Method_Coefficients coefficients);





    float getA() const;
    void setA(float value);
    float getB() const;
    void setB(float value);
    float getFac() const;
    void setFac(float value);
};

#endif // WORKERSPHEREFOLLOWING_H
