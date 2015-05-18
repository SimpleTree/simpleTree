#ifndef WORKERSPHEREFOLLOWING_H
#define WORKERSPHEREFOLLOWING_H

#include <QRunnable>

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
    #include <QReadWriteLock>

#include "../../src/Model/modelAdjustment/allometry.h"

#include "../../src/method/method_coefficients.h"
#include "optimization.h"

class WorkerSphereFollowing : public QRunnable
{
    Q_OBJECT
public:
    boost::shared_ptr<Controller> control;
    boost::shared_ptr<Optimization> optimize;

    explicit WorkerSphereFollowing(QObject *parent = 0);

    Method_Coefficients coeff;
    float meanDist;

    void
    run();

    void
    setOptimize(boost::shared_ptr<Optimization> optimize);





signals:

public slots:

};

#endif // WORKERSPHEREFOLLOWING_H
