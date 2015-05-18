#ifndef OPTIMIZATION_H
#define OPTIMIZATION_H

#include <QThreadPool>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/enable_shared_from_this.hpp>

#include "../../src/method/method_coefficients.h"
#include "workerspherefollowing.h"
#include "../controller.h"

class Optimization : public QThreadPool,  public boost::enable_shared_from_this<Optimization>
{
    Q_OBJECT
private :
    Method_Coefficients coefficients_old, coefficients_new;
    float min_thresh = 0.001;
    float min_adjust = 0.0005;
    int max_iterations = 30;

    int numberTrials = 15;

    float current_thresh = 0.1;
    float current_adjust = 0.01;
    float current_iteration = 0;

    int numberThreads;

        QReadWriteLock lock;



        boost::shared_ptr<Controller> controller;
public:

        void
        setControl(boost::shared_ptr<Controller> control);


    void
    updateCoeff(Method_Coefficients coeff, float dist);

    boost::shared_ptr<simpleTree::Tree> tree_ptr;


    explicit Optimization(QObject *parent = 0);

    void
    setCoefficients(Method_Coefficients coeffcients);

    void
    optimize();



    std::vector<float> generateCoefficients(float coeff);


signals:

public slots:

};

#endif // OPTIMIZATION_H
