/*
 * down_hill_simplex.h
 *
 *  Created on: 15.06.2015
 *      Author: hackenberg
 */

#ifndef DOWN_HILL_SIMPLEX_H_
#define DOWN_HILL_SIMPLEX_H_

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/make_shared.hpp>
#include <QString>
#include <QThreadPool>

#include "../method_coefficients.h"
#include <pcl/console/time.h>
#include "../../Model/modelAdjustment/allometry.h"
#include "workerspherefollowing.h"



class DownHillSimplex: public QThreadPool,  public boost::enable_shared_from_this<DownHillSimplex> {
	Q_OBJECT
public:

//	DownHillSimplex();

//	virtual ~DownHillSimplex();


//	void
//	setCoefficients(Method_Coefficients & coeff);

//	void
//	set_error(int i, float error);

//	Method_Coefficients
//	getCoefficients();

//	void
//	optimize();

//	float getA() const {
//		return a;
//	}

//	void setA(float a) {
//		this->a = a;
//	}

//	float getB() const {
//		return b;
//	}

//	void setB(float b) {
//		this->b = b;
//	}


//	float getFac() const {
//		return fac;
//	}

//	void setFac(float fac) {
//		this->fac = fac;
//	}

//	const Method_Coefficients& getMax() const {
//		return max;
//	}

//	void setMax(const Method_Coefficients& max) {
//		this->max = max;
//	}

//	const Method_Coefficients& getMin() const {
//		return min;
//	}

//	void setMin(const Method_Coefficients& min) {
//		this->min = min;
//	}

//	const QString& getResultString() const {
//		return resultString;
//	}



//private:
//	float a,b;

//	float
//	rand(float a, float b);

//	float fac;

//	float alpha = 1;
//	float gamma = 2;
//	float rho = -0.5f;
//	float sigma = 0.5f;


//	Method_Coefficients min,max;
//	std::vector<std::pair<Method_Coefficients, float> > simplex;

//	std::pair<Method_Coefficients, float> reflection_coeff, expansion_coeff, contraction_coeff;

//	int max_iterations = 80;

////	float criterion = 0.0002f;

//	void
//	make_coefficients_positive(Method_Coefficients & coeff);


//	void
//	generate_start_simplex();

//	std::pair<Method_Coefficients, float>
//	evaluateCoefficients (Method_Coefficients coeff);

//	void
//	sortSimplex();

//	Method_Coefficients
//	computeCentroid();

//	Method_Coefficients
//	reflection(Method_Coefficients centroid);

//	Method_Coefficients
//	expansion(Method_Coefficients centroid);

//	Method_Coefficients
//	contraction(Method_Coefficients centroid);

//	void
//	reduction();



//	QString resultString;

};




#endif /* DOWN_HILL_SIMPLEX_H_ */
