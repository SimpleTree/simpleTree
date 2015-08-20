/*
 * down_hill_simplex.cpp
 *
 *  Created on: 15.06.2015
 *      Author: hackenberg
 */

#include "down_hill_simplex.h"

//DownHillSimplex::DownHillSimplex() {

//	int numberThreads = QThread::idealThreadCount()-1;
//	//    std::cout << "Optimization Constructor : number of threads :" << numberThreads << std::endl;
//	this->setMaxThreadCount(std::max<int>(numberThreads,1));

//}

//DownHillSimplex::~DownHillSimplex() {
//	// TODO Auto-generated destructor stub
//}

//void
//DownHillSimplex::set_error(int i, float error)
//{
//	error = std::abs(error);
//	if(i>=0 && i <= 10)
//	{
//		simplex.at(i).second = error;
//	} else if (i == -1)

//	{
//		reflection_coeff.second = error;

//	} else if (i == -2)

//	{
//		expansion_coeff.second = error;

//	}else if (i == -3)

//	{
//		contraction_coeff.second = error;

//	}else {
//		std::cout << "Critical error in DownHillSimplex::set_error" << std::endl;
//	}
//}

//float
//DownHillSimplex::rand(float a, float b)
//{
//	int i = std::rand()%98;
//	float perc = i/100.0f;
//	float rnd  = a - perc*(b-a);
//	return rnd;
//}

//void
//DownHillSimplex::generate_start_simplex() {
//	simplex.resize(11);
//	int i = 0;
//	{
//		Method_Coefficients coeff = min;

//		simplex.at(i).first = coeff;
//		make_coefficients_positive(coeff);
//		WorkerSphereFollowing * worker = new WorkerSphereFollowing(i);
//		worker->set_control(this->getControl());
//		worker->setOptimize(shared_from_this());
//		worker->set_coefficients(coeff);
//		worker->setA(a);
//		worker->setB(b);
//		worker->setFac(fac);
//		this->start(worker);
//		//simplex.push_back(evaluateCoefficients(coeff) );
//	}

//	{
//		assert(min.epsilon_sphere!= max.epsilon_sphere);
//		Method_Coefficients coeff = min;
//		coeff.epsilon_sphere = rand(min.epsilon_sphere,max.epsilon_sphere);
//		coeff.epsilon_cluster_branch = rand(min.epsilon_cluster_branch,max.epsilon_cluster_branch);
//		coeff.epsilon_cluster_stem = rand(min.epsilon_cluster_stem,max.epsilon_cluster_stem);
//		coeff.minPts_cluster_branch = rand(min.minPts_cluster_branch,max.minPts_cluster_branch);
//		coeff.minPts_cluster_stem = rand(min.minPts_cluster_stem,max.minPts_cluster_stem);
//		coeff.minPts_ransac_branch = rand(min.minPts_ransac_branch,max.minPts_ransac_branch);
//		coeff.minPts_ransac_stem = rand(min.minPts_ransac_stem,max.minPts_ransac_stem);
//		coeff.min_radius_sphere_branch = rand(min.min_radius_sphere_branch,max.min_radius_sphere_branch);
//		coeff.min_radius_sphere_stem = rand(min.min_radius_sphere_stem,max.min_radius_sphere_stem);
//		coeff.sphere_radius_multiplier = rand(min.sphere_radius_multiplier,max.sphere_radius_multiplier);

//		coeff.epsilon_sphere = max.epsilon_sphere;
//		i++;
//		simplex.at(i).first = coeff;
//		make_coefficients_positive(coeff);
//		WorkerSphereFollowing * worker = new WorkerSphereFollowing(i);
//		worker->set_control(this->getControl());
////		worker->setOptimize(shared_from_this());
//		worker->set_coefficients(coeff);
//		worker->setA(a);
//		worker->setB(b);
//		worker->setFac(fac);
//		this->start(worker);
//		//		simplex.push_back(evaluateCoefficients(coeff) );
//	}

//	{
//		assert(min.epsilon_cluster_branch!= max.epsilon_cluster_branch);
//		Method_Coefficients coeff = min;
//		coeff.epsilon_sphere = rand(min.epsilon_sphere,max.epsilon_sphere);
//		coeff.epsilon_cluster_branch = rand(min.epsilon_cluster_branch,max.epsilon_cluster_branch);
//		coeff.epsilon_cluster_stem = rand(min.epsilon_cluster_stem,max.epsilon_cluster_stem);
//		coeff.minPts_cluster_branch = rand(min.minPts_cluster_branch,max.minPts_cluster_branch);
//		coeff.minPts_cluster_stem = rand(min.minPts_cluster_stem,max.minPts_cluster_stem);
//		coeff.minPts_ransac_branch = rand(min.minPts_ransac_branch,max.minPts_ransac_branch);
//		coeff.minPts_ransac_stem = rand(min.minPts_ransac_stem,max.minPts_ransac_stem);
//		coeff.min_radius_sphere_branch = rand(min.min_radius_sphere_branch,max.min_radius_sphere_branch);
//		coeff.min_radius_sphere_stem = rand(min.min_radius_sphere_stem,max.min_radius_sphere_stem);
//		coeff.sphere_radius_multiplier = rand(min.sphere_radius_multiplier,max.sphere_radius_multiplier);
//		coeff.epsilon_cluster_branch = max.epsilon_cluster_branch;
//		i++;
//		simplex.at(i).first = coeff;
//		make_coefficients_positive(coeff);
//		WorkerSphereFollowing * worker = new WorkerSphereFollowing(i);
//		worker->set_control(this->getControl());
////		worker->setOptimize(shared_from_this());
//		worker->set_coefficients(coeff);
//		worker->setA(a);
//		worker->setB(b);
//		worker->setFac(fac);
//		this->start(worker);
//		//		simplex.push_back(evaluateCoefficients(coeff) );
//	}

//	{
//		assert(min.epsilon_cluster_stem!=max.epsilon_cluster_stem);
//		Method_Coefficients coeff = min;
//		coeff.epsilon_sphere = rand(min.epsilon_sphere,max.epsilon_sphere);
//		coeff.epsilon_cluster_branch = rand(min.epsilon_cluster_branch,max.epsilon_cluster_branch);
//		coeff.epsilon_cluster_stem = rand(min.epsilon_cluster_stem,max.epsilon_cluster_stem);
//		coeff.minPts_cluster_branch = rand(min.minPts_cluster_branch,max.minPts_cluster_branch);
//		coeff.minPts_cluster_stem = rand(min.minPts_cluster_stem,max.minPts_cluster_stem);
//		coeff.minPts_ransac_branch = rand(min.minPts_ransac_branch,max.minPts_ransac_branch);
//		coeff.minPts_ransac_stem = rand(min.minPts_ransac_stem,max.minPts_ransac_stem);
//		coeff.min_radius_sphere_branch = rand(min.min_radius_sphere_branch,max.min_radius_sphere_branch);
//		coeff.min_radius_sphere_stem = rand(min.min_radius_sphere_stem,max.min_radius_sphere_stem);
//		coeff.sphere_radius_multiplier = rand(min.sphere_radius_multiplier,max.sphere_radius_multiplier);
//		coeff.epsilon_cluster_stem = max.epsilon_cluster_branch;
//		i++;
//		simplex.at(i).first = coeff;
//		make_coefficients_positive(coeff);
//		WorkerSphereFollowing * worker = new WorkerSphereFollowing(i);
//		worker->set_control(this->getControl());
////		worker->setOptimize(shared_from_this());
//		worker->set_coefficients(coeff);
//		worker->setA(a);
//		worker->setB(b);
//		worker->setFac(fac);
//		this->start(worker);
//		//		simplex.push_back(evaluateCoefficients(coeff) );
//	}

//	{
//		assert(min.minPts_cluster_branch!=max.minPts_cluster_branch);
//		Method_Coefficients coeff = min;
//		coeff.epsilon_sphere = rand(min.epsilon_sphere,max.epsilon_sphere);
//		coeff.epsilon_cluster_branch = rand(min.epsilon_cluster_branch,max.epsilon_cluster_branch);
//		coeff.epsilon_cluster_stem = rand(min.epsilon_cluster_stem,max.epsilon_cluster_stem);
//		coeff.minPts_cluster_branch = rand(min.minPts_cluster_branch,max.minPts_cluster_branch);
//		coeff.minPts_cluster_stem = rand(min.minPts_cluster_stem,max.minPts_cluster_stem);
//		coeff.minPts_ransac_branch = rand(min.minPts_ransac_branch,max.minPts_ransac_branch);
//		coeff.minPts_ransac_stem = rand(min.minPts_ransac_stem,max.minPts_ransac_stem);
//		coeff.min_radius_sphere_branch = rand(min.min_radius_sphere_branch,max.min_radius_sphere_branch);
//		coeff.min_radius_sphere_stem = rand(min.min_radius_sphere_stem,max.min_radius_sphere_stem);
//		coeff.sphere_radius_multiplier = rand(min.sphere_radius_multiplier,max.sphere_radius_multiplier);
//		coeff.minPts_cluster_branch = max.minPts_cluster_branch;
//		i++;
//		simplex.at(i).first = coeff;
//		make_coefficients_positive(coeff);
//		WorkerSphereFollowing * worker = new WorkerSphereFollowing(i);
//		worker->set_control(this->getControl());
////		worker->setOptimize(shared_from_this());
//		worker->set_coefficients(coeff);
//		worker->setA(a);
//		worker->setB(b);
//		worker->setFac(fac);
//		this->start(worker);
//		//		simplex.push_back(evaluateCoefficients(coeff) );
//	}

//	{
//		assert(min.minPts_cluster_stem!=max.minPts_cluster_stem);
//		Method_Coefficients coeff = min;
//		coeff.epsilon_sphere = rand(min.epsilon_sphere,max.epsilon_sphere);
//		coeff.epsilon_cluster_branch = rand(min.epsilon_cluster_branch,max.epsilon_cluster_branch);
//		coeff.epsilon_cluster_stem = rand(min.epsilon_cluster_stem,max.epsilon_cluster_stem);
//		coeff.minPts_cluster_branch = rand(min.minPts_cluster_branch,max.minPts_cluster_branch);
//		coeff.minPts_cluster_stem = rand(min.minPts_cluster_stem,max.minPts_cluster_stem);
//		coeff.minPts_ransac_branch = rand(min.minPts_ransac_branch,max.minPts_ransac_branch);
//		coeff.minPts_ransac_stem = rand(min.minPts_ransac_stem,max.minPts_ransac_stem);
//		coeff.min_radius_sphere_branch = rand(min.min_radius_sphere_branch,max.min_radius_sphere_branch);
//		coeff.min_radius_sphere_stem = rand(min.min_radius_sphere_stem,max.min_radius_sphere_stem);
//		coeff.sphere_radius_multiplier = rand(min.sphere_radius_multiplier,max.sphere_radius_multiplier);
//		coeff.minPts_cluster_stem = max.minPts_cluster_stem;
//		i++;
//		simplex.at(i).first = coeff;
//		make_coefficients_positive(coeff);
//		WorkerSphereFollowing * worker = new WorkerSphereFollowing(i);
//		worker->set_control(this->getControl());
////		worker->setOptimize(shared_from_this());
//		worker->set_coefficients(coeff);
//		worker->setA(a);
//		worker->setB(b);
//		worker->setFac(fac);
//		this->start(worker);
//		//		simplex.push_back(evaluateCoefficients(coeff) );
//	}
//	{
//		assert(min.minPts_ransac_branch!=max.minPts_ransac_branch);
//		Method_Coefficients coeff = min;
//		coeff.epsilon_sphere = rand(min.epsilon_sphere,max.epsilon_sphere);
//		coeff.epsilon_cluster_branch = rand(min.epsilon_cluster_branch,max.epsilon_cluster_branch);
//		coeff.epsilon_cluster_stem = rand(min.epsilon_cluster_stem,max.epsilon_cluster_stem);
//		coeff.minPts_cluster_branch = rand(min.minPts_cluster_branch,max.minPts_cluster_branch);
//		coeff.minPts_cluster_stem = rand(min.minPts_cluster_stem,max.minPts_cluster_stem);
//		coeff.minPts_ransac_branch = rand(min.minPts_ransac_branch,max.minPts_ransac_branch);
//		coeff.minPts_ransac_stem = rand(min.minPts_ransac_stem,max.minPts_ransac_stem);
//		coeff.min_radius_sphere_branch = rand(min.min_radius_sphere_branch,max.min_radius_sphere_branch);
//		coeff.min_radius_sphere_stem = rand(min.min_radius_sphere_stem,max.min_radius_sphere_stem);
//		coeff.sphere_radius_multiplier = rand(min.sphere_radius_multiplier,max.sphere_radius_multiplier);
//		coeff.minPts_ransac_branch = max.minPts_ransac_branch;
//		i++;
//		simplex.at(i).first = coeff;
//		make_coefficients_positive(coeff);
//		WorkerSphereFollowing * worker = new WorkerSphereFollowing(i);
//		worker->set_control(this->getControl());
////		worker->setOptimize(shared_from_this());
//		worker->set_coefficients(coeff);
//		worker->setA(a);
//		worker->setB(b);
//		worker->setFac(fac);
//		this->start(worker);
//		//		simplex.push_back(evaluateCoefficients(coeff) );
//	}

//	{
//		assert(min.minPts_ransac_stem!=max.minPts_ransac_stem);
//		Method_Coefficients coeff = min;
//		coeff.epsilon_sphere = rand(min.epsilon_sphere,max.epsilon_sphere);
//		coeff.epsilon_cluster_branch = rand(min.epsilon_cluster_branch,max.epsilon_cluster_branch);
//		coeff.epsilon_cluster_stem = rand(min.epsilon_cluster_stem,max.epsilon_cluster_stem);
//		coeff.minPts_cluster_branch = rand(min.minPts_cluster_branch,max.minPts_cluster_branch);
//		coeff.minPts_cluster_stem = rand(min.minPts_cluster_stem,max.minPts_cluster_stem);
//		coeff.minPts_ransac_branch = rand(min.minPts_ransac_branch,max.minPts_ransac_branch);
//		coeff.minPts_ransac_stem = rand(min.minPts_ransac_stem,max.minPts_ransac_stem);
//		coeff.min_radius_sphere_branch = rand(min.min_radius_sphere_branch,max.min_radius_sphere_branch);
//		coeff.min_radius_sphere_stem = rand(min.min_radius_sphere_stem,max.min_radius_sphere_stem);
//		coeff.sphere_radius_multiplier = rand(min.sphere_radius_multiplier,max.sphere_radius_multiplier);
//		coeff.minPts_ransac_stem = max.minPts_ransac_stem;
//		i++;
//		simplex.at(i).first = coeff;
//		make_coefficients_positive(coeff);
//		WorkerSphereFollowing * worker = new WorkerSphereFollowing(i);
//		worker->set_control(this->getControl());
////		worker->setOptimize(shared_from_this());
//		worker->set_coefficients(coeff);
//		worker->setA(a);
//		worker->setB(b);
//		worker->setFac(fac);
//		this->start(worker);
//		//		simplex.push_back(evaluateCoefficients(coeff) );
//	}

//	{
//		assert(min.min_radius_sphere_branch!=max.min_radius_sphere_branch);
//		Method_Coefficients coeff = min;
//		coeff.epsilon_sphere = rand(min.epsilon_sphere,max.epsilon_sphere);
//		coeff.epsilon_cluster_branch = rand(min.epsilon_cluster_branch,max.epsilon_cluster_branch);
//		coeff.epsilon_cluster_stem = rand(min.epsilon_cluster_stem,max.epsilon_cluster_stem);
//		coeff.minPts_cluster_branch = rand(min.minPts_cluster_branch,max.minPts_cluster_branch);
//		coeff.minPts_cluster_stem = rand(min.minPts_cluster_stem,max.minPts_cluster_stem);
//		coeff.minPts_ransac_branch = rand(min.minPts_ransac_branch,max.minPts_ransac_branch);
//		coeff.minPts_ransac_stem = rand(min.minPts_ransac_stem,max.minPts_ransac_stem);
//		coeff.min_radius_sphere_branch = rand(min.min_radius_sphere_branch,max.min_radius_sphere_branch);
//		coeff.min_radius_sphere_stem = rand(min.min_radius_sphere_stem,max.min_radius_sphere_stem);
//		coeff.sphere_radius_multiplier = rand(min.sphere_radius_multiplier,max.sphere_radius_multiplier);
//		coeff.min_radius_sphere_branch = max.min_radius_sphere_branch;
//		i++;
//		simplex.at(i).first = coeff;
//		make_coefficients_positive(coeff);
//		WorkerSphereFollowing * worker = new WorkerSphereFollowing(i);
//		worker->set_control(this->getControl());
////		worker->setOptimize(shared_from_this());
//		worker->set_coefficients(coeff);
//		worker->setA(a);
//		worker->setB(b);
//		worker->setFac(fac);
//		this->start(worker);
//		//		simplex.push_back(evaluateCoefficients(coeff) );
//	}

//	{
//		assert(min.min_radius_sphere_stem!=max.min_radius_sphere_stem);
//		Method_Coefficients coeff = min;
//		coeff.epsilon_sphere = rand(min.epsilon_sphere,max.epsilon_sphere);
//		coeff.epsilon_cluster_branch = rand(min.epsilon_cluster_branch,max.epsilon_cluster_branch);
//		coeff.epsilon_cluster_stem = rand(min.epsilon_cluster_stem,max.epsilon_cluster_stem);
//		coeff.minPts_cluster_branch = rand(min.minPts_cluster_branch,max.minPts_cluster_branch);
//		coeff.minPts_cluster_stem = rand(min.minPts_cluster_stem,max.minPts_cluster_stem);
//		coeff.minPts_ransac_branch = rand(min.minPts_ransac_branch,max.minPts_ransac_branch);
//		coeff.minPts_ransac_stem = rand(min.minPts_ransac_stem,max.minPts_ransac_stem);
//		coeff.min_radius_sphere_branch = rand(min.min_radius_sphere_branch,max.min_radius_sphere_branch);
//		coeff.min_radius_sphere_stem = rand(min.min_radius_sphere_stem,max.min_radius_sphere_stem);
//		coeff.sphere_radius_multiplier = rand(min.sphere_radius_multiplier,max.sphere_radius_multiplier);
//		coeff.min_radius_sphere_stem = max.min_radius_sphere_stem;
//		i++;
//		simplex.at(i).first = coeff;

//		make_coefficients_positive(coeff);
//		WorkerSphereFollowing * worker = new WorkerSphereFollowing(i);
//		worker->set_control(this->getControl());
////		worker->setOptimize(shared_from_this());
//		worker->set_coefficients(coeff);
//		worker->setA(a);
//		worker->setB(b);
//		worker->setFac(fac);
//		this->start(worker);
//		//		simplex.push_back(evaluateCoefficients(coeff) );
//	}

//	{
//		assert(min.sphere_radius_multiplier!=max.sphere_radius_multiplier);
//		Method_Coefficients coeff = min;
//		coeff.epsilon_sphere = rand(min.epsilon_sphere,max.epsilon_sphere);
//		coeff.epsilon_cluster_branch = rand(min.epsilon_cluster_branch,max.epsilon_cluster_branch);
//		coeff.epsilon_cluster_stem = rand(min.epsilon_cluster_stem,max.epsilon_cluster_stem);
//		coeff.minPts_cluster_branch = rand(min.minPts_cluster_branch,max.minPts_cluster_branch);
//		coeff.minPts_cluster_stem = rand(min.minPts_cluster_stem,max.minPts_cluster_stem);
//		coeff.minPts_ransac_branch = rand(min.minPts_ransac_branch,max.minPts_ransac_branch);
//		coeff.minPts_ransac_stem = rand(min.minPts_ransac_stem,max.minPts_ransac_stem);
//		coeff.min_radius_sphere_branch = rand(min.min_radius_sphere_branch,max.min_radius_sphere_branch);
//		coeff.min_radius_sphere_stem = rand(min.min_radius_sphere_stem,max.min_radius_sphere_stem);
//		coeff.sphere_radius_multiplier = rand(min.sphere_radius_multiplier,max.sphere_radius_multiplier);
//		coeff.sphere_radius_multiplier = max.sphere_radius_multiplier;
//		i++;
//		simplex.at(i).first = coeff;
//		make_coefficients_positive(coeff);
//		WorkerSphereFollowing * worker = new WorkerSphereFollowing(i);
//		worker->set_control(this->getControl());
////		worker->setOptimize(shared_from_this());
//		worker->set_coefficients(coeff);
//		worker->setA(a);
//		worker->setB(b);
//		worker->setFac(fac);
//		this->start(worker);
//		//		simplex.push_back(evaluateCoefficients(coeff) );
//	}
//	this->waitForDone();

//	assert(simplex.size()==11);

//}

//void
//DownHillSimplex::make_coefficients_positive(Method_Coefficients & coeff)
//{

//	if(coeff.sphere_radius_multiplier<1.2)
//	{
//		coeff.sphere_radius_multiplier = 1.2;
//	}

//	if(coeff.epsilon_cluster_branch<0.01)
//	{
//		coeff.epsilon_cluster_branch = 0.01;
//	}

//	if(coeff.epsilon_cluster_stem<0.02)
//	{
//		coeff.epsilon_cluster_stem = 0.02;
//	}

//	if(coeff.epsilon_sphere<0.01)
//	{
//		coeff.epsilon_sphere = 0.01;
//	}



//	//    if(coeff.epsilon_sphere>0.03)
//	//    {
//	//        coeff.epsilon_sphere = 0.03;
//	//    }

//	if(coeff.minPts_ransac_stem<10)
//	{
//		coeff.minPts_ransac_stem = 10;
//	}

//	if(coeff.minPts_ransac_branch<10)
//	{
//		coeff.minPts_ransac_branch = 10;
//	}

//	if(coeff.minPts_cluster_stem<3)
//	{
//		coeff.minPts_cluster_stem = 3;
//	}

//	if(coeff.minPts_cluster_branch<3)
//	{
//		coeff.minPts_cluster_branch = 3;
//	}

//	//	if(coeff.min_radius_sphere_branch<0.03)
//	//	{
//	//		coeff.min_radius_sphere_branch = 0.03;
//	//	}
//	//
//	//	if(coeff.min_radius_sphere_stem<0.07)
//	//	{
//	//		coeff.min_radius_sphere_stem = 0.07;
//	//	}
//}


//Method_Coefficients
//DownHillSimplex::getCoefficients()
//{
//	return simplex.at(0).first;
//}

//boost::shared_ptr<Controller>
//DownHillSimplex::get_control()
//{
//	return this->control.lock();
//}

//void
//DownHillSimplex::reduction()
//{
//	for(size_t i = 1; i < simplex.size(); i ++)
//	{
//		Method_Coefficients coeff = simplex.at(0).first;
//		coeff.epsilon_cluster_branch += sigma*(simplex.at(i).first.epsilon_cluster_branch - simplex.at(0).first.epsilon_cluster_branch);
//		coeff.epsilon_cluster_stem += sigma*(simplex.at(i).first.epsilon_cluster_stem - simplex.at(0).first.epsilon_cluster_stem);
//		coeff.epsilon_sphere += sigma*(simplex.at(i).first.epsilon_sphere - simplex.at(0).first.epsilon_sphere);
//		coeff.minPts_cluster_branch += sigma*(simplex.at(i).first.minPts_cluster_branch - simplex.at(0).first.minPts_cluster_branch);
//		coeff.minPts_cluster_stem += sigma*(simplex.at(i).first.minPts_cluster_stem - simplex.at(0).first.minPts_cluster_stem);
//		coeff.minPts_ransac_branch += sigma*(simplex.at(i).first.minPts_ransac_branch - simplex.at(0).first.minPts_ransac_branch);
//		coeff.minPts_ransac_stem += sigma*(simplex.at(i).first.minPts_ransac_stem - simplex.at(0).first.minPts_ransac_stem);
//		coeff.min_radius_sphere_branch += sigma*(simplex.at(i).first.min_radius_sphere_branch - simplex.at(0).first.min_radius_sphere_branch);
//		coeff.min_radius_sphere_stem += sigma*(simplex.at(i).first.min_radius_sphere_stem - simplex.at(0).first.min_radius_sphere_stem);
//		coeff.sphere_radius_multiplier += sigma*(simplex.at(i).first.sphere_radius_multiplier - simplex.at(0).first.sphere_radius_multiplier);
//		make_coefficients_positive(coeff);
//		WorkerSphereFollowing * worker = new WorkerSphereFollowing(i);
//		worker->set_control(this->getControl());
////		worker->setOptimize(shared_from_this());
//		worker->set_coefficients(coeff);
//		worker->setA(a);
//		worker->setB(b);
//		worker->setFac(fac);
//		this->start(worker);
//		simplex.at(i).first = coeff;
//		//		simplex.at(i) = evaluateCoefficients(coeff);
//	}
//	waitForDone();
//}

//Method_Coefficients
//DownHillSimplex::contraction(Method_Coefficients centroid){

//	Method_Coefficients coeff = centroid;
//	coeff.epsilon_cluster_branch += rho*(centroid.epsilon_cluster_branch - simplex.at(10).first.epsilon_cluster_branch);
//	coeff.epsilon_cluster_stem += rho*(centroid.epsilon_cluster_stem - simplex.at(10).first.epsilon_cluster_stem);
//	coeff.epsilon_sphere += rho*(centroid.epsilon_sphere - simplex.at(10).first.epsilon_sphere);
//	coeff.minPts_cluster_branch += rho*(centroid.minPts_cluster_branch - simplex.at(10).first.minPts_cluster_branch);
//	coeff.minPts_cluster_stem += rho*(centroid.minPts_cluster_stem - simplex.at(10).first.minPts_cluster_stem);
//	coeff.minPts_ransac_branch += rho*(centroid.minPts_ransac_branch - simplex.at(10).first.minPts_ransac_branch);
//	coeff.minPts_ransac_stem += rho*(centroid.minPts_ransac_stem - simplex.at(10).first.minPts_ransac_stem);
//	coeff.min_radius_sphere_branch += rho*(centroid.min_radius_sphere_branch - simplex.at(10).first.min_radius_sphere_branch);
//	coeff.min_radius_sphere_stem += rho*(centroid.min_radius_sphere_stem - simplex.at(10).first.min_radius_sphere_stem);
//	coeff.sphere_radius_multiplier += rho*(centroid.sphere_radius_multiplier - simplex.at(10).first.sphere_radius_multiplier);
//	make_coefficients_positive(coeff);
//	return coeff;
//}


//Method_Coefficients
//DownHillSimplex::expansion(Method_Coefficients centroid){

//	Method_Coefficients coeff = centroid;
//	coeff.epsilon_cluster_branch += gamma*(centroid.epsilon_cluster_branch - simplex.at(10).first.epsilon_cluster_branch);
//	coeff.epsilon_cluster_stem += gamma*(centroid.epsilon_cluster_stem - simplex.at(10).first.epsilon_cluster_stem);
//	coeff.epsilon_sphere += gamma*(centroid.epsilon_sphere - simplex.at(10).first.epsilon_sphere);
//	coeff.minPts_cluster_branch += gamma*(centroid.minPts_cluster_branch - simplex.at(10).first.minPts_cluster_branch);
//	coeff.minPts_cluster_stem += gamma*(centroid.minPts_cluster_stem - simplex.at(10).first.minPts_cluster_stem);
//	coeff.minPts_ransac_branch += gamma*(centroid.minPts_ransac_branch - simplex.at(10).first.minPts_ransac_branch);
//	coeff.minPts_ransac_stem += gamma*(centroid.minPts_ransac_stem - simplex.at(10).first.minPts_ransac_stem);
//	coeff.min_radius_sphere_branch += gamma*(centroid.min_radius_sphere_branch - simplex.at(10).first.min_radius_sphere_branch);
//	coeff.min_radius_sphere_stem += gamma*(centroid.min_radius_sphere_stem - simplex.at(10).first.min_radius_sphere_stem);
//	coeff.sphere_radius_multiplier += gamma*(centroid.sphere_radius_multiplier - simplex.at(10).first.sphere_radius_multiplier);
//	make_coefficients_positive(coeff);
//	return coeff;
//}


//Method_Coefficients
//DownHillSimplex::reflection(Method_Coefficients centroid){

//	Method_Coefficients coeff = centroid;
//	coeff.epsilon_cluster_branch += alpha*(centroid.epsilon_cluster_branch - simplex.at(10).first.epsilon_cluster_branch);
//	coeff.epsilon_cluster_stem += alpha*(centroid.epsilon_cluster_stem - simplex.at(10).first.epsilon_cluster_stem);
//	coeff.epsilon_sphere += alpha*(centroid.epsilon_sphere - simplex.at(10).first.epsilon_sphere);
//	coeff.minPts_cluster_branch += alpha*(centroid.minPts_cluster_branch - simplex.at(10).first.minPts_cluster_branch);
//	coeff.minPts_cluster_stem += alpha*(centroid.minPts_cluster_stem - simplex.at(10).first.minPts_cluster_stem);
//	coeff.minPts_ransac_branch += alpha*(centroid.minPts_ransac_branch - simplex.at(10).first.minPts_ransac_branch);
//	coeff.minPts_ransac_stem += alpha*(centroid.minPts_ransac_stem - simplex.at(10).first.minPts_ransac_stem);
//	coeff.min_radius_sphere_branch += alpha*(centroid.min_radius_sphere_branch - simplex.at(10).first.min_radius_sphere_branch);
//	coeff.min_radius_sphere_stem += alpha*(centroid.min_radius_sphere_stem - simplex.at(10).first.min_radius_sphere_stem);
//	coeff.sphere_radius_multiplier += alpha*(centroid.sphere_radius_multiplier - simplex.at(10).first.sphere_radius_multiplier);
//	make_coefficients_positive(coeff);
//	return coeff;
//}

//void
//DownHillSimplex::optimize()
//{
//	float iterations = 0;
//	generate_start_simplex();
//	while (iterations<max_iterations)
//	{
//		sortSimplex();
//		resultString.append ("In the ").append(QString::number(iterations + 1)).
//				append("th Iteration the best found threshold set leads to").
//				append(QString::number(simplex.at(0).second)).append(" value of fit.\n");
//		Method_Coefficients centroid = computeCentroid();
//		//std::cout << " controid "<<centroid.toQString().toStdString() << std::endl;
//		Method_Coefficients reflect = reflection(centroid);
//		//std::cout << reflect.toQString().toStdString() << std::endl;
//		Method_Coefficients expand = expansion(centroid);
//		//	std::cout << expand.toQString().toStdString() << std::endl;
//		Method_Coefficients contract = contraction(centroid);
//		//		std::cout << contract.toQString().toStdString() << std::endl;

//		//		std::pair<Method_Coefficients,float> reflection;
//		reflection_coeff.first = reflect;
//		//		std::pair<Method_Coefficients,float> expansion;
//		expansion_coeff.first = expand;
//		//		std::pair<Method_Coefficients,float> contraction;
//		contraction_coeff.first = contract;

//		{
//			//make_coefficients_positive(reflect);
//			WorkerSphereFollowing * worker = new WorkerSphereFollowing(-1);
//			worker->set_control(this->getControl());
////			worker->setOptimize(shared_from_this());
//			worker->set_coefficients(reflect);
//			worker->setA(a);
//			worker->setB(b);
//			worker->setFac(fac);
//			this->start(worker);
//		}
//		{
//			//make_coefficients_positive(expand);
//			WorkerSphereFollowing * worker = new WorkerSphereFollowing(-2);
//			worker->set_control(this->getControl());
////			worker->setOptimize(shared_from_this());
//			worker->set_coefficients(expand);
//			worker->setA(a);
//			worker->setB(b);
//			worker->setFac(fac);
//			this->start(worker);
//		}
//		{
//			//make_coefficients_positive(contract);
//			WorkerSphereFollowing * worker = new WorkerSphereFollowing(-3);
//			worker->set_control(this->getControl());
////			worker->setOptimize(shared_from_this());
//			worker->set_coefficients(contract);
//			worker->setA(a);
//			worker->setB(b);
//			worker->setFac(fac);
//			this->start(worker);
//		}


//		//	std::cout << "wait" << std::endl;
//		waitForDone();
//		//		std::cout << "done" << std::endl;
//		std::cout << " -------------------Iteration " <<  iterations  <<" ----------------------" << std::endl;
//		std::cout << " Reflection : " <<  reflection_coeff.second << std::endl;
//		std::cout << " Expansion : " <<  expansion_coeff.second << std::endl;
//		std::cout << " Contraction : " <<  reflection_coeff.second << std::endl;
//		std::cout << " best : " << simplex.at(0).second << std::endl;
//		std::cout << " worst : " << simplex.at(10).second << std::endl;
//		std::cout << " --------------------------------------------------------" << std::endl;




//		if((simplex.at(0).second<=reflection_coeff.second) && (reflection_coeff.second< simplex.at(10).second))
//		{
//			simplex.at(10) = reflection_coeff;
//			std::cout << "reflection" << std::endl;

//		} else if(reflection_coeff.second<simplex.at(0).second)
//		{
//			//			Method_Coefficients expand = expansion(centroid);
//			//			std::pair<Method_Coefficients,float> expansion = evaluateCoefficients(expand);
//			if(expansion_coeff.second<reflection_coeff.second)
//			{
//				simplex.at(10) = expansion_coeff;
//				std::cout << "expansion" << std::endl;
//			} else {
//				simplex.at(10) = reflection_coeff;
//				std::cout << "reflection" << std::endl;
//			}
//		} else {
//			//			Method_Coefficients contract = contraction(centroid);

//			if(contraction_coeff.second<simplex.at(10).second){
//				simplex.at(10) = contraction_coeff;
//				std::cout << "contraction" << std::endl;
//			} else {
//				reduction();
//				std::cout << "reduction" << std::endl;
//			}
//		}
//		iterations ++;
//	}
//	sortSimplex();
//	resultString.append("The final thresholdset :").append(simplex.at(0).first.toQString());
//	std::cout << resultString.toStdString() << std::endl;
//}

//void
//DownHillSimplex::setCoefficients(Method_Coefficients & coeff)
//{
//	min.epsilon_cluster_branch = coeff.epsilon_cluster_branch;
//	max.epsilon_cluster_branch = 1.5*coeff.epsilon_cluster_branch;

//	min.epsilon_cluster_stem = coeff.epsilon_cluster_stem;
//	max.epsilon_cluster_stem = 1.5*coeff.epsilon_cluster_stem;

//	min.epsilon_sphere = coeff.epsilon_sphere;
//	max.epsilon_sphere = 1.5*coeff.epsilon_sphere;

//	min.minPts_cluster_branch = coeff.minPts_cluster_branch;
//	max.minPts_cluster_branch = 3*coeff.minPts_cluster_branch;

//	min.minPts_cluster_stem = coeff.minPts_cluster_stem;
//	max.minPts_cluster_stem = 3*coeff.minPts_cluster_stem;

//	min.minPts_ransac_branch = coeff.minPts_ransac_branch;
//	max.minPts_ransac_branch = 3*coeff.minPts_ransac_branch;

//	min.minPts_ransac_stem = coeff.minPts_ransac_stem;
//	max.minPts_ransac_stem = 3*coeff.minPts_ransac_stem;


//	min.min_radius_sphere_branch = coeff.min_radius_sphere_branch;
//	max.min_radius_sphere_branch = 1.5*coeff.min_radius_sphere_branch;

//	min.min_radius_sphere_stem = coeff.min_radius_sphere_stem;
//	max.min_radius_sphere_stem = 1.5*coeff.min_radius_sphere_stem;

//	min.sphere_radius_multiplier = coeff.sphere_radius_multiplier;
//	max.sphere_radius_multiplier = 1.5*coeff.sphere_radius_multiplier;
//}

//Method_Coefficients
//DownHillSimplex::computeCentroid()
//{
//	assert(simplex.size()==11);

//	Method_Coefficients centroid;
//	for(int i = 0; i < 9; i++)
//	{
//		centroid.epsilon_cluster_branch += simplex.at(i).first.epsilon_cluster_branch;
//		centroid.epsilon_cluster_stem += simplex.at(i).first.epsilon_cluster_stem;
//		centroid.epsilon_sphere += simplex.at(i).first.epsilon_sphere;
//		centroid.minPts_cluster_branch += simplex.at(i).first.minPts_cluster_branch;
//		centroid.minPts_cluster_stem += simplex.at(i).first.minPts_cluster_stem;
//		centroid.minPts_ransac_branch += simplex.at(i).first.minPts_ransac_branch;
//		centroid.minPts_ransac_stem += simplex.at(i).first.minPts_ransac_stem;
//		centroid.min_radius_sphere_branch += simplex.at(i).first.min_radius_sphere_branch;
//		centroid.min_radius_sphere_stem += simplex.at(i).first.min_radius_sphere_stem;
//		centroid.sphere_radius_multiplier += simplex.at(i).first.sphere_radius_multiplier;
//	}
//	centroid.epsilon_cluster_branch /=10;
//	centroid.epsilon_cluster_stem /=10;
//	centroid.epsilon_sphere /=10;
//	centroid.minPts_cluster_branch /=10;
//	centroid.minPts_cluster_stem /=10;
//	centroid.minPts_ransac_branch /=10;
//	centroid.minPts_ransac_stem /=10;
//	centroid.min_radius_sphere_branch /=10;
//	centroid.min_radius_sphere_stem /=10;
//	centroid.sphere_radius_multiplier /=10;
//	make_coefficients_positive(centroid);
//	return centroid;
//}

//void
//DownHillSimplex::sortSimplex()
//{
//	std::sort(simplex.begin(),
//			simplex.end(),
//			[] (const std::pair<Method_Coefficients ,float> &left, const std::pair<Method_Coefficients ,float> &right) {
//		return std::abs(left.second) < std::abs(right.second);
//	});
//}

//std::pair<Method_Coefficients, float>
//DownHillSimplex::evaluateCoefficients(Method_Coefficients coeff)
//{
//	std::pair<Method_Coefficients, float> pair;
//	pair.first = coeff;
//	pcl::console::TicToc tt;
//	tt.tic();
//	SphereFollowing sphereFollowing ( get_control()->getCloudPtr (), getControl()->getIsStem(), 1, coeff);

//	boost::shared_ptr<simpleTree::Tree> tree = boost::make_shared<simpleTree::Tree> (
//			sphereFollowing.getCylinders (), get_control()->getCloudPtr (),
//			get_control()->getTreeID (), get_control() ,false);
//	simpleTree::Allometry allom;
//	allom.setTree(tree);
//	allom.setCoefficients(a,b);
//	allom.setFac(fac);
//	allom.setMinRad(0.02);
//	allom.improveTree();
//	std::vector<float> distances = tree->distancesToModel ();
//	float meanDist = get_control()->getGuiPtr()->mean(distances);
//	meanDist = std::abs(meanDist);
//	pair.second = meanDist;

//	return pair;
//}



