#include "workerspherefollowing.h"


float WorkerSphereFollowing::getA() const
{
    return a;
}

void WorkerSphereFollowing::setA(float value)
{
    a = value;
}

float WorkerSphereFollowing::getB() const
{
    return b;
}

void WorkerSphereFollowing::setB(float value)
{
    b = value;
}

float WorkerSphereFollowing::getFac() const
{
    return fac;
}

void WorkerSphereFollowing::setFac(float value)
{
    fac = value;
}
WorkerSphereFollowing::WorkerSphereFollowing(int i,QObject *parent)
{
	this->i = i;
}

void
WorkerSphereFollowing::run()
{

    try {
        SphereFollowing sphereFollowing ( _cloud_ptr, isStem, 1, coefficients );
        boost::shared_ptr<simpleTree::Tree> tree = boost::make_shared<simpleTree::Tree> ( sphereFollowing.getCylinders (), _cloud_ptr,
                                                                                         treeID ,false);

        simpleTree::Allometry allom;
        allom.setTree(tree);
        allom.setCoefficients(a,b);
        allom.setFac(fac);

        allom.setMinRad(minRad);
        allom.improveTree();


        std::vector<float> distances = tree->distancesToModel ();
        std::vector<float> distances_abs = getAbsDistances(distances);
        meanDist = mean(distances_abs);


        optimize->updateCoeff(coefficients, meanDist, minRad);




    } catch (...) {
        std::cout << "critical error in WorkerSphereFollowing" << std::endl;
        std::cout << coefficients.toQString().toStdString() << std::endl;
        std::cout << "critical error in WorkerSphereFollowing" << std::endl;
    }




}

std::vector<float>
WorkerSphereFollowing::getAbsDistances(std::vector<float> distances)
{
	std::vector<float> abs_distances;
	abs_distances.resize(distances.size());
	for(size_t i = 0; i < distances.size(); i++)
	{
		abs_distances.at(i) = std::abs(distances.at(i));
	}
	return abs_distances;
}


void
WorkerSphereFollowing::setOptimize(boost::shared_ptr<Optimization> optimize)
{
    this->optimize = optimize;
}



void
WorkerSphereFollowing::set_coefficients(Method_Coefficients coefficients)
{
    this->coefficients = coefficients;
}

float
WorkerSphereFollowing::median(std::vector<float> values)
{
    float median;
    size_t size = values.size();

    sort(values.begin(), values.end());

    if (size  % 2 == 0)
    {
        median = (values[size / 2 - 1] + values[size / 2]) / 2;
    }
    else
    {
        median = values[size / 2];
    }

    return median;
}

float
WorkerSphereFollowing::mean(std::vector<float> const &v)
{
    std::vector<float>::size_type taille = v.size();
    float sum = 0;
    for(std::vector<float>::const_iterator i = v.begin(); i != v.end(); ++i)
        sum += *i;
    return sum/taille;
}


