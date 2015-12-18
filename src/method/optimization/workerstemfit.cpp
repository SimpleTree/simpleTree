#include "workerstemfit.h"

WorkerStemFit::WorkerStemFit(PointCloudI::Ptr cloud, float min_height, float bin_width, float epsilon)
{
    this->_min_height = min_height;
    this->_bin_width = bin_width;
    this->_epsilon = epsilon;
    this->_cloud = cloud;
}

WorkerStemFit::~WorkerStemFit()
{

}

float
WorkerStemFit::median(std::vector<float>  &values)
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
WorkerStemFit::squared_mean(std::vector<float>  &v)
{
    std::vector<float>::size_type taille = v.size();
    float sum = 0;
    for(std::vector<float>::iterator i = v.begin(); i != v.end(); ++i)
    {
        float x1 = *i;
                float xy = x1*x1;
        sum += xy;
    }
    return sum/taille;
}

void
WorkerStemFit::run()
{
    Stem_fit fit(_cloud,_min_height, _bin_width, _epsilon);
    std::vector<pcl::ModelCoefficients> cylinders = fit.getCylinders();
    std::vector<float> distances = compute_distances(cylinders);
    std::vector<float> distances_only_fitted_points = reduce_distances(distances,cylinders);
    float dist = squared_mean(distances_only_fitted_points);
    get_optimization()->updateCoeff(dist, _bin_width, _epsilon);
}

std::vector<int>
WorkerStemFit::indexOfPointsNearCylinder ( pcl::octree::OctreePointCloudSearch<PointI>& octree,
                                  pcl::ModelCoefficients& cylinder ) {
    PointI queryPoint;
    queryPoint.x = cylinder.values.at(0)+(cylinder.values.at(3)/2);
    queryPoint.y = cylinder.values.at(1)+(cylinder.values.at(4)/2);
    queryPoint.z = cylinder.values.at(2)+(cylinder.values.at(5)/2);

    float half_length = std::sqrt((cylinder.values.at(3)/2)*(cylinder.values.at(3)/2)+(cylinder.values.at(4)/2)*(cylinder.values.at(4)/2)+(cylinder.values.at(5)/2)*(cylinder.values.at(5)/2));
    float radius = std::sqrt(half_length*half_length+radius*radius);
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    float r = std::max<float> ( radius +  0.1, 0.2 );
    octree.radiusSearch ( queryPoint, r, pointIdxRadiusSearch, pointRadiusSquaredDistance );
    std::vector<int> indices;
    for ( size_t i = 0; i < pointIdxRadiusSearch.size (); i++ ) {
        int index = ( pointIdxRadiusSearch.at ( i ) );
        indices.push_back ( index );
    }
    return indices;
}


std::vector<float>
WorkerStemFit::reduce_distances(std::vector<float>   & distances_in, std::vector<pcl::ModelCoefficients>  &cylinders)
{
    std::cout << "a1" << std::endl;
    std::vector<float> distances_out;
    float max_height = 0;

    for(size_t i = 0; i < cylinders.size(); i++)
    {
        if(cylinders.at(i).values.at(2)+cylinders.at(i).values.at(5)>max_height)
        {
            max_height = cylinders.at(i).values.at(2)+cylinders.at(i).values.at(5);
        }
    }
    std::cout << "a2" << std::endl;
    std::cout << distances_in.size() << std::endl;
    std::cout << _cloud->points.size() << std::endl;
    for(size_t i = 0; i < _cloud->points.size(); i++)
    {
        if(_cloud->points.at(i).z<max_height)
        {
            distances_out.push_back(distances_in.at(i));
        }
    }
    std::cout << "a3" << std::endl;

    std::cout << distances_out.size() << std::endl;
    return distances_out;
}

std::vector<float>
WorkerStemFit::compute_distances(std::vector<pcl::ModelCoefficients>  &cylinders)
{

    std::vector<float> distances ( _cloud->points.size () );
    std::fill ( distances.begin (), distances.end (), 0.5 );

    pcl::octree::OctreePointCloudSearch<PointI> octree ( 0.02f );
    octree.setInputCloud ( _cloud );
    octree.addPointsFromInputCloud ();
    for (size_t i = 0; i < cylinders.size(); i++) {

        pcl::ModelCoefficients cylinder = cylinders.at(i);
        std::vector<int> indices = indexOfPointsNearCylinder ( octree, cylinder );
        for ( size_t i = 0; i < indices.size (); i++ ) {
            PointI point = _cloud->points[indices[i]];
            simpleTree::Cylinder cyl (cylinder);

            float dist = cyl.distToPoint ( point );
            if ( std::abs ( dist ) < std::abs ( distances[indices[i]] ) ) {
                distances[indices[i]] = dist;
            }
        }
    }
    return distances;
}
