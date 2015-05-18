#include "workerspherefollowing.h"

WorkerSphereFollowing::WorkerSphereFollowing(QObject *parent) :
    QRunnable(parent)
{
}

WorkerSphereFollowing::run()
{


    SphereFollowing sphereFollowing ( this->control->getCloudPtr (), control, 1, coeff );
    boost::shared_ptr<simpleTree::Tree> tree = boost::make_shared<simpleTree::Tree> ( sphereFollowing.getCylinders (), control->getCloudPtr (),
                                                                                     control->getTreeID (), this->getControl() );

    simpleTree::Allometry allom;
    allom.setTree(tree);
    allom.setCoefficients(2101,3.775);
    allom.setFac(1.6);
    allom.improveTree();
    getControl ()->setTreePtr ( tree );


    std::vector<float> distances = getControl()->getTreePtr()->distancesToModel ();
    meanDist = average(distances);



}
