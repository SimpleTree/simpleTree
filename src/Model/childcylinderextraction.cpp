#include "childcylinderextraction.h"
namespace simpleTree {
ChildCylinderExtraction::ChildCylinderExtraction(boost::shared_ptr<Tree> tree)
{
    this->tree = tree;
    this->cylinders = this->tree->getCylinders();


}
}
