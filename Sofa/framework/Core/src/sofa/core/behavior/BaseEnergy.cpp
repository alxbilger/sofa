#include <sofa/core/behavior/BaseEnergy.h>
#include <sofa/core/objectmodel/BaseNode.h>

namespace sofa::core::behavior
{

void BaseEnergy::init()
{
    StateAccessor::init();

    if (!this->isComponentStateInvalid())
    {
        initBaseEnergy();
    }
}
bool BaseEnergy::insertInNode(objectmodel::BaseNode* node)
{
    node->addEnergy(this);
    Inherit1::insertInNode(node);
    return true;
}
bool BaseEnergy::removeInNode(objectmodel::BaseNode* node)
{
    node->addEnergy(this);
    Inherit1::removeInNode(node);
    return true;
}

}  // namespace sofa::core::behavior
