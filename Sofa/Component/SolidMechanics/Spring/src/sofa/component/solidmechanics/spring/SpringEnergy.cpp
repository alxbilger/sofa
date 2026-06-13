#include <sofa/component/solidmechanics/spring/SpringEnergy.inl>
#include <sofa/core/ObjectFactory.h>

namespace sofa::component::solidmechanics::spring
{

template class SOFA_COMPONENT_SOLIDMECHANICS_SPRING_API SpringEnergy<sofa::defaulttype::Vec3Types>;

void registerSpringEnergy(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(core::ObjectRegistrationData("Elastic springs")
        .add< SpringEnergy<sofa::defaulttype::Vec3Types> >()
    );
}

}
