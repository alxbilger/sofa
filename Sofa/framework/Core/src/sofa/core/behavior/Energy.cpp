#define SOFA_CORE_BEHAVIOR_POTENTIAL_CPP
#include <sofa/core/behavior/Energy.inl>

namespace sofa::core::behavior
{

template class SOFA_CORE_API Energy<sofa::defaulttype::Vec1Types>;
template class SOFA_CORE_API Energy<sofa::defaulttype::Vec2Types>;
template class SOFA_CORE_API Energy<sofa::defaulttype::Vec3Types>;
template class SOFA_CORE_API Energy<sofa::defaulttype::Rigid2Types>;
template class SOFA_CORE_API Energy<sofa::defaulttype::Rigid3Types>;

}
