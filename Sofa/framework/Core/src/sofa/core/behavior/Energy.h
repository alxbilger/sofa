#pragma once
#include <sofa/core/behavior/BaseEnergy.h>

namespace sofa::core::behavior
{

template<class TDataTypes>
class Energy : public virtual BaseEnergy, public virtual SingleStateAccessor<TDataTypes>
{
public:
    SOFA_CLASS2(Energy<TDataTypes>, BaseEnergy, SingleStateAccessor<TDataTypes>);

    void init() final;

    void accumulateGradient(
        const VecDerivId& out_gradientId,
        const ConstVecCoordId& /*q*/in_coordinatesId,
        const ConstVecDerivId& /*v*/in_timeDerivativesId,
        SReal k_q, SReal k_v) override;

protected:
    virtual void initPotential() {}

    void initBasePotential() final;

    virtual void doAccumulateGradient(
        VecDeriv_t<TDataTypes>& out_gradient,
        const VecCoord_t<TDataTypes>& in_coordinates,
        const VecDeriv_t<TDataTypes>& in_timeDerivatives,
        SReal k_q, SReal k_v) = 0;
};

#if !defined(SOFA_CORE_BEHAVIOR_POTENTIAL_CPP)
extern template class SOFA_CORE_API Potential<sofa::defaulttype::Vec1Types>;
extern template class SOFA_CORE_API Potential<sofa::defaulttype::Vec2Types>;
extern template class SOFA_CORE_API Potential<sofa::defaulttype::Vec3Types>;
extern template class SOFA_CORE_API Potential<sofa::defaulttype::Rigid2Types>;
extern template class SOFA_CORE_API Potential<sofa::defaulttype::Rigid3Types>;
#endif

}  // namespace sofa::core::behavior
