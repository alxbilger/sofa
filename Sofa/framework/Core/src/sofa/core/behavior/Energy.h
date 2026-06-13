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

    SReal computeEnergy(const ConstVecCoordId& in_coordinatesId,
                        const ConstVecDerivId& in_timeDerivativesId) override;

    void accumulateGradient(
        const VecDerivId& out_gradientId,
        const ConstVecCoordId& /*q*/in_coordinatesId,
        const ConstVecDerivId& /*v*/in_timeDerivativesId,
        SReal k_q, SReal k_v) override;

    void accumulateHessianVectorProduct(
        VecDerivId outVector, ConstVecDerivId inU,
        const ConstVecCoordId& in_coordinatesId,
        const ConstVecDerivId& in_timeDerivativesId,
        SReal k_q, SReal k_vv, SReal k_qv, SReal k_vq) override;

    void accumulateHessianMatrix(const ConstVecCoordId& in_coordinatesId,
                                 const ConstVecDerivId& in_timeDerivativesId, SReal k_q, SReal k_vv,
                                 SReal k_qv, SReal k_vq) override {}

protected:
    virtual void initPotential() {}

    void initBasePotential() final;

    virtual Real_t<TDataTypes> doComputeEnergy(
        const VecCoord_t<TDataTypes>& in_coordinates,
        const VecDeriv_t<TDataTypes>& in_timeDerivative
    ) = 0;

    virtual void doAccumulateGradient(
        VecDeriv_t<TDataTypes>& out_gradient,
        const VecCoord_t<TDataTypes>& in_coordinates,
        const VecDeriv_t<TDataTypes>& in_timeDerivatives,
        SReal k_q, SReal k_v
    ) = 0;

    virtual void doAccumulateHessianVectorProduct(
        VecDeriv_t<TDataTypes>& outVector,
        const VecDeriv_t<TDataTypes>& inU,
        const VecCoord_t<TDataTypes>& in_coordinates,
        const VecDeriv_t<TDataTypes>& in_timeDerivatives,
        Real_t<TDataTypes> k_q, Real_t<TDataTypes> k_vv, Real_t<TDataTypes> k_qv, Real_t<TDataTypes> k_vq
    ) = 0;

};

#if !defined(SOFA_CORE_BEHAVIOR_POTENTIAL_CPP)
extern template class SOFA_CORE_API Energy<sofa::defaulttype::Vec1Types>;
extern template class SOFA_CORE_API Energy<sofa::defaulttype::Vec2Types>;
extern template class SOFA_CORE_API Energy<sofa::defaulttype::Vec3Types>;
extern template class SOFA_CORE_API Energy<sofa::defaulttype::Rigid2Types>;
extern template class SOFA_CORE_API Energy<sofa::defaulttype::Rigid3Types>;
#endif

}  // namespace sofa::core::behavior
