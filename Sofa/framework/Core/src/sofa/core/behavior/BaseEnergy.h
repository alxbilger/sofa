#pragma once
#include <sofa/core/behavior/SingleStateAccessor.h>

namespace sofa::core::behavior
{

class SOFA_CORE_API BaseEnergy : public StateAccessor
{
public:
    SOFA_CLASS(BaseEnergy, StateAccessor);

    void init() override;

    /**
     * Returns the energy E(q, v), where v = dq/dt, q is the coordinates, and t is the time.
     *
     * q and v are provided as ids pointing to a vector in the state associated with this component.
     */
    virtual SReal computeEnergy(
        const ConstVecCoordId& /*q*/in_coordinatesId,
        const ConstVecDerivId& /*v*/in_timeDerivativesId) = 0;

    /**
     * Accumulate the energy gradient into a vector
     *
     * Compute out += ∇E(q,v),
     * where ∇E(q,v) = k_q * (дE/дq)(q,v) + k_v * (дE/дv)(q,v),
     * E = E(q, v) is the energy, and v = dq/dt.
     *
     * Warning: This component must handle the sign in front of the energy.
     * For example, a potential energy has a minus sign compared to a kinetic
     * energy which has a plus sign.
     */
    virtual void accumulateGradient(
        const VecDerivId& out_gradientId,
        const ConstVecCoordId& /*q*/in_coordinatesId,
        const ConstVecDerivId& /*v*/in_timeDerivativesId,
        SReal k_q, SReal k_v) = 0;

    /**
     * Accumulate the Hessian-Vector product
     *
     * Compute out += (k_qq * H_qq(q,v) + k_vv * H_vv(q,v) + k_qv * H_qv(q,v) + k_vq * H_qv(q,v) ) * u,
     * where:
     * q is the coordinates and v=dq/dt
     * u is a given input vector
     * H_qq = д²E/д²q
     * H_vv = д²E/д²q
     * H_qv = д²E/дqдv
     * H_vq = д²E/дvдq
     * k_qq, k_vv, k_qv, k_vq are given factors
     *
     * Note that some Hessian terms may be zero. In this case, the associated factor won't be used.
     */
    virtual void accumulateHessianVectorProduct(
        VecDerivId outVector,
        ConstVecDerivId /*u*/inU,
        const ConstVecCoordId& /*q*/in_coordinatesId,
        const ConstVecDerivId& /*v*/in_timeDerivativesId,
        SReal k_q, SReal k_vv, SReal k_qv, SReal k_vq) = 0;

    virtual void accumulateHessianMatrix(
        const ConstVecCoordId& /*q*/in_coordinatesId,
        const ConstVecDerivId& /*v*/in_timeDerivativesId,
        SReal k_q, SReal k_vv, SReal k_qv, SReal k_vq) = 0;

    bool insertInNode( objectmodel::BaseNode* node ) override;
    bool removeInNode( objectmodel::BaseNode* node ) override;

protected:

    virtual void initBasePotential() {}
};

}  // namespace sofa::core::behavior
