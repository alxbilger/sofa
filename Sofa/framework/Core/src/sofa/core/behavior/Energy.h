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

    template<class T>
    static bool canCreate(T*& obj, objectmodel::BaseContext* context, objectmodel::BaseObjectDescription* arg)
    {
        if (context)
        {
            if (arg)
            {
                static const std::string attributeName {"mstate"};
                const std::string mstateLink = arg->getAttribute(attributeName,"");
                if (mstateLink.empty())
                {
                    if (dynamic_cast<MechanicalState<TDataTypes>*>(context->getMechanicalState()) == nullptr)
                    {
                        arg->logError("Since the attribute '" + attributeName + "' has not been specified, a mechanical state "
                            "with the datatype '" + TDataTypes::Name() + "' has been searched in the current context, but not found.");
                        return false;
                    }
                }
                else
                {
                    MechanicalState<TDataTypes>* mstate = nullptr;
                    context->findLinkDest(mstate, mstateLink, nullptr);
                    if (!mstate)
                    {
                        arg->logError("Data attribute '" + attributeName + "' does not point to a valid mechanical state of datatype '" + std::string(TDataTypes::Name()) + "'.");
                        return false;
                    }
                }
            }
            else
            {
                if (dynamic_cast<MechanicalState<TDataTypes>*>(context->getMechanicalState()) == nullptr)
                {
                    return false;
                }
            }

            return sofa::core::objectmodel::BaseComponent::canCreate(obj, context, arg);
        }
        return false;
    }

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
        Real_t<TDataTypes> k_q, Real_t<TDataTypes> k_v
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
