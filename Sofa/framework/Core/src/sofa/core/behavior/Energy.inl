#pragma once
#include <sofa/core/behavior/Energy.h>
namespace sofa::core::behavior
{

template <class TDataTypes>
void Energy<TDataTypes>::init()
{
    BaseEnergy::init();
}

template <class TDataTypes>
void Energy<TDataTypes>::accumulateGradient(
    const VecDerivId& out_gradientId,
    const ConstVecCoordId& /*q*/in_coordinatesId,
    const ConstVecDerivId& /*v*/in_timeDerivativesId,
    SReal k_q, SReal k_v)
{
    if (this->isComponentStateInvalid()) return;

    if (!this->mstate)
    {
        msg_error() << "Cannot access to the associated state";
        return;
    }

    // input
    const auto* coordinatesDataPtr = this->mstate->read(in_coordinatesId);
    const auto* timeDerivativesDataPtr = this->mstate->read(in_timeDerivativesId);
    // output
    auto* gradientDataPtr = this->mstate->write(out_gradientId);

    msg_error_when(!coordinatesDataPtr)
        << "Cannot access to the coordinates through the id " << in_coordinatesId;
    msg_error_when(!timeDerivativesDataPtr)
        << "Cannot access to the time derivatives through the id " << in_timeDerivativesId;
    msg_error_when(!gradientDataPtr)
        << "Cannot access to the gradient through the id " << out_gradientId;

    if (!coordinatesDataPtr || !timeDerivativesDataPtr || !gradientDataPtr) return;

    const auto coordinatesAccessor = sofa::helper::getReadAccessor(*coordinatesDataPtr);
    const auto timeDerivativesAccessor = sofa::helper::getReadAccessor(*timeDerivativesDataPtr);
    auto gradientAccessor = sofa::helper::getWriteAccessor(*gradientDataPtr);

    doAccumulateGradient(
        gradientAccessor.wref(),
        coordinatesAccessor.ref(),
        timeDerivativesAccessor.ref(),
        k_q, k_v);
}

template <class TDataTypes>
void Energy<TDataTypes>::initBasePotential()
{
    SingleStateAccessor<TDataTypes>::init();

    if (!this->isComponentStateInvalid())
    {
        initPotential();
    }
}

}  // namespace sofa::core::behavior
