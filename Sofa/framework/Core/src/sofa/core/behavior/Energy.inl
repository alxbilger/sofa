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
SReal Energy<TDataTypes>::computeEnergy(const ConstVecCoordId& in_coordinatesId,
                                        const ConstVecDerivId& in_timeDerivativesId)
{
    if (this->isComponentStateInvalid()) return 0_sreal;

    if (!this->mstate)
    {
        msg_error() << "Cannot access to the associated state";
        return 0_sreal;
    }

    // input
    const auto* coordinatesDataPtr = this->mstate->read(in_coordinatesId);
    const auto* timeDerivativesDataPtr = this->mstate->read(in_timeDerivativesId);

    msg_error_when(!coordinatesDataPtr)
        << "Cannot access to the coordinates through the id " << in_coordinatesId;
    msg_error_when(!timeDerivativesDataPtr)
        << "Cannot access to the time derivatives through the id " << in_timeDerivativesId;

    if (!coordinatesDataPtr || !timeDerivativesDataPtr) return 0_sreal;

    const auto coordinatesAccessor = sofa::helper::getReadAccessor(*coordinatesDataPtr);
    const auto timeDerivativesAccessor = sofa::helper::getReadAccessor(*timeDerivativesDataPtr);

    const auto energy = doComputeEnergy(coordinatesAccessor.ref(), timeDerivativesAccessor.ref());
    return static_cast<SReal>(energy);
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
void Energy<TDataTypes>::accumulateHessianVectorProduct(
    VecDerivId outVector, ConstVecDerivId inU,
    const ConstVecCoordId& in_coordinatesId,
    const ConstVecDerivId& in_timeDerivativesId,
    SReal k_q, SReal k_vv, SReal k_qv, SReal k_vq)
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
    const auto* uDataPtr = this->mstate->read(inU);
    // output
    auto* out = this->mstate->write(outVector);

    msg_error_when(!coordinatesDataPtr)
        << "Cannot access to the coordinates through the id " << in_coordinatesId;
    msg_error_when(!timeDerivativesDataPtr)
        << "Cannot access to the time derivatives through the id " << in_timeDerivativesId;
    msg_error_when(!uDataPtr)
        << "Cannot access to the u vector through the id " << inU;
    msg_error_when(!out)
        << "Cannot access to the output through the id " << outVector;

    if (!coordinatesDataPtr || !timeDerivativesDataPtr || !uDataPtr || !out) return;

    const auto coordinatesAccessor = sofa::helper::getReadAccessor(*coordinatesDataPtr);
    const auto timeDerivativesAccessor = sofa::helper::getReadAccessor(*timeDerivativesDataPtr);
    const auto uAccessor = sofa::helper::getReadAccessor(*uDataPtr);
    auto outAccessor = sofa::helper::getWriteAccessor(*out);

    doAccumulateHessianVectorProduct(
        outAccessor.wref(), uAccessor.ref(),
        coordinatesAccessor.ref(), timeDerivativesAccessor.ref(),
        static_cast<Real_t<TDataTypes>>(k_q),
        static_cast<Real_t<TDataTypes>>(k_vv),
        static_cast<Real_t<TDataTypes>>(k_qv),
        static_cast<Real_t<TDataTypes>>(k_vq)
    );

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
