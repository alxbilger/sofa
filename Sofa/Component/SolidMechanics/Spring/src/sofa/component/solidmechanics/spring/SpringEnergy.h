#pragma once
#include <sofa/component/solidmechanics/spring/config.h>
#include <sofa/core/behavior/Energy.h>

namespace sofa::component::solidmechanics::spring
{

template<class DataTypes>
class SpringEnergy : public core::behavior::Energy<DataTypes>
{
public:
    SOFA_CLASS(SpringEnergy, core::behavior::Energy<DataTypes>);

    using Real = Real_t<DataTypes>;

    Data<type::vector<size_t>> d_indices1;
    Data<type::vector<size_t>> d_indices2;
    Data<type::vector<Real>> d_ks;
    Data<type::vector<Real>> d_lengths;

protected:
    explicit SpringEnergy(Real ks = 100.0, Real l0 = 0.0);

    Real_t<DataTypes> doComputeEnergy(
        const VecCoord_t<DataTypes>& in_coordinates,
        const VecDeriv_t<DataTypes>& in_timeDerivative) override;

    void doAccumulateGradient(
        VecDeriv_t<DataTypes>& out_gradient,
       const VecCoord_t<DataTypes>& in_coordinates,
       const VecDeriv_t<DataTypes>& in_timeDerivatives,
       SReal k_q, SReal k_v) override;

    void doAccumulateHessianVectorProduct(
        VecDeriv_t<DataTypes>& outVector,
        const VecDeriv_t<DataTypes>& inU,
        const VecCoord_t<DataTypes>& in_coordinates,
        const VecDeriv_t<DataTypes>& in_timeDerivatives,
        Real_t<DataTypes> k_q, Real_t<DataTypes> k_vv,
        Real_t<DataTypes> k_qv, Real_t<DataTypes> k_vq) override;
};


}  // namespace sofa::component::solidmechanics::spring
