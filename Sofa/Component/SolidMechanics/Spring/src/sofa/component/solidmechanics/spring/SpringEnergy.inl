#pragma once

#include <sofa/component/solidmechanics/spring/SpringEnergy.h>
#include <sofa/core/behavior/Energy.inl>

namespace sofa::component::solidmechanics::spring
{

template <class DataTypes>
SpringEnergy<DataTypes>::SpringEnergy(Real ks, Real l0)
    : d_indices1(initData(&d_indices1, "indices1", "Indices of the first particles"))
    , d_indices2(initData(&d_indices2, "indices2", "Indices of the second particles"))
    , d_ks(initData(&d_ks, type::vector<Real>{ks}, "stiffness", "Stiffness of the springs"))
    , d_lengths(initData(&d_lengths, type::vector<Real>{l0}, "restLength", "Rest length of the springs"))
{
}

template <class DataTypes>
Real_t<DataTypes> SpringEnergy<DataTypes>::doComputeEnergy(
    const VecCoord_t<DataTypes>& in_coordinates,
    const VecDeriv_t<DataTypes>& in_timeDerivative)
{
    Real_t<DataTypes> energy{};

    const auto indices1 = sofa::helper::getReadAccessor(d_indices1);
    const auto indices2 = sofa::helper::getReadAccessor(d_indices2);
    const auto ks = sofa::helper::getReadAccessor(d_ks);
    const auto l0 = sofa::helper::getReadAccessor(d_lengths);

    const std::size_t nbSprings = std::min({indices1.size(), indices2.size(), ks.size(), l0.size()});

    for (std::size_t i = 0; i < nbSprings; ++i)
    {
        const auto i1 = indices1[i];
        const auto i2 = indices2[i];
        const auto k = ks[i];
        const auto l = l0[i];

        energy += 0.5 * std::pow( (in_coordinates[i1] - in_coordinates[i2]).norm() - l, 2);
    }

    return energy;
}

template <class DataTypes>
void SpringEnergy<DataTypes>::doAccumulateGradient(
    VecDeriv_t<DataTypes>& out_gradient,
    const VecCoord_t<DataTypes>& in_coordinates,
    const VecDeriv_t<DataTypes>& in_timeDerivatives,
    SReal k_q, SReal k_v)
{
}

template <class DataTypes>
void SpringEnergy<DataTypes>::doAccumulateHessianVectorProduct(
    VecDeriv_t<DataTypes>& outVector,
    const VecDeriv_t<DataTypes>& inU,
    const VecCoord_t<DataTypes>& in_coordinates,
    const VecDeriv_t<DataTypes>& in_timeDerivatives,
    Real_t<DataTypes> k_q, Real_t<DataTypes> k_vv, Real_t<DataTypes> k_qv, Real_t<DataTypes> k_vq)
{
}

}  // namespace sofa::component::solidmechanics::spring
