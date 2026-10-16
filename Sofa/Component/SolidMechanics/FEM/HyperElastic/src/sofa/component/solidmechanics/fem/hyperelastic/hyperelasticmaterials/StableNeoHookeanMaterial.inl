/******************************************************************************
*                 SOFA, Simulation Open-Framework Architecture                *
*                    (c) 2006 INRIA, USTL, UJF, CNRS, MGH                     *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#pragma once

#include <sofa/component/solidmechanics/fem/hyperelastic/hyperelasticmaterials/StableNeoHookeanMaterial.h>
#include <sofa/component/solidmechanics/fem/hyperelastic/PK2HyperelasticMaterial.inl>

namespace sofa::component::solidmechanics::fem::hyperelastic
{

template <class DataTypes>
auto StableNeoHookeanMaterial<DataTypes>::alpha() const -> Real
{
    return static_cast<Real>(1) + m_mu / (m_lambda + m_mu);
}

template <class DataTypes>
auto StableNeoHookeanMaterial<DataTypes>::secondPiolaKirchhoffStress(Strain<DataTypes>& strain) -> StressTensor
{
    static constexpr auto& I = Strain<DataTypes>::identity;

    const auto& C = strain.getRightCauchyGreenTensor();
    const RightCauchyGreenTensor C_1 = sofa::type::inverse(C);

    const Real J = strain.getDeterminantDeformationGradient();
    const Real a = alpha();

    return m_mu * I + ((m_lambda + m_mu) * J * (J - a)) * C_1;
}

template <class DataTypes>
auto StableNeoHookeanMaterial<DataTypes>::elasticityTensor(Strain<DataTypes>& strain) -> ElasticityTensor
{
    const auto& C = strain.getRightCauchyGreenTensor();
    const RightCauchyGreenTensor C_1 = sofa::type::inverse(C);

    const Real J = strain.getDeterminantDeformationGradient();
    const Real a = alpha();

    return ElasticityTensor(
        [
            lambdaMu = m_lambda + m_mu,
            J,
            a,
            &C_1
        ](sofa::Index i, sofa::Index j, sofa::Index k, sofa::Index l)
        {
            return lambdaMu *
                (
                    -J * (J - a) * (C_1(i, k) * C_1(l, j) + C_1(i, l) * C_1(k, j))
                    + J * (static_cast<Real>(2) * J - a) * C_1(l, k) * C_1(i, j)
                );
        });
}

} // namespace sofa::component::solidmechanics::fem::hyperelastic
