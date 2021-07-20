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

#include <sofa/component/linearsystem/matrixaccumulators/BaseAssemblingMatrixAccumulator.h>

namespace sofa::component::linearsystem
{

/**
 * Local matrix component associated to force fields, masses and mappings
 *
 * They are associated to components by the assembling matrix system AssemblingMatrixSystem.
 * AssemblingMatrixAccumulator is not added to the factory. It is not up to the user to add this component
 * in the scene graph.
 *
 * AssemblingMatrixAccumulator inherits from MatrixAccumulator and is defined as a child of components.
 * This allows components to add their contributions to the global matrix through their associated
 * local matrices.
 *
 * This matrix accumulator has a direct link to the global matrix so it can add its contributions directly
 * into it. It also knows where to add in the matrix using an offset parameter, set by the assembling
 * matrix system.
 *
 * @tparam TComponentType core::behavior::BaseForceField, core::behavior::BaseMass or core::BaseMapping
 */
template<class TComponentType, class TStrategy = sofa::core::matrixaccumulator::NoIndexVerification>
class AssemblingMatrixAccumulator : public virtual sofa::core::MatrixAccumulatorIndexChecker<BaseAssemblingMatrixAccumulator<TComponentType>, TStrategy>
{
public:
    SOFA_CLASS(AssemblingMatrixAccumulator, SOFA_TEMPLATE2(sofa::core::MatrixAccumulatorIndexChecker, BaseAssemblingMatrixAccumulator<TComponentType>, TStrategy));

    using MatrixAccumulator = Inherit1;
    using ComponentType = TComponentType;
    using Strategy = TStrategy;

    using Inherit1::m_globalMatrix;
    using Inherit1::m_cachedPositionInGlobalMatrix;
    using Inherit1::m_cachedFactor;

    void clear() override;

    using Inherit1::add;

protected:

    void add(const core::matrixaccumulator::no_check_policy&, sofa::SignedIndex row, sofa::SignedIndex col, float value) override;
    void add(const core::matrixaccumulator::no_check_policy&, sofa::SignedIndex row, sofa::SignedIndex col, double value) override;
    void add(const core::matrixaccumulator::no_check_policy&, sofa::SignedIndex row, sofa::SignedIndex col, const sofa::type::Mat<3, 3, float>& value) override;
    void add(const core::matrixaccumulator::no_check_policy&, sofa::SignedIndex row, sofa::SignedIndex col, const sofa::type::Mat<3, 3, double>& value) override;
};



template<class ComponentType, class TStrategy>
void AssemblingMatrixAccumulator<ComponentType, TStrategy>::add(const core::matrixaccumulator::no_check_policy&, const sofa::SignedIndex row, const sofa::SignedIndex col, const float value)
{
    m_globalMatrix->add(row + m_cachedPositionInGlobalMatrix[0],
                              col + m_cachedPositionInGlobalMatrix[1],
                              m_cachedFactor * value);
}
template<class ComponentType, class TStrategy>
void AssemblingMatrixAccumulator<ComponentType, TStrategy>::add(const core::matrixaccumulator::no_check_policy&, const sofa::SignedIndex row, const sofa::SignedIndex col, const double value)
{
    m_globalMatrix->add(row + m_cachedPositionInGlobalMatrix[0],
                              col + m_cachedPositionInGlobalMatrix[1],
                              m_cachedFactor * value);
}
template<class ComponentType, class TStrategy>
void AssemblingMatrixAccumulator<ComponentType, TStrategy>::add(const core::matrixaccumulator::no_check_policy&, const sofa::SignedIndex row, const sofa::SignedIndex col, const sofa::type::Mat<3, 3, float>& value)
{
    m_globalMatrix->add(row + m_cachedPositionInGlobalMatrix[0],
                              col + m_cachedPositionInGlobalMatrix[1],
                              value * static_cast<float>(m_cachedFactor));
}
template<class ComponentType, class TStrategy>
void AssemblingMatrixAccumulator<ComponentType, TStrategy>::add(const core::matrixaccumulator::no_check_policy&, const sofa::SignedIndex row, const sofa::SignedIndex col, const sofa::type::Mat<3, 3, double>& value)
{
    m_globalMatrix->add(row + m_cachedPositionInGlobalMatrix[0],
                              col + m_cachedPositionInGlobalMatrix[1],
                              value * static_cast<double>(m_cachedFactor));
}
template<class ComponentType, class TStrategy>
void AssemblingMatrixAccumulator<ComponentType, TStrategy>::clear()
{}


}
