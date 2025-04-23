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

#include <sofa/component/constraint/lagrangian/solver/AssembleConstraints.h>
#include <sofa/component/constraint/lagrangian/solver/config.h>
#include <sofa/component/linearsystem/MatrixLinearSystem.h>

namespace sofa::component::constraint::lagrangian::solver
{

template<class TMatrix, class TVector>
class LagrangianConstraintLinearSystem
    : public sofa::component::linearsystem::MatrixLinearSystem<TMatrix, TVector >
    , public AssembleConstraints
{
public:
    SOFA_CLASS(
        SOFA_TEMPLATE2(LagrangianConstraintLinearSystem, TMatrix, TVector),
        SOFA_TEMPLATE2(sofa::component::linearsystem::MatrixLinearSystem, TMatrix, TVector));

    using Matrix = TMatrix;
    using Vector = TVector;
    using Real = typename TMatrix::Real;

    void init() override;
    void assembleSystem(const core::MechanicalParams* mparams) override;

protected:
    sofa::Size computeLocalLagrangianConstraintMatrices(const core::MechanicalParams* mparams);
    void assembleConstraints();

    sofa::core::MultiVecDerivId m_lambdaId;

    core::ConstraintParams cparams;

    const core::objectmodel::BaseContext* getLagrangianConstraintsContext() const override;
    core::objectmodel::BaseContext* getLagrangianConstraintsContext() override;

    std::size_t computeSystemSize(const core::MechanicalParams* mparams) override;
};



}
