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
#include <sofa/component/constraint/lagrangian/solver/AssembleConstraints.h>
#include <sofa/helper/ScopedAdvancedTimer.h>
#include <sofa/simulation/mechanicalvisitor/MechanicalAccumulateMatrixDeriv.h>
#include <sofa/simulation/mechanicalvisitor/MechanicalBuildConstraintMatrix.h>
#include <sofa/simulation/mechanicalvisitor/MechanicalProjectJacobianMatrixVisitor.h>
#include <sofa/simulation/mechanicalvisitor/MechanicalResetConstraintVisitor.h>


namespace sofa::component::constraint::lagrangian::solver
{

void AssembleConstraints::resetConstraints(const core::ConstraintParams* cParams)
{
    assert(getLagrangianConstraintsContext());
    SCOPED_TIMER("Reset Constraint");
    simulation::mechanicalvisitor::MechanicalResetConstraintVisitor(cParams).execute(getLagrangianConstraintsContext());
}

void AssembleConstraints::buildLocalConstraintMatrix(const core::ConstraintParams* cparams, unsigned int &constraintId)
{
    assert(getLagrangianConstraintsContext());
    SCOPED_TIMER("Build Local Constraint Matrix");
    simulation::mechanicalvisitor::MechanicalBuildConstraintMatrix buildConstraintMatrix(cparams, cparams->j(), constraintId );
    buildConstraintMatrix.execute(getLagrangianConstraintsContext());
}

void AssembleConstraints::accumulateMatrixDeriv(const core::ConstraintParams* cparams)
{
    assert(getLagrangianConstraintsContext());
    SCOPED_TIMER("Project Mapped Constraint Matrix");
    simulation::mechanicalvisitor::MechanicalAccumulateMatrixDeriv accumulateMatrixDeriv(cparams, cparams->j());
    accumulateMatrixDeriv.execute(getLagrangianConstraintsContext());
}

void AssembleConstraints::applyProjectiveConstraintOnConstraintMatrix(
    const core::ConstraintParams* cparams)
{
    assert(getLagrangianConstraintsContext());
    SCOPED_TIMER("Projective Constraints on Constraint Jacobian");
    core::MechanicalParams mparams = core::MechanicalParams(*cparams);
    simulation::mechanicalvisitor::MechanicalProjectJacobianMatrixVisitor(&mparams).execute(getLagrangianConstraintsContext());
}
}
