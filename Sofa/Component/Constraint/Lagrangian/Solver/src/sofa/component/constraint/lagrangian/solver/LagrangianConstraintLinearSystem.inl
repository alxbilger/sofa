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

#include <sofa/component/constraint/lagrangian/solver/LagrangianConstraintLinearSystem.h>
#include <sofa/core/behavior/MultiVec.h>
#include <sofa/simulation/VectorOperations.h>
#include <sofa/simulation/mechanicalvisitor/MechanicalGetConstraintJacobianVisitor.h>
#include <sofa/simulation/mechanicalvisitor/MechanicalVOpVisitor.h>

#include <sofa/component/linearsystem/MatrixLinearSystem.inl>

#include "visitors/MechanicalGetConstraintViolationVisitor.h"

namespace sofa::component::constraint::lagrangian::solver
{

template<class TMatrix>
class ConstraintMatrixProxy;

template<class TVector>
class ConstraintViolationProxy;

template< typename TMultiVecId >
void clearMultiVecId(sofa::core::objectmodel::BaseContext* ctx, const sofa::core::ConstraintParams* cParams, const TMultiVecId& vid)
{
    sofa::simulation::mechanicalvisitor::MechanicalVOpVisitor clearVisitor(cParams, vid, sofa::core::ConstMultiVecDerivId::null(), sofa::core::ConstMultiVecDerivId::null(), 1.0);
    clearVisitor.setMapped(true);
    ctx->executeVisitor(&clearVisitor);
}

template <class TMatrix, class TVector>
LagrangianConstraintLinearSystem<TMatrix, TVector>::LagrangianConstraintLinearSystem()
    : d_lambda(initData(&d_lambda, "lambda", "Lagrange multiplier"))
{
}

template <class TMatrix, class TVector>
void LagrangianConstraintLinearSystem<TMatrix, TVector>::init()
{
    linearsystem::MatrixLinearSystem<TMatrix, TVector>::init();

    simulation::common::VectorOperations vop(sofa::core::execparams::defaultInstance(),
                                             this->getContext());
    {
        sofa::core::behavior::MultiVecDeriv lambda(&vop, m_lambdaId);
        lambda.realloc(&vop, false, true, core::VecIdProperties{"lambda", GetClass()->className});
        m_lambdaId = lambda.id();
    }
}
template <class TMatrix, class TVector>
void LagrangianConstraintLinearSystem<TMatrix, TVector>::reset()
{
    lagrangianConstraintsSize = 0;
}

template <class TMatrix, class TVector>
void LagrangianConstraintLinearSystem<TMatrix, TVector>::assembleConstraintsJacobian()
{
    const auto mechanicalSize = this->getMappingGraph().getTotalNbMainDofs();
    ConstraintMatrixProxy intermediate(this->getSystemMatrix(), mechanicalSize);

    simulation::mechanicalvisitor::MechanicalGetConstraintJacobianVisitor(&cparams, &intermediate)
        .execute(this->getLagrangianConstraintsContext());
}

template <class TMatrix, class TVector>
void LagrangianConstraintLinearSystem<TMatrix, TVector>::assembleConstraintViolation()
{
    const auto mechanicalSize = this->getMappingGraph().getTotalNbMainDofs();
    ConstraintViolationProxy constraintViolation(this->getRHSVector(), mechanicalSize);
    MechanicalGetConstraintViolationVisitor(&cparams, &constraintViolation)
        .execute(this->getLagrangianConstraintsContext());
}

template <class TMatrix, class TVector>
void LagrangianConstraintLinearSystem<TMatrix, TVector>::assembleSystem(
    const core::MechanicalParams* mparams)
{
    Inherit1::assembleSystem(mparams);

    const auto nbConstraints = computeLocalLagrangianConstraintMatrices(mparams);
    if (nbConstraints != lagrangianConstraintsSize)
    {
        if (lagrangianConstraintsSize > 0)
        {
            msg_warning() << "Lagrangian constraints size changed from " << lagrangianConstraintsSize << " to " << nbConstraints << " and it is not tested";
        }
        lagrangianConstraintsSize = nbConstraints;
    }
    const std::size_t totalSize = this->computeSystemSize(mparams);

    assert(this->getSystemMatrix()->rowSize() == this->getSystemMatrix()->colSize());
    if (totalSize > this->getSystemMatrix()->rowSize())
    {
        msg_info() << "Extend the linear system to " << totalSize << "x" << totalSize << " to include " << lagrangianConstraintsSize << " lagrangian constraints";
        this->m_linearSystem.extendSystem(totalSize);
        this->d_matrixSize.setValue({totalSize, totalSize});
    }
    else
    {
        if (lagrangianConstraintsSize == 0)
        {
            //nothing to do
            msg_info() << "No lagrangian constraints";
            return;
        }
    }

    assembleConstraintsJacobian();
    assembleConstraintViolation();
}

template <class TMatrix, class TVector>
void LagrangianConstraintLinearSystem<TMatrix, TVector>::dispatchSystemSolution(
    core::MultiVecDerivId v)
{
    linearsystem::MatrixLinearSystem<TMatrix, TVector>::dispatchSystemSolution(v);

    if (auto* solution = this->getSolutionVector())
    {
        auto lambda = sofa::helper::getWriteOnlyAccessor(d_lambda);
        lambda.resize(lagrangianConstraintsSize);
        const auto mechanicalSize = this->getMappingGraph().getTotalNbMainDofs();
        for (std::size_t i = 0; i < lagrangianConstraintsSize; ++i)
        {
            lambda[i] = solution->element(i + mechanicalSize);
        }
    }
}


template <class TMatrix, class TVector>
sofa::Size LagrangianConstraintLinearSystem<TMatrix, TVector>::
computeLocalLagrangianConstraintMatrices(const core::MechanicalParams* mparams)
{
    cparams = *mparams;
    cparams.setLambda(m_lambdaId);
    cparams.setX(core::vec_id::read_access::position);
    cparams.setV(core::vec_id::read_access::velocity);

    SCOPED_TIMER("Build Constraint Matrix");
    unsigned int constraintId {};
    resetConstraints(&cparams);
    buildLocalConstraintMatrix(&cparams, constraintId);
    accumulateMatrixDeriv(&cparams);
    applyProjectiveConstraintOnConstraintMatrix(&cparams);

    return constraintId;
}

template <class TMatrix, class TVector>
const core::objectmodel::BaseContext*
LagrangianConstraintLinearSystem<TMatrix, TVector>::getLagrangianConstraintsContext() const
{
    return this->getContext();
}

template <class TMatrix, class TVector>
core::objectmodel::BaseContext*
LagrangianConstraintLinearSystem<TMatrix, TVector>::getLagrangianConstraintsContext()
{
    return this->getContext();
}

template <class TMatrix, class TVector>
std::size_t LagrangianConstraintLinearSystem<TMatrix, TVector>::
computeSystemSize(const core::MechanicalParams* mparams)
{
    return this->m_mappingGraph.getTotalNbMainDofs() + lagrangianConstraintsSize;
}


/// Dispatch the constraints matrix entries into the global system
template<class TMatrix>
class ConstraintMatrixProxy : public linearalgebra::BaseMatrix
{
public:
    ConstraintMatrixProxy(TMatrix* originalMatrix, sofa::Size offset) : m_originalMatrix(originalMatrix), m_offset(offset) {}
    ConstraintMatrixProxy() = delete;
    Index rowSize() const override { return m_originalMatrix->rowSize(); }
    Index colSize() const override { return m_originalMatrix->colSize(); }
    SReal element(Index i, Index j) const override { return 0; }
    void resize(Index nbRow, Index nbCol) override {}
    void clear() override {}
    void set(Index i, Index j, double v) override {}
    void add(Index row, Index col, double v) override
    {
        //the constraint matrix appears twice in the global system:
        //1) below the mechanical matrix (down-left)
        m_originalMatrix->add(row + m_offset, col, v);
        //2) right to the mechanical matrix (top-right)
        m_originalMatrix->add(col, row + m_offset, v);
    }

private:
    TMatrix* m_originalMatrix { nullptr };
    const sofa::Size m_offset {};
};

template<class TVector>
class ConstraintViolationProxy : public linearalgebra::BaseVector
{
public:
    ConstraintViolationProxy(TVector* originalVector, sofa::Size offset) : m_originalVector(originalVector), m_offset(offset) {}
    ConstraintViolationProxy() = delete;
    Index size() const override { return m_originalVector->size(); }
    SReal element(Index i) const override { return 0; }
    void resize(Index dim) override { }
    void clear() override {}
    void set(Index i, SReal v) override {}
    void add(Index i, SReal v) override { m_originalVector->add(i + m_offset, v); }

private:
    TVector* m_originalVector { nullptr };
    const sofa::Size m_offset {};
};

}
