/*********************************************************************************
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
#include <sofa/component/linearsystem/IndirectAssemblingMatrixSystem.h>

#include <sofa/component/linearsystem/BlockTypeDeducerLocalMatrix.inl>
#include <sofa/component/linearsystem/MatrixMapping.h>

#include <sofa/simulation/mechanicalvisitor/MechanicalIdentityBlocksInJacobianVisitor.h>
using sofa::simulation::mechanicalvisitor::MechanicalIdentityBlocksInJacobianVisitor;

#include <sofa/simulation/mechanicalvisitor/MechanicalBuildMBKMatrixVisitor.h>
using sofa::simulation::mechanicalvisitor::MechanicalBuildMBKMatrixVisitor;

#include <sofa/simulation/mechanicalvisitor/MechanicalResetConstraintVisitor.h>
using sofa::simulation::mechanicalvisitor::MechanicalResetConstraintVisitor;

#include <sofa/simulation/mechanicalvisitor/MechanicalAccumulateJacobian.h>
using sofa::simulation::mechanicalvisitor::MechanicalAccumulateJacobian;

#include <sofa/core/behavior/BaseForceField.h>
#include <sofa/core/behavior/BaseMass.h>
#include <sofa/core/behavior/BaseProjectiveConstraintSet.h>
#include <sofa/simulation/Node.h>
#include <sofa/linearalgebra/BaseMatrix.h>
#include <sofa/helper/ScopedAdvancedTimer.h>
#include <sofa/component/linearsystem/MatrixMapping.h>

namespace sofa::component::linearsystem
{

/*****************************************
 * MatrixEntry
 *****************************************/

template<class TBlockType>
std::istream& operator>>( std::istream& in, MatrixEntry<TBlockType>& s )
{
    in >> s.row >> s.col >> s.value;
    return in;
}

template<class TBlockType>
std::ostream& operator<<( std::ostream& out, const MatrixEntry<TBlockType>& s)
{
    out << s.row << " " << s.col << " " << s.value;
    return out;
}

/*****************************************
 * BlockTypeDeducer_IndirectAssemblingLocalMatrix
 *****************************************/

template<class TMatrixAccumulator>
class BlockTypeDeducer_IndirectAssemblingLocalMatrix : public BlockTypeDeducerLocalMatrix<TMatrixAccumulator, BaseIndirectAssemblingLocalMatrix<TMatrixAccumulator> >
{
public:
    using Base = BlockTypeDeducerLocalMatrix<TMatrixAccumulator, BaseIndirectAssemblingLocalMatrix<TMatrixAccumulator> >;
    SOFA_CLASS(BlockTypeDeducer_IndirectAssemblingLocalMatrix, Base);
    using ReassignedMatrixType = typename Inherit1::ReassignedMatrixType;

protected:

    typename ReassignedMatrixType::SPtr instantiateLocalMatrix(core::behavior::BlockType blocType) override
    {
        msg_info() << "Instantiate a IndirectAssemblingLocalMatrix for blocks of type " << blocType;
        if (blocType == core::behavior::getScalarBlocType<float>())
        {
            const auto mat = sofa::core::objectmodel::New<IndirectAssemblingLocalMatrix<TMatrixAccumulator, float> >();
            this->getContext()->addObject(mat);
            return mat;
        }
        if (blocType == core::behavior::getMat33BlocType<float>())
        {
            const auto mat = sofa::core::objectmodel::New<IndirectAssemblingLocalMatrix<TMatrixAccumulator, sofa::type::Mat<3, 3, float> > >();
            this->getContext()->addObject(mat);
            return mat;
        }
        if (blocType == core::behavior::getMat33BlocType<double>())
        {
            const auto mat = sofa::core::objectmodel::New<IndirectAssemblingLocalMatrix<TMatrixAccumulator, sofa::type::Mat<3, 3, double> > >();
            this->getContext()->addObject(mat);
            return mat;
        }

        const auto mat = sofa::core::objectmodel::New<IndirectAssemblingLocalMatrix<TMatrixAccumulator, double> >();
        this->getContext()->addObject(mat);
        return mat;
    }
};

/*****************************************
 * IndirectAssemblingLocalMatrix
 *****************************************/

template<class TMatrixAccumulator, class BlocType, class TStrategy>
IndirectAssemblingLocalMatrix<TMatrixAccumulator, BlocType, TStrategy>::IndirectAssemblingLocalMatrix()
: Inherit1()
, d_entries(initData(&d_entries, "matrixEntries", "Contributions to be added to the system matrix. The indices are expressed in the local matrix."))
{}

template<class TMatrixAccumulator, class BlocType, class TStrategy>
void IndirectAssemblingLocalMatrix<TMatrixAccumulator, BlocType, TStrategy>::add(const no_check_policy&, sofa::SignedIndex row, sofa::SignedIndex col, float value)
{
    if constexpr (std::is_same_v<BlocType, float>)
    {
        sofa::helper::getWriteAccessor(d_entries).push_back({row, col, value});
    }
    else
    {
        SOFA_UNUSED(row);
        SOFA_UNUSED(col);
        SOFA_UNUSED(value);
        dmsg_error() << "Adding matrix bloc of type float is not supported";
    }
}
template<class TMatrixAccumulator, class BlocType, class TStrategy>
void IndirectAssemblingLocalMatrix<TMatrixAccumulator, BlocType, TStrategy>::add(const no_check_policy&, sofa::SignedIndex row, sofa::SignedIndex col, double value)
{
    if constexpr (std::is_same_v<BlocType, double>)
    {
        sofa::helper::getWriteAccessor(d_entries).push_back({row, col, value});
    }
    else
    {
        SOFA_UNUSED(row);
        SOFA_UNUSED(col);
        SOFA_UNUSED(value);
        dmsg_error() << "Adding matrix bloc of type double is not supported";
    }
}
template<class TMatrixAccumulator, class BlocType, class TStrategy>
void IndirectAssemblingLocalMatrix<TMatrixAccumulator, BlocType, TStrategy>::add(const no_check_policy&, sofa::SignedIndex row, sofa::SignedIndex col, const sofa::type::Mat<3, 3, float>& value)
{
    if constexpr (std::is_same_v<BlocType, sofa::type::Mat<3, 3, float>>)
    {
        sofa::helper::getWriteAccessor(d_entries).push_back({row, col, value});
    }
    else
    {
        SOFA_UNUSED(row);
        SOFA_UNUSED(col);
        SOFA_UNUSED(value);
        dmsg_error() << "Adding matrix bloc of type sofa::type::Mat<3, 3, float> is not supported";
    }
}
template<class TMatrixAccumulator, class BlocType, class TStrategy>
void IndirectAssemblingLocalMatrix<TMatrixAccumulator, BlocType, TStrategy>::add(const no_check_policy&, sofa::SignedIndex row, sofa::SignedIndex col, const sofa::type::Mat<3, 3, double>& value)
{
    if constexpr (std::is_same_v<BlocType, sofa::type::Mat<3, 3, double>>)
    {
        sofa::helper::getWriteAccessor(d_entries).push_back({row, col, value});
    }
    else
    {
        SOFA_UNUSED(row);
        SOFA_UNUSED(col);
        SOFA_UNUSED(value);
        dmsg_error() << "Adding matrix bloc of type sofa::type::Mat<3, 3, double> is not supported";
    }
}

template<class TMatrixAccumulator, class BlocType, class TStrategy>
void IndirectAssemblingLocalMatrix<TMatrixAccumulator, BlocType, TStrategy>::clear()
{
    sofa::helper::getWriteAccessor(d_entries).clear();
}

template<class TMatrixAccumulator, class BlocType, class TStrategy>
void IndirectAssemblingLocalMatrix<TMatrixAccumulator, BlocType, TStrategy>::addContributionsToMatrix(sofa::linearalgebra::BaseMatrix* globalMatrix, SReal factor, const sofa::type::Vec2u& positionInMatrix)
{
    if (globalMatrix)
    {
        for (const auto& matrixEntry : sofa::helper::getReadAccessor(d_entries))
        {
            globalMatrix->add(matrixEntry.row + positionInMatrix[0], matrixEntry.col + positionInMatrix[1], matrixEntry.value * factor);
        }
    }
}

/*****************************************
 * IndirectAssemblingMatrixSystem
 *****************************************/

template<class TMatrix, class TVector>
void IndirectAssemblingMatrixSystem<TMatrix, TVector>::assembleSystem(const core::MechanicalParams* mparams)
{
    sofa::helper::ScopedAdvancedTimer timer("AssembleSystem");

    {
        sofa::helper::ScopedAdvancedTimer buildMBKMatrixTimer("buildMatrices");

        //Visitor calling buildStiffnessMatrix() on all force fields, and buildMassMatrix on all masses in the current context.
        //This will accumulate all the contributions from the force field into all the local
        //matrices associated to the force field. Among the matrices, at least one matrix will be
        //used to assemble the global system.
        MechanicalBuildMBKMatrixVisitor(mparams).execute(this->getContext());
    }

    if (m_mappingGraph.hasAnyMapping())
    {
        auto cparams = core::ConstraintParams(*mparams);
        {
            sofa::helper::ScopedAdvancedTimer resetConstraintTimer("resetConstraint");
            MechanicalResetConstraintVisitor(&cparams).execute(this->getContext());
        }

        {
            // The matrix blocks stored into mapped mechanical states are reset to identity
            sofa::helper::ScopedAdvancedTimer identityTimer("jacobianIdentity");
            MechanicalIdentityBlocksInJacobianVisitor(mparams, sofa::core::MatrixDerivId::mappingJacobian()).execute(this->getContext());
        }

        {
            sofa::helper::ScopedAdvancedTimer identityTimer("accumulateJacobian");
            MechanicalAccumulateJacobian(&cparams, core::MatrixDerivId::mappingJacobian()).execute(this->getContext());
        }
    }

    {
        sofa::helper::ScopedAdvancedTimer assembleTimer("Assemble");

        for (auto* m : this->m_masses)
        {
            assembleComponent(m, mparams);
        }
        for (auto* ff : this->m_forceFields)
        {
            assembleComponent(ff, mparams);
        }
        for (auto* c : this->m_projectiveConstraints)
        {
            assembleComponent(c);
        }
    }

    {
        applyZeroDirichletCondition();
    }
}

template<class TMatrix, class TVector>
void IndirectAssemblingMatrixSystem<TMatrix, TVector>::associateLocalMatrixToComponents(const core::MechanicalParams* mparams)
{
    sofa::helper::ScopedAdvancedTimer timer("InitializeSystem");

    const sofa::Size totalSize = m_mappingGraph.getTotalNbMainDofs();
    this->d_matrixSize.setValue({totalSize, totalSize});

    {
        sofa::helper::ScopedAdvancedTimer resizeTimer("clearSystem");
        this->clearSystem();
    }
    {
        sofa::helper::ScopedAdvancedTimer resizeTimer("resizeSystem");
        this->resizeSystem(totalSize);
    }


    for(auto* m : this->m_masses)
    {
        associateLocalMatrixTo(m);
    }
    for(auto* ff : this->m_forceFields)
    {
        associateLocalMatrixTo(ff);
    }
    for(auto* c : this->m_projectiveConstraints)
    {
        associateLocalMatrixTo(c);
    }
}

template <class TMatrix, class TVector>
template <class ComponentType>
BaseIndirectAssemblingLocalMatrix<ComponentType>* IndirectAssemblingMatrixSystem<TMatrix, TVector>::createLocalMatrix(
    ComponentType* object) const
{
    msg_info() << "Adding a local matrix to object " << object->getPathName();
    // add a local matrix that will create the appropriate type of local matrix based on the type of blocks the object manipulates
    const auto mat = sofa::core::objectmodel::New<BlockTypeDeducer_IndirectAssemblingLocalMatrix<ComponentType> >();
    mat->setName(object->getName() + "_deducer_matrix");
    mat->f_printLog.setValue(this->f_printLog.getValue());
    mat->addTag(core::objectmodel::Tag(core::behavior::tagSetupByMatrixLinearSystem));
    object->addSlave(mat);
    return mat.get();
}

template <class TMatrix, class TVector>
template <class ComponentType>
void IndirectAssemblingMatrixSystem<TMatrix, TVector>::associateLocalMatrixTo(ComponentType* component)
{
    sofa::type::vector<BaseIndirectAssemblingLocalMatrix<ComponentType>*> listLocalMatrices;

    if (component)
    {
        for (const auto slave : component->getSlaves())
        {
            if (auto mat = sofa::core::objectmodel::SPtr_dynamic_cast<BaseIndirectAssemblingLocalMatrix<ComponentType> >(slave))
            {
                listLocalMatrices.push_back(mat.get());
            }
        }

        if (listLocalMatrices.empty())
        {
            listLocalMatrices.push_back(createLocalMatrix<ComponentType>(component));
        }
    }

    const auto& mstates = component->getMechanicalStates();
    if (!mstates.empty())
    {
        const auto matrixSize = mstates.front()->getMatrixSize();
        for (auto* mat : listLocalMatrices)
        {
            mat->setMatrixSize({matrixSize, matrixSize});
        }
    }
}

template <class TMatrix, class TVector>
void IndirectAssemblingMatrixSystem<TMatrix, TVector>::associateLocalMatrixTo(
    sofa::core::behavior::BaseProjectiveConstraintSet* c)
{
    sofa::type::vector<IndirectProjectiveConstraintLocalMatrix*> listLocalMatrices;

    if (c)
    {
        for (const auto slave : c->getSlaves())
        {
            if (auto mat = sofa::core::objectmodel::SPtr_dynamic_cast<IndirectProjectiveConstraintLocalMatrix>(slave))
            {
                listLocalMatrices.push_back(mat.get());
            }
        }

        if (listLocalMatrices.empty())
        {
            msg_info() << "Adding a local matrix to projective constraint " << c->getPathName();
            const auto mat = sofa::core::objectmodel::New<IndirectProjectiveConstraintLocalMatrix>();
            mat->setName(c->getName() + "_matrix");
            mat->f_printLog.setValue(this->f_printLog.getValue());

            c->addSlave(mat);
            listLocalMatrices.push_back(mat.get());
        }
    }

    const auto states = c->getMechanicalStates();
    msg_warning_when(states.size() > 1) << "Multiple states not supported";
    if (!states.empty())
    {
        if (auto* mstate = states.front())
        {
            const auto pos = m_mappingGraph.getPositionInGlobalMatrix(mstate);
            for (auto* mat : listLocalMatrices)
            {
                mat->setPositionInGlobalMatrix(pos, mstate);
            }
        }
    }
}

template <class TMatrix, class TVector>
template <class ComponentType>
void IndirectAssemblingMatrixSystem<TMatrix, TVector>::assembleComponent(ComponentType* component, const sofa::core::MechanicalParams *mparams)
{
    if (component)
    {
        const auto factor = Inherit1::getFactorIncludingRayleighDamping(mparams, component);
        if (!m_mappingGraph.hasAnyMappingInput(component))
        {
            if (this->getSystemMatrix())
            {
                auto mstates = component->getMechanicalStates();
                if (!mstates.empty())
                {
                    for (auto slave : component->getSlaves())
                    {
                        if (auto mat = sofa::core::objectmodel::SPtr_dynamic_cast<BaseIndirectAssemblingLocalMatrix<ComponentType> >(slave))
                        {
                            if (factor != 0.0)
                            {
                                msg_info(mat.get()) << "Adding contribution with factor " << factor;
                                mat->addContributionsToMatrix(this->getSystemMatrix(), factor, m_mappingGraph.getPositionInGlobalMatrix(mstates.front()));
                            }
                        }
                    }
                }
            }
        }
        else
        {
            const auto inputs = m_mappingGraph.getTopMostMechanicalStates(component);
            const auto mstates = component->getMechanicalStates();
            if (!mstates.empty())
            {
                msg_error_when(mstates.size() > 1) << "Multiple mstates not supported";
                auto* associatedMechanicalState = mstates.front();
                // Get the stiffness (or mass) matrix as a CompressedRowSparseMatrix
                linearalgebra::CompressedRowSparseMatrix<SReal> K {};
                K.resize(associatedMechanicalState->getMatrixSize(), associatedMechanicalState->getMatrixSize());
                msg_info() << component->getPathName() << ": Matrix size: " << K.rowSize() << " x " << K.colSize();

                bool found = false;
                for (const auto slave : component->getSlaves())
                {
                    if (const auto mat = sofa::core::objectmodel::SPtr_dynamic_cast<BaseIndirectAssemblingLocalMatrix_IndirectAssemblingMatrixSystem<ComponentType> >(slave))
                    {
                        mat->addContributionsToMatrix(&K, factor, {0,0});
                        found = true;
                        break;
                    }
                }

                if (!found)
                {
                    return;
                }

                addMappedMatrixToGlobalMatrix(associatedMechanicalState, &K, m_mappingGraph, this->getSystemMatrix());

            }
        }
    }
}

template <class TMatrix, class TVector>
void IndirectAssemblingMatrixSystem<TMatrix, TVector>::assembleComponent(
    sofa::core::behavior::BaseProjectiveConstraintSet* c)
{
    for (const auto slave : c->getSlaves())
    {
        if (auto mat = sofa::core::objectmodel::SPtr_dynamic_cast<IndirectProjectiveConstraintLocalMatrix>(slave))
        {
            sofa::helper::getWriteAccessor(mat->d_discardedRowCol).clear();
            m_discarder.m_localProjectiveConstraintMatrix = mat.get();
            c->applyConstraint(&m_discarder);
        }
    }
}

template <class TMatrix, class TVector>
void IndirectAssemblingMatrixSystem<TMatrix, TVector>::Dirichlet::discardRowCol(sofa::Index row, sofa::Index col)
{
    if (m_localProjectiveConstraintMatrix)
    {
        sofa::helper::getWriteAccessor(m_localProjectiveConstraintMatrix->d_discardedRowCol)
            .push_back({row, col});
    }
}

template <class TMatrix, class TVector>
void IndirectAssemblingMatrixSystem<TMatrix, TVector>::applyZeroDirichletCondition()
{
    for (auto* c : this->m_projectiveConstraints)
    {
        for (const auto slave : c->getSlaves())
        {
            if (const auto mat = sofa::core::objectmodel::SPtr_dynamic_cast<IndirectProjectiveConstraintLocalMatrix>(slave))
            {
                const auto offset = mat->d_positionInGlobalMatrix.getValue();
                const bool symmetric = offset[0] == offset[1];
                for (const auto& rowCol  : mat->d_discardedRowCol.getValue())
                {
                    if (rowCol[0] == rowCol[1] && symmetric)
                    {
                        this->getSystemMatrix()->clearRowCol(rowCol[0] + offset[0]);
                    }
                    else
                    {
                        this->getSystemMatrix()->clearRow(rowCol[0] + offset[0]);
                        this->getSystemMatrix()->clearCol(rowCol[1] + offset[1]);
                    }
                    this->getSystemMatrix()->set(rowCol[0] + offset[0], rowCol[1]  + offset[1], 1.);
                }
            }
        }
    }
}
}
