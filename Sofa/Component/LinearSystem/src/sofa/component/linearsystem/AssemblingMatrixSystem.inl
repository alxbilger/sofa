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
#include <optional>
#include <unordered_set>
#include <mutex>
#include <sofa/component/linearsystem/AssemblingMatrixSystem.h>

#include <sofa/helper/ScopedAdvancedTimer.h>
#include <sofa/helper/AdvancedTimer.h>
#include <sofa/core/behavior/BaseForceField.h>
#include <sofa/core/behavior/BaseMass.h>
#include <sofa/core/behavior/BaseProjectiveConstraintSet.h>
#include <sofa/simulation/Node.h>
#include <sofa/component/linearsystem/MatrixMapping.h>
#include <sofa/core/behavior/BaseLocalForceFieldMatrix.h>
#include <sofa/core/behavior/BaseLocalMassMatrix.h>

#include <sofa/simulation/mechanicalvisitor/MechanicalBuildMBKMatrixVisitor.h>
using sofa::simulation::mechanicalvisitor::MechanicalBuildMBKMatrixVisitor;

#include <sofa/simulation/mechanicalvisitor/MechanicalIdentityBlocksInJacobianVisitor.h>
using sofa::simulation::mechanicalvisitor::MechanicalIdentityBlocksInJacobianVisitor;

#include <sofa/simulation/mechanicalvisitor/MechanicalResetConstraintVisitor.h>
using sofa::simulation::mechanicalvisitor::MechanicalResetConstraintVisitor;

#include <sofa/simulation/mechanicalvisitor/MechanicalAccumulateJacobian.h>
using sofa::simulation::mechanicalvisitor::MechanicalAccumulateJacobian;

namespace sofa::component::linearsystem
{

template <class TMatrix, class TVector>
AssemblingMatrixSystem<TMatrix, TVector>::AssemblingMatrixSystem()
    : Inherit1()
    , d_assembleStiffness         (initData(&d_assembleStiffness,          true,  "assembleStiffness",          "If true, the stiffness is added to the global matrix"))
    , d_assembleMass              (initData(&d_assembleMass,               true,  "assembleMass",               "If true, the mass is added to the global matrix"))
    , d_assembleMappings          (initData(&d_assembleMappings,           true,  "assembleMappings",           "If true, the geometric stiffness of mappings is added to the global matrix"))
    , d_applyProjectiveConstraints(initData(&d_applyProjectiveConstraints, true,  "applyProjectiveConstraints", "If true, projective constraints are applied on the global matrix"))
    , d_applyMappedComponents     (initData(&d_applyMappedComponents,      true,  "applyMappedComponents",      "If true, mapped components contribute to the global matrix"))
    , d_checkIndices              (initData(&d_checkIndices,               false, "checkIndices",      "If true, indices are verified before being added in to the global matrix, favoring security over speed"))
{}

template<class TMatrix, class TVector>
void AssemblingMatrixSystem<TMatrix, TVector>::assembleSystem(const core::MechanicalParams* mparams)
{
    if (this->getSystemMatrix()->rowSize() == 0 || this->getSystemMatrix()->colSize() == 0)
    {
        msg_error() << "Global system matrix is not resized appropriatly (" << this->getPathName() << ")";
        return;
    }

    sofa::helper::ScopedAdvancedTimer assembleSystemTimer("AssembleSystem");

    {
        sofa::helper::ScopedAdvancedTimer buildMatricesTimer("buildMatrices");

        if (d_assembleStiffness.getValue())
        {
            sofa::helper::ScopedAdvancedTimer buildStiffnessTimer("buildStiffness");
            for (auto* ff : this->m_forceFields)
            {
                if (Inherit1::getFactorIncludingRayleighDamping(mparams, ff) != 0.)
                {
                    auto& accumulators = stiffness.accumulators[ff];
                    if (!accumulators.empty())
                    {
                        ff->buildStiffnessMatrix(&accumulators);
                    }
                }
            }
        }
        if (d_assembleMass.getValue())
        {
            sofa::helper::ScopedAdvancedTimer buildMassTimer("buildMass");
            for (auto* m : this->m_masses)
            {
                if (Inherit1::getFactorIncludingRayleighDamping(mparams, m) != 0.)
                {
                    auto& accumulators = mass.accumulators[m];
                    if (!accumulators.empty())
                    {
                        m->buildMassMatrix(&accumulators);
                    }
                }
            }
        }
        if (d_assembleMappings.getValue())
        {
            sofa::helper::ScopedAdvancedTimer buildMapppingTimer("buildMapping");
            for (auto* m : this->m_mechanicalMappings)
            {
                auto& accumulators = mapping.accumulators[m];
                if (!accumulators.empty())
                {
                    m->buildGeometricStiffnessMatrix(&accumulators);
                }
            }
        }

    }

    if (d_applyMappedComponents.getValue() && m_mappingGraph.hasAnyMapping())
    {
        assembleMappedMatrices(mparams);
    }

    if (d_applyProjectiveConstraints.getValue())
    {
        applyProjectiveConstraints(mparams);
    }
}

template <class TMatrix, class TVector>
void AssemblingMatrixSystem<TMatrix, TVector>::makeLocalMatrixGroups(const core::MechanicalParams* mparams)
{
    m_localMappedMatrices.clear();
    const auto groups = m_mappingGraph.makeComponentGroups(mparams);
    for (const auto group : groups)
    {
        auto mstate = group.first;
        if (mstate && m_mappingGraph.hasAnyMappingInput(mstate))
        {
            std::shared_ptr<LocalMappedMatrixType<Real> > mat;

            for (const auto component : group.second.masses)
            {
                auto* massMat = mass.mappedLocalMatrix[component];
                if (massMat && massMat->getMatrix())
                {
                    mat = massMat->getMatrix();
                }
            }
            for (const auto component : group.second.forceFields)
            {
                auto* ffMat = stiffness.mappedLocalMatrix[component];
                if (ffMat && ffMat->getMatrix())
                {
                    mat = ffMat->getMatrix();
                }
            }

            if (!mat)
            {
                std::stringstream ss;
                for (const auto component : group.second.masses)
                {
                    ss << component->getPathName() << " ";
                }
                for (const auto component : group.second.forceFields)
                {
                    ss << component->getPathName() << " ";
                }
                msg_info() << "Create a matrix to be mapped, shared among the following components: " << ss.str();
                mat = std::make_shared<LocalMappedMatrixType<Real> >();
            }

            std::optional<sofa::type::Vec2u> matrixSize;
            for (const auto component : group.second.masses)
            {
                auto* massMat = mass.mappedLocalMatrix[component];
                if (massMat)
                {
                    massMat->shareMatrix(mat);
                    assert(!matrixSize.has_value() || matrixSize.value() == massMat->getMatrixSize());
                    matrixSize = massMat->getMatrixSize();
                }
            }
            for (const auto component : group.second.forceFields)
            {
                auto* ffMat = stiffness.mappedLocalMatrix[component];
                if (ffMat)
                {
                    ffMat->shareMatrix(mat);
                    assert(!matrixSize.has_value() || matrixSize.value() == ffMat->getMatrixSize());
                    matrixSize = ffMat->getMatrixSize();
                }
            }

            if (matrixSize)
            {
                mat->resize((*matrixSize)[0], (*matrixSize)[1]);
            }
            m_localMappedMatrices.emplace_back(mstate, mat);
        }
    }
}

template<class TMatrix, class TVector>
void AssemblingMatrixSystem<TMatrix, TVector>::associateLocalMatrixToComponents(const core::MechanicalParams* mparams)
{
    sofa::helper::ScopedAdvancedTimer timer("InitializeSystem");

    const sofa::Size totalSize = m_mappingGraph.getTotalNbMainDofs();
    this->d_matrixSize.setValue({totalSize, totalSize});
    m_discarder.m_globalMatrix = this->getSystemMatrix();

    {
        sofa::helper::ScopedAdvancedTimer resizeTimer("resizeSystem");
        const auto rowSize = this->getSystemMatrix() ? this->getSystemMatrix()->rowSize() : 0;
        const auto colSize = this->getSystemMatrix() ? this->getSystemMatrix()->colSize() : 0;
        this->resizeSystem(totalSize);
        const auto newRowSize = this->getSystemMatrix() ? this->getSystemMatrix()->rowSize() : 0;
        const auto newColSize = this->getSystemMatrix() ? this->getSystemMatrix()->colSize() : 0;
        msg_info_when(newRowSize != rowSize || newColSize != colSize) <<
            "System matrix is resized from " << rowSize << " x " << colSize << " to " << newRowSize << " x " << newColSize;
    }
    {
        sofa::helper::ScopedAdvancedTimer clearSystemTimer("clearSystem");
        this->clearSystem();
    }

    {
        sofa::helper::ScopedAdvancedTimer localMatricesTimer("initializeLocalMatrices");

        if (d_assembleMass.getValue())
        {
            for (auto* m : this->m_masses)
            {
                associateLocalMatrixTo(m, mparams, mass);
            }
        }

        if (d_assembleStiffness.getValue())
        {
            for (auto* ff : this->m_forceFields)
            {
                associateLocalMatrixTo(ff, mparams, stiffness);
            }
        }

        if (d_assembleMappings.getValue())
        {
            for (auto* m : this->m_mechanicalMappings)
            {
                associateLocalMatrixTo(m, mparams);
            }
        }

        makeLocalMatrixGroups(mparams);
    }
}

template <class TMatrix, class TVector>
void AssemblingMatrixSystem<TMatrix, TVector>::Dirichlet::discardRowCol(sofa::Index row, sofa::Index col)
{
    if (row == col && m_offset[0] == m_offset[1])
    {
        m_globalMatrix->clearRowCol(row + m_offset[0]);
    }
    else
    {
        m_globalMatrix->clearRow(row + m_offset[0]);
        m_globalMatrix->clearCol(col + m_offset[1]);
    }
    m_globalMatrix->set(row + m_offset[0], col  + m_offset[1], 1.);
}

template <class TMatrix, class TVector>
const MappingGraph& AssemblingMatrixSystem<TMatrix, TVector>::getMappingGraph() const
{
    return m_mappingGraph;
}

template <class TMatrix, class TVector>
template <class ComponentType>
void AssemblingMatrixSystem<TMatrix, TVector>::associateLocalMatrixTo(ComponentType* component, const core::MechanicalParams* mparams, LocalMatrixMaps<ComponentType, Real>& matrixMaps)
{
    const auto mstatesLinks = component->getMechanicalStates();

    if (mstatesLinks.empty())
    {
        return;
    }

    sofa::type::vector<core::behavior::BaseMechanicalState*> mstates;
    mstates.reserve(mstatesLinks.size());
    for (auto m : mstatesLinks)
    {
        mstates.push_back(m);
    }

    //remove duplicates: it may happen for InteractionForceFields
    std::sort( mstates.begin(), mstates.end() );
    mstates.erase( std::unique( mstates.begin(), mstates.end() ), mstates.end() );

    // Mechanical state associatd to this component (more than one association is not supported)
    core::behavior::BaseMechanicalState* mstate = mstates.front();
    msg_error_when(mstates.size() > 1) << "The component '" << component->getPathName() << "' is associated"
        " to multiple states: not supported";

    const auto factor = Inherit1::getFactorIncludingRayleighDamping(mparams, component);
    auto& strategy = matrixMaps.indexVerificationStrategy[component];
    if (d_checkIndices.getValue() && !strategy)
    {
        strategy = std::make_shared<core::matrixaccumulator::RangeVerification>();
        strategy->m_messageComponent = component;
    }

    const auto it = matrixMaps.localMatrix.find(component);
    if (it == matrixMaps.localMatrix.end())
    {
        BaseAssemblingMatrixAccumulator<ComponentType>* mat { nullptr };
        const bool isMapped = this->getMappingGraph().hasAnyMappingInput(component);
        if (isMapped) //is component mapped?
        {
            auto mappedMatrix = createLocalMappedMatrix(component, factor);
            msg_info() << "No local matrix found: a new local matrix of type " << mappedMatrix->getClass()->className << " is created and associated to " << component->getPathName();
            matrixMaps.mappedLocalMatrix.insert({component, mappedMatrix});
            mat = mappedMatrix;
        }
        else
        {
            mat = createLocalMatrix(component, factor);
            msg_info() << "No local matrix found: a new local matrix of type " << mat->getClass()->className << " is created and associated to " << component->getPathName();
        }

        matrixMaps.localMatrix.insert({component, mat});
        matrixMaps.accumulators[component].push_back(mat);

        if (mstate)
        {
            const auto matrixSize = mstate->getMatrixSize();
            mat->setGlobalMatrix(this->getSystemMatrix());
            if (!isMapped)
            {
                mat->setPositionInGlobalMatrix(this->m_mappingGraph.getPositionInGlobalMatrix(mstate));
            }
            mat->setMatrixSize({matrixSize, matrixSize});
            if (strategy)
            {
                strategy->maxRowIndex = matrixSize;
                strategy->maxColIndex = matrixSize - 1;
            }
        }
    }
    else
    {
        if (mstate)
        {
            const auto matrixSize = mstate->getMatrixSize();
            if (!this->m_mappingGraph.hasAnyMappingInput(component)) // mapped components don't add their contributions directly into the global matrix
            {
                it->second->setGlobalMatrix(this->getSystemMatrix());
                it->second->setPositionInGlobalMatrix(this->m_mappingGraph.getPositionInGlobalMatrix(mstate));
            }
            it->second->setMatrixSize({matrixSize, matrixSize});
            if (strategy)
            {
                strategy->maxRowIndex = matrixSize;
                strategy->maxColIndex = matrixSize - 1;
            }
        }
    }



}

template <class TMatrix, class TVector>
void AssemblingMatrixSystem<TMatrix, TVector>::associateLocalMatrixTo(BaseMapping* m, const core::MechanicalParams* mparams)
{
    const auto factor = sofa::core::mechanicalparams::kFactor(mparams);

    const auto mstates = m->getMechFrom();
    if (mstates.size() > 1)
    {
        static std::map<BaseMapping*, bool> emittedWarning;
        msg_warning_when(!emittedWarning[m]) << "Trying to associate a local matrix to mapping " << m->getPathName() <<
            ": Multiple mstates is not supported. It has no consequence and this warning can be ignored if the mapping is linear.";
        emittedWarning[m] = true;
        return;
    }


    // Mechanical state associatd to this component (more than one association is not supported)
    core::behavior::BaseMechanicalState* mstate = mstates.front();

    const auto it = mapping.localMatrix.find(m);
    if (it == mapping.localMatrix.end())
    {
        if (!m_mappingGraph.hasAnyMappingInput(mstate))
        {
            msg_info() << "No local matrix found: a new local matrix is created and associated to " << m->getPathName();
            auto mat = createLocalMatrix(m, factor);
            mapping.accumulators[m].push_back(mat);
            mapping.localMatrix.insert({m, mat});

            if (mstate)
            {
                const auto matrixSize = mstate->getMatrixSize();
                mat->setGlobalMatrix(this->getSystemMatrix());
                mat->setPositionInGlobalMatrix(this->m_mappingGraph.getPositionInGlobalMatrix(mstate));
                mat->setMatrixSize({matrixSize, matrixSize});
            }
        }
        else
        {
            static std::map<std::string, std::once_flag> flag;
            std::call_once(flag[m->getPathName()], [this, m]()
            {
                msg_warning() << "Geometric stiffness of mapped mapping is not supported (" << m->getPathName() << ")" ;
            });
        }
    }
    else
    {
        if (mstate && !m_mappingGraph.hasAnyMappingInput(mstate))
        {
            const auto matrixSize = mstate->getMatrixSize();
            it->second->setGlobalMatrix(this->getSystemMatrix());
            it->second->setPositionInGlobalMatrix(m_mappingGraph.getPositionInGlobalMatrix(mstate));
            it->second->setMatrixSize({matrixSize, matrixSize});
        }
    }
}

template <class TMatrix, class TVector>
template <class TLocalMatrix>
TLocalMatrix* AssemblingMatrixSystem<TMatrix, TVector>::createLocalMatrixT(typename TLocalMatrix::ComponentType* object, const SReal factor) const
{
    static_assert(std::is_base_of_v<core::objectmodel::BaseObject, TLocalMatrix>, "Template argument must be a BaseObject");
    const auto mat = sofa::core::objectmodel::New<TLocalMatrix>();
    mat->setName(object->getName() + "_matrix");
    mat->f_printLog.setValue(this->notMuted());
    mat->setFactor(factor);
    mat->associateObject(object);
    mat->addTag(core::objectmodel::Tag(core::behavior::tagSetupByMatrixLinearSystem));
    object->addSlave(mat);
    return mat.get();
}

template <class TMatrix, class TVector>
BaseAssemblingMatrixAccumulator<BaseForceField>* AssemblingMatrixSystem<TMatrix, TVector>::
createLocalMatrix(BaseForceField* object, SReal factor) const
{
    if (d_checkIndices.getValue())
    {
        auto mat = createLocalMatrixT<AssemblingMatrixAccumulator<BaseForceField, core::matrixaccumulator::RangeVerification> >(object, factor);
        const auto it = stiffness.indexVerificationStrategy.find(object);
        if (it != stiffness.indexVerificationStrategy.end())
        {
            mat->indexVerificationStrategy = it->second;
        }
        return mat;
    }
    return createLocalMatrixT<AssemblingMatrixAccumulator<BaseForceField> >(object, factor);
}

template <class TMatrix, class TVector>
BaseAssemblingMatrixAccumulator<BaseMass>* AssemblingMatrixSystem<TMatrix, TVector>::createLocalMatrix(
    BaseMass* object, SReal factor) const
{
    if (d_checkIndices.getValue())
    {
        auto mat = createLocalMatrixT<AssemblingMatrixAccumulator<BaseMass, core::matrixaccumulator::RangeVerification> >(object, factor);
        auto it = mass.indexVerificationStrategy.find(object);
        if (it != mass.indexVerificationStrategy.end())
        {
            mat->indexVerificationStrategy = it->second;
        }
        return mat;
    }
    return createLocalMatrixT<AssemblingMatrixAccumulator<BaseMass> >(object, factor);
}

template <class TMatrix, class TVector>
BaseAssemblingMatrixAccumulator<BaseMapping>* AssemblingMatrixSystem<TMatrix, TVector>::createLocalMatrix(
    BaseMapping* object, SReal factor) const
{
    return createLocalMatrixT<AssemblingMatrixAccumulator<BaseMapping> >(object, factor);
}

template <class TMatrix, class TVector>
auto AssemblingMatrixSystem<TMatrix, TVector>::
createLocalMappedMatrix(BaseForceField* object, SReal factor) const
    -> AssemblingMappedMatrixAccumulator<BaseForceField, Real>*
{
    return createLocalMatrixT<AssemblingMappedMatrixAccumulator<BaseForceField, Real> >(object, factor);
}

template <class TMatrix, class TVector>
auto AssemblingMatrixSystem<TMatrix, TVector>::
createLocalMappedMatrix(BaseMass* object, SReal factor) const
    -> AssemblingMappedMatrixAccumulator<BaseMass, Real>*
{
    return createLocalMatrixT<AssemblingMappedMatrixAccumulator<BaseMass, Real> >(object, factor);
}

template <class TMatrix, class TVector>
void AssemblingMatrixSystem<TMatrix, TVector>::projectMappedMatrices()
{
    for (const auto& mappedMatrix : m_localMappedMatrices)
    {
        LocalMappedMatrixType<Real>* crs = mappedMatrix.second.get();

        sofa::component::linearsystem::addMappedMatrixToGlobalMatrixEigen(
            mappedMatrix.first, crs, m_mappingGraph, this->getSystemMatrix());
    }
}

template <class TMatrix, class TVector>
void AssemblingMatrixSystem<TMatrix, TVector>::assembleMappedMatrices(const core::MechanicalParams* mparams)
{
    if (this->getSystemMatrix()->rowSize() == 0 || this->getSystemMatrix()->colSize() == 0)
    {
        msg_error() << "Global system matrix is not resized appropriatly";
        return;
    }

    sofa::helper::ScopedAdvancedTimer buildMappedMatricesTimer("buildMappedMatrices");
    auto cparams = core::ConstraintParams(*mparams);
    {
        sofa::helper::ScopedAdvancedTimer resetConstraintTimer("resetConstraint");
        MechanicalResetConstraintVisitor(&cparams).execute(this->getSolveContext());
    }

    {
        // The matrix blocks stored into mapped mechanical states are reset to identity
        sofa::helper::ScopedAdvancedTimer jacobianIdentityTimer("jacobianIdentity");
        MechanicalIdentityBlocksInJacobianVisitor(mparams, sofa::core::MatrixDerivId::mappingJacobian()).execute(this->getSolveContext());
    }

    {
        sofa::helper::ScopedAdvancedTimer accumulateJacobianTimer("accumulateJacobian");
        MechanicalAccumulateJacobian(&cparams, core::MatrixDerivId::mappingJacobian()).execute(this->getSolveContext());
    }

    {
        sofa::helper::ScopedAdvancedTimer matrixMappingTimer("matrixMapping");

        projectMappedMatrices();
    }
}

template <class TMatrix, class TVector>
void AssemblingMatrixSystem<TMatrix, TVector>::applyProjectiveConstraints(const core::MechanicalParams* mparams)
{
    SOFA_UNUSED(mparams);
    sofa::helper::ScopedAdvancedTimer applyProjectiveConstraintTimer("applyProjectiveConstraint");
    for (auto* constraint : this->m_projectiveConstraints)
    {
        if (constraint)
        {
            const auto& mstates = constraint->getMechanicalStates();
            if (!mstates.empty())
            {
                m_discarder.m_offset = this->getMappingGraph().getPositionInGlobalMatrix(mstates.front());
                constraint->applyConstraint(&m_discarder);
            }
        }
    }
}

}
