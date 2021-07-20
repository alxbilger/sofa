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
#include <sofa/component/linearsystem/ConstantSparsityPatternSystem.h>
#include <sofa/component/linearsystem/AssemblingMatrixSystem.inl>
#include <sofa/component/linearsystem/matrixaccumulators/SparsityPatternLocalMappedMatrix.h>
#include <sofa/component/linearsystem/matrixaccumulators/ConstantLocalMappedMatrix.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::component::linearsystem
{

ConstantSparsityPatternSystem::ConstantSparsityPatternSystem()
    : Inherit1()
{
}

template <core::matrixaccumulator::Contribution c>
void ConstantSparsityPatternSystem::replaceLocalMatrixMapped(const core::MechanicalParams* mparams, LocalMatrixMaps<c, Real>& matrixMaps)
{
    for (auto& [component, localMatrix] : matrixMaps.mappedLocalMatrix)
    {
        if (auto* sparsityPatternMatrix = dynamic_cast<SparsityPatternLocalMappedMatrix<c, SReal>*>(localMatrix))
        {
            // component->removeSlave(localMatrix);
            // component->getContext()->removeObject(localMatrix);
            matrixMaps.accumulators[component].remove(localMatrix);

            const auto& insertionOrderList = sparsityPatternMatrix->getInsertionOrderList();

            const auto factor = Inherit1::getContributionFactor<c>(mparams, component);
            auto* mat = this->createLocalMatrixComponent<ConstantLocalMappedMatrix<c, SReal> >(component, factor);

            msg_info() << "Replacing " << sparsityPatternMatrix->getPathName() << " (class "
                << sparsityPatternMatrix->getClass()->className << ") by " << mat->getPathName() << " (class "
                << mat->getClass()->className << ")";

            mat->setMatrixSize(localMatrix->getMatrixSize());
            const auto& sharedMatrix = localMatrix->getMatrix();
            mat->shareMatrix(sharedMatrix);

            const auto it = std::find_if(this->m_localMappedMatrices.begin(), this->m_localMappedMatrices.end(),
                                         [&sharedMatrix](const auto& el){ return el.second == sharedMatrix; });
            if (it != this->m_localMappedMatrices.end())
            {
                const auto id = std::distance(this->m_localMappedMatrices.begin(), it);
                const auto& mapping = m_constantCRSMappingMappedMatrices[id];
                mat->insertionOrderList.reserve(insertionOrderList.size());
                for (const auto& [row, col] : insertionOrderList)
                {
                    mat->insertionOrderList.push_back(mapping.at(row + col * sharedMatrix->rows()));
                }
            }
            else
            {
                dmsg_error() << "Cannot find a matrix for this component";
            }

            component->removeSlave(localMatrix);
            localMatrix = mat;
            matrixMaps.accumulators[component].push_back(mat);
            matrixMaps.localMatrix[component] = mat;
        }
        else
        {
            dmsg_error() << "not a sparsity pattern matrix";
        }
    }
}

template <core::matrixaccumulator::Contribution c>
void ConstantSparsityPatternSystem::replaceLocalMatricesNonMapped(const core::MechanicalParams* mparams, LocalMatrixMaps<c, Real>& matrixMaps)
{
    for (auto& [component, localMatrix] : matrixMaps.localMatrix)
    {
        if (auto* sparsityPatternMatrix = dynamic_cast<SparsityPatternLocalMatrix<c>*>(localMatrix))
        {
            // component->removeSlave(localMatrix);
            // component->getContext()->removeObject(localMatrix);
            matrixMaps.accumulators[component].remove(localMatrix);

            const auto& insertionOrderList = sparsityPatternMatrix->getInsertionOrderList();

            SReal factor = Inherit1::getContributionFactor<c>(mparams, component);
            auto mat = createLocalMatrixComponent<ConstantLocalMatrix<c> >(component, factor);

            msg_info() << "Replacing " << sparsityPatternMatrix->getPathName() << " (class "
                << sparsityPatternMatrix->getClass()->className << ") by " << mat->getPathName() << " (class "
                << mat->getClass()->className << ")";

            mat->setMatrixSize(localMatrix->getMatrixSize());
            mat->setGlobalMatrix(this->getSystemMatrix());
            mat->setPositionInGlobalMatrix(localMatrix->getPositionInGlobalMatrix());

            mat->insertionOrderList.reserve(insertionOrderList.size());
            for (const auto& [row, col] : insertionOrderList)
            {
                const auto flatIndex = row + col * this->getSystemMatrix()->rows();
                const auto it = m_constantCRSMapping->find(flatIndex);
                if (it != m_constantCRSMapping->end())
                {
                    mat->insertionOrderList.push_back(it->second);
                }
                else
                {
                    msg_error() << "Could not find index " << flatIndex << " (row " << row <<
                    ", col " << col << ") in the hash table";
                }
            }

            component->removeSlave(localMatrix);
            localMatrix = mat;
            matrixMaps.accumulators[component].push_back(mat);
        }
    }
}

template<core::matrixaccumulator::Contribution c>
void ConstantSparsityPatternSystem::replaceLocalMatrices(const core::MechanicalParams* mparams,
    LocalMatrixMaps<c, Real>& matrixMaps)
{
    replaceLocalMatrixMapped<c>(mparams, matrixMaps);
    replaceLocalMatricesNonMapped<c>(mparams, matrixMaps);
}

template<>
void ConstantSparsityPatternSystem::replaceLocalMatrices(const core::MechanicalParams* mparams,
    LocalMatrixMaps<core::matrixaccumulator::Contribution::GEOMETRIC_STIFFNESS, Real>& matrixMaps)
{
    replaceLocalMatricesNonMapped<core::matrixaccumulator::Contribution::GEOMETRIC_STIFFNESS>(mparams, matrixMaps);
}

template <core::matrixaccumulator::Contribution c>
void ConstantSparsityPatternSystem::reinitLocalMatrices(LocalMatrixMaps<c, Real>& matrixMaps)
{
    for (auto& m : matrixMaps.localMatrix)
    {
        if (auto* local = dynamic_cast<ConstantLocalMatrix<c>* >(m.second))
        {
            local->currentId = 0;
        }
    }
    for (auto& m : matrixMaps.mappedLocalMatrix)
    {
        if (auto* local = dynamic_cast<ConstantLocalMappedMatrix<c, SReal>* >(m.second))
        {
            local->currentId = 0;
        }
    }
}

template <>
void ConstantSparsityPatternSystem::reinitLocalMatrices(LocalMatrixMaps<core::matrixaccumulator::Contribution::GEOMETRIC_STIFFNESS, Real>& matrixMaps)
{
    for (auto& m : matrixMaps.localMatrix)
    {
        if (auto* local = dynamic_cast<ConstantLocalMatrix<core::matrixaccumulator::Contribution::GEOMETRIC_STIFFNESS>* >(m.second))
        {
            local->currentId = 0;
        }
    }
}

void ConstantSparsityPatternSystem::buildHashTable(linearalgebra::CompressedRowSparseMatrix<SReal>& M, ConstantCRSMapping& mapping)
{
    for (unsigned int it_rows_k = 0; it_rows_k < M.rowIndex.size() ; it_rows_k ++)
    {
        const auto row = M.rowIndex[it_rows_k];
        Matrix::Range rowRange( M.rowBegin[it_rows_k], M.rowBegin[it_rows_k+1] );
        for(auto xj = rowRange.begin() ; xj < rowRange.end() ; ++xj )  // for each non-null block
        {
            const auto col = M.colsIndex[xj];
            mapping.emplace(row + col * M.rows(), xj);
        }
    }
}

void ConstantSparsityPatternSystem::applyProjectiveConstraints(const core::MechanicalParams* mparams)
{
    if (!isConstantSparsityPatternUsedYet())
    {
        auto& M = *this->getSystemMatrix();
        M.compress();

        m_constantCRSMapping = std::make_unique<ConstantCRSMapping>();

        // build the hash table from the compressed matrix
        buildHashTable(M, *m_constantCRSMapping);

        m_constantCRSMappingMappedMatrices.resize(this->m_localMappedMatrices.size());
        std::size_t i {};
        for (const auto& mat : this->m_localMappedMatrices)
        {
            mat.second->fullRows();
            buildHashTable(*mat.second, m_constantCRSMappingMappedMatrices[i++]);
        }

        //replace the local matrix components by new ones that use the hash table
        replaceLocalMatrices(mparams, getLocalMatrixMap<Contribution::STIFFNESS>());
        replaceLocalMatrices(mparams, getLocalMatrixMap<Contribution::MASS>());
        replaceLocalMatrices(mparams, getLocalMatrixMap<Contribution::DAMPING>());
        replaceLocalMatrices(mparams, getLocalMatrixMap<Contribution::GEOMETRIC_STIFFNESS>());

        m_isConstantSparsityPatternUsedYet = true;
    }
    else
    {
        dmsg_error_when(!this->getSystemMatrix()->btemp.empty()) << "Matrix is not compressed";
        for (const auto& mat : this->m_localMappedMatrices)
        {
            dmsg_error_when(!mat.second->btemp.empty()) << "Matrix is not compressed";
        }
    }

    //the hash table and the ordered lists must be created BEFORE the application of the projective constraints
    Inherit1::applyProjectiveConstraints(mparams);
}

void ConstantSparsityPatternSystem::resizeSystem(sofa::Size n)
{
    this->allocateSystem();

    if (getSystemMatrix())
    {
        if (n != getSystemMatrix()->rowSize() || n != getSystemMatrix()->colSize())
        {
            getSystemMatrix()->resize(n, n);
        }
        else
        {
            // In the CRS format, the pattern is unchanged from a time step to the next
            // Only the values are reset to 0

            auto& values = this->getSystemMatrix()->colsValue;
            std::fill(values.begin(), values.end(), 0_sreal);

            for (auto& m : m_localMappedMatrices)
            {
                std::fill(m.second->colsValue.begin(), m.second->colsValue.end(), 0_sreal);
            }
        }
    }

    this->resizeVectors(n);
}

void ConstantSparsityPatternSystem::clearSystem()
{
    this->allocateSystem();

    if (this->getSystemMatrix())
    {
        if (!isConstantSparsityPatternUsedYet())
        {
            this->getSystemMatrix()->clear();
        }
        else
        {
            auto& values = this->getSystemMatrix()->colsValue;
            std::fill(values.begin(), values.end(), 0_sreal);

            unsigned int i = 0;
            for (auto& m : m_localMappedMatrices)
            {
                std::fill(m.second->colsValue.begin(), m.second->colsValue.end(), 0_sreal);
                ++i;
            }


        }
    }

    if (getRHSVector())
    {
        getRHSVector()->clear();
    }

    if (getSolutionVector())
    {
        getSolutionVector()->clear();
    }
}

bool ConstantSparsityPatternSystem::isConstantSparsityPatternUsedYet() const
{
    return m_isConstantSparsityPatternUsedYet;
}

void ConstantSparsityPatternSystem::preAssembleSystem(const core::MechanicalParams* mechanical_params)
{
    Inherit1::preAssembleSystem(mechanical_params);

    if (isConstantSparsityPatternUsedYet())
    {
        this->getSystemMatrix()->compressed = true;
        for (const auto& mat : this->m_localMappedMatrices)
        {
            mat.second->compressed = true;
        }

        reinitLocalMatrices(getLocalMatrixMap<Contribution::STIFFNESS>());
        reinitLocalMatrices(getLocalMatrixMap<Contribution::MASS>());
        reinitLocalMatrices(getLocalMatrixMap<Contribution::DAMPING>());
        reinitLocalMatrices(getLocalMatrixMap<Contribution::GEOMETRIC_STIFFNESS>());
    }
}

BaseAssemblingMatrixAccumulator<core::matrixaccumulator::Contribution::STIFFNESS>* ConstantSparsityPatternSystem::createLocalStiffnessMatrix(
    BaseForceField* object, SReal factor) const
{
    return createLocalMatrixComponent<SparsityPatternLocalMatrix<core::matrixaccumulator::Contribution::STIFFNESS> >(object, factor);
}

BaseAssemblingMatrixAccumulator<core::matrixaccumulator::Contribution::MASS>* ConstantSparsityPatternSystem::createLocalMassMatrix(
    BaseMass* object, SReal factor) const
{
    return createLocalMatrixComponent<SparsityPatternLocalMatrix<core::matrixaccumulator::Contribution::MASS> >(object, factor);
}

BaseAssemblingMatrixAccumulator<core::matrixaccumulator::Contribution::DAMPING>*
ConstantSparsityPatternSystem::createLocalDampingMatrix(BaseForceField* object, SReal factor) const
{
    return createLocalMatrixComponent<SparsityPatternLocalMatrix<core::matrixaccumulator::Contribution::DAMPING> >(object, factor);
}

BaseAssemblingMatrixAccumulator<core::matrixaccumulator::Contribution::GEOMETRIC_STIFFNESS>* ConstantSparsityPatternSystem::createLocalGeometricStiffnessMatrix(
    BaseMapping* object, SReal factor) const
{
    return createLocalMatrixComponent<SparsityPatternLocalMatrix<core::matrixaccumulator::Contribution::GEOMETRIC_STIFFNESS> >(object, factor);
}

AssemblingMappedMatrixAccumulator<core::matrixaccumulator::Contribution::STIFFNESS, SReal>* ConstantSparsityPatternSystem::
createLocalMappedStiffnessMatrix(BaseForceField* object, SReal factor) const
{
    return createLocalMatrixComponent<SparsityPatternLocalMappedMatrix<core::matrixaccumulator::Contribution::STIFFNESS, SReal> >(object, factor);
}

AssemblingMappedMatrixAccumulator<core::matrixaccumulator::Contribution::MASS, SReal>* ConstantSparsityPatternSystem::
createLocalMappedMassMatrix(BaseMass* object, SReal factor) const
{
    return createLocalMatrixComponent<SparsityPatternLocalMappedMatrix<core::matrixaccumulator::Contribution::MASS, SReal> >(object, factor);
}

AssemblingMappedMatrixAccumulator<core::matrixaccumulator::Contribution::DAMPING, SReal>*
ConstantSparsityPatternSystem::createLocalMappedDampingMatrix(BaseForceField* object,
    SReal factor) const
{
    return createLocalMatrixComponent<SparsityPatternLocalMappedMatrix<core::matrixaccumulator::Contribution::DAMPING, SReal> >(object, factor);
}

int ConstantSparsityPatternSystemClass = core::RegisterObject("Linear system taking advantage of the constant sparsity pattern of the global matrix to speed up the matrix assembly. Do not use if sparsity pattern is not constant (topological changes, ...).")
    .add<ConstantSparsityPatternSystem>();

}
