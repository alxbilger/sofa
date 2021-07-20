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

template <class ComponentType>
void ConstantSparsityPatternSystem::replaceLocalMatrixMapped(const core::MechanicalParams* mparams, LocalMatrixMaps<ComponentType, Real>& matrixMaps)
{
    for (auto& [component, localMatrix] : matrixMaps.mappedLocalMatrix)
    {
        if (auto* sparsityPatternMatrix = dynamic_cast<SparsityPatternLocalMappedMatrix<ComponentType, SReal>*>(localMatrix))
        {
            // component->removeSlave(localMatrix);
            // component->getContext()->removeObject(localMatrix);
            matrixMaps.accumulators[component].remove(localMatrix);

            const auto& insertionOrderList = sparsityPatternMatrix->getInsertionOrderList();

            const auto factor = Inherit1::getFactorIncludingRayleighDamping(mparams, component);
            auto* mat = createLocalMatrixT<ConstantLocalMappedMatrix<ComponentType, SReal> >(component, factor);

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

template <class ComponentType>
void ConstantSparsityPatternSystem::replaceLocalMatricesNonMapped(const core::MechanicalParams* mparams, LocalMatrixMaps<ComponentType, Real>& matrixMaps)
{
    for (auto& [component, localMatrix] : matrixMaps.localMatrix)
    {
        if (auto* sparsityPatternMatrix = dynamic_cast<SparsityPatternLocalMatrix<ComponentType>*>(localMatrix))
        {
            // component->removeSlave(localMatrix);
            // component->getContext()->removeObject(localMatrix);
            matrixMaps.accumulators[component].remove(localMatrix);

            const auto& insertionOrderList = sparsityPatternMatrix->getInsertionOrderList();

            SReal factor = Inherit1::getFactorIncludingRayleighDamping(mparams, component);
            auto mat = createLocalMatrixT<ConstantLocalMatrix<ComponentType> >(component, factor);

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

template<class ComponentType>
void ConstantSparsityPatternSystem::replaceLocalMatrices(const core::MechanicalParams* mparams,
    LocalMatrixMaps<ComponentType, Real>& matrixMaps)
{
    replaceLocalMatrixMapped<ComponentType>(mparams, matrixMaps);
    replaceLocalMatricesNonMapped<ComponentType>(mparams, matrixMaps);
}

template<>
void ConstantSparsityPatternSystem::replaceLocalMatrices(const core::MechanicalParams* mparams,
    LocalMatrixMaps<sofa::core::BaseMapping, Real>& matrixMaps)
{
    replaceLocalMatricesNonMapped<sofa::core::BaseMapping>(mparams, matrixMaps);
}

template <class ComponentType>
void ConstantSparsityPatternSystem::reinitLocalMatrices(LocalMatrixMaps<ComponentType, Real>& matrixMaps)
{
    for (auto& m : matrixMaps.localMatrix)
    {
        if (auto* local = dynamic_cast<ConstantLocalMatrix<ComponentType>* >(m.second))
        {
            local->currentId = 0;
        }
    }
    for (auto& m : matrixMaps.mappedLocalMatrix)
    {
        if (auto* local = dynamic_cast<ConstantLocalMappedMatrix<ComponentType, SReal>* >(m.second))
        {
            local->currentId = 0;
        }
    }
}

template <>
void ConstantSparsityPatternSystem::reinitLocalMatrices(LocalMatrixMaps<sofa::core::BaseMapping, Real>& matrixMaps)
{
    for (auto& m : matrixMaps.localMatrix)
    {
        if (auto* local = dynamic_cast<ConstantLocalMatrix<sofa::core::BaseMapping>* >(m.second))
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
        replaceLocalMatrices(mparams, this->stiffness);
        replaceLocalMatrices(mparams, this->mass);
        replaceLocalMatrices(mparams, this->mapping);

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
            std::fill(values.begin(), values.end(), static_cast<SReal>(0.));

            for (auto& m : m_localMappedMatrices)
            {
                std::fill(m.second->colsValue.begin(), m.second->colsValue.end(), static_cast<SReal>(0.));
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
            std::fill(values.begin(), values.end(), static_cast<SReal>(0.));

            unsigned int i = 0;
            for (auto& m : m_localMappedMatrices)
            {
                std::fill(m.second->colsValue.begin(), m.second->colsValue.end(), static_cast<SReal>(0.));
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

        reinitLocalMatrices(this->stiffness);
        reinitLocalMatrices(this->mass);
        reinitLocalMatrices(this->mapping);
    }
}

BaseAssemblingMatrixAccumulator<BaseForceField>* ConstantSparsityPatternSystem::createLocalMatrix(
    BaseForceField* object, SReal factor) const
{
    return createLocalMatrixT<SparsityPatternLocalMatrix<BaseForceField> >(object, factor);
}

BaseAssemblingMatrixAccumulator<BaseMass>* ConstantSparsityPatternSystem::createLocalMatrix(
    BaseMass* object, SReal factor) const
{
    return createLocalMatrixT<SparsityPatternLocalMatrix<BaseMass> >(object, factor);
}

BaseAssemblingMatrixAccumulator<BaseMapping>* ConstantSparsityPatternSystem::createLocalMatrix(
    BaseMapping* object, SReal factor) const
{
    return createLocalMatrixT<SparsityPatternLocalMatrix<BaseMapping> >(object, factor);
}

AssemblingMappedMatrixAccumulator<BaseForceField, SReal>* ConstantSparsityPatternSystem::
createLocalMappedMatrix(BaseForceField* object, SReal factor) const
{
    return createLocalMatrixT<SparsityPatternLocalMappedMatrix<BaseForceField, SReal> >(object, factor);
}

AssemblingMappedMatrixAccumulator<BaseMass, SReal>* ConstantSparsityPatternSystem::
createLocalMappedMatrix(BaseMass* object, SReal factor) const
{
    return createLocalMatrixT<SparsityPatternLocalMappedMatrix<BaseMass, SReal> >(object, factor);
}

int ConstantSparsityPatternSystemClass = core::RegisterObject("Linear system taking advantage of the constant sparsity pattern of the global matrix to speed up the matrix assembly. Do not use if sparsity pattern is not constant (topological changes, ...).")
    .add<ConstantSparsityPatternSystem>();

}
