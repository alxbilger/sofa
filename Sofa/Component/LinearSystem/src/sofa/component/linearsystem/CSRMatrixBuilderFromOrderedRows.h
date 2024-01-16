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
#include <sofa/linearalgebra/BaseMatrix.h>
#include <sofa/linearalgebra/CompressedRowSparseMatrixGeneric.h>


namespace sofa::component::linearsystem
{

template<typename TBlock, typename TPolicy = linearalgebra::CRSDefaultPolicy>
struct CSRMatrixBuilderFromOrderedRows : linearalgebra::BaseMatrix
{
    using MatrixBackend = linearalgebra::CompressedRowSparseMatrixGeneric<TBlock, TPolicy>;
    using VecIndex = typename MatrixBackend::VecIndex;
    using VecBlock = typename MatrixBackend::VecBlock;

    MatrixBackend* m_matrix { nullptr };
    explicit CSRMatrixBuilderFromOrderedRows(MatrixBackend* matrix) : m_matrix(matrix) {}

    Index rowSize() const override { return 0; }
    Index colSize() const override { return 0; }
    SReal element(Index i, Index j) const override { return 0;}
    void resize(Index nbRow, Index nbCol) override {}
    void clear() override {}
    void set(Index i, Index j, double v) override {}

    Index m_currentRow = -1;
    bool m_isOrdered { true };

    void add(Index row, Index col, double v) override
    {
        assert(m_matrix != nullptr);

        if (v == 0)
        {
            return;
        }

        if (!m_isOrdered)
        {
            m_matrix->add(row, col, v);
            return;
        }

        if (row != m_currentRow)
        {
            m_isOrdered = row > m_currentRow;

            m_matrix->rowBegin.push_back(m_matrix->colsValue.size());
            if (m_isOrdered)
            {
                m_currentRow = row;
                m_matrix->rowIndex.push_back(row);
            }
            else
            {
                m_currentRow = -1;
                m_matrix->add(row, col, v);
                return;
            }
        }
        m_matrix->colsIndex.push_back(col);
        m_matrix->colsValue.push_back(v);
    }

    void finalize()
    {
        if (m_isOrdered)
        {
            m_matrix->rowBegin.push_back(m_matrix->colsValue.size());
        }
        m_matrix->compress();
    }
};

}
