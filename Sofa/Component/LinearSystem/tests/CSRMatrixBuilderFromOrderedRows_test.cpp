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
#include <sofa/testing/BaseTest.h>
#include <sofa/component/linearsystem/CSRMatrixBuilderFromOrderedRows.h>
#include <sofa/linearalgebra/CompressedRowSparseMatrix.h>
#include <sofa/testing/LinearCongruentialRandomGenerator.h>

template<typename TBlock>
void generateMatrix(sofa::linearalgebra::CompressedRowSparseMatrix<TBlock>& matrix,
    sofa::SignedIndex nbRows, sofa::SignedIndex nbCols,
    typename sofa::linearalgebra::CompressedRowSparseMatrix<TBlock>::Real sparsity)
{
    using Real = typename sofa::linearalgebra::CompressedRowSparseMatrix<TBlock>::Real;
    const auto nbNonZero = static_cast<sofa::SignedIndex>(sparsity * static_cast<Real>(nbRows*nbCols));

    sofa::testing::LinearCongruentialRandomGenerator lcg(46515387);

    matrix.resize(nbRows, nbCols);

    for (sofa::SignedIndex i = 0; i < nbNonZero; ++i)
    {
        const auto value = static_cast<Real>(lcg.generateInUnitRange<Real>());
        const auto row = lcg.generateInRange(0_sreal, static_cast<SReal>(nbRows));
        const auto col = lcg.generateInRange(0_sreal, static_cast<SReal>(nbCols));
        matrix.add(row, col, value);
    }
    matrix.compress();
}

TEST(CSRMatrixBuilderFromOrderedRows, fromScalarToScalar)
{
    using InitialMatrixType = sofa::linearalgebra::CompressedRowSparseMatrix<SReal>;
    InitialMatrixType initialMatrix;
    generateMatrix(initialMatrix, 1500, 1500, 0.05_sreal);

    InitialMatrixType builtMatrix;
    sofa::component::linearsystem::CSRMatrixBuilderFromOrderedRows<InitialMatrixType::Block, InitialMatrixType::Policy> intermediateMatrix(&builtMatrix);

    for (InitialMatrixType::Index xi = 0; xi < (InitialMatrixType::Index)initialMatrix.rowIndex.size(); ++xi)
    {
        const auto row = initialMatrix.rowIndex[xi];
        InitialMatrixType::Range rowRange(initialMatrix.rowBegin[xi], initialMatrix.rowBegin[xi + 1]);
        for (InitialMatrixType::Index xj = rowRange.begin(); xj < rowRange.end(); ++xj)
        {
            const SReal& v = initialMatrix.colsValue[xj];
            const auto col = initialMatrix.colsIndex[xj];

            intermediateMatrix.add(row, col, v);
        }
    }

    intermediateMatrix.finalize();

    EXPECT_EQ(builtMatrix.rowIndex, initialMatrix.rowIndex);
    EXPECT_EQ(builtMatrix.rowBegin, initialMatrix.rowBegin);
    EXPECT_EQ(builtMatrix.colsIndex, initialMatrix.colsIndex);
    EXPECT_EQ(builtMatrix.colsValue, initialMatrix.colsValue);
}

template<class Block>
void foo()
{
    using InitialMatrixType = sofa::linearalgebra::CompressedRowSparseMatrix<Block>;
    InitialMatrixType initialMatrix;
    generateMatrix(initialMatrix, 1500, 1500, 0.05_sreal);

    using Index = typename InitialMatrixType::Index;

    sofa::linearalgebra::CompressedRowSparseMatrix<SReal> builtMatrix;
    sofa::component::linearsystem::CSRMatrixBuilderFromOrderedRows<SReal, typename InitialMatrixType::Policy> intermediateMatrix(&builtMatrix);

    using Triplet = std::tuple<Index, Index, SReal >;
    sofa::type::vector<Triplet> initialMatrixTriplets;

    for (Index xi = 0; xi < (Index)initialMatrix.rowIndex.size(); ++xi)
    {
        const auto row = initialMatrix.rowIndex[xi];
        typename InitialMatrixType::Range rowRange(initialMatrix.rowBegin[xi], initialMatrix.rowBegin[xi + 1]);
        for (Index xj = rowRange.begin(); xj < rowRange.end(); ++xj)
        {
            const auto& v = initialMatrix.colsValue[xj];
            const auto col = initialMatrix.colsIndex[xj];
            for (Index bi = 0; bi < InitialMatrixType::NC; ++bi)
            {
                for (Index bj = 0; bj < InitialMatrixType::NC; ++bj)
                {
                    intermediateMatrix.add(row, col + bj, InitialMatrixType::traits::v(v, bi, bj));
                    initialMatrixTriplets.emplace_back(row, col + bj, InitialMatrixType::traits::v(v, bi, bj));
                }
            }
        }
    }

    intermediateMatrix.finalize();
    EXPECT_TRUE(intermediateMatrix.m_isOrdered);

    sofa::type::vector<Triplet> builtMatrixTriplets;

    for (Index xi = 0; xi < (Index)builtMatrix.rowIndex.size(); ++xi)
    {
        const auto row = builtMatrix.rowIndex[xi];
        typename InitialMatrixType::Range rowRange(builtMatrix.rowBegin[xi], builtMatrix.rowBegin[xi + 1]);
        for (Index xj = rowRange.begin(); xj < rowRange.end(); ++xj)
        {
            const SReal& v = builtMatrix.colsValue[xj];
            const auto col = builtMatrix.colsIndex[xj];

            builtMatrixTriplets.emplace_back(row, col, v);
        }
    }
    EXPECT_EQ(initialMatrixTriplets, builtMatrixTriplets);
}


TEST(CSRMatrixBuilderFromOrderedRows, from1x3toScalar)
{
    foo<sofa::type::Vec<3, SReal> >();
}

TEST(CSRMatrixBuilderFromOrderedRows, from3x3toScalar)
{
    foo<sofa::type::Mat<3, 3, SReal> >();
}


TEST(CSRMatrixBuilderFromOrderedRows, fromScalartoScalarBis)
{
    foo<SReal>();
}
