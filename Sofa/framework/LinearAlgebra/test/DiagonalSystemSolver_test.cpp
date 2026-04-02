
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
#include <sofa/linearalgebra/DiagonalSystemSolver.h>
#include <gtest/gtest.h>
#include <sofa/testing/NumericTest.h>
#include <sofa/type/Mat.h>
#include <sofa/type/Vec.h>

namespace sofa
{

TEST(DiagonalSystemSolver, scalar)
{
    constexpr std::array<SReal, 3> b { 6, 8, 10 };
    constexpr std::array<SReal, 3> D { 2, 4, 5 };
    std::array<SReal, 3> x {};

    sofa::linearalgebra::solveDiagonalSystem(3, b.data(), x.data(), D.data());
    EXPECT_FLOATINGPOINT_EQ(x[0], static_cast<SReal>(3));
    EXPECT_FLOATINGPOINT_EQ(x[1], static_cast<SReal>(2));
    EXPECT_FLOATINGPOINT_EQ(x[2], static_cast<SReal>(2));
}

TEST(DiagonalSystemSolver, scalarInverted)
{
    constexpr std::array<SReal, 3> b { 6, 8, 10 };
    constexpr std::array<SReal, 3> Dinv { 0.5, 0.25, 0.2 };
    std::array<SReal, 3> x {};

    sofa::linearalgebra::solveDiagonalSystemUsingInvertedValues(3, b.data(), x.data(), Dinv.data());
    EXPECT_FLOATINGPOINT_EQ(x[0], static_cast<SReal>(3));
    EXPECT_FLOATINGPOINT_EQ(x[1], static_cast<SReal>(2));
    EXPECT_FLOATINGPOINT_EQ(x[2], static_cast<SReal>(2));
}

TEST(DiagonalSystemSolver, block2x2)
{
    using Block = sofa::type::Mat<2, 2, SReal>;
    constexpr std::array<SReal, 4> b { 1, 2, 3, 4 };
    std::array<SReal, 4> x {};

    /**
     * D = [ D0 0 ]
     *     [ 0 D1 ]
     *
     * D0 = [ 2 0 ]  => D0inv = [ 0.5 0   ]
     *      [ 0 4 ]             [ 0   0.25]
     *
     * D1 = [ 5 0 ]  => D1inv = [ 0.2 0   ]
     *      [ 0 8 ]             [ 0   0.125]
     *
     * x0 = D0inv * b0 = [ 0.5 0   ] * [ 1 ] = [ 0.5 ]
     *                   [ 0   0.25]   [ 2 ]   [ 0.5 ]
     *
     * x1 = D1inv * b1 = [ 0.2 0    ] * [ 3 ] = [ 0.6 ]
     *                   [ 0   0.125]   [ 4 ]   [ 0.5 ]
     */

    Block D0; D0.clear(); D0[0][0] = 2; D0[1][1] = 4;
    Block D1; D1.clear(); D1[0][0] = 5; D1[1][1] = 8;
    std::array<Block, 2> D_values { D0, D1 };

    sofa::linearalgebra::solveDiagonalSystem<SReal, Block>(2, b.data(), x.data(), D_values.data());

    EXPECT_FLOATINGPOINT_EQ(x[0], static_cast<SReal>(0.5));
    EXPECT_FLOATINGPOINT_EQ(x[1], static_cast<SReal>(0.5));
    EXPECT_FLOATINGPOINT_EQ(x[2], static_cast<SReal>(0.6));
    EXPECT_FLOATINGPOINT_EQ(x[3], static_cast<SReal>(0.5));
}

TEST(DiagonalSystemSolver, block2x2_nonDiagonal)
{
    using Block = sofa::type::Mat<2, 2, SReal>;
    constexpr std::array<SReal, 4> b { 1, 2, 3, 4 };
    std::array<SReal, 4> x {};

    /**
     * D = [ D0 0 ]
     *     [ 0 D1 ]
     *
     * D0 = [ 2 1 ]  => det = 2*4 - 1*1 = 7
     *      [ 1 4 ]     D0inv = 1/7 * [ 4 -1 ] = [ 4/7 -1/7 ]
     *                                [ -1 2 ]   [ -1/7 2/7 ]
     *
     * D1 = [ 5 2 ]  => det = 5*8 - 2*2 = 36
     *      [ 2 8 ]     D1inv = 1/36 * [ 8 -2 ] = [ 8/36 -2/36 ] = [ 2/9 -1/18 ]
     *                                 [ -2 5 ]   [ -2/36 5/36 ]   [ -1/18 5/36 ]
     *
     * x0 = D0inv * b0 = [ 4/7 -1/7 ] * [ 1 ] = [ 4/7 - 2/7 ] = [ 2/7 ]
     *                   [ -1/7 2/7 ]   [ 2 ]   [ -1/7 + 4/7 ]  [ 3/7 ]
     *
     * x1 = D1inv * b1 = [ 2/9 -1/18 ] * [ 3 ] = [ 6/9 - 4/18 ] = [ 2/3 - 2/9 ] = [ 4/9 ]
     *                   [ -1/18 5/36 ]  [ 4 ]   [ -3/18 + 20/36 ] = [ -1/6 + 5/9 ] = [ -3/18 + 10/18 ] = [ 7/18 ]
     */

    Block D0; D0[0][0] = 2; D0[0][1] = 1; D0[1][0] = 1; D0[1][1] = 4;
    Block D1; D1[0][0] = 5; D1[0][1] = 2; D1[1][0] = 2; D1[1][1] = 8;
    std::array<Block, 2> D_values { D0, D1 };

    sofa::linearalgebra::solveDiagonalSystem<SReal, Block>(2, b.data(), x.data(), D_values.data());

    EXPECT_FLOATINGPOINT_EQ(x[0], static_cast<SReal>(2.0/7.0));
    EXPECT_FLOATINGPOINT_EQ(x[1], static_cast<SReal>(3.0/7.0));
    EXPECT_FLOATINGPOINT_EQ(x[2], static_cast<SReal>(4.0/9.0));
    EXPECT_FLOATINGPOINT_EQ(x[3], static_cast<SReal>(7.0/18.0));
}

TEST(DiagonalSystemSolver, block2x2Inverted_nonDiagonal)
{
    using Block = sofa::type::Mat<2, 2, SReal>;
    constexpr std::array<SReal, 4> b { 1, 2, 3, 4 };
    std::array<SReal, 4> x {};

    /**
     * Same as above but providing Dinv directly
     */
    Block Dinv0; Dinv0[0][0] = 4.0/7.0; Dinv0[0][1] = -1.0/7.0; Dinv0[1][0] = -1.0/7.0; Dinv0[1][1] = 2.0/7.0;
    Block Dinv1; Dinv1[0][0] = 2.0/9.0; Dinv1[0][1] = -1.0/18.0; Dinv1[1][0] = -1.0/18.0; Dinv1[1][1] = 5.0/36.0;
    std::array<Block, 2> Dinv_values { Dinv0, Dinv1 };

    sofa::linearalgebra::solveDiagonalSystemUsingInvertedValues<SReal, Block>(2, b.data(), x.data(), Dinv_values.data());

    EXPECT_FLOATINGPOINT_EQ(x[0], static_cast<SReal>(2.0/7.0));
    EXPECT_FLOATINGPOINT_EQ(x[1], static_cast<SReal>(3.0/7.0));
    EXPECT_FLOATINGPOINT_EQ(x[2], static_cast<SReal>(4.0/9.0));
    EXPECT_FLOATINGPOINT_EQ(x[3], static_cast<SReal>(7.0/18.0));
}

TEST(DiagonalSystemSolver, block2x2Inverted)
{
    using Block = sofa::type::Mat<2, 2, SReal>;
    constexpr std::array<SReal, 4> b { 1, 2, 3, 4 };
    std::array<SReal, 4> x {};

    Block Dinv0; Dinv0.clear(); Dinv0[0][0] = 0.5; Dinv0[1][1] = 0.25;
    Block Dinv1; Dinv1.clear(); Dinv1[0][0] = 0.2; Dinv1[1][1] = 0.125;
    std::array<Block, 2> Dinv_values { Dinv0, Dinv1 };

    sofa::linearalgebra::solveDiagonalSystemUsingInvertedValues<SReal, Block>(2, b.data(), x.data(), Dinv_values.data());

    EXPECT_FLOATINGPOINT_EQ(x[0], static_cast<SReal>(0.5));
    EXPECT_FLOATINGPOINT_EQ(x[1], static_cast<SReal>(0.5));
    EXPECT_FLOATINGPOINT_EQ(x[2], static_cast<SReal>(0.6));
    EXPECT_FLOATINGPOINT_EQ(x[3], static_cast<SReal>(0.5));
}

}
