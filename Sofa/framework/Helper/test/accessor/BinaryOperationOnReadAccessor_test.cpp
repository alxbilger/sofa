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
#include <gtest/gtest.h>
#include <sofa/defaulttype/RigidCoord.h>

#include <sofa/helper/accessor.h>
#include <sofa/type/vector.h>

namespace sofa
{

TEST(SumOperationOnReadAccessor, VectorTypes_float)
{
    const sofa::type::vector<float> vector { 0.f, 1.f, 2.f, 3.f, 4.f};
    const sofa::helper::ReadAccessor accessorA(vector);
    const sofa::helper::ReadAccessor accessorB(vector);

    const sofa::helper::SumOperationOnReadAccessor sum(accessorA, accessorB);
    EXPECT_EQ(sum[0], 0.f);
    EXPECT_EQ(sum[1], 2.f);
    EXPECT_EQ(sum[2], 4.f);
    EXPECT_EQ(sum[3], 6.f);
    EXPECT_EQ(sum[4], 8.f);
}

TEST(SumOperationOnReadAccessor, VectorTypes_mixFloatDouble)
{
    const sofa::type::vector<float> vectorA { 0.f, 1.f, 2.f, 3.f, 4.f};
    const sofa::type::vector<double> vectorB { 0., 1., 2., 3., 4.};

    const sofa::helper::ReadAccessor accessorA(vectorA);
    const sofa::helper::ReadAccessor accessorB(vectorB);

    const sofa::helper::SumOperationOnReadAccessor sum(accessorA, accessorB);
    static_assert(std::is_same_v<decltype(sum)::value_type, double>);

    EXPECT_EQ(sum[0], 0.);
    EXPECT_EQ(sum[1], 2.);
    EXPECT_EQ(sum[2], 4.);
    EXPECT_EQ(sum[3], 6.);
    EXPECT_EQ(sum[4], 8.);
}

TEST(SumOperationOnReadAccessor, VectorTypes_chain)
{
    const sofa::type::vector<float> vector { 0.f, 1.f, 2.f, 3.f, 4.f};

    const sofa::helper::ReadAccessor accessorA(vector);
    const sofa::helper::ReadAccessor accessorB(vector);
    const sofa::helper::ReadAccessor accessorC(vector);

    const sofa::helper::SumOperationOnReadAccessor sumA(accessorA, accessorB);
    const sofa::helper::SumOperationOnReadAccessor sumB(sumA, accessorB);

    EXPECT_EQ(sumB[0], 0.f);
    EXPECT_EQ(sumB[1], 3.f);
    EXPECT_EQ(sumB[2], 6.f);
    EXPECT_EQ(sumB[3], 9.f);
    EXPECT_EQ(sumB[4], 12.f);
}

TEST(SumOperationOnReadAccessor, operatorSum2)
{
    const sofa::type::vector<float> vector { 0.f, 1.f, 2.f, 3.f, 4.f};

    const sofa::helper::ReadAccessor accessorA(vector);
    const sofa::helper::ReadAccessor accessorB(vector);

    const auto sum = accessorA + accessorB;

    EXPECT_EQ(sum[0], 0.);
    EXPECT_EQ(sum[1], 2.);
    EXPECT_EQ(sum[2], 4.);
    EXPECT_EQ(sum[3], 6.);
    EXPECT_EQ(sum[4], 8.);
}

TEST(SumOperationOnReadAccessor, operatorSum5)
{
    const sofa::type::vector<float> vector { 0.f, 1.f, 2.f, 3.f, 4.f};

    const sofa::helper::ReadAccessor A(vector);
    const sofa::helper::ReadAccessor B(vector);
    const sofa::helper::ReadAccessor C(vector);
    const sofa::helper::ReadAccessor D(vector);
    const sofa::helper::ReadAccessor E(vector);

    const auto sum1 = A + B + C + D + E;

    EXPECT_EQ(sum1[0], 0.f);
    EXPECT_EQ(sum1[1], 5.f);
    EXPECT_EQ(sum1[2], 10.f);
    EXPECT_EQ(sum1[3], 15.f);
    EXPECT_EQ(sum1[4], 20.f);

    const auto sum2 = (A + B) + (C + D + E);

    EXPECT_EQ(sum2[0], 0.f);
    EXPECT_EQ(sum2[1], 5.f);
    EXPECT_EQ(sum2[2], 10.f);
    EXPECT_EQ(sum2[3], 15.f);
    EXPECT_EQ(sum2[4], 20.f);

    const auto sum3 = A + (B + C + D + E);

    EXPECT_EQ(sum3[0], 0.f);
    EXPECT_EQ(sum3[1], 5.f);
    EXPECT_EQ(sum3[2], 10.f);
    EXPECT_EQ(sum3[3], 15.f);
    EXPECT_EQ(sum3[4], 20.f);
}

TEST(SumDiffOperationOnReadAccessor, operatorDiff5)
{
    const sofa::type::vector<float> vector { 0.f, 1.f, 2.f, 3.f, 4.f};

    const sofa::helper::ReadAccessor A(vector);
    const sofa::helper::ReadAccessor B(vector);
    const sofa::helper::ReadAccessor C(vector);
    const sofa::helper::ReadAccessor D(vector);
    const sofa::helper::ReadAccessor E(vector);

    const auto sum1 = A + B - (C + D + E);

    EXPECT_EQ(sum1[0], 0.f);
    EXPECT_EQ(sum1[1], -1.f);
    EXPECT_EQ(sum1[2], -2.f);
    EXPECT_EQ(sum1[3], -3.f);
    EXPECT_EQ(sum1[4], -4.f);

    const auto sum2 = (A - B) + (C - D - E);

    EXPECT_EQ(sum2[0], 0.f);
    EXPECT_EQ(sum2[1], -1.f);
    EXPECT_EQ(sum2[2], -2.f);
    EXPECT_EQ(sum2[3], -3.f);
    EXPECT_EQ(sum2[4], -4.f);

    const auto sum3 = A - (B + C + D + E);

    EXPECT_EQ(sum3[0], 0.f);
    EXPECT_EQ(sum3[1], -3.f);
    EXPECT_EQ(sum3[2], -6.f);
    EXPECT_EQ(sum3[3], -9.f);
    EXPECT_EQ(sum3[4], -12.f);
}

TEST(MultOperationOnReadAccessor, operatorMult2)
{
    const sofa::type::vector<float> vector { 0.f, 1.f, 2.f, 3.f, 4.f};

    const sofa::helper::ReadAccessor accessorA(vector);
    const sofa::helper::ReadAccessor accessorB(vector);

    const auto product = accessorA * accessorB;

    EXPECT_EQ(product[0], 0.f);
    EXPECT_EQ(product[1], 1.f);
    EXPECT_EQ(product[2], 4.f);
    EXPECT_EQ(product[3], 9.f);
    EXPECT_EQ(product[4], 16.f);
}

TEST(MultOperationOnReadAccessor, operatorMultScalarRight)
{
    const sofa::type::vector<float> vector { 0.f, 1.f, 2.f, 3.f, 4.f};

    const sofa::helper::ReadAccessor accessorA(vector);

    const auto product = accessorA * 6.f;

    EXPECT_EQ(product[0], 0.f);
    EXPECT_EQ(product[1], 6.f);
    EXPECT_EQ(product[2], 12.f);
    EXPECT_EQ(product[3], 18.f);
    EXPECT_EQ(product[4], 24.f);
}

TEST(MultOperationOnReadAccessor, operatorMultScalarRightAndAdd)
{
    const sofa::type::vector<float> vector { 0.f, 1.f, 2.f, 3.f, 4.f};

    const sofa::helper::ReadAccessor accessorA(vector);

    const auto product = accessorA * 6.f + accessorA;

    EXPECT_EQ(product[0], 0.f);
    EXPECT_EQ(product[1], 7.f);
    EXPECT_EQ(product[2], 14.f);
    EXPECT_EQ(product[3], 21.f);
    EXPECT_EQ(product[4], 28.f);
}

TEST(MultOperationOnReadAccessor, operatorMultScalarLeft)
{
    const sofa::type::vector<float> vector { 0.f, 1.f, 2.f, 3.f, 4.f};

    const sofa::helper::ReadAccessor accessorA(vector);

    const auto product = 6.f * accessorA;

    EXPECT_EQ(product[0], 0.f);
    EXPECT_EQ(product[1], 6.f);
    EXPECT_EQ(product[2], 12.f);
    EXPECT_EQ(product[3], 18.f);
    EXPECT_EQ(product[4], 24.f);
}

// This test makes sure that it is possible to manipulate two accessors which the underlying type
// is not the same. It works as long as the binary operation exists.
TEST(SumOperationOnReadAccessor, operatorSumRigid)
{
    const sofa::type::vector<defaulttype::RigidCoord<3, float>> rigids {
        {{}, {}}
    };

    const sofa::type::vector<defaulttype::RigidDeriv<3, float>> derivs {
        {{}, {}}
    };

    const sofa::helper::ReadAccessor accessorA(rigids);
    const sofa::helper::ReadAccessor accessorB(derivs);

    const auto sum = accessorA + accessorB;
    static_assert(std::is_same_v<decltype(sum)::value_type, defaulttype::RigidCoord<3, float>>);

    const defaulttype::RigidCoord<3, float> empty{};
    EXPECT_EQ(sum[0], empty);
}


TEST(DivOperationOnReadAccessor, operatorDiv)
{
    const sofa::type::vector<float> vectorA { 0.f, 1.f, 2.f, 3.f, 4.f};
    const sofa::type::vector<float> vectorB { 1.f, 1.f, 2.f, 3.f, 4.f};

    const sofa::helper::ReadAccessor accessorA(vectorA);
    const sofa::helper::ReadAccessor accessorB(vectorB);

    const auto product = accessorA / accessorB;

    EXPECT_EQ(product[0], 0.f);
    EXPECT_EQ(product[1], 1.f);
    EXPECT_EQ(product[2], 1.f);
    EXPECT_EQ(product[3], 1.f);
    EXPECT_EQ(product[4], 1.f);
}

TEST(DivOperationOnReadAccessor, operatorDivScalarRight)
{
    const sofa::type::vector<float> vector { 0.f, 1.f, 2.f, 3.f, 4.f};

    const sofa::helper::ReadAccessor accessorA(vector);

    const auto product = accessorA / 0.1f;

    EXPECT_EQ(product[0], 0.f);
    EXPECT_EQ(product[1], 10.f);
    EXPECT_EQ(product[2], 20.f);
    EXPECT_EQ(product[3], 30.f);
    EXPECT_EQ(product[4], 40.f);
}

}
