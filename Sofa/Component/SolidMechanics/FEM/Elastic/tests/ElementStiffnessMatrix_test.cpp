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
#include <sofa/component/solidmechanics/fem/elastic/finiteelement/FiniteElement[Tetrahedron].h>
#include <sofa/component/solidmechanics/fem/elastic/impl/ElementStiffnessMatrix.h>
#include <sofa/component/solidmechanics/fem/elastic/impl/FullySymmetric4Tensor.h>
#include <sofa/component/solidmechanics/fem/elastic/impl/StrainDisplacement.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/geometry/Tetrahedron.h>

namespace sofa::component::solidmechanics::fem::elastic
{

using DataTypes = sofa::defaulttype::Vec3Types;
using ElementType = sofa::geometry::Tetrahedron;
using Real = sofa::Real_t<DataTypes>;

// Reference stiffness via full ijkl contraction; the computation is brute-forced to
// avoid replicating the logic of makeStrainDisplacement in this test.
// K[ap, bq] = factor * sum_{ijkl} B[i][j][a][p] * C[i][j][k][l] * B[k][l][b][q]
// B[i][j][a][p] = 0.5 * (gradN[a][j]*delta(i,p) + gradN[a][i]*delta(j,p))
template <class Callable>
static sofa::type::Mat<12, 12, Real> computeKRef(Callable elasticityComponents)
{
    Real C[3][3][3][3] = {};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
                for (int l = 0; l < 3; ++l) C[i][j][k][l] = elasticityComponents(i, j, k, l);

    using FE = FiniteElement<ElementType, DataTypes>;
    const auto gradN = FE::gradientShapeFunctions(FE::quadraturePoints()[0].first);

    Real B[3][3][4][3] = {};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int a = 0; a < 4; ++a)
                for (int p = 0; p < 3; ++p)
                    B[i][j][a][p] =
                        0.5 *  // This factor accounts for symmetry double counting
                        (gradN[a][j] * (i == p ? 1.0 : 0.0) + gradN[a][i] * (j == p ? 1.0 : 0.0));

    constexpr Real factor = static_cast<Real>(1) / static_cast<Real>(6);
    sofa::type::Mat<12, 12, Real> K_ref;
    for (int a = 0; a < 4; ++a)
        for (int p = 0; p < 3; ++p)
            for (int b = 0; b < 4; ++b)
                for (int q = 0; q < 3; ++q)
                {
                    Real sum = 0;
                    for (int i = 0; i < 3; ++i)
                        for (int j = 0; j < 3; ++j)
                            for (int k = 0; k < 3; ++k)
                                for (int l = 0; l < 3; ++l)
                                    sum += B[i][j][a][p] * C[i][j][k][l] * B[k][l][b][q];
                    K_ref(a * 3 + p, b * 3 + q) = factor * sum;
                }
    return K_ref;
}

// Stiffness via the Voigt path: FullySymmetric4Tensor -> integrate() -> B^T * C_voigt * B.
template <class Callable>
static sofa::type::Mat<12, 12, Real> computeKVoigt(Callable elasticityComponents)
{
    const FullySymmetric4Tensor<DataTypes> elasticityTensor{elasticityComponents};
    const std::array<sofa::Coord_t<DataTypes>, 4> nodes{
        {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    const auto K = integrate<DataTypes, ElementType>(nodes, elasticityTensor);
    return K.getAssembledMatrix();
}

static void compareStiffness(const sofa::type::Mat<12, 12, Real>& K1,
                             const sofa::type::Mat<12, 12, Real>& K2, Real tol)
{
    for (sofa::Size row = 0; row < 12; ++row)
        for (sofa::Size col = 0; col < 12; ++col)
            EXPECT_NEAR(K1(row, col), K2(row, col), tol)
                << "K(" << row << "," << col << ") differs from K_ref";
}

// Checks that row alpha of B represents the strain pair toTensorIndices(alpha),
// matching the convention used by FullySymmetric4Tensor when storing C_voigt.
TEST(makeStrainDisplacement, voigtRowOrdering)
{
    using FE = FiniteElement<ElementType, DataTypes>;
    const auto gradN = FE::gradientShapeFunctions(FE::quadraturePoints()[0].first);
    const auto B = makeStrainDisplacement<DataTypes, ElementType>(gradN);

    // Row 3 -> (1,2) = yz: B(3, ne*3+1) = gradN[ne][2], B(3, ne*3+2) = gradN[ne][1]
    EXPECT_EQ(B(3, 1), static_cast<Real>(-1));
    EXPECT_EQ(B(3, 2), static_cast<Real>(-1));
    EXPECT_EQ(B(3, 0), static_cast<Real>(0));
    EXPECT_EQ(B(3, 7), static_cast<Real>(0));
    EXPECT_EQ(B(3, 8), static_cast<Real>(1));
    EXPECT_EQ(B(3, 6), static_cast<Real>(0));
    EXPECT_EQ(B(3, 10), static_cast<Real>(1));
    EXPECT_EQ(B(3, 11), static_cast<Real>(0));

    // Row 5 -> (0,1) = xy: B(5, ne*3+0) = gradN[ne][1], B(5, ne*3+1) = gradN[ne][0]
    EXPECT_EQ(B(5, 6), static_cast<Real>(1));
    EXPECT_EQ(B(5, 7), static_cast<Real>(0));
    EXPECT_EQ(B(5, 8), static_cast<Real>(0));
    EXPECT_EQ(B(5, 9), static_cast<Real>(0));
    EXPECT_EQ(B(5, 10), static_cast<Real>(0));

    // Row 4 -> (0,2) = xz: B(4, ne*3+0) = gradN[ne][2], B(4, ne*3+2) = gradN[ne][0]
    EXPECT_EQ(B(4, 3), static_cast<Real>(0));
    EXPECT_EQ(B(4, 5), static_cast<Real>(1));
}

// Isotropic sanity check: K_ref and K_voigt must agree regardless of shear row ordering
// because all shear moduli are equal (C_yzyz = C_xyxy = mu).
// isotropicElasticity will generate the following tensor:
//        xx      yy      zz      yz      xz      xy
//  xx  [ λ+2μ    λ       λ       0       0       0  ]
//  yy  [ λ       λ+2μ    λ       0       0       0  ]
//  zz  [ λ       λ       λ+2μ    0       0       0  ]
//  yz  [ 0       0       0       μ       0       0  ]
//  xz  [ 0       0       0       0       μ       0  ]
//  xy  [ 0       0       0       0       0       μ  ]
TEST(makeStrainDisplacement, stiffnessIsotropic)
{
    constexpr Real mu = 2, lambda = 1;
    auto isotropicElasticity = [](int i, int j, int k, int l) -> Real
    {
        auto d = [](int a, int b) { return Real(a == b); };
        return lambda * d(i, j) * d(k, l) + mu * (d(i, k) * d(j, l) + d(i, l) * d(j, k));
    };
    compareStiffness(computeKRef(isotropicElasticity), computeKVoigt(isotropicElasticity), 1e-10);
}

// Orthotropic case: K_ref and K_voigt must agree when shear moduli differ across planes.
// orthotropicElasticity will generate the following tensor:
//        xx      yy      zz      yz      xz      xy
//  xx  [ 10       4       5       0       0       0  ]
//  yy  [  4      11       6       0       0       0  ]
//  zz  [  5       6      12       0       0       0  ]
//  yz  [  0       0       0       2       0       0  ]
//  xz  [  0       0       0       0       3       0  ]
//  xy  [  0       0       0       0       0       7  ]
TEST(makeStrainDisplacement, stiffnessAnisotropic)
{
    auto orthotropicElasticity = [](int i, int j, int k, int l) -> Real
    {
        auto pairMatches = [](int a, int b, int p, int q)
        { return (a == p && b == q) || (a == q && b == p); };
        if (pairMatches(i, j, 0, 0) && pairMatches(k, l, 0, 0)) return Real(10);  // xxxx
        if (pairMatches(i, j, 1, 1) && pairMatches(k, l, 1, 1)) return Real(11);  // yyyy
        if (pairMatches(i, j, 2, 2) && pairMatches(k, l, 2, 2)) return Real(12);  // zzzz
        if ((pairMatches(i, j, 0, 0) && pairMatches(k, l, 1, 1)) ||
            (pairMatches(i, j, 1, 1) && pairMatches(k, l, 0, 0)))
            return Real(4);  // xxyy
        if ((pairMatches(i, j, 0, 0) && pairMatches(k, l, 2, 2)) ||
            (pairMatches(i, j, 2, 2) && pairMatches(k, l, 0, 0)))
            return Real(5);  // xxzz
        if ((pairMatches(i, j, 1, 1) && pairMatches(k, l, 2, 2)) ||
            (pairMatches(i, j, 2, 2) && pairMatches(k, l, 1, 1)))
            return Real(6);                                                         // yyzz
        if (pairMatches(i, j, 1, 2) && pairMatches(k, l, 1, 2)) return Real(2);  // yzyz
        if (pairMatches(i, j, 0, 2) && pairMatches(k, l, 0, 2)) return Real(3);  // xzxz
        if (pairMatches(i, j, 0, 1) && pairMatches(k, l, 0, 1)) return Real(7);  // xyxy
        return Real(0);
    };
    compareStiffness(computeKRef(orthotropicElasticity), computeKVoigt(orthotropicElasticity), 1e-10);
}

}  // namespace sofa::component::solidmechanics::fem::elastic
