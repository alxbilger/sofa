#include <gtest/gtest.h>
#include <sofa/fem/MonomialBasisSet.h>
#include <sofa/type/Vec.h>

namespace sofa::fem
{

TEST(MonomialBasisSetTest, Eval)
{
    // Basis: 1, x, y
    constexpr std::array<std::array<std::size_t, 2>, 3> exponents {{
        {0, 0},
        {1, 0},
        {0, 1}
    }};
    using Basis = MonomialBasisSet<double, 2, 3, exponents>;

    sofa::type::Vec<2, double> q;
    q.fill(0.0);
    q[0] = 0.5;
    q[1] = 0.2;

    EXPECT_DOUBLE_EQ(Basis::eval<0>(q), 1.0);
    EXPECT_DOUBLE_EQ(Basis::eval<1>(q), 0.5);
    EXPECT_DOUBLE_EQ(Basis::eval<2>(q), 0.2);
}

TEST(MonomialBasisSetTest, Derivative)
{
    // Basis: 1, x, y, x^2, xy, y^2
    constexpr std::array<std::array<std::size_t, 2>, 6> exponents {{
        {0, 0}, // 1
        {1, 0}, // x
        {0, 1}, // y
        {2, 0}, // x^2
        {1, 1}, // xy
        {0, 2}  // y^2
    }};
    using Basis = MonomialBasisSet<double, 2, 6, exponents>;

    sofa::type::Vec<2, double> q;
    q.fill(0.0);
    q[0] = 0.5;
    q[1] = 0.2;

    // d/dx
    EXPECT_DOUBLE_EQ((Basis::derivative<0, 0>(q)), 0.0);
    EXPECT_DOUBLE_EQ((Basis::derivative<1, 0>(q)), 1.0);
    EXPECT_DOUBLE_EQ((Basis::derivative<2, 0>(q)), 0.0);
    EXPECT_DOUBLE_EQ((Basis::derivative<3, 0>(q)), 1.0); // 2*x = 2*0.5 = 1.0
    EXPECT_DOUBLE_EQ((Basis::derivative<4, 0>(q)), 0.2); // y = 0.2
    EXPECT_DOUBLE_EQ((Basis::derivative<5, 0>(q)), 0.0);

    // d/dy
    EXPECT_DOUBLE_EQ((Basis::derivative<0, 1>(q)), 0.0);
    EXPECT_DOUBLE_EQ((Basis::derivative<1, 1>(q)), 0.0);
    EXPECT_DOUBLE_EQ((Basis::derivative<2, 1>(q)), 1.0);
    EXPECT_DOUBLE_EQ((Basis::derivative<3, 1>(q)), 0.0);
    EXPECT_DOUBLE_EQ((Basis::derivative<4, 1>(q)), 0.5); // x = 0.5
    EXPECT_DOUBLE_EQ((Basis::derivative<5, 1>(q)), 0.4); // 2*y = 2*0.2 = 0.4
}

}
