#include <gtest/gtest.h>
#include <sofa/fem/ShapeFunction.h>
#include <sofa/fem/MonomialBasisSet.h>
#include <sofa/type/Vec.h>

namespace sofa::fem
{

TEST(ShapeFunctionTest, EvaluateAt)
{
    // Basis: 1, x, y
    static constexpr std::array<std::array<std::size_t, 2>, 3> exponents {{
        {0, 0},
        {1, 0},
        {0, 1}
    }};
    using Coord = sofa::type::Vec<2, double>;
    using Basis = MonomialBasisSet<double, exponents>;
    using SF = ShapeFunction<Coord, Basis>;

    SF sf;
    sf.coefficients = { 1.0, 2.0, 3.0 }; // N(x, y) = 1 + 2x + 3y

    Coord q1; q1.fill(0.0);
    EXPECT_DOUBLE_EQ(sf.evaluateAt(q1), 1.0);

    Coord q2; q2[0] = 0.5; q2[1] = 0.0;
    EXPECT_DOUBLE_EQ(sf.evaluateAt(q2), 2.0); // 1 + 2*0.5 = 2.0

    Coord q3; q3[0] = 0.0; q3[1] = 0.5;
    EXPECT_DOUBLE_EQ(sf.evaluateAt(q3), 2.5); // 1 + 3*0.5 = 2.5

    Coord q4; q4[0] = 0.5; q4[1] = 0.5;
    EXPECT_DOUBLE_EQ(sf.evaluateAt(q4), 3.5); // 1 + 2*0.5 + 3*0.5 = 3.5
}

TEST(ShapeFunctionTest, EvaluateAtAllZeroCoefficients)
{
    // Basis: 1, x, y
    static constexpr std::array<std::array<std::size_t, 2>, 3> exponents {{
        {0, 0},
        {1, 0},
        {0, 1}
    }};
    using Coord = sofa::type::Vec<2, double>;
    using Basis = MonomialBasisSet<double, exponents>;
    using SF = ShapeFunction<Coord, Basis>;

    SF sf;
    sf.coefficients.fill(0.0);

    Coord q1; q1[0] = 0.5; q1[1] = 0.5;
    EXPECT_DOUBLE_EQ(sf.evaluateAt(q1), 0.0);
}

TEST(ShapeFunctionTest, EvaluateAtDifferentBasis)
{
    // Basis: x^2, y^2
    static constexpr std::array<std::array<std::size_t, 2>, 2> exponents {{
        {2, 0},
        {0, 2}
    }};
    using Coord = sofa::type::Vec<2, double>;
    using Basis = MonomialBasisSet<double, exponents>;
    using SF = ShapeFunction<Coord, Basis>;

    SF sf;
    sf.coefficients = { 1.0, 1.0 }; // N(x, y) = x^2 + y^2

    Coord q1; q1[0] = 1.0; q1[1] = 0.0;
    EXPECT_DOUBLE_EQ(sf.evaluateAt(q1), 1.0);

    Coord q2; q2[0] = 0.0; q2[1] = 1.0;
    EXPECT_DOUBLE_EQ(sf.evaluateAt(q2), 1.0);

    Coord q3; q3[0] = 2.0; q3[1] = 3.0;
    EXPECT_DOUBLE_EQ(sf.evaluateAt(q3), 13.0); // 4 + 9 = 13.0
}

}
