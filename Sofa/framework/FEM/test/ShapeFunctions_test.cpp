#include <gtest/gtest.h>
#include <sofa/fem/ShapeFunction.h>
#include <sofa/fem/MonomialBasisSet.h>
#include <sofa/type/Vec.h>

namespace sofa::fem
{

TEST(ShapeFunctionsTest, Edge1D)
{
    // Linear edge: 1, x
    static constexpr std::array<std::array<std::size_t, 1>, 2> exponents {{
        {0},
        {1}
    }};
    using Coord = sofa::type::Vec<1, double>;
    using Basis = MonomialBasisSet<double, exponents>;
    using SF = ShapeFunction<Coord, Basis>;
    using SFS = ShapeFunctions<SF>;

    // Unit edge: x=0, x=1
    std::array<Coord, 2> nodes;
    nodes[0][0] = 0.0;
    nodes[1][0] = 1.0;

    SFS shapeFunctions(nodes);

    // Test Kronecker delta property at nodes
    for (std::size_t i = 0; i < 2; ++i)
    {
        auto vals = shapeFunctions.evaluateAt(nodes[i]);
        for (std::size_t j = 0; j < 2; ++j)
        {
            EXPECT_NEAR(vals[j], (i == j ? 1.0 : 0.0), 1e-12);
        }
    }

    // Test partition of unity at various points
    for (double x = 0.0; x <= 1.0; x += 0.25)
    {
        Coord q; q[0] = x;
        auto vals = shapeFunctions.evaluateAt(q);
        double sum = 0.0;
        for (std::size_t i = 0; i < 2; ++i) sum += vals[i];
        EXPECT_NEAR(sum, 1.0, 1e-12);
    }
}

TEST(ShapeFunctionsTest, Triangle2D)
{
    // Linear triangle: 1, x, y
    static constexpr std::array<std::array<std::size_t, 2>, 3> exponents {{
        {0, 0},
        {1, 0},
        {0, 1}
    }};
    using Coord = sofa::type::Vec<2, double>;
    using Basis = MonomialBasisSet<double, exponents>;
    using SF = ShapeFunction<Coord, Basis>;
    using SFS = ShapeFunctions<SF>;

    // Unit triangle: (0,0), (1,0), (0,1)
    std::array<Coord, 3> nodes;
    nodes[0].fill(0.0);
    nodes[1][0] = 1.0; nodes[1][1] = 0.0;
    nodes[2][0] = 0.0; nodes[2][1] = 1.0;

    SFS shapeFunctions(nodes);

    // Test Kronecker delta property at nodes
    for (std::size_t i = 0; i < 3; ++i)
    {
        auto vals = shapeFunctions.evaluateAt(nodes[i]);
        for (std::size_t j = 0; j < 3; ++j)
        {
            EXPECT_NEAR(vals[j], (i == j ? 1.0 : 0.0), 1e-12);
        }
    }

    // Test partition of unity
    Coord q; q[0] = 0.3; q[1] = 0.4;
    auto vals = shapeFunctions.evaluateAt(q);
    double sum = 0.0;
    for (std::size_t i = 0; i < 3; ++i) sum += vals[i];
    EXPECT_NEAR(sum, 1.0, 1e-12);
}

TEST(ShapeFunctionsTest, Quad2D)
{
    // Bilinear quad: 1, x, y, xy
    static constexpr std::array<std::array<std::size_t, 2>, 4> exponents {{
        {0, 0},
        {1, 0},
        {0, 1},
        {1, 1}
    }};
    using Coord = sofa::type::Vec<2, double>;
    using Basis = MonomialBasisSet<double, exponents>;
    using SF = ShapeFunction<Coord, Basis>;
    using SFS = ShapeFunctions<SF>;

    // Unit quad: (0,0), (1,0), (1,1), (0,1)
    std::array<Coord, 4> nodes;
    nodes[0][0] = 0.0; nodes[0][1] = 0.0;
    nodes[1][0] = 1.0; nodes[1][1] = 0.0;
    nodes[2][0] = 1.0; nodes[2][1] = 1.0;
    nodes[3][0] = 0.0; nodes[3][1] = 1.0;

    SFS shapeFunctions(nodes);

    // Test Kronecker delta property at nodes
    for (std::size_t i = 0; i < 4; ++i)
    {
        auto vals = shapeFunctions.evaluateAt(nodes[i]);
        for (std::size_t j = 0; j < 4; ++j)
        {
            EXPECT_NEAR(vals[j], (i == j ? 1.0 : 0.0), 1e-12);
        }
    }

    // Test partition of unity
    Coord q; q[0] = 0.5; q[1] = 0.5;
    auto vals = shapeFunctions.evaluateAt(q);
    double sum = 0.0;
    for (std::size_t i = 0; i < 4; ++i) sum += vals[i];
    EXPECT_NEAR(sum, 1.0, 1e-12);
}

TEST(ShapeFunctionsTest, Hexahedron3D)
{
    // Trilinear hexahedron: 1, x, y, z, xy, yz, zx, xyz
    static constexpr std::array<std::array<std::size_t, 3>, 8> exponents {{
        {0, 0, 0},
        {1, 0, 0},
        {0, 1, 0},
        {1, 1, 0},
        {0, 0, 1},
        {1, 0, 1},
        {0, 1, 1},
        {1, 1, 1}
    }};
    using Coord = sofa::type::Vec<3, double>;
    using Basis = MonomialBasisSet<double, exponents>;
    using SF = ShapeFunction<Coord, Basis>;
    using SFS = ShapeFunctions<SF>;

    // Unit hex: (0,0,0) to (1,1,1)
    std::array<Coord, 8> nodes;
    for (int i = 0; i < 8; ++i)
    {
        nodes[i][0] = (i & 1) ? 1.0 : 0.0;
        nodes[i][1] = (i & 2) ? 1.0 : 0.0;
        nodes[i][2] = (i & 4) ? 1.0 : 0.0;
    }

    SFS shapeFunctions(nodes);

    // Test Kronecker delta property at nodes
    for (std::size_t i = 0; i < 8; ++i)
    {
        auto vals = shapeFunctions.evaluateAt(nodes[i]);
        for (std::size_t j = 0; j < 8; ++j)
        {
            EXPECT_NEAR(vals[j], (i == j ? 1.0 : 0.0), 1e-12);
        }
    }

    // Test partition of unity
    Coord q; q[0] = 0.2; q[1] = 0.4; q[2] = 0.6;
    auto vals = shapeFunctions.evaluateAt(q);
    double sum = 0.0;
    for (std::size_t i = 0; i < 8; ++i) sum += vals[i];
    EXPECT_NEAR(sum, 1.0, 1e-12);
}

TEST(ShapeFunctionsTest, Tetrahedron3D)
{
    // Linear tetrahedron: 1, x, y, z
    static constexpr std::array<std::array<std::size_t, 3>, 4> exponents {{
        {0, 0, 0},
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
    }};
    using Coord = sofa::type::Vec<3, double>;
    using Basis = MonomialBasisSet<double, exponents>;
    using SF = ShapeFunction<Coord, Basis>;
    using SFS = ShapeFunctions<SF>;

    // Unit tet: (0,0,0), (1,0,0), (0,1,0), (0,0,1)
    std::array<Coord, 4> nodes;
    nodes[0].fill(0.0);
    nodes[1][0] = 1.0; nodes[1][1] = 0.0; nodes[1][2] = 0.0;
    nodes[2][0] = 0.0; nodes[2][1] = 1.0; nodes[2][2] = 0.0;
    nodes[3][0] = 0.0; nodes[3][1] = 0.0; nodes[3][2] = 1.0;

    SFS shapeFunctions(nodes);

    // Test Kronecker delta property at nodes
    for (std::size_t i = 0; i < 4; ++i)
    {
        auto vals = shapeFunctions.evaluateAt(nodes[i]);
        for (std::size_t j = 0; j < 4; ++j)
        {
            EXPECT_NEAR(vals[j], (i == j ? 1.0 : 0.0), 1e-12);
        }
    }

    // Test partition of unity
    Coord q; q[0] = 0.1; q[1] = 0.2; q[2] = 0.3;
    auto vals = shapeFunctions.evaluateAt(q);
    double sum = 0.0;
    for (std::size_t i = 0; i < 4; ++i) sum += vals[i];
    EXPECT_NEAR(sum, 1.0, 1e-12);
}

}
