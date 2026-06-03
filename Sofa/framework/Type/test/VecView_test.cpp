
#include <gtest/gtest.h>
#include <sofa/type/Mat.h>
#include <sofa/type/Vec.h>
#include <sofa/type/VecView.h>

#include <chrono>

using namespace sofa::type;

// Helper constant sizes for tests
constexpr sofa::Size TEST_SIZE = 5;

namespace sofa
{

// ===============================================================
// Test Fixture Setup
// ===============================================================
class VecViewTest : public ::testing::Test
{
   protected:
    // Concrete vector used for testing, initialized with distinct values
    sofa::type::Vec<TEST_SIZE, double> source_vec;

    void SetUp() override { source_vec.set(1.0, 2.0, 3.0, 4.0, 5.0); }
};

// ===============================================================
// Test Constructors and Initialization
// ===============================================================

TEST_F(VecViewTest, ConstructorFromVectorReference)
{
    // VecView derived from the whole vector (L=TEST_SIZE)
    VecView<TEST_SIZE, double> view1(source_vec);

    // Check if accessing elements via the view gives correct data
    EXPECT_DOUBLE_EQ(view1[0], 1.0);
    EXPECT_DOUBLE_EQ(view1[TEST_SIZE - 1], 5.0);

    // Test another instantiation (e.g., L=3)
    sofa::type::Vec<3, double> short_vec;
    short_vec.set(10.0, 20.0, 30.0);
    VecView<3, double> view2(short_vec);

    EXPECT_DOUBLE_EQ(view2[0], 10.0);
    EXPECT_DOUBLE_EQ(view2[2], 30.0);
}

TEST_F(VecViewTest, ConstructorFromVectorReferenceAndIndex)
{
    // Create a view starting at index 2 (value 3.0) with size L=3
    VecView<3, double> view(source_vec, 2);  // Start at index 2 (value 3.0)

    // We expect the internal pointer to point to source_vec[2]
    EXPECT_DOUBLE_EQ(view[0], 3.0);
    EXPECT_DOUBLE_EQ(view[1], 4.0);
    EXPECT_DOUBLE_EQ(view[2], 5.0);
}

// ===============================================================
// Test Operator Overloading (Accessors)
// ===============================================================

TEST_F(VecViewTest, IndexOperatorConstRead)
{
    // Constant access should read the value correctly
    const VecView<TEST_SIZE, double> view(source_vec);
    EXPECT_DOUBLE_EQ(view[3], 4.0);
}

TEST_F(VecViewTest, IndexOperatorNonConstWrite)
{
    // Non-constant access should allow modification of the underlying vector data
    VecView<TEST_SIZE, double> view(source_vec);
    double original_value = source_vec[1];  // Should be 2.0

    view[1] = 99.9;  // Modify element 1 via the view

    // Check if the underlying vector was actually modified
    EXPECT_DOUBLE_EQ(source_vec[1], 99.9);
    EXPECT_DOUBLE_EQ(source_vec[0], 1.0);  // Ensure other elements are untouched
}

// ===============================================================
// Test Arithmetic and Utility Methods (Member Operators)
// ===============================================================

TEST_F(VecViewTest, NegationOperatorMinus)
{
    VecView<TEST_SIZE, double> view(source_vec);
    sofa::type::Vec<TEST_SIZE, double> negated_vec = -view;  // Operator-()

    // Expected result: {-1.0, -2.0, -3.0, -4.0, -5.0}
    EXPECT_DOUBLE_EQ(negated_vec[0], -1.0);
    EXPECT_DOUBLE_EQ(negated_vec[TEST_SIZE - 1], -5.0);
}

TEST_F(VecViewTest, ToVectorConversion)
{
    VecView<TEST_SIZE, double> view(source_vec);
    sofa::type::Vec<TEST_SIZE, double> result = view.toVec();  // Method toVec()

    // Check if the resulting vector holds copies of the data
    EXPECT_DOUBLE_EQ(result[0], 1.0);
    EXPECT_DOUBLE_EQ(result[TEST_SIZE - 1], 5.0);

    // Crucially, modifying the result should NOT modify the source view/vector
    result[0] = 999.0;
    EXPECT_DOUBLE_EQ(source_vec[0], 1.0);
}

TEST_F(VecViewTest, AssignmentOperatorEquals)
{
    // Reassigning a view from a concrete vector copy (This tests the assignment implementation:
    // m_data[i] = vec[i])
    sofa::type::Vec<TEST_SIZE, double> target_vec;
    target_vec.set(100.0, 200.0, 300.0, 400.0, 500.0);

    // Initialize view with original data
    VecView<TEST_SIZE, double> view(source_vec);

    // Assignment: view = target_vec (This copies the values into whatever memory space m_data
    // currently points to)
    view = target_vec;

    // Check if the elements visible through 'view' are now from target_vec
    EXPECT_DOUBLE_EQ(view[0], 100.0);
    EXPECT_DOUBLE_EQ(view[TEST_SIZE - 1], 500.0);
}

TEST_F(VecViewTest, CompoundAssignmentOperatorPlusEquals)
{
    // Test += operation: view += vec (Copies values and adds them element-wise into m_data)
    sofa::type::Vec<TEST_SIZE, double> addition_vec;
    addition_vec.set(10.0, 10.0, 10.0, 10.0, 10.0);

    // Initialize view with original data (source_vec: {1, 2, 3, 4, 5})
    VecView<TEST_SIZE, double> view(source_vec);

    // Perform operation
    view += addition_vec;

    // Check if the elements visible through 'view' are now updated (e.g., 1+10=11)
    EXPECT_DOUBLE_EQ(view[0], 11.0);
    EXPECT_DOUBLE_EQ(view[TEST_SIZE - 1], 15.0);
}

TEST_F(VecViewTest, CompoundAssignmentOperatorMinusEquals)
{
    // Test -= operation: view -= vec (Copies values and subtracts them element-wise into m_data)
    sofa::type::Vec<TEST_SIZE, double> subtraction_vec;
    subtraction_vec.set(10.0, 10.0, 10.0, 10.0, 10.0);

    // Initialize view with original data (source_vec: {1, 2, 3, 4, 5})
    VecView<TEST_SIZE, double> view(source_vec);

    // Perform operation
    view -= subtraction_vec;

    // Check if the elements visible through 'view' are now updated (e.g., 1-10=-9)
    EXPECT_DOUBLE_EQ(view[0], -9.0);
    EXPECT_DOUBLE_EQ(view[TEST_SIZE - 1],
                     -5.0);  // Original: 5, Subtracted: 10 -> Result: -5. Wait, the setup uses
                             // addition_vec (10). Let's correct the expectation.

    // Re-running calculation based on subtraction_vec set to {10, 10, 10, 10, 10}
    EXPECT_DOUBLE_EQ(view[0], 1.0 - 10.0);              // = -9.0
    EXPECT_DOUBLE_EQ(view[TEST_SIZE - 1], 5.0 - 10.0);  // = -5.0
}

TEST_F(VecViewTest, CompoundAssignmentOperatorMultiplyEquals)
{
    // Test *= operation: view *= scalar (Multiplies elements in place by a scalar)
    double scalar = 2.5;

    // Initialize view with original data (source_vec: {1, 2, 3, 4, 5})
    VecView<TEST_SIZE, double> view(source_vec);

    // Perform operation
    view *= scalar;

    // Check if the elements visible through 'view' are updated (e.g., 1*2.5 = 2.5)
    EXPECT_DOUBLE_EQ(view[0], 2.5);
    EXPECT_DOUBLE_EQ(view[TEST_SIZE - 1], 12.5);  // 5 * 2.5 = 12.5
}

// ===============================================================
// Test Global Operator Overloads (Arithmetic Mixing)
// ===============================================================

// Test: Mat * VecView
TEST_F(VecViewTest, MatrixVectorMultiplicationMatTimesView)
{
    // Setup a mock matrix M[3x2] and an input view V[2]. Result should be Vout[3].
    sofa::type::Mat<3, 2, double> mat;

    // Manually set the mock matrix values for clarity:
    // M = | 1.0  2.0 |
    //     | 3.0  4.0 |
    //     | 5.0  6.0 |
    mat(0, 0) = 1.0;
    mat(0, 1) = 2.0;
    mat(1, 0) = 3.0;
    mat(1, 1) = 4.0;
    mat(2, 0) = 5.0;
    mat(2, 1) = 6.0;

    // Create a test vector {1.0, 1.0} viewed (C=2). We use size L=3 for the resulting vec.
    sofa::type::Vec<2, double> source_vec2;
    source_vec2.set(1.0, 1.0);
    VecView<2, double> view(source_vec2);  // View V = {1.0, 1.0}

    // Expected result:
    // R[0] = M[0][0]*V[0] + M[0][1]*V[1] = 1*1 + 2*1 = 3.0
    // R[1] = M[1][0]*V[0] + M[1][1]*V[1] = 3*1 + 4*1 = 7.0
    // R[2] = M[2][0]*V[0] + M[2][1]*V[1] = 5*1 + 6*1 = 11.0

    sofa::type::Vec<3, double> result = mat * view;  // Operator*

    EXPECT_DOUBLE_EQ(result[0], 3.0);
    EXPECT_DOUBLE_EQ(result[1], 7.0);
    EXPECT_DOUBLE_EQ(result[2], 11.0);
}

// Test: View * Scalar (LHS is view)
TEST_F(VecViewTest, GlobalMultiplicationViewTimesScalar)
{
    double scalar = 3.0;
    VecView<TEST_SIZE, double> view(source_vec);  // V = {1.0, 2.0, 3.0, 4.0, 5.0}

    // Operation: View * Scalar (v * f)
    sofa::type::Vec<TEST_SIZE, double> result = view * scalar;

    // Expected result: {1*3, 2*3, 3*3, 4*3, 5*3} = {3.0, 6.0, 9.0, 12.0, 15.0}
    EXPECT_DOUBLE_EQ(result[0], 3.0);
    EXPECT_DOUBLE_EQ(result[2], 9.0);
    EXPECT_DOUBLE_EQ(result[TEST_SIZE - 1], 15.0);
}

// Test: Scalar * View (RHS is view)
TEST_F(VecViewTest, GlobalMultiplicationScalarTimesView)
{
    double scalar = 3.0;
    VecView<TEST_SIZE, double> view(source_vec);  // V = {1.0, 2.0, 3.0, 4.0, 5.0}

    // Operation: Scalar * View (f * v)
    sofa::type::Vec<TEST_SIZE, double> result = scalar * view;

    // Expected result: {1*3, 2*3, 3*3, 4*3, 5*3} = {3.0, 6.0, 9.0, 12.0, 15.0}
    EXPECT_DOUBLE_EQ(result[0], 3.0);
    EXPECT_DOUBLE_EQ(result[2], 9.0);
    EXPECT_DOUBLE_EQ(result[TEST_SIZE - 1], 15.0);
}

// Test: Vec + View (LHS is concrete Vec)
TEST_F(VecViewTest, VectorPlusViewOrder1)
{
    sofa::type::Vec<TEST_SIZE, double> vec_a;
    vec_a.set(10.0, 20.0, 30.0, 40.0, 50.0);  // a

    // Use the view V = {1.0, 2.0, 3.0, 4.0, 5.0}
    VecView<TEST_SIZE, double> view(source_vec);  // b

    // Operation: Vec + View (a + b)
    sofa::type::Vec<TEST_SIZE, double> result = vec_a + view;  // Operator+

    EXPECT_DOUBLE_EQ(result[0], 11.0);
    EXPECT_DOUBLE_EQ(result[2], 33.0);
    EXPECT_DOUBLE_EQ(result[4], 55.0);
}

// Test: View + Vec (LHS is view)
TEST_F(VecViewTest, ViewPlusVectorOrder2)
{
    sofa::type::Vec<TEST_SIZE, double> vec_b;
    vec_b.set(10.0, 20.0, 30.0, 40.0, 50.0);  // b

    // Use the view V = {1.0, 2.0, 3.0, 4.0, 5.0}
    VecView<TEST_SIZE, double> view(source_vec);  // a

    // Operation: View + Vec (a + b). Should call implementation that swaps order.
    sofa::type::Vec<TEST_SIZE, double> result = view + vec_b;  // Operator+

    EXPECT_DOUBLE_EQ(result[0], 11.0);
    EXPECT_DOUBLE_EQ(result[2], 33.0);
    EXPECT_DOUBLE_EQ(result[4], 55.0);
}

// ===============================================================
// Performance Benchmarks (Matrix-Vector Multiplication)
// ===============================================================

TEST_F(VecViewTest, BenchmarkMatTimesSubvector_CopyVsView)
{
    using namespace std::chrono;
    constexpr sofa::Size VEC_SIZE = 12;
    constexpr sofa::Size SUB_SIZE = 3; // Dimension of the sub-vector/result
    constexpr sofa::Size NB_SUB = VEC_SIZE / SUB_SIZE;
    constexpr int ITERATIONS = 1000;  // Number of iterations for benchmarking

    // Setup: M[3x3] and V[3]. Result R[3].
    sofa::type::Mat<SUB_SIZE, SUB_SIZE, double> mat;
    mat(0, 0) = 1.0; mat(0, 1) = 2.0; mat(0, 2) = 3.0;
    mat(1, 0) = 4.0; mat(1, 1) = 5.0; mat(1, 2) = 6.0;
    mat(2, 0) = 7.0; mat(2, 1) = 8.0; mat(2, 2) = 9.0;

    // Input vector V (must be sized SUB_SIZE=3). We use the first 3 elements of source_vec
    sofa::type::Vec<VEC_SIZE, double> input_vec_storage { 1., 2., 3., 4., 5., 6., 7., 8., 9., 10., 11., 12. };
    sofa::type::Vec<VEC_SIZE, double> output;

    // --- Benchmark using Vec Copy (Explicit copy) ---
    auto start_copy = high_resolution_clock::now();
    for (int i = 0; i < ITERATIONS; ++i)
    {
        sofa::type::Vec<SUB_SIZE, double> subVec { sofa::type::NOINIT };
        for (sofa::Size j = 0; j < NB_SUB; ++j)
        {
            input_vec_storage.getsub(j * SUB_SIZE, subVec);
            auto r = mat * subVec;
            output.setsub(j * SUB_SIZE, r);
        }
    }
    auto end_copy = high_resolution_clock::now();
    long long duration_copy = duration_cast<nanoseconds>(end_copy - start_copy).count();

    // --- Benchmark using VecView (Explicit View) ---
    auto start_view = high_resolution_clock::now();
    for (int i = 0; i < ITERATIONS; ++i)
    {
        for (sofa::Size j = 0; j < NB_SUB; ++j)
        {
            // Create view from storage (avoids copying underlying data)
            auto in_view = sofa::type::makeVecView<SUB_SIZE>(input_vec_storage, j * SUB_SIZE);
            auto out_view = sofa::type::makeVecView<SUB_SIZE>(output, j * SUB_SIZE);
            sofa::type::matrixProduct(out_view, mat, in_view);
        }
    }
    auto end_view = high_resolution_clock::now();
    long long duration_view = duration_cast<nanoseconds>(end_view - start_view).count();


    // Reporting results to standard output (Requires <chrono> and <iostream>)
    std::cout << "\n--- Benchmark Results (" << ITERATIONS << " iterations) ---\n";
    std::cout << "Mat * SubVec (using Vec copy): " << duration_copy << " ns\n";
    std::cout << "Mat * SubVec (using VecView): " << duration_view << " ns\n";
}

}  // end namespace sofa
