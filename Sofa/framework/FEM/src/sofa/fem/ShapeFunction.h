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

#include <sofa/fem/config.h>
#include <utility>

namespace sofa::fem
{

/**
 * A linear combination of basis functions.
 *
 * A ShapeFunction is defined as:
 * \f$ N(x) = \sum_i c_i b_i(x) \f$
 * where \f$ c_i \f$ are the coefficients and \f$ b_i(x) \f$ are the basis functions.
 *
 * @tparam TCoord The coordinate type (e.g., Vec3).
 * @tparam TBasisSet The set of basis functions to use. Must provide a static `eval<I>(q)` method.
 */
template <class TCoord, class TBasisSet>
struct ShapeFunction
{
    using Coord = TCoord;
    using Real = typename Coord::value_type;
    using BasisSet = TBasisSet;
    static constexpr std::size_t BasisSize = BasisSet::BasisSize;

    /** The coefficients \f$ c_i \f$ of the linear combination. */
    std::array<Real, BasisSize> coefficients {};

    /**
     * Evaluate the shape function at a given coordinate.
     * @param q The coordinate where to evaluate the shape function.
     * @return The value of the shape function at q.
     */
    constexpr Real evaluateAt(const Coord& q) const
    {
        return evaluateImpl(q, std::make_index_sequence<BasisSize>());
    }

private:

    /**
     * Evaluate one basis function scaled by its coefficient.
     * @tparam I The index of the basis function.
     * @param q The coordinate where to evaluate.
     * @return \f$ c_I \cdot b_I(q) \f$
     */
    template<std::size_t I>
    constexpr Real evaluateScaledBasis(const Coord& q) const
    {
        return coefficients[I] * BasisSet::template eval<I>(q);
    }

    /**
     * Implementation of the evaluation using parameter pack expansion.
     * @tparam I Indices of the basis functions.
     * @param q The coordinate where to evaluate.
     * @return The sum of all scaled basis functions.
     */
    template <std::size_t... I>
    constexpr Real evaluateImpl(const Coord& q, std::index_sequence<I...>) const
    {
        return (evaluateScaledBasis<I>(q) + ...);
    }
};

/**
 * A set of shape functions \f$ N_i \f$ satisfying the Kronecker delta property at the element nodes:
 * \f$ N_i(x_j) = \delta_{ij} \f$
 *
 * @tparam TShapeFunction The type of individual shape function.
 */
template<class TShapeFunction>
struct ShapeFunctions
{
    using ShapeFunctionType = TShapeFunction;
    using Coord = typename ShapeFunctionType::Coord;
    using Real = typename ShapeFunctionType::Real;
    using BasisSet = typename ShapeFunctionType::BasisSet;
    static constexpr std::size_t BasisSize = ShapeFunctionType::BasisSize;

    /** The array of shape functions, one for each node/basis function. */
    std::array<ShapeFunctionType, BasisSize> functions {};

    /**
     * Construct the shape functions from the element nodes.
     *
     * It computes the coefficients \f$ c_{ik} \f$ such that:
     * \f$ N_i(x_j) = \sum_k c_{ik} b_k(x_j) = \delta_{ij} \f$
     *
     * In matrix form: \f$ B C^T = I \f$ where \f$ B_{jk} = b_k(x_j) \f$
     * Thus \f$ C^T = B^{-1} \f$ or \f$ C = (B^{-1})^T \f$.
     *
     * @param elementNodes The coordinates of the element nodes.
     */
    constexpr ShapeFunctions(const std::array<Coord, BasisSize>& elementNodes)
    {
        computeCoefficients(elementNodes);
    }

    /**
     * Evaluate all shape functions at a given coordinate.
     * @param q The coordinate where to evaluate.
     * @return A vector containing the values of all shape functions.
     */
    constexpr sofa::type::Vec<BasisSize, Real> evaluateAt(const Coord& q) const
    {
        sofa::type::Vec<BasisSize, Real> result { sofa::type::NOINIT };
        for (std::size_t i = 0; i < BasisSize; ++i)
        {
            result[i] = functions[i].evaluateAt(q);
        }
        return result;
    }

private:
    /**
     * Computes the coefficients for each shape function based on the element nodes.
     * @param elementNodes The coordinates of the element nodes.
     */
    constexpr void computeCoefficients(const std::array<Coord, BasisSize>& elementNodes)
    {
        // We want N_i(x_j) = δ_ij
        // Given N_i(x) = ∑_k c_ik b_k(x)
        // We need ∑_k c_ik b_k(x_j) = δ_ij
        // Let M be the matrix such that M_jk = b_k(x_j)
        // Then the equation is M C^T = I, where C_ik is the k-th coefficient of the i-th shape function.
        // Thus C^T = M^-1

        sofa::type::Mat<BasisSize, BasisSize, Real> M { sofa::type::NOINIT };

        for (std::size_t j = 0; j < BasisSize; ++j)
        {
            sofa::type::Vec<BasisSize, Real> basisValuesAtNode { sofa::type::NOINIT };
            evaluateAllBasisFunctions<BasisSize-1>(elementNodes[j], basisValuesAtNode);

            for (std::size_t k = 0; k < BasisSize; ++k)
            {
                M(j, k) = basisValuesAtNode[k];
            }
        }

        const auto canInvert = M.invert(M);
        (void)canInvert; // TODO: handle non-invertible case if necessary

        for (std::size_t i = 0; i < BasisSize; ++i)
        {
            for (std::size_t j = 0; j < BasisSize; ++j)
            {
                // C_ij = (M^-1)_ji (since C^T = M^-1)
                functions[i].coefficients[j] = M(j, i);
            }
        }
    }

    /**
     * Helper to evaluate all basis functions at a given coordinate.
     * @tparam I Index of the basis function to evaluate.
     * @param q Coordinate where to evaluate.
     * @param result Vector where to store the results.
     */
    template<std::size_t I>
    static constexpr void evaluateAllBasisFunctions(const Coord& q, sofa::type::Vec<BasisSize, Real>& result)
    {
        result[I] = BasisSet::template eval<I>(q);
        if constexpr (I > 0)
        {
            evaluateAllBasisFunctions<I - 1>(q, result);
        }
    }
};

}
