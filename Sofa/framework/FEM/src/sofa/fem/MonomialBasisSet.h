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

#include <sofa/type/Vec.h>

namespace sofa::fem
{
/**
 * @brief Set of monomial basis functions.
 *
 * This class provides methods to evaluate monomial basis functions and their derivatives.
 * The basis functions are defined by their exponents for each dimension.
 *
 * @tparam Real The real type used for calculations.
 * @tparam Dim The dimension of the space.
 * @tparam TBasisSize The number of basis functions in the set.
 * @tparam Exponents An array containing the exponents for each basis function in each dimension.
 */
template <class Real, std::size_t Dim, std::size_t TBasisSize,
    std::array<std::array<std::size_t, Dim>, TBasisSize> Exponents>
struct MonomialBasisSet
{
    static constexpr std::size_t BasisSize = TBasisSize;
    static constexpr std::size_t Dimension = Dim;

    /**
     * @brief Evaluates the I-th basis function at the given coordinate.
     *
     * The evaluation is performed as: product(q[d]^Exponents[I][d]) for d from 0 to Dim-1.
     *
     * @tparam I The index of the basis function to evaluate.
     * @param q The coordinate at which to evaluate the basis function.
     * @return The value of the I-th basis function at q.
     */
    template<std::size_t I>
    static constexpr Real eval(const sofa::type::Vec<Dim, Real>& q)
    {
        static_assert(I < BasisSize, "Basis index out of range");

        Real value = static_cast<Real>(1);
        for (std::size_t d = 0; d < Dim; ++d)
        {
            const auto exponent = Exponents[I][d];
            for (std::size_t k = 0; k < exponent; ++k)
            {
                value *= q[d];
            }
        }
        return value;
    }

    /**
     * @brief Evaluates the derivative of the I-th basis function with respect to the D-th dimension.
     *
     * @tparam I The index of the basis function.
     * @tparam D The dimension with respect to which the derivative is taken.
     * @param q The coordinate at which to evaluate the derivative.
     * @return The value of the derivative.
     */
    template<std::size_t I, std::size_t D>
    static constexpr Real derivative(const sofa::type::Vec<Dim, Real>& q)
    {
        static_assert(I < BasisSize, "Basis index out of range");
        static_assert(D < Dim, "Derivative direction out of range");

        constexpr std::size_t exponentD = Exponents[I][D];
        if constexpr (exponentD == 0)
        {
            return static_cast<Real>(0);
        }
        else
        {
            Real value = static_cast<Real>(exponentD);

            for (std::size_t d = 0; d < Dim; ++d)
            {
                std::size_t exponent = Exponents[I][d];
                if (d == D)
                {
                    --exponent;
                }

                for (std::size_t k = 0; k < exponent; ++k)
                {
                    value *= q[d];
                }
            }

            return value;
        }
    }
};
}
