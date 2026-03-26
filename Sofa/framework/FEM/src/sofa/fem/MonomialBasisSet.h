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
#include <utility> // for std::index_sequence, std::make_index_sequence

namespace sofa::fem
{

template <typename T, std::size_t Row, std::size_t Col>
using Array2d = std::array<std::array<T, Col>, Row>;

template <std::size_t Row, std::size_t Col>
using Exponent = Array2d<std::size_t, Row, Col>;


/**
 * @brief Set of monomial basis functions.
 *
 * This class provides methods to evaluate monomial basis functions and their derivatives.
 * The basis functions are defined by their exponents for each dimension.
 *
 * @tparam Real The real type used for calculations.
 * @tparam Exponents An array containing the exponents for each basis function in each dimension.
 */
template <class Real, auto& Exponents>
struct MonomialBasisSet
{
    static constexpr std::size_t BasisSize = Exponents.size();
    static constexpr std::size_t Dimension = Exponents[0].size();

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
    static constexpr Real eval(const sofa::type::Vec<Dimension, Real>& q)
    {
        static_assert(I < BasisSize, "Basis index out of range");
        return eval_impl<I>(q, std::make_index_sequence<Dimension>());
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
    static constexpr Real derivative(const sofa::type::Vec<Dimension, Real>& q)
    {
        static_assert(I < BasisSize, "Basis index out of range");
        static_assert(D < Dimension, "Derivative direction out of range");

        constexpr std::size_t exponentD = Exponents[I][D];
        if constexpr (exponentD == 0)
        {
            return static_cast<Real>(0);
        }
        else
        {
            return static_cast<Real>(exponentD) * derivative_impl<I, D>(q, std::make_index_sequence<Dimension>());
        }
    }

    /**
     * @brief Evaluates the gradient of the I-th basis function.
     *
     * @tparam I The index of the basis function.
     * @param q The coordinate at which to evaluate the gradient.
     * @return The gradient vector.
     */
    template<std::size_t I>
    static constexpr sofa::type::Vec<Dimension, Real> gradient(const sofa::type::Vec<Dimension, Real>& q)
    {
        static_assert(I < BasisSize, "Basis index out of range");
        return gradient_impl<I>(q, std::make_index_sequence<Dimension>());
    }

private:
    /**
     * @brief Internal helper to evaluate the power of a coordinate at compile-time.
     */
    template<std::size_t Exp>
    static constexpr Real pow_impl(Real base)
    {
        if constexpr (Exp == 0)
        {
            return static_cast<Real>(1);
        }
        else if constexpr (Exp == 1)
        {
            return base;
        }
        else if constexpr (Exp == 2)
        {
            return base * base;
        }
        else
        {
            Real result = base;
            for (std::size_t i = 1; i < Exp; ++i)
            {
                result *= base;
            }
            return result;
        }
    }

    template<std::size_t I, std::size_t... Dims>
    static constexpr Real eval_impl(const sofa::type::Vec<Dimension, Real>& q, std::index_sequence<Dims...>)
    {
        return (pow_impl<Exponents[I][Dims]>(q[Dims]) * ...);
    }

    template <std::size_t I, std::size_t D, std::size_t... Dims>
    static constexpr Real derivative_impl(const sofa::type::Vec<Dimension, Real>& q, std::index_sequence<Dims...>)
    {
        return (pow_impl<(Dims == D ? Exponents[I][Dims] - 1 : Exponents[I][Dims])>(q[Dims]) * ...);
    }

    template <std::size_t I, std::size_t... D>
    static constexpr sofa::type::Vec<Dimension, Real> gradient_impl(const sofa::type::Vec<Dimension, Real>& q, std::index_sequence<D...>)
    {
        return sofa::type::Vec<Dimension, Real>{ derivative<I, D>(q)... };
    }

};
}
