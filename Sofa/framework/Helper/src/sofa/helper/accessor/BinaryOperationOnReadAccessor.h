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

#include <sofa/config.h>
#include <sofa/helper/accessor/ReadAccessorVector.h>

namespace sofa::helper
{

/**
 * \brief Expression template object representing a binary operation on ReadAccessor. It is used for
 * lazy evaluation of container elements, therefore it makes sense to use this class only for
 * containers.
 * \tparam T1 A ReadAccessor on a container, but it can also be a BinaryOperationOnReadAccessor to
 * be able to chain binary operations
* \tparam T2 A ReadAccessor on a container, but it can also be a BinaryOperationOnReadAccessor to
 * be able to chain binary operations
 * \tparam BinaryOperation Type of the binary operation to apply between elements
 */
template<class T1, class T2, class BinaryOperation>
struct BinaryOperationOnReadAccessor
{
protected:

    using value_type1 = typename T1::value_type;
    using value_type2 = typename T2::value_type;

    const T1& m_left;
    const T2& m_right;
    inline static const BinaryOperation s_binaryOp {};

public:
    BinaryOperationOnReadAccessor(const T1& left, const T2& right) : m_left(left), m_right(right) {}
    BinaryOperationOnReadAccessor() = delete;

    using value_type = decltype(BinaryOperation()(value_type1{}, value_type2{}));
    value_type operator[](Size i) const
    {
        return s_binaryOp(m_left[i], m_right[i]);
    }
};

template<class T1, class Scalar, class BinaryOperation>
struct ScalarBinaryOperationOnReadAccessor
{
    static_assert(std::is_scalar_v<Scalar>);
protected:

    using value_type1 = typename T1::value_type;
    using value_type2 = Scalar;

    const T1& m_left;
    Scalar m_right;
    inline static const BinaryOperation s_binaryOp {};

public:
    ScalarBinaryOperationOnReadAccessor(const T1& left, Scalar right) : m_left(left), m_right(right) {}
    ScalarBinaryOperationOnReadAccessor() = delete;

    using value_type = decltype(BinaryOperation()(value_type1{}, value_type2{}));
    value_type operator[](Size i) const
    {
        return s_binaryOp(m_left[i], m_right);
    }
};

template<class T1, class BinaryOperation>
struct BinaryOperationOnReadAccessor<T1, float, BinaryOperation> : ScalarBinaryOperationOnReadAccessor<T1, float, BinaryOperation>
{
    BinaryOperationOnReadAccessor(const T1& left, float right)
    : ScalarBinaryOperationOnReadAccessor<T1, float, BinaryOperation>(left, right) {}
};

template<class T1, class BinaryOperation>
struct BinaryOperationOnReadAccessor<T1, double, BinaryOperation> : ScalarBinaryOperationOnReadAccessor<T1, double, BinaryOperation>
{
    BinaryOperationOnReadAccessor(const T1& left, double right)
    : ScalarBinaryOperationOnReadAccessor<T1, double, BinaryOperation>(left, right) {}
};

/*************
 * SUM
 *************/

template<class T1, class T2>
struct SumOperationOnReadAccessor : BinaryOperationOnReadAccessor<T1, T2, std::plus<>>
{
    using Inherit = BinaryOperationOnReadAccessor<T1, T2, std::plus<>>;
    SumOperationOnReadAccessor(const T1& left, const T2& right) : Inherit(left, right) {}
};


template<class TA, class TB>
SumOperationOnReadAccessor<ReadAccessorVector<TA>, ReadAccessorVector<TB>>
operator+(const ReadAccessorVector<TA>& a, const ReadAccessorVector<TB>& b)
{
    return {a, b};
}

template<class TA1, class TA2, class BinaryOperator, class TB>
SumOperationOnReadAccessor<BinaryOperationOnReadAccessor<TA1, TA2, BinaryOperator>, ReadAccessorVector<TB>>
operator+(const BinaryOperationOnReadAccessor<TA1, TA2, BinaryOperator>& a, const ReadAccessorVector<TB>& b)
{
    return {a, b};
}

template<class TA, class TB1, class TB2, class BinaryOperator>
SumOperationOnReadAccessor<ReadAccessorVector<TA>, BinaryOperationOnReadAccessor<TB1, TB2, BinaryOperator>>
operator+(const ReadAccessorVector<TA>& a, const BinaryOperationOnReadAccessor<TB1, TB2, BinaryOperator>& b)
{
    return {a, b};
}

template<class TA1, class TA2, class BinaryOperatorA, class TB1, class TB2, class BinaryOperatorB>
SumOperationOnReadAccessor<BinaryOperationOnReadAccessor<TA1, TA2, BinaryOperatorA>, BinaryOperationOnReadAccessor<TB1, TB2, BinaryOperatorB>>
operator+(const BinaryOperationOnReadAccessor<TA1, TA2, BinaryOperatorA>& a, const BinaryOperationOnReadAccessor<TB1, TB2, BinaryOperatorB>& b)
{
    return {a, b};
}

/*************
 * DIFFERENCE
 *************/


template<class T1, class T2>
struct DiffOperationOnReadAccessor : BinaryOperationOnReadAccessor<T1, T2, std::minus<>>
{
    using Inherit = BinaryOperationOnReadAccessor<T1, T2, std::minus<>>;
    DiffOperationOnReadAccessor(const T1& left, const T2& right) : Inherit(left, right) {}
};

template<class TA, class TB>
DiffOperationOnReadAccessor<ReadAccessorVector<TA>, ReadAccessorVector<TB>>
operator-(const ReadAccessorVector<TA>& a, const ReadAccessorVector<TB>& b)
{
    return {a, b};
}

template<class TA1, class TA2, class BinaryOperator, class TB>
DiffOperationOnReadAccessor<BinaryOperationOnReadAccessor<TA1, TA2, BinaryOperator>, ReadAccessorVector<TB>>
operator-(const BinaryOperationOnReadAccessor<TA1, TA2, BinaryOperator>& a, const ReadAccessorVector<TB>& b)
{
    return {a, b};
}

template<class TA, class TB1, class TB2, class BinaryOperator>
DiffOperationOnReadAccessor<ReadAccessorVector<TA>, BinaryOperationOnReadAccessor<TB1, TB2, BinaryOperator>>
operator-(const ReadAccessorVector<TA>& a, const BinaryOperationOnReadAccessor<TB1, TB2, BinaryOperator>& b)
{
    return {a, b};
}

template<class TA1, class TA2, class BinaryOperatorA, class TB1, class TB2, class BinaryOperatorB>
DiffOperationOnReadAccessor<BinaryOperationOnReadAccessor<TA1, TA2, BinaryOperatorA>, BinaryOperationOnReadAccessor<TB1, TB2, BinaryOperatorB>>
operator-(const BinaryOperationOnReadAccessor<TA1, TA2, BinaryOperatorA>& a, const BinaryOperationOnReadAccessor<TB1, TB2, BinaryOperatorB>& b)
{
    return {a, b};
}

/*************
 * PRODUCT
 *************/

template<class T1, class T2>
struct MultOperationOnReadAccessor : BinaryOperationOnReadAccessor<T1, T2, std::multiplies<>>
{
    using Inherit = BinaryOperationOnReadAccessor<T1, T2, std::multiplies<>>;
    MultOperationOnReadAccessor(const T1& left, const T2& right) : Inherit(left, right) {}
};

template<class TA, class TB>
MultOperationOnReadAccessor<ReadAccessorVector<TA>, ReadAccessorVector<TB>>
operator*(const ReadAccessorVector<TA>& a, const ReadAccessorVector<TB>& b)
{
    return {a, b};
}

template<class TA1, class TA2, class BinaryOperator, class TB>
MultOperationOnReadAccessor<BinaryOperationOnReadAccessor<TA1, TA2, BinaryOperator>, ReadAccessorVector<TB>>
operator*(const BinaryOperationOnReadAccessor<TA1, TA2, BinaryOperator>& a, const ReadAccessorVector<TB>& b)
{
    return {a, b};
}

template<class TA, class TB1, class TB2, class BinaryOperator>
MultOperationOnReadAccessor<ReadAccessorVector<TA>, BinaryOperationOnReadAccessor<TB1, TB2, BinaryOperator>>
operator*(const ReadAccessorVector<TA>& a, const BinaryOperationOnReadAccessor<TB1, TB2, BinaryOperator>& b)
{
    return {a, b};
}

template<class TA1, class TA2, class BinaryOperatorA, class TB1, class TB2, class BinaryOperatorB>
MultOperationOnReadAccessor<BinaryOperationOnReadAccessor<TA1, TA2, BinaryOperatorA>, BinaryOperationOnReadAccessor<TB1, TB2, BinaryOperatorB>>
operator*(const BinaryOperationOnReadAccessor<TA1, TA2, BinaryOperatorA>& a, const BinaryOperationOnReadAccessor<TB1, TB2, BinaryOperatorB>& b)
{
    return {a, b};
}

template<class TA, class Scalar, std::enable_if_t<std::is_scalar_v<Scalar>, bool> = true>
MultOperationOnReadAccessor<ReadAccessorVector<TA>, Scalar>
operator*(const ReadAccessorVector<TA>& a, Scalar b)
{
    return {a, b};
}

template<class TA, class Scalar, std::enable_if_t<std::is_scalar_v<Scalar>, bool> = true>
MultOperationOnReadAccessor<ReadAccessorVector<TA>, Scalar>
operator*(Scalar a, const ReadAccessorVector<TA>& b)
{
    return {b, a};
}

/*************
 * DIVISION
 *************/

template<class T1, class T2>
struct DivOperationOnReadAccessor : BinaryOperationOnReadAccessor<T1, T2, std::divides<>>
{
    using Inherit = BinaryOperationOnReadAccessor<T1, T2, std::divides<>>;
    DivOperationOnReadAccessor(const T1& left, const T2& right) : Inherit(left, right) {}
};


template<class TA, class TB>
DivOperationOnReadAccessor<ReadAccessorVector<TA>, ReadAccessorVector<TB>>
operator/(const ReadAccessorVector<TA>& a, const ReadAccessorVector<TB>& b)
{
    return {a, b};
}

template<class TA1, class TA2, class BinaryOperator, class TB>
DivOperationOnReadAccessor<BinaryOperationOnReadAccessor<TA1, TA2, BinaryOperator>, ReadAccessorVector<TB>>
operator/(const BinaryOperationOnReadAccessor<TA1, TA2, BinaryOperator>& a, const ReadAccessorVector<TB>& b)
{
    return {a, b};
}

template<class TA, class TB1, class TB2, class BinaryOperator>
DivOperationOnReadAccessor<ReadAccessorVector<TA>, BinaryOperationOnReadAccessor<TB1, TB2, BinaryOperator>>
operator/(const ReadAccessorVector<TA>& a, const BinaryOperationOnReadAccessor<TB1, TB2, BinaryOperator>& b)
{
    return {a, b};
}

template<class TA1, class TA2, class BinaryOperatorA, class TB1, class TB2, class BinaryOperatorB>
DivOperationOnReadAccessor<BinaryOperationOnReadAccessor<TA1, TA2, BinaryOperatorA>, BinaryOperationOnReadAccessor<TB1, TB2, BinaryOperatorB>>
operator/(const BinaryOperationOnReadAccessor<TA1, TA2, BinaryOperatorA>& a, const BinaryOperationOnReadAccessor<TB1, TB2, BinaryOperatorB>& b)
{
    return {a, b};
}

template<class TA, class Scalar, std::enable_if_t<std::is_scalar_v<Scalar>, bool> = true>
DivOperationOnReadAccessor<ReadAccessorVector<TA>, Scalar>
operator/(const ReadAccessorVector<TA>& a, Scalar b)
{
    return {a, b};
}


}
