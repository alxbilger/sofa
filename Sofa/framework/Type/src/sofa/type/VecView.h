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

namespace sofa::type
{

/**
 * @class VecView
 * @brief A utility class designed to provide a lightweight view into a contiguous range
 * of elements in a vector or similar container.
 *
 * This class allows users to access and iterate over a subset of a container
 * without copying or modifying the underlying data. It operates on any data
 * structure exposing a pointer to its elements or raw memory, such as a standard
 * vector, array, or similar constructs.
 */
template<sofa::Size N, typename ValueType>
struct VecView
{
    typedef sofa::Size       Size;
    typedef ValueType        value_type;
    typedef sofa::Size       size_type;
    typedef std::ptrdiff_t   difference_type;

    static constexpr sofa::Size static_size = N;
    static constexpr sofa::Size size() { return static_size; }

    template<sofa::Size L>
    VecView(sofa::type::Vec<L, ValueType>& vec)
        : m_data((ValueType*)vec.data())
    {}

    template<sofa::Size L>
    VecView(sofa::type::Vec<L, ValueType>& vec, sofa::Size i)
        : m_data(&vec.elems.data()[i])
    {}

    ValueType* data() const
    {
        return m_data;
    }

    const value_type& operator[](sofa::Size i) const
    {
        return m_data[i];
    }

    value_type& operator[](sofa::Size i)
    {
        return m_data[i];
    }

    sofa::type::Vec<N, ValueType> operator-() const
    {
        sofa::type::Vec<N, ValueType> res;
        for (sofa::Size i = 0; i < N; ++i)
            res[i] = -m_data[i];
        return res;
    }

    sofa::type::Vec<N, ValueType> toVec() const
    {
        sofa::type::Vec<N, ValueType> res;
        for (sofa::Size i = 0; i < N; ++i)
            res[i] = m_data[i];
        return res;
    }

    template<class T> requires (T::static_size == N)
    VecView& operator=(const T& vec)
    {
        for (sofa::Size i = 0; i < N; ++i)
            m_data[i] = vec[i];
        return *this;
    }

    template<class T> requires (T::static_size == N)
    VecView& operator+=(const T& vec)
    {
        for (sofa::Size i = 0; i < N; ++i)
            m_data[i] += vec[i];
        return *this;
    }

    template<class T> requires (T::static_size == N)
    VecView& operator-=(const T& vec)
    {
        for (sofa::Size i = 0; i < N; ++i)
            m_data[i] -= vec[i];
        return *this;
    }

    template<std::floating_point T>
    VecView& operator*=(const T& scalar)
    {
        for (sofa::Size i = 0; i < N; ++i)
            m_data[i] *= scalar;
        return *this;
    }

    template<std::floating_point T>
    VecView& operator/=(const T& scalar)
    {
        for (sofa::Size i = 0; i < N; ++i)
            m_data[i] /= scalar;
        return *this;
    }

private:
    ValueType* m_data { nullptr };
};

template<sofa::Size N, class T>
auto makeVecView(T& v)
{
    return VecView<N, typename T::value_type>(v);
}

template<sofa::Size N, class T>
auto makeVecView(T& v, sofa::Size i)
{
    return VecView<N, typename T::value_type>(v, i);
}


template<sofa::Size N, typename ValueType>
sofa::type::Vec<N, ValueType> operator*(const VecView<N, ValueType>& v, ValueType f)
{
    sofa::type::Vec<N, ValueType> res { NOINIT };
    for (sofa::Size i = 0; i < N; ++i)
        res[i] = v[i] * f;
    return res;
}

template<sofa::Size N, typename ValueType>
sofa::type::Vec<N, ValueType> operator*(ValueType f, const VecView<N, ValueType>& v)
{
    return v * f;
}

template<sofa::Size L, sofa::Size C, typename ValueType>
sofa::type::Vec<L, ValueType> operator*(const sofa::type::Mat<L, C, ValueType>& mat, const VecView<C, ValueType>& vec)
{
    sofa::type::Vec<L, ValueType> res { NOINIT};

    for (sofa::Size i = 0; i < L; ++i)
    {
        res[i] = mat(i,0) * vec[0];
        for (sofa::Size j = 1; j < C; ++j)
        {
            res[i] += mat(i, j) * vec[j];
        }
    }

    return res;
}

template<sofa::Size L, sofa::Size C, typename ValueType>
void matrixProduct(VecView<L, ValueType>& result, const sofa::type::Mat<L, C, ValueType>& mat, const VecView<C, ValueType>& vec)
{
    for (sofa::Size i = 0; i < L; ++i)
    {
        ValueType value = mat(i, 0) * vec[0];
        for (sofa::Size j = 1; j < C; ++j)
        {
            value += mat(i, j) * vec[j];
        }
        result[i] = value;
    }
}

template<sofa::Size N, typename ValueType>
sofa::type::Vec<N, ValueType> operator+(const sofa::type::Vec<N, ValueType>& a, const VecView<N, ValueType>& b)
{
    sofa::type::Vec<N, ValueType> result { NOINIT };
    for (sofa::Size i = 0; i < N; ++i)
    {
        result[i] = a[i] + b[i];
    }
    return result;
}

template<sofa::Size N, typename ValueType>
sofa::type::Vec<N, ValueType> operator+(const VecView<N, ValueType>& a, const sofa::type::Vec<N, ValueType>& b)
{
    return b + a;
}

template<sofa::Size N, typename ValueType>
sofa::type::Vec<N, ValueType> operator-(const sofa::type::Vec<N, ValueType>& a, const VecView<N, ValueType>& b)
{
    sofa::type::Vec<N, ValueType> result { NOINIT };
    for (sofa::Size i = 0; i < N; ++i)
    {
        result[i] = a[i] - b[i];
    }
    return result;
}

template<sofa::Size N, typename ValueType>
sofa::type::Vec<N, ValueType> operator-(const VecView<N, ValueType>& a, const sofa::type::Vec<N, ValueType>& b)
{
    sofa::type::Vec<N, ValueType> result { NOINIT };
    for (sofa::Size i = 0; i < N; ++i)
    {
        result[i] = a[i] - b[i];
    }
    return result;
}


}
