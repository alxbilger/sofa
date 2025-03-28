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
#include <functional>

namespace sofa::type::trait
{

// Helper to extract the argument type from a callable
template <typename T>
struct callable_argument;

// Specialization for function pointers
template <typename T>
struct callable_argument<void(*)(T*)>
{
    using argument_type = T;
};

// Specialization for std::function
template <typename T>
struct callable_argument<std::function<void(T*)>>
{
    using argument_type = T;
};

// Specialization for member function pointers
template <typename C, typename T>
struct callable_argument<void(C::*)(T*)>
{
    using argument_type = T;
};

// Specialization for const member function pointers
template <typename C, typename T>
struct callable_argument<void(C::*)(T*) const>
{
    using argument_type = T;
};

// Specialization for functors and lambdas
template <typename F>
struct callable_argument : public callable_argument<decltype(&F::operator())> {};

template <typename T>
using callable_argument_t = typename callable_argument<T>::argument_type;


}
