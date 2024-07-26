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

#include <iostream>
#include <sofa/helper/logging/Messaging.h>


namespace sofa::helper
{

template<typename T, typename Bounds>
class BoundedScalar
{
    static_assert(std::is_floating_point_v<T>, "T must be a floating point type");

    static constexpr T defaultValue = Bounds::defaultValue;

public:
    BoundedScalar() = default;

    BoundedScalar(T value)
    {
        setValue(value);
    }

    operator T() const
    {
        return value;
    }

    BoundedScalar& operator=(T newValue)
    {
        setValue(value);
        return *this;
    }

    friend std::ostream& operator<<(std::ostream& os, const BoundedScalar& scalar)
    {
        os << scalar.value;
        return os;
    }

    friend std::istream& operator>>(std::istream& in, BoundedScalar& scalar)
    {
        T tmp;
        in >> tmp;
        scalar.setValue(tmp);
        return in;
    }

private:

    void setValue(T newValue)
    {
        if (Bounds::IsOutOfBounds(newValue))
        {
            msg_error("BoundedScalar") << Bounds::ErrorMessage(newValue) <<
                ". It is set to " << defaultValue << " to ensure correct behavior";
            value = defaultValue;
        }
        else
        {
            value = newValue;
        }
    }

    T value;
};

}
