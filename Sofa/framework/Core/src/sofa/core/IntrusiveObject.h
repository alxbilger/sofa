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
#include <sofa/core/config.h>
#include <atomic>

namespace sofa::core
{

template<class T>
class WeakPtr;

/**
 * The `IntrusiveObject` class implements an internal reference counting mechanism
 * to manage its lifetime. It is intended to work with intrusive smart pointers like
 * `boost::intrusive_ptr`.
 */
class SOFA_CORE_API IntrusiveObject
{
    std::atomic<int> m_strongCounter { 0 };
    std::atomic<int> m_weakCounter { 0 };

    void incrementStrong();
    void incrementWeak();
    void decrementStrong();

    bool expired() const;

    friend inline void intrusive_ptr_add_ref(IntrusiveObject* p)
    {
        p->incrementStrong();
    }

    friend inline void intrusive_ptr_release(IntrusiveObject* p)
    {
        p->decrementStrong();
    }

    template<class T>
    friend class WeakPtr;

    void kill() { delete this; }

public:
    virtual ~IntrusiveObject();

};

}
