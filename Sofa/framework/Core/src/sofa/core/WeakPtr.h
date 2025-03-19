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

#include <sofa/core/IntrusiveObject.h>
#include <sofa/core/sptr.h>

namespace sofa::core
{

template<class T>
class WeakPtr {
public:
    // static_assert(std::is_base_of_v<IntrusiveObject, T>, "T must derive from IntrusiveObject");

    WeakPtr() : m_ptr(nullptr) {}

    WeakPtr(const sptr<T>& other) : m_ptr(other.get())
    {
        if (m_ptr)
        {
            m_ptr->incrementWeak();
        }
    }

    template<class U>
    WeakPtr(const sptr<U>& other) : m_ptr(other.get())
    {
        if (m_ptr)
        {
            m_ptr->incrementWeak();
        }
    }

    WeakPtr(const WeakPtr& other) : m_ptr(other.m_ptr)
    {
        if (m_ptr)
        {
            m_ptr->incrementWeak();
        }
    }

    template<class U>
    WeakPtr(const WeakPtr<U>& other) : m_ptr(other.m_ptr)
    {
        if (m_ptr)
        {
            m_ptr->incrementWeak();
        }
    }

    WeakPtr(T* other) : m_ptr(other)
    {
        if (m_ptr)
        {
            m_ptr->incrementWeak();
        }
    }

    template<class U>
    WeakPtr(U* other) : m_ptr(other)
    {
        if (m_ptr)
        {
            m_ptr->incrementWeak();
        }
    }

    WeakPtr(const nullptr_t&) : m_ptr(nullptr)
    {}

    ~WeakPtr()
    {
        deref();
    }

    WeakPtr& operator=(const sptr<T>& other)
    {
        deref();
        m_ptr = other.get();
        m_ptr->incrementWeak();
        return *this;
    }

    template<class U>
    WeakPtr& operator=(const sptr<U>& other)
    {
        deref();
        m_ptr = other.get();
        m_ptr->incrementWeak();
        return *this;
    }

    WeakPtr& operator=(const WeakPtr& other)
    {
        if (&other != this)
        {
            deref();
            m_ptr = other.m_ptr;
            m_ptr->incrementWeak();
        }
        return *this;
    }

    WeakPtr& operator=(T* other)
    {
        if (other != m_ptr)
        {
            deref();
            m_ptr = other;
            m_ptr->incrementWeak();
        }
        return *this;
    }

    template<class U>
    WeakPtr& operator=(U* other)
    {
        if (other != m_ptr)
        {
            deref();
            m_ptr = other;
            m_ptr->incrementWeak();
        }
        return *this;
    }

    WeakPtr& operator=(const nullptr_t& other)
    {
        deref();
        m_ptr = other;
        return *this;
    }

    bool operator==(const WeakPtr& other) const
    {
        return m_ptr == other.m_ptr;
    }

    [[nodiscard]] bool expired() const
    {
        return m_ptr && m_ptr->m_strongCounter.load() == 0;
    }

    operator bool() const
    {
        return !expired();
    }

    const T* operator ->() const
    {
        return lock().get();
    }

    T* operator ->()
    {
        return lock().get();
    }

    operator T*() const
    {
        return lock().get();
    }

    sptr<T> lock() const
    {
        if (m_ptr && m_ptr->m_strongCounter)
            return sptr<T>(m_ptr);
        return sptr<T>();
    }

private:
    T* m_ptr { nullptr };

    void deref()
    {
        if (m_ptr &&
            --m_ptr->m_weakCounter <= 0 && m_ptr->m_strongCounter.load() == 0)
        {
            m_ptr->kill();
            m_ptr = nullptr;
        }
    }
};
}
