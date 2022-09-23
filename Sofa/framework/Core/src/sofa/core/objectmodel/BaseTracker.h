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

#include <mutex>
#include <unordered_map>
#include <iostream>
#include <sstream>

namespace sofa::core::objectmodel
{



class SOFA_CORE_API BaseTracker
{
public:

    static BaseTracker& getInstance()
    {
        static BaseTracker tracker;
        return tracker;
    }

    void allocateBase()
    {
        ++m_totalAllocationsBase;
    }

    void destroyBase()
    {
        ++m_totalDestructionsBase;
    }

    void allocate(const std::string& className)
    {
        std::lock_guard lock(m_allocationMutex);
        m_nbAllocations[className]++;
    }

    void destroy(const std::string& className)
    {
        std::lock_guard lock(m_destructionMutex);
        m_nbDestructions[className]++;
    }

    ~BaseTracker()
    {
        print();
    }

    void print()
    {
        std::stringstream ss;
        for (const auto& [className, nbAllocation] : m_nbAllocations)
        {
            ss << className << " " << nbAllocation << " " << m_nbDestructions[className] << '\n';
        }
        std::cout << '\n' << ss.str() << std::endl;

        std::cout << "\nTotal Base: " << m_totalAllocationsBase << " " << m_totalDestructionsBase << std::endl;
    }

private:
    BaseTracker() = default;

    std::unordered_map<std::string, sofa::Size> m_nbAllocations;
    std::unordered_map<std::string, sofa::Size> m_nbDestructions;

    std::atomic<sofa::Size> m_totalAllocationsBase {};
    std::atomic<sofa::Size> m_totalDestructionsBase {};

    std::mutex m_allocationMutex, m_destructionMutex;
};

template<class T>
class BaseCounter
{
public:
    BaseCounter()
    {
        BaseTracker::getInstance().allocate(T::GetClass()->className);
    }

    ~BaseCounter()
    {
        BaseTracker::getInstance().destroy(T::GetClass()->className);
    }
};

}
