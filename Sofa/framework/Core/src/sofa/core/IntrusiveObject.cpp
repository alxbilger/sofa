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
#include <sofa/core/IntrusiveObject.h>

#include <cassert>

namespace sofa::core
{
void IntrusiveObject::incrementStrong()
{
    ++m_strongCounter;
}

void IntrusiveObject::incrementWeak()
{
    ++m_weakCounter;
}

void IntrusiveObject::decrementStrong()
{
    if (--m_strongCounter == 0)
    {
        if (m_weakCounter.load() < 1)
        {
            delete this;
        }
    }
}

bool IntrusiveObject::expired() const { return m_strongCounter.load() == 0; }

IntrusiveObject::~IntrusiveObject()
{
    assert(m_strongCounter.load() == 0 && m_weakCounter.load() == 0);
}

}  // namespace sofa::core
