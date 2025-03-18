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
#include <sofa/core/objectmodel/Link.h>

namespace sofa::simulation
{
class Node;

/// Class to hold 0-or-1 object. Public access is only readonly using an interface similar to std::vector (size/[]/begin/end), plus an automatic conversion to one pointer.
/// UPDATE: it is now an alias for the Link pointer container
template < class T, bool duplicate = true >
class NodeSingle : public SingleLink<Node, T, BaseLink::FLAG_DOUBLELINK|(duplicate ? BaseLink::FLAG_DUPLICATE : BaseLink::FLAG_NONE)>
{
public:
    typedef SingleLink<Node, T, BaseLink::FLAG_DOUBLELINK|(duplicate ? BaseLink::FLAG_DUPLICATE : BaseLink::FLAG_NONE)> Inherit;
    typedef T pointed_type;
    typedef typename Inherit::DestPtr value_type;
    typedef typename Inherit::const_iterator const_iterator;
    typedef typename Inherit::const_reverse_iterator const_reverse_iterator;
    typedef const_iterator iterator;
    typedef const_reverse_iterator reverse_iterator;

    NodeSingle(const BaseLink::InitLink<Node>& init)
        : Inherit(init)
    {
    }

    T* operator->() const
    {
        return this->get();
    }
    T& operator*() const
    {
        return *this->get();
    }
    operator T*() const
    {
        return this->get();
    }
};

#if !defined(SOFA_SIMULATION_NODESINGLE_CPP_)
extern template class NodeSingle<sofa::core::behavior::BaseAnimationLoop>;
extern template class NodeSingle<sofa::core::visual::VisualLoop>;
extern template class NodeSingle<sofa::core::visual::BaseVisualStyle>;
extern template class NodeSingle<sofa::core::topology::Topology>;
extern template class NodeSingle<sofa::core::topology::BaseMeshTopology>;
extern template class NodeSingle<sofa::core::BaseState>;
extern template class NodeSingle<sofa::core::behavior::BaseMechanicalState>;
extern template class NodeSingle<sofa::core::BaseMapping>;
extern template class NodeSingle<sofa::core::behavior::BaseMass>;
extern template class NodeSingle<sofa::core::collision::Pipeline>;
#endif
}
