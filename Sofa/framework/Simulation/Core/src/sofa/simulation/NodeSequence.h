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

/// @name Component containers
/// @{
/// Sequence class to hold a list of objects. Public access is only readonly using an interface similar to std::vector (size/[]/begin/end).
/// UPDATE: it is now an alias for the Link pointer container
template < class T, bool strong = false >
class NodeSequence : public MultiLink<Node, T, BaseLink::FLAG_DOUBLELINK|(strong ? BaseLink::FLAG_STRONGLINK : BaseLink::FLAG_DUPLICATE)>
{
public:
    typedef MultiLink<Node, T, BaseLink::FLAG_DOUBLELINK|(strong ? BaseLink::FLAG_STRONGLINK : BaseLink::FLAG_DUPLICATE)> Inherit;
    typedef T pointed_type;
    typedef typename Inherit::DestPtr value_type;
    typedef typename Inherit::const_iterator const_iterator;
    typedef typename Inherit::const_reverse_iterator const_reverse_iterator;
    typedef const_iterator iterator;
    typedef const_reverse_iterator reverse_iterator;

    NodeSequence(const BaseLink::InitLink<Node>& init)
        : Inherit(init)
    {
    }

    value_type operator[](std::size_t i) const
    {
        return this->get(i);
    }

    /// Swap two values in the list. Uses a const_cast to violate the read-only iterators.
    void swap( iterator a, iterator b )
    {
        value_type& wa = const_cast<value_type&>(*a);
        value_type& wb = const_cast<value_type&>(*b);
        value_type tmp = *a;
        wa = *b;
        wb = tmp;
    }
};

#if !defined(SOFA_SIMULATION_NODESEQUENCE_CPP_)
extern template class NodeSequence<Node,true>;
extern template class NodeSequence<sofa::core::objectmodel::BaseObject,true>;
extern template class NodeSequence<sofa::core::BehaviorModel>;
extern template class NodeSequence<sofa::core::BaseMapping>;
extern template class NodeSequence<sofa::core::behavior::OdeSolver>;
extern template class NodeSequence<sofa::core::behavior::ConstraintSolver>;
extern template class NodeSequence<sofa::core::behavior::BaseLinearSolver>;
extern template class NodeSequence<sofa::core::topology::BaseTopologyObject>;
extern template class NodeSequence<sofa::core::behavior::BaseForceField>;
extern template class NodeSequence<sofa::core::behavior::BaseInteractionForceField>;
extern template class NodeSequence<sofa::core::behavior::BaseProjectiveConstraintSet>;
extern template class NodeSequence<sofa::core::behavior::BaseConstraintSet>;
extern template class NodeSequence<sofa::core::objectmodel::ContextObject>;
extern template class NodeSequence<sofa::core::objectmodel::ConfigurationSetting>;
extern template class NodeSequence<sofa::core::visual::Shader>;
extern template class NodeSequence<sofa::core::visual::VisualModel>;
extern template class NodeSequence<sofa::core::visual::VisualManager>;
extern template class NodeSequence<sofa::core::CollisionModel>;
extern template class NodeSequence<sofa::core::objectmodel::BaseObject>;
#endif
}
