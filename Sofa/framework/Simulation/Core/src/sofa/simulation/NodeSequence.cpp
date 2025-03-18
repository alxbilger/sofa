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
#define SOFA_SIMULATION_NODESEQUENCE_CPP_

#include <sofa/simulation/Node.h>

namespace sofa::simulation
{
template class NodeSequence<Node,true>;
template class NodeSequence<sofa::core::objectmodel::BaseObject,true>;
template class NodeSequence<sofa::core::BehaviorModel>;
template class NodeSequence<sofa::core::BaseMapping>;
template class NodeSequence<sofa::core::behavior::OdeSolver>;
template class NodeSequence<sofa::core::behavior::ConstraintSolver>;
template class NodeSequence<sofa::core::behavior::BaseLinearSolver>;
template class NodeSequence<sofa::core::topology::BaseTopologyObject>;
template class NodeSequence<sofa::core::behavior::BaseForceField>;
template class NodeSequence<sofa::core::behavior::BaseInteractionForceField>;
template class NodeSequence<sofa::core::behavior::BaseProjectiveConstraintSet>;
template class NodeSequence<sofa::core::behavior::BaseConstraintSet>;
template class NodeSequence<sofa::core::objectmodel::ContextObject>;
template class NodeSequence<sofa::core::objectmodel::ConfigurationSetting>;
template class NodeSequence<sofa::core::visual::Shader>;
template class NodeSequence<sofa::core::visual::VisualModel>;
template class NodeSequence<sofa::core::visual::VisualManager>;
template class NodeSequence<sofa::core::CollisionModel>;
template class NodeSequence<sofa::core::objectmodel::BaseObject>;
}
