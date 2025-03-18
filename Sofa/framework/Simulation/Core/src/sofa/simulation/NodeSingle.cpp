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
#define SOFA_SIMULATION_NODESINGLE_CPP_

#include <sofa/simulation/Node.h>

namespace sofa::simulation
{
template class NodeSingle<sofa::core::behavior::BaseAnimationLoop>;
template class NodeSingle<sofa::core::visual::VisualLoop>;
template class NodeSingle<sofa::core::visual::BaseVisualStyle>;
template class NodeSingle<sofa::core::topology::Topology>;
template class NodeSingle<sofa::core::topology::BaseMeshTopology>;
template class NodeSingle<sofa::core::BaseState>;
template class NodeSingle<sofa::core::behavior::BaseMechanicalState>;
template class NodeSingle<sofa::core::BaseMapping>;
template class NodeSingle<sofa::core::behavior::BaseMass>;
template class NodeSingle<sofa::core::collision::Pipeline>;
}
