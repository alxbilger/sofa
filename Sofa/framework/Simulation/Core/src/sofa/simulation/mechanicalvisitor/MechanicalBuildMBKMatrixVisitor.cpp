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

#include <sofa/simulation/mechanicalvisitor/MechanicalBuildMBKMatrixVisitor.h>

#include <sofa/core/behavior/BaseForceField.h>
#include <sofa/core/behavior/BaseMass.h>

namespace sofa::simulation::mechanicalvisitor
{

Visitor::Result MechanicalBuildMBKMatrixVisitor::fwdMass(simulation::Node* /*node*/, sofa::core::behavior::BaseMass* mass)
{
    if (m_buildMass && mass != nullptr)
    {
        mass->buildMassMatrix();
    }
    return RESULT_CONTINUE;
}
    
Visitor::Result MechanicalBuildMBKMatrixVisitor::fwdForceField(simulation::Node* /*node*/, core::behavior::BaseForceField* ff)
{
    if (m_buildStiffness && ff != nullptr)
    {
        ff->buildStiffnessMatrix();
    }
    return RESULT_CONTINUE;
}

Visitor::Result MechanicalBuildMBKMatrixVisitor::fwdMechanicalMapping(simulation::Node* /*node*/, sofa::core::BaseMapping* map)
{
    if (m_buildMappings && map != nullptr)
    {
        map->buildGeometricStiffnessMatrix();
    }
    return RESULT_CONTINUE;
}
} //namespace sofa::simulation::mechanicalvisitor