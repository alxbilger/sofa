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

#include <sofa/simulation/MechanicalVisitor.h>

namespace sofa::simulation::mechanicalvisitor
{

/** Accumulate the entries of a mechanical matrix (mass or stiffness) of the whole scene */
class SOFA_SIMULATION_CORE_API MechanicalBuildMBKMatrixVisitor : public MechanicalVisitor
{
public:
    explicit MechanicalBuildMBKMatrixVisitor(const sofa::core::MechanicalParams* m_mparams,
        bool buildMass = true,
        bool buildStiffness = true,
        bool buildMappings = true
        )
        : MechanicalVisitor(m_mparams)
        , m_buildMass(buildMass)
        , m_buildStiffness(buildStiffness)
        , m_buildMappings(buildMappings)
    {}

    Result fwdMass(simulation::Node* /*node*/, sofa::core::behavior::BaseMass* mass) override;
    Result fwdForceField(simulation::Node* /*node*/, core::behavior::BaseForceField* ff) override;
    Result fwdMechanicalMapping(simulation::Node*, sofa::core::BaseMapping*) override;
    const char* getClassName() const override {  return "MechanicalBuildMBKMatrixVisitor";}

    bool m_buildMass { true };
    bool m_buildStiffness { true };
    bool m_buildMappings { true };
};

} //namespace sofa::simulation::mechanicalvisitor