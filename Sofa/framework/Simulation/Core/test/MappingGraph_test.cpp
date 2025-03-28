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
#include <gtest/gtest.h>
#include <sofa/core/BaseMapping.h>
#include <sofa/core/MechanicalParams.h>
#include <sofa/core/behavior/BaseMechanicalState.h>
#include <sofa/helper/system/FileRepository.h>
#include <sofa/simulation/MappingGraph.h>
#include <sofa/simulation/Node.h>
#include <sofa/simulation/Simulation.h>

namespace sofa
{
TEST(MappingGraph_Test, CollectPathNameVisitor)
{
    constexpr std::string_view filename { "Component/LinearSystem/MatrixLinearSystem.scn" };
    const std::string path = sofa::helper::system::DataRepository.getFile(std::string{filename});

    const simulation::Node::SPtr groot = sofa::simulation::node::load(path, false, {});
    ASSERT_NE(groot, nullptr);

    sofa::simulation::node::initRoot(groot.get());

    auto* mparams = sofa::core::MechanicalParams::defaultInstance();
    sofa::simulation::MappingGraph graph(mparams, groot.get());

    sofa::type::vector<std::string> visitForward, visitBackward;
    const auto visitObjectForward = [&visitForward](const core::objectmodel::BaseObject* object)
    {
        visitForward.push_back(object->getPathName());
    };
    const auto visitObjectBackward = [&visitBackward](const core::objectmodel::BaseObject* object)
    {
        visitBackward.push_back(object->getPathName());
    };

    auto visitor =
        simulation::mapping_graph::makeForwardVisitor(
            [&visitObjectForward](core::behavior::BaseMechanicalState* state){visitObjectForward(state);},
            [&visitObjectForward](core::BaseMapping* mapping){visitObjectForward(mapping);},
            [&visitObjectForward](core::behavior::BaseForceField* forcefield){visitObjectForward(forcefield);},
            [&visitObjectForward](core::behavior::BaseMass* mass){visitObjectForward(mass);},
            [&visitObjectForward](core::behavior::BaseProjectiveConstraintSet* constraint){visitObjectForward(constraint);}
        ) +
        simulation::mapping_graph::makeBackwardVisitor(
            [&visitObjectBackward](core::behavior::BaseMechanicalState* state){visitObjectBackward(state);},
            [&visitObjectBackward](core::BaseMapping* mapping){visitObjectBackward(mapping);},
            [&visitObjectBackward](core::behavior::BaseForceField* forcefield){visitObjectBackward(forcefield);},
            [&visitObjectBackward](core::behavior::BaseMass* mass){visitObjectBackward(mass);},
            [&visitObjectBackward](core::behavior::BaseProjectiveConstraintSet* constraint){visitObjectBackward(constraint);}
        );

    graph.accept(visitor, false);

    const auto compare = [&visitForward](const std::string& A, const std::string& B)
    {
        const auto itA = std::find(visitForward.begin(), visitForward.end(), A);
        EXPECT_NE(itA, visitForward.end());
        const auto itB = std::find(visitForward.begin(), visitForward.end(), B);
        EXPECT_NE(itB, visitForward.end());

        return std::distance(itA, itB);
    };

    EXPECT_GT(compare("/rigidSections/blue/DOFs", "/rigidSections/blue/intermediateMapping/IdentityMapping1"), 0);
    EXPECT_LT(compare("/rigidSections/blue/intermediateMapping/IdentityMapping1", "/rigidSections/blue/DOFs"), 0);
    EXPECT_GT(compare("/rigidSections/blue/intermediateMapping/IdentityMapping1", "/rigidSections/blue/intermediateMapping/DOFs"), 0);

    EXPECT_GT(compare("/rigidSections/red/DOFs", "/rigidSections/red/FEM/RigidMapping1"), 0);
    EXPECT_GT(compare("/rigidSections/red/FEM/RigidMapping1", "/rigidSections/red/FEM/DOFs"), 0);

}

}
