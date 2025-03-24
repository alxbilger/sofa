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
TEST(MappingGraph_Test, dsafads)
{
    constexpr std::string_view filename { "Component/LinearSystem/MatrixLinearSystem.scn" };
    const std::string path = sofa::helper::system::DataRepository.getFile(std::string{filename});

    const simulation::Node::SPtr groot = sofa::simulation::node::load(path, false, {});
    ASSERT_NE(groot, nullptr);

    sofa::simulation::node::initRoot(groot.get());

    auto* mparams = sofa::core::MechanicalParams::defaultInstance();
    sofa::simulation::MappingGraph graph(mparams, groot.get());

    struct PrintNameVisitor : simulation::MappingGraphVisitor
    {
        sofa::type::vector<std::string> visit;
        std::mutex mutex;
        ~PrintNameVisitor() override = default;
        void forwardVisit(sofa::core::behavior::BaseMechanicalState* state) override
        {
            // std::lock_guard lock(mutex);
            visit.push_back(state->getPathName());
        }
        void forwardVisit(sofa::core::BaseMapping* mapping) override
        {
            // std::lock_guard lock(mutex);
            visit.push_back(mapping->getPathName());
        }

    } visitor;

    graph.accept(visitor, false);

    const auto compare = [&visitor](const std::string& A, const std::string& B)
    {
        const auto itA = std::find(visitor.visit.begin(), visitor.visit.end(), A);
        EXPECT_NE(itA, visitor.visit.end());
        const auto itB = std::find(visitor.visit.begin(), visitor.visit.end(), B);
        EXPECT_NE(itB, visitor.visit.end());

        return std::distance(itA, itB);
    };

    EXPECT_GT(compare("/rigidSections/blue/DOFs", "/rigidSections/blue/intermediateMapping/IdentityMapping1"), 0);
    EXPECT_LT(compare("/rigidSections/blue/intermediateMapping/IdentityMapping1", "/rigidSections/blue/DOFs"), 0);
    EXPECT_GT(compare("/rigidSections/blue/intermediateMapping/IdentityMapping1", "/rigidSections/blue/intermediateMapping/DOFs"), 0);

    EXPECT_GT(compare("/rigidSections/red/DOFs", "/rigidSections/red/FEM/RigidMapping1"), 0);
    EXPECT_GT(compare("/rigidSections/red/FEM/RigidMapping1", "/rigidSections/red/FEM/DOFs"), 0);

}

}
