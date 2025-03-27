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
#define SOFA_SIMULATION_MAPPINGGRAPH_CPP
#include <sofa/simulation/MappingGraph.inl>

#include <sofa/helper/taskflow.h>

#include <sofa/simulation/MechanicalVisitor.h>
#include <sofa/simulation/Node.h>


namespace sofa::simulation
{

template struct SOFA_SIMULATION_CORE_API details::TasksContainer<mapping_graph::VisitorDirection::FORWARD>;
template struct SOFA_SIMULATION_CORE_API details::TasksContainer<mapping_graph::VisitorDirection::BACKWARD>;

size_t details::SetHash::operator()(
    const std::set<sofa::core::behavior::BaseMechanicalState*>& objSet) const
{
    size_t hashValue = 0;
    for (sofa::core::behavior::BaseMechanicalState* obj : objSet)
    {
        hashValue ^= std::hash<sofa::core::behavior::BaseMechanicalState*>{}(obj) + 0x9e3779b9 +
                     (hashValue << 6) + (hashValue >> 2);
    }
    return hashValue;
}

bool details::SetEqual::operator()(
    const std::set<sofa::core::behavior::BaseMechanicalState*>& lhs,
    const std::set<sofa::core::behavior::BaseMechanicalState*>& rhs) const
{
    return lhs == rhs;  // Directly compare sets
}

}  // namespace sofa::simulation
