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

#include <sofa/simulation/mechanicalvisitor/MechanicalResetConstraintVisitor.h>

#include <sofa/core/behavior/BaseMechanicalState.h>
#include <sofa/core/behavior/BaseConstraintSet.h>
#include <sofa/simulation/TaskScheduler.h>
#include <sofa/helper/ScopedAdvancedTimer.h>

namespace sofa::simulation::mechanicalvisitor
{
void MechanicalResetConstraintVisitor::processNodeBottomUp(simulation::Node* node)
{
    if (!m_tasks.empty())
    {
        auto* taskScheduler = sofa::simulation::TaskScheduler::getInstance();
        assert(taskScheduler != nullptr);
        sofa::helper::ScopedAdvancedTimer parallelSolveTimer("waitParallelTasks");
        taskScheduler->workUntilDone(&m_status);
    }
    m_tasks.clear();

    BaseMechanicalVisitor::processNodeBottomUp(node);
}

Visitor::Result MechanicalResetConstraintVisitor::fwdMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState* mm)
{
    // mm->setC(res);
    if (m_parallelReset)
    {
        auto* taskScheduler = sofa::simulation::TaskScheduler::getInstance();
        assert(taskScheduler != nullptr);

        m_tasks.emplace_back(&m_status, m_cparams, mm);
        taskScheduler->addTask(&m_tasks.back());
    }
    else
    {
        mm->resetConstraint(m_cparams);
    }

    return RESULT_CONTINUE;
}


Visitor::Result MechanicalResetConstraintVisitor::fwdMappedMechanicalState(simulation::Node* /*node*/, core::behavior::BaseMechanicalState* mm)
{
    if (m_parallelReset)
    {
        auto* taskScheduler = sofa::simulation::TaskScheduler::getInstance();
        assert(taskScheduler != nullptr);

        m_tasks.emplace_back(&m_status, m_cparams, mm);
        taskScheduler->addTask(&m_tasks.back());
    }
    else
    {
        mm->resetConstraint(m_cparams);
    }

    return RESULT_CONTINUE;
}


Visitor::Result MechanicalResetConstraintVisitor::fwdConstraintSet(simulation::Node* /*node*/, core::behavior::BaseConstraintSet* c)
{
    c->resetConstraint();
    return RESULT_CONTINUE;
}

ResetConstraintVisitorTask::ResetConstraintVisitorTask(sofa::simulation::CpuTask::Status* status,
    const sofa::core::ConstraintParams* cparams, core::behavior::BaseMechanicalState* mstate)
    : sofa::simulation::CpuTask(status)
    , m_cparams(cparams)
    , m_mstate(mstate)
{
}

sofa::simulation::Task::MemoryAlloc ResetConstraintVisitorTask::run()
{
    m_mstate->resetConstraint(m_cparams);
    return Task::Stack;
}

}
