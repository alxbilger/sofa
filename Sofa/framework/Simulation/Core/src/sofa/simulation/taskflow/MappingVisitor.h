#pragma once

#include <sofa/core/BaseMapping.h>
#include <sofa/helper/ScopedAdvancedTimer.h>
#include <sofa/simulation/taskflow/TaskflowVisitor.h>
#include <sofa/simulation/taskflow/detail.h>
#include <sofa/simulation/taskflow/VisitDirection.h>

namespace sofa::simulation::taskflow
{

template<VisitDirection Direction>
struct MappingVisitor : public TaskflowVisitor
{
    using TaskflowVisitor::TaskflowVisitor;
    using TaskflowVisitor::s_executor;

    void run(Node* node) override
    {
        SCOPED_TIMER_TR("MappingVisitor");
        processNode(node);
        //m_taskflow.dump(std::cout);
        s_executor.run(m_taskflow).wait();
    }

    virtual void apply(core::BaseMapping* mapping) = 0;

private:

    enum class MappingIO : bool { INPUT, OUTPUT};

    template<MappingIO io>
    static type::vector<core::behavior::BaseMechanicalState*> getMappingIO(core::BaseMapping* mapping)
    {
        if constexpr (io == MappingIO::INPUT)
        {
            return mapping->getMechFrom();
        }
        else
        {
            return mapping->getMechTo();
        }
    }

    template<MappingIO io>
    void linkMappingTaskAndStateTasks(tf::Task& mappingTask, core::BaseMapping* mapping)
    {
        for (auto* state : getMappingIO<io>(mapping))
        {
            if (state)
            {
                auto stateTaskIt = findOrCreateStateTask(state);
                if constexpr ((io == MappingIO::INPUT) == (Direction == VisitDirection::Forward))
                {
                    mappingTask.succeed(stateTaskIt->second);
                }
                else
                {
                    mappingTask.precede(stateTaskIt->second);
                }
            }
        }
    }

    auto findOrCreateStateTask(core::behavior::BaseMechanicalState* state)
    {
        auto stateTaskIt = m_stateTasks.find(state);
        if (stateTaskIt == m_stateTasks.end())
        {
            stateTaskIt = m_stateTasks.emplace(state,
                m_taskflow.emplace([](){}).name(state->getPathName())).first;
        }
        return stateTaskIt;
    }

    void processNode(Node* node)
    {
        if (auto* mapping = node->mechanicalMapping.get())
        {
            tf::Task mappingTask =
                m_taskflow.emplace([this, mapping]() { this->apply(mapping); })
                .name(mapping->getPathName());
            linkMappingTaskAndStateTasks<MappingIO::INPUT>(mappingTask, mapping);
            linkMappingTaskAndStateTasks<MappingIO::OUTPUT>(mappingTask, mapping);
        }

        if (auto* mstate = node->mechanicalState.get())
        {
            findOrCreateStateTask(mstate);
        }

        for (auto& child : node->child)
        {
            this->processNode(child.get());
        }
    }

    tf::Taskflow m_taskflow;

    std::unordered_map<core::behavior::BaseMechanicalState*, tf::Task> m_stateTasks;
    std::unordered_map<core::BaseMapping*, tf::Task> m_mappingTasks;
};

}
