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
        sortTasks();
        // m_taskflow.dump(std::cout);
        s_executor.run(m_taskflow).wait();
    }

    virtual void apply(core::BaseMapping* mapping) = 0;

private:

    void processNode(Node* node)
    {
        if (auto* mapping = node->mechanicalMapping.get())
        {
            m_mappingTasks[mapping] = m_taskflow.emplace([this, mapping]()
           {
               this->apply(mapping);
           }).name(mapping->getPathName());
        }

        if (auto* mstate = node->mechanicalState.get())
        {
            m_stateTasks[mstate] = m_taskflow.emplace([](){}).name(mstate->getPathName());;
        }

        for (auto& child : node->child)
        {
            this->processNode(child.get());
        }
    }

    void sortTasks()
    {
        for (auto& [mapping, mappingTask] : m_mappingTasks)
        {
            for (auto* input : mapping->getMechFrom())
            {
                if (auto stateTaskIt = m_stateTasks.find(input); stateTaskIt != m_stateTasks.end())
                {
                    if constexpr (Direction == VisitDirection::Forward)
                    {
                        mappingTask.succeed(stateTaskIt->second);
                    }
                    else
                    {
                        mappingTask.precede(stateTaskIt->second);
                    }
                }
            }


            for (auto* output : mapping->getMechTo())
            {
                if (auto stateTaskIt = m_stateTasks.find(output); stateTaskIt != m_stateTasks.end())
                {
                    if constexpr (Direction == VisitDirection::Forward)
                    {
                        mappingTask.precede(stateTaskIt->second);
                    }
                    else
                    {
                        mappingTask.succeed(stateTaskIt->second);
                    }
                }
            }
        }
    }

    tf::Taskflow m_taskflow;

    std::unordered_map<core::behavior::BaseMechanicalState*, tf::Task> m_stateTasks;
    std::unordered_map<core::BaseMapping*, tf::Task> m_mappingTasks;
};

}
