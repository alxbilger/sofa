#pragma once

#include <sofa/helper/ScopedAdvancedTimer.h>
#include <sofa/simulation/taskflow/GlobalExecutor.h>
#include <sofa/simulation/taskflow/TaskflowVisitor.h>
#include <sofa/simulation/taskflow/detail.h>

#include <sofa/simulation/taskflow/BuildTaskflow.h>

namespace sofa::simulation::taskflow
{

template<TaskflowType type, class... Callable>
struct AbstractTaskflowVisitor : public TaskflowVisitor
{
    AbstractTaskflowVisitor(const sofa::core::ExecParams* params, Callable... callable)
        : TaskflowVisitor(params), m_callable(callable...) {}

    void run(Node* node) override
    {
        SCOPED_TIMER_TR("Visitor");
        createTaskflow(taskflow, node, std::get<Callable>(m_callable)...);
        getExecutor().run(m_taskflow).wait();
    }

private:

    std::tuple<Callable...> m_callable;
    tf::Taskflow m_taskflow;
    TaskflowContainer<type> taskflow { &m_taskflow };
};


template<TaskflowType type>
struct VisitorBuilder
{
    VisitorBuilder(const sofa::core::ExecParams* params) : m_params(params) {}

    template<class... Callable>
    AbstractTaskflowVisitor<type, Callable...> from(Callable... callable)
    {
        return AbstractTaskflowVisitor<type, Callable...>(m_params, callable...);
    }

private:
    const sofa::core::ExecParams* m_params { nullptr };
};

template<TaskflowType type>
VisitorBuilder<type> make(const sofa::core::ExecParams* params)
{
    return VisitorBuilder<type>(params);
}

}
