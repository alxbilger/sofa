#pragma once

#include <sofa/simulation/config.h>
#include <sofa/simulation/fwd.h>
#include <sofa/helper/taskflow.h>

namespace sofa::simulation::taskflow
{

struct SOFA_SIMULATION_CORE_API TaskflowVisitor
{
    explicit TaskflowVisitor(const sofa::core::ExecParams* params) : m_params(params) {}
    virtual ~TaskflowVisitor() {}
    virtual void run(sofa::simulation::Node* node) = 0;

protected:
    const sofa::core::ExecParams* m_params = nullptr;
};

}
