#pragma once
#include <sofa/simulation/config.h>
#include <sofa/helper/taskflow.h>

namespace sofa::simulation::taskflow
{
SOFA_SIMULATION_CORE_API
tf::Executor& getExecutor();
}
