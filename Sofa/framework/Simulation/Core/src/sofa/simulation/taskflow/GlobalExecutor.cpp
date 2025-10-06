#include <sofa/simulation/taskflow/GlobalExecutor.h>
tf::Executor& sofa::simulation::taskflow::getExecutor()
{
    static tf::Executor executor;
    return executor;
}
