#include "Task.hpp"

using namespace camera_trigger;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
}

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;
    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();

    bool triggerValue = false;
    RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> frame;

    // Check triggering condition (New sample & value TRUE)
    if ((_trigger.read(triggerValue) == RTT::NewData) && (triggerValue == true))
    {
        // We got triggered, but we have to check if there is ANY sample at all (not necessarily a new one though) :)
        // Pass frame from input to output
        if (_frame_in.read(frame))
        {
            _frame_out.write(frame);
        }
    }
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}
