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

    // something went wrong
    if (!( _telecommand_in.connected() && _telemetry_out.connected()))
    {
        return false;
    }

    /*  _______________________________________________
     * |                   |             |       |     |
     * | Input ports, from | Stereo Pair | LiDAR | ToF |
     * |___________________|_____________|_______|_____|
     * | Frame (Left)      | Yes         | Yes   | Yes |
     * | Frame Right       | Yes         |       |     |
     * | Distance Frame    |             | Yes   |     |
     * | Laser Scan        |             | Yes   | Yes |
     * | Pointcloud        |             |       | Yes |
     * |___________________|_____________|_______|_____/
     *
     */

    // determine which type of sensor the trigger is connected to
    if (_frame_left_in.connected() && _frame_right_in.connected())
    {
        // stereo pair
        connectedSensor = telemetry_telecommand::messages::MAST;
    }
    else if (_frame_left_in.connected() && _distance_frame_in.connected() && _laser_scan_in.connected())
    {
        // lidar
        connectedSensor = telemetry_telecommand::messages::LIDAR;
    }
    else if (_frame_left_in.connected() && _distance_frame_in.connected() && _pointcloud_in.connected())
    {
        // tof
        connectedSensor = telemetry_telecommand::messages::TOF;
    }
    else if (_frame_left_in.connected())
    {
        // bb2/3
        connectedSensor = telemetry_telecommand::messages::FRONT; //TODO could also be rear. adjust messages.h?
    }
    else
    {
        // could not derive which sensor this trigger is connected to
        return false;
    }

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

    RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> frame;

    // Check triggering condition (New sample & value TRUE)
    //if ((_trigger.read(triggerValue) == RTT::NewData) && (triggerValue == true))
    //{
    //    // We got triggered, but we have to check if there is ANY sample at all (not necessarily a new one though) :)
    //    // Pass frame from input to output
    //    if (_frame_in.read(frame))
    //    {
    //        _frame_out.write(frame);
    //    }
    //}
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

// _telecommand_in
// _frame_left_in
// _frame_right_in
// _laser_scan_in
// _distance_frame_in
// _pointcloud_in
// _frame_out
// _laser_scan_out
// _distance_frame_out
// _pointcloud_out
// _telemetry_out
// _telecommand_out
