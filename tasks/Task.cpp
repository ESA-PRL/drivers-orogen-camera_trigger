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
        sensor = CAMERA;
    }
    else if (_frame_left_in.connected() && _distance_frame_in.connected() && _laser_scan_in.connected())
    {
        // lidar
        sensor = LIDAR;
    }
    else if (_frame_left_in.connected() && _distance_frame_in.connected() && _pointcloud_in.connected())
    {
        // tof
        sensor = TOF;
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

    // measure how much time has elapsed
    // (this way the trigger's periodicity can be changed)
    base::Time curTime = base::Time::now();
    int64_t elapsedTime = curTime.toMicroseconds() - lastTime.toMicroseconds();
    lastTime = curTime;

    RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> frame;

    if (_telecommand_in.read(command) == RTT::NewData)
    {
        // take note of newly set mode for productType
        productModes[command.productType] = command.productMode;
        // setting a new mode resets the timer for the chosen productType
        productTimes[command.productType] = curTime;
    }

    // check if any product needs to be produced/forwarded
    typedef std::map<telemetry_telecommand::messages::ProductType, telemetry_telecommand::messages::Mode>::iterator it_type;
    for (it_type it = productModes.begin(); it != productModes.end(); it++)
    {
        telemetry_telecommand::messages::ProductType type = it->first;
        telemetry_telecommand::messages::Mode mode = it->second;

        if (mode == telemetry_telecommand::messages::ONE_SHOT)
        {
            //TODO output/forward

            // just for completeness, not needed
            productTimes[type] = curTime;
        }
        else if (mode < telemetry_telecommand::messages::STOP)
        {
            if (elapsedTime >= command.usecPeriod) //TODO fix comp. between uint and int
            {
                //TODO output/forward

                productTimes[type] = curTime;
            }
        }
        else if (mode == telemetry_telecommand::messages::STOP)
        {
            // no action required
        }
        else
        {
            //TODO output error message
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
