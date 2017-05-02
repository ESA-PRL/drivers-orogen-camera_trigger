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
    {
        std::cout << "Parent configureHook failed\n";
        return false;
    }

    // something went wrong
    if (!( _telecommand_in.connected() && _telecommands_out.connected()))
    {
        std::cout << "Telecommand in/out connection missing\n";
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
        sensor = CAMERA;
    }
    else if (_frame_left_in.connected() && _laser_scan_in.connected())
    {
        sensor = LIDAR;
    }
    else if (_frame_left_in.connected() && _pointcloud_in.connected())
    {
        sensor = TOF;
    }
    else
    {
        // could not derive which sensor this trigger is connected to
        std::cout << "Could not derive which sensor this trigger is connected to\n";
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

    base::Time curTime = base::Time::now();
    RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> frame;

    // read telecommand
    telemetry_telecommand::messages::Mode lastMode;
    if (_telecommand_in.read(command) == RTT::NewData)
    {
        // do we need to STOP requesting ALL products?
        if (command.productType == telemetry_telecommand::messages::ALL && command.productMode == telemetry_telecommand::messages::STOP)
        {
            // set mode of all products to STOP
            typedef std::map<telemetry_telecommand::messages::ProductType, telemetry_telecommand::messages::Mode>::iterator it_type;
            for (it_type it = productModes.begin(); it != productModes.end(); it++)
            {
                productModes[ it->first ] = telemetry_telecommand::messages::STOP;
            }
        }
        // command is not supported
        else if (command.productType == telemetry_telecommand::messages::ALL)
        {
            std::cout << "WARNING: This command combination (type==ALL, mode!=STOP) is currently not supported. Ignoring command.\n";
        }
        // product type is an actual type, not ALL
        else
        {
            // Remember old mode in case a single shot should be taken.
            // In case the product type never had a mode until now, set (last) mode to default STOP
            if (productModes.count(command.productType) == 0)
            {
                productModes[command.productType] = telemetry_telecommand::messages::STOP;
            }
            lastMode = productModes[command.productType];

            // take note of newly set mode for productType
            productModes[command.productType] = command.productMode;
            // setting a new mode resets the timer for the chosen productType
            productTimes[command.productType] = curTime;

            if (command.productMode == telemetry_telecommand::messages::PERIODIC)
            {
                productPeriods[command.productType] = command.usecPeriod;
            }
        }
		std::cout << "Went through new telecommand\n";
    }

    bool sendAnything = false;
    // check if any product needs to be produced/forwarded
    typedef std::map<telemetry_telecommand::messages::ProductType, telemetry_telecommand::messages::Mode>::iterator it_type;
    for (it_type it = productModes.begin(); it != productModes.end(); it++)
    {
        telemetry_telecommand::messages::ProductType type = it->first;
        telemetry_telecommand::messages::Mode mode = it->second;

        // Populate commandsMap (the map with which we build the command vector to be sent to DEM/Stereo/...
        // The following components do not care about the periodicity; only this trigger has to take care of it
        // Unless the current type is in one shot or the period is reached (NOT just if the mode is periodic),
        // leave it at stop (don't process type at next component)
        commandsMap[type].productType = type;
        commandsMap[type].productMode = telemetry_telecommand::messages::STOP;

        if (mode == telemetry_telecommand::messages::ONE_SHOT)
        {
            std::cout << "Went through one_shot if\n";
            sendAnything = true;
            commandsMap[type].productMode = mode;

            // reset mode to last one before One Shot.
            it->second = lastMode;
        }
        else if (mode == telemetry_telecommand::messages::PERIODIC)
        {
            // how much time has elapsed since the last time this product was forwarded
            int64_t elapsedTime = curTime.toMicroseconds() - productTimes[type].toMicroseconds();

            // CHECK IF TYPE NEEDS TO BE FORWARDED DUE TO PERIODICITY:
            // The DEM generation always produces all the types we send to it via a telecommand,
            // and we send everything which is NOT set to STOP.
            // This is why we only set the current type's mode to periodic (!= STOP) if it has to be sent because of its periodicity.
            // Thus, when one product type is in CONTINUOUS mode and another one is in PERIODIC but has not completed its period,
            // only the necessary/desired product (CONTINUOUS) will be produced.
            if (elapsedTime >= int64_t(productPeriods[type]))
            {
                sendAnything = true;
                commandsMap[type].productMode = mode;
                // update time at which product was last sent
                productTimes[type] = curTime;
            }
        }
        else if (mode == telemetry_telecommand::messages::CONTINUOUS)
        {
            sendAnything = true;
            commandsMap[type].productMode = mode;
        }
        else if (mode == telemetry_telecommand::messages::STOP)
        {
            // no action required:
            // sendAnything stays unchanged, current product type's mode stays STOP
        }
        else
        {
            //TODO output error message
        }
    }
    if (sendAnything)
    {
        forwardToPorts();
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
void Task::forwardToPorts()
{
    bool onlyFrameRequested = true;
    // prepare telecommands and check which products are required
    std::vector<telemetry_telecommand::messages::Telecommand> commandVec;
    typedef std::map<telemetry_telecommand::messages::ProductType, telemetry_telecommand::messages::Telecommand>::iterator it_type;
    for (it_type it = commandsMap.begin(); it != commandsMap.end(); it++)
    {
        if (it->second.productMode != telemetry_telecommand::messages::STOP)
        {
            if (it->second.productType != telemetry_telecommand::messages::IMAGE)
            {
                onlyFrameRequested = false;
            }
            commandVec.push_back( it->second );
        }
    }
    std::cout << "Went through forward to ports if\n";

    switch (sensor)
    {
        case CAMERA:
        {
            bool validPair = false;
            // check if only frame will be sent
            if(onlyFrameRequested)
            {
                while (_frame_left_in.read(frameLeft) != RTT::NewData);
                _frame_left_out.write(frameLeft);
                std::cout << "Went through forwarding of camera single " << frameLeft->time << " " <<  onlyFrameRequested << "\n";

            }
            else
            {
                while (_frame_left_in.read(frameLeft) != RTT::NewData);
                while (_frame_right_in.read(frameRight) != RTT::NewData);
                while(!validPair)
                {
                    if(abs(frameLeft->time.toMicroseconds() - frameRight->time.toMicroseconds()) < 1000) // time difference less 1ms
                        validPair = true;
                    else if(frameLeft->time > frameRight->time) // frame right in the past
                    {
                        std::cout << "Went through older right" << frameLeft->time << " " << frameRight->time << " " << onlyFrameRequested << "\n";
                        while (_frame_right_in.read(frameRight) != RTT::NewData);
                    }
                    else if(frameLeft->time < frameRight->time) // frame left in the past
                    {
                        std::cout << "Went through older left" << frameLeft->time << " " << frameRight->time << " " << onlyFrameRequested << "\n";
                        while (_frame_left_in.read(frameLeft) != RTT::NewData);

                    }
                    std::cout << "Went through forwarding of camera stereo " << frameLeft->time << " " << frameRight->time << " " << onlyFrameRequested << "\n";

                }
                _frame_right_out.write(frameRight);
                _frame_left_out.write(frameLeft);
            }

            break;
        }
        case LIDAR:
        {
            while (_frame_left_in.read(frame) != RTT::NewData);
            _frame_left_out.write(frame);

            //while (_distance_frame_in.read(distanceFrame) != RTT::NewData);
            //_distance_frame_out.write(distanceFrame);

            while (_laser_scan_in.read(laserScans) != RTT::NewData);
            _laser_scan_out.write(laserScans);

            break;
        }
        case TOF:
        {
            while (_frame_left_in.read(frame) != RTT::NewData);
            _frame_left_out.write(frame);

            //while (_distance_frame_in.read(distanceFrame) != RTT::NewData);
            //_distance_frame_out.write(distanceFrame);

            while (_pointcloud_in.read(pointcloud) != RTT::NewData);
            _pointcloud_out.write(pointcloud);

            break;
        }
        default:
        {
            //TODO error
            break;
        }
    }
    _telecommands_out.write(commandVec);
}
