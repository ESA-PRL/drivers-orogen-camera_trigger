#ifndef CAMERA_TRIGGER_TASK_TASK_HPP
#define CAMERA_TRIGGER_TASK_TASK_HPP

#include "camera_trigger/TaskBase.hpp"
#include <telemetry_telecommand/Messages.hpp>
#include <base-logging/Logging.hpp>

#define BASE_LOG_NAMESPACE camera_trigger
#define BASE_LOG_WARN

namespace camera_trigger {
    enum connectedSensor
    {
        CAMERA,
        LIDAR,
        TOF
    };

    class Task : public TaskBase
    {
    friend class TaskBase;
    protected:
        connectedSensor sensor;

        base::Time lastTime;

        // variables to read and write contents of ports
        RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> frame;
        RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> frameLeft;
        RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> frameRight;
        RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> distanceFrame;
        base::samples::Pointcloud pointcloud;
        velodyne_lidar::MultilevelLaserScan laserScans;
        telemetry_telecommand::messages::Telecommand command;

        std::map<telemetry_telecommand::messages::ProductType, telemetry_telecommand::messages::Mode> productModes;
        std::map<telemetry_telecommand::messages::ProductType, base::Time> productTimes;
        std::map<telemetry_telecommand::messages::ProductType, uint64_t> productPeriods;
        std::map<telemetry_telecommand::messages::ProductType, telemetry_telecommand::messages::Telecommand> commandsMap;

        void forwardToPorts();

    public:
        Task(std::string const& name = "camera_trigger::Task");
        Task(std::string const& name, RTT::ExecutionEngine* engine);

        ~Task();

        bool configureHook();
        bool startHook();
        void updateHook();
        void errorHook();
        void stopHook();
        void cleanupHook();
    };
}

#endif
