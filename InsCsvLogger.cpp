#include "InsCsvLogger.h"

#include "gz/msgs/imu.pb.h"
#include "gz/msgs/navsat.pb.h"
#include "gz/msgs/odometry.pb.h"
#include "gz/sim/Model.hh"
#include "gz/sim/components/CanonicalLink.hh"
#include "gz/sim/components/JointPosition.hh"
#include <gz/common/Util.hh>
#include <gz/math/Quaternion.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <filesystem>
#include <fstream>
#include <iomanip>

using namespace gz;
using namespace sim;
using namespace systems;

using namespace csv_logger;

namespace csv_logger
{

class InsCsvLoggerPrivate
{
public:
    /// \brief Node for communication.
    transport::Node node;

    /// \brief Model.
    Model model;

    math::SphericalCoordinates coordinateSystem{
        math::SphericalCoordinates::SurfaceType::EARTH_WGS84,
        0, 0, 0, 0
    };

    std::ofstream imuFile;
    std::ofstream positionFile;
    std::ofstream truePoseFile;

    void OnImu(const msgs::IMU& _msg)
    {
        const auto& a = _msg.linear_acceleration();
        const auto& w = _msg.angular_velocity();
        imuFile << _msg.header().stamp().sec() + _msg.header().stamp().nsec() / 1.e9
                << "," << a.x() << "," << a.y() << "," << a.z()
                << "," << w.x() << "," << w.y() << "," << w.z()
                << std::endl;
    }

    void OnOdometry(const msgs::Odometry& _msg)
    {
        const auto& pose = _msg.pose();
        math::Quaterniond
            q{pose.orientation().w(), pose.orientation().x(), pose.orientation().y(), pose.orientation().z()};
        truePoseFile << _msg.header().stamp().sec() + _msg.header().stamp().nsec() / 1.e9
                     << "," << pose.position().x() << "," << pose.position().y() << "," << pose.position().z()
                     << "," << q.Roll() << "," << q.Pitch() << "," << q.Yaw()
                     << "," << _msg.twist().linear().x() << std::endl;
    }

    void OnGnss(const msgs::NavSat& _msg)
    {
        const math::Vector3d position = coordinateSystem
            .LocalFromSphericalPosition(math::Vector3d{_msg.latitude_deg(), _msg.longitude_deg(), _msg.altitude()});
        const auto velocity = coordinateSystem.LocalFromGlobalVelocity(
            math::Vector3d{_msg.velocity_east(), _msg.velocity_north(), _msg.velocity_up()});
        positionFile << _msg.header().stamp().sec() + _msg.header().stamp().nsec() / 1.e9
                     << "," << position[0] << "," << position[1] << "," << position[2]
                     << "," << velocity[0] << "," << velocity[1] << "," << velocity[2]
                     << std::endl;
    }
};

InsCsvLogger::InsCsvLogger()
    : p(std::make_unique<InsCsvLoggerPrivate>())
{
}

InsCsvLogger::~InsCsvLogger() = default;

void InsCsvLogger::PostUpdate(const gz::sim::UpdateInfo& _info,
                              const gz::sim::EntityComponentManager& _ecm)
{
    if(_info.paused)
        return;
}

void InsCsvLogger::Configure(const gz::sim::Entity& _entity,
                             const std::shared_ptr<const sdf::Element>& _sdf,
                             gz::sim::EntityComponentManager& _ecm,
                             gz::sim::EventManager& _eventMgr)
{
    this->p->model = Model(_entity);
    if(!this->p->model.Valid(_ecm))
    {
        gzerr << "SampleSystem system plugin should be attached to a model"
              << " entity. Failed to initialize." << std::endl;
        return;
    }
    gzdbg << "Creating csv logger" << std::endl;

    const auto heightStdDev = 0.05; // meters
    const auto heightSphError = p->coordinateSystem.SphericalFromLocalPosition(math::Vector3d{0, -heightStdDev, 0});
    gzdbg << "Good rtk gnss height std dev of " << heightStdDev << "m in spherical coordinates: "
          << std::setprecision(12) << heightSphError[0] << std::endl;
    const auto planeStdDev = 0.02; // meters
    const auto planeSphError = p->coordinateSystem.SphericalFromLocalPosition(math::Vector3d{0, -planeStdDev, 0});
    gzdbg << "Good rtk gnss xy-plane std dev of " << planeStdDev << "m in spherical coordinates: "
          << std::setprecision(12) << planeSphError[0] << std::endl;


    p->node.Subscribe("/imu", &InsCsvLoggerPrivate::OnImu,
                      this->p.get());
    p->node.Subscribe("/base_link_true_pose/odom", &InsCsvLoggerPrivate::OnOdometry,
                      this->p.get());
    p->node.Subscribe("/gnss_pos", &InsCsvLoggerPrivate::OnGnss,
                      this->p.get());
    std::filesystem::create_directory(DATA_DIR);
    p->imuFile.open(std::filesystem::path(DATA_DIR) / "imu.csv");
    p->imuFile << "time,acc x,acc y,acc z,gyro x,gyro y,gyro z" << std::endl;
    p->imuFile << std::setprecision(12);
    p->positionFile.open(std::filesystem::path(DATA_DIR) / "gnss.csv");
    p->positionFile << "time,pos x,pos y,pos z,vel x,vel y,vel z" << std::endl;
    p->positionFile << std::setprecision(12);
    p->truePoseFile.open(std::filesystem::path(DATA_DIR) / "true_pose_speed.csv");
    p->truePoseFile << "time,pos x,pos y,pos z,roll,pitch,yaw,speed" << std::endl;
    p->truePoseFile << std::setprecision(12);
}

// Include a line in your source file for each interface implemented.
GZ_ADD_PLUGIN(
    InsCsvLogger,
    ::gz::sim::System,
    InsCsvLogger::ISystemConfigure,
    InsCsvLogger::ISystemPostUpdate
)

GZ_ADD_PLUGIN_ALIAS(InsCsvLogger, "csv_logger::csv_logger")
}
