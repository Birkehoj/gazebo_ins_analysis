#pragma once
#include <gz/sim/System.hh>

namespace csv_logger
{
class InsCsvLoggerPrivate;
/// \brief Inersial navigation csv logger
class InsCsvLogger:
    public gz::sim::System,
    public gz::sim::ISystemPostUpdate,
    public gz::sim::ISystemConfigure
{
public:
    InsCsvLogger();
    ~InsCsvLogger() override;
    void PostUpdate(const gz::sim::UpdateInfo& _info,
                    const gz::sim::EntityComponentManager& _ecm) override;
    void Configure(const gz::sim::Entity& _entity,
                   const std::shared_ptr<const sdf::Element>& _sdf,
                   gz::sim::EntityComponentManager& _ecm,
                   gz::sim::EventManager& _eventMgr) override;
private:
    std::unique_ptr<InsCsvLoggerPrivate> p;
};
}