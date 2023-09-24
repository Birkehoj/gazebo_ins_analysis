/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

////////////////////////////////////////////////////////////
//! [include statements]
#include <gz/physics/FeatureList.hh>
#include <gz/physics/FeaturePolicy.hh>
#include <gz/physics/GetEntities.hh>
#include <gz/physics/Register.hh>
//! [include statements]

namespace sample_system
{
////////////////////////////////////////////////////////
//! [feature list]
// List of all features that this plugin will implement
struct HelloWorldFeatureList : gz::physics::FeatureList<
    gz::physics::GetEngineInfo
> { };
//! [feature list]

////////////////////////////////////////////////////////
//! [implementation]
class SampleSystem:
    // This class is a system.
    public ignition::gazebo::System,
    // This class also implements the ISystemPostUpdate interface.
    public ignition::gazebo::ISystemPostUpdate
{
public: SampleSystem();
public: ~SampleSystem() override;
public: void PostUpdate(const ignition::gazebo::UpdateInfo &_info,
                        const ignition::gazebo::EntityComponentManager &_ecm) override;
};
//! [implementation]

////////////////////////////////////////////////////////
//! [register]
// Register plugin
// Include a line in your source file for each interface implemented.
IGNITION_ADD_PLUGIN(
    sample_system::SampleSystem,
    ignition::gazebo::System,
    sample_system::SampleSystem::ISystemPostUpdate)
//! [register]
}