// Copyright 2023 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef BETAFLIGHT_GAZEBO__BETAFLIGHTPLUGIN_HPP_
#define BETAFLIGHT_GAZEBO__BETAFLIGHTPLUGIN_HPP_

#include <memory>

#include <gz/sim/System.hh>
#include <sdf/sdf.hh>

namespace betaflight_gazebo
{
// The servo packet received from ArduPilot SITL. Defined in SIM_JSON.h.
struct servo_packet
{
  uint16_t magic;           // 18458 expected magic value
  uint16_t frame_rate;
  uint32_t frame_count;
  uint16_t pwm[16];
};

class BetaFlightPluginPrivate;

class GZ_SIM_VISIBLE BetaFlightPlugin
  : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPostUpdate,
  public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemReset
{
  /// \brief Constructor.

public:
  BetaFlightPlugin();

  /// \brief Destructor.

public:
  ~BetaFlightPlugin();

public:
  void Reset(
    const gz::sim::UpdateInfo & _info,
    gz::sim::EntityComponentManager & _ecm) final;

  /// \brief Load configuration from SDF on startup.

public:
  void Configure(
    const gz::sim::Entity & _entity,
    const std::shared_ptr<const sdf::Element> & _sdf,
    gz::sim::EntityComponentManager & _ecm,
    gz::sim::EventManager & _eventMgr) final;

  /// \brief Do the part of one update loop that involves making
  ///        changes to simulation.

public:
  void PreUpdate(
    const gz::sim::UpdateInfo & _info,
    gz::sim::EntityComponentManager & _ecm) final;

  /// \brief Do the part of one update loop that involves
  ///        reading results from simulation.

public:
  void PostUpdate(
    const gz::sim::UpdateInfo & _info,
    const gz::sim::EntityComponentManager & _ecm) final;

  /// \brief Load control channels

private:
  void LoadControlChannels(
    sdf::ElementPtr _sdf,
    gz::sim::EntityComponentManager & _ecm);

  /// \brief Load IMU sensors

private:
  void LoadImuSensors(
    sdf::ElementPtr _sdf,
    gz::sim::EntityComponentManager & _ecm);

  /// \brief Update the control surfaces controllers.
  /// \param[in] _info Update information provided by the server.

private:
  void OnUpdate();

  /// \brief Update PID Joint controllers.
  /// \param[in] _dt time step size since last update.

private:
  void ApplyMotorForces(
    const double _dt,
    gz::sim::EntityComponentManager & _ecm);

  /// \brief Reset PID Joint controllers.

private:
  void ResetPIDs();

  /// \brief Receive a servo packet from ArduPilot
  ///
  /// Returns true if a servo packet was received, otherwise false.

private:
  bool ReceiveServoPacket(double _simTime, const gz::sim::EntityComponentManager & _ecm);

  /// \brief Send state to ArduPilot

private:
  void SendState(double _simTime, const gz::sim::EntityComponentManager & _ecm) const;

  /// \brief Initialise flight dynamics model socket

private:
  bool InitSockets(sdf::ElementPtr _sdf) const;

  /// \brief Private data pointer.

private:
  std::unique_ptr<BetaFlightPluginPrivate> dataPtr;
};

}  // namespace betaflight_gazebo

#endif  // BETAFLIGHT_GAZEBO__BETAFLIGHTPLUGIN_HPP_
