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


#ifndef DRONEHMI_HPP_
#define DRONEHMI_HPP_

#include <gz/gui/qt.h>

#include <gz/msgs/float.pb.h>

#include <memory>
#include <string>

#include <gz/gui/Plugin.hh>
#include <gz/transport/Node.hh>

namespace gz
{
namespace aerial
{
namespace plugins
{

class AttitudeDisplayPrivate;

class DroneHmi : public gz::gui::Plugin
{
  Q_OBJECT

public:
  /// \brief Constructor
  DroneHmi();

  /// \brief Destructor
  virtual ~DroneHmi();

  /// \brief Called by Gazebo GUI when plugin is instantiated.
  /// \param[in] _pluginElem XML configuration for this plugin.
  void LoadConfig(const tinyxml2::XMLElement * _pluginElem) override;

  /// \brief Notify that a new image has been received.

  void Update();

  void DrawRoll(QPainter & painter);
  void DrawPitch(QPainter & painter);
  void DrawBackground(QPainter & painter);

  /// \brief Frequency
  Q_PROPERTY(
    double heading
    READ Heading
    WRITE SetHeading
    NOTIFY newHeading
  )

public:
  double Heading();

public:
  Q_INVOKABLE void SetHeading(const double);

signals:
  void newImage();
  void newHeading();

private:
  gz::transport::Node node_;

  void onHeadingReceived(const gz::msgs::Float & _msg);
  void onPitchReceived(const gz::msgs::Float & _msg);
  void onRollReceived(const gz::msgs::Float & _msg);

  /// \internal
  /// \brief Pointer to private data.

private:
  std::unique_ptr<AttitudeDisplayPrivate> dataPtr;
};

}  // namespace plugins
}  // namespace aerial
}  // namespace gz

#endif  // DRONEHMI_HPP_
