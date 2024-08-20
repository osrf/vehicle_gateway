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

#include "betaflight_gazebo/BetaflightPlugin.hpp"
#include "betaflight_gazebo/Util.hpp"

#include <gz/sim/components/Imu.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/World.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/PID.hh>
#include <gz/math/Filter.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <gz/sim/Util.hh>

#include "betaflight_gazebo/BetaflightSocket.hpp"


// MAX_MOTORS limits the maximum number of <control> elements that
// can be defined in the <plugin>.
#define MAX_MOTORS 255

// SITL JSON interface supplies 16 servo channels
#define MAX_SERVO_CHANNELS 16

// Register plugin
GZ_ADD_PLUGIN(
  betaflight_gazebo::BetaFlightPlugin,
  gz::sim::System,
  betaflight_gazebo::BetaFlightPlugin::ISystemConfigure,
  betaflight_gazebo::BetaFlightPlugin::ISystemPostUpdate,
  betaflight_gazebo::BetaFlightPlugin::ISystemReset,
  betaflight_gazebo::BetaFlightPlugin::ISystemPreUpdate)
// Add plugin alias so that we can refer to the plugin without the version
// namespace
GZ_ADD_PLUGIN_ALIAS(betaflight_gazebo::BetaFlightPlugin, "BetaFlightPlugin")

/// \brief A servo packet.
struct ServoPacket
{
  /// \brief Motor speed data.
  /// should rename to servo_command here and in Betaflight SIM_Gazebo.cpp
  float motorSpeed[MAX_MOTORS] = {0.0f};
};

/// \brief Flight Dynamics Model packet that is sent back to the Betaflight
struct fdmPacket
{
  /// \brief packet timestamp
  double timestamp;

  /// \brief IMU angular velocity
  double imuAngularVelocityRPY[3];

  /// \brief IMU linear acceleration
  double imuLinearAccelerationXYZ[3];

  /// \brief IMU quaternion orientation
  double imuOrientationQuat[4];

  /// \brief Model velocity in NEED frame
  double velocityXYZ[3];

  /// \brief Model position in NEED frame
  double positionXYZ[3];

  /// \brief Pressure value
  double pressure;
};

/// \brief Control class
class Control
{
  /// \brief Constructor

public:
  Control()
  {
    // most of these coefficients are not used yet.
    this->rotorVelocitySlowdownSim = this->kDefaultRotorVelocitySlowdownSim;
    this->frequencyCutoff = this->kDefaultFrequencyCutoff;
    this->samplingRate = this->kDefaultSamplingRate;

    this->pid.Init(0.1, 0, 0, 0, 0, 1.0, -1.0);
  }

public:
  ~Control() {}

  /// \brief The PWM channel used to command this control

public:
  int channel = 0;

  /// \brief Next command to be applied to the joint

public:
  double cmd = 0;

  /// \brief Velocity PID for motor control

public:
  gz::math::PID pid;

  /// \brief The controller type
  ///
  /// Valid controller types are:
  ///   VELOCITY control velocity of joint
  ///   POSITION control position of joint
  ///   EFFORT control effort of joint
  ///   COMMAND control sends command to topic

public:
  std::string type;

  /// \brief Use force controller

public:
  bool useForce = true;

  /// \brief The name of the joint being controlled

public:
  std::string jointName;

  /// \brief The name of the topic to forward this command

public:
  std::string cmdTopic;

  /// \brief The joint being controlled

public:
  gz::sim::Entity joint;

  /// \brief A multiplier to scale the raw input command

public:
  double multiplier = 1.0;

  /// \brief An offset to shift the zero-point of the raw input command

public:
  double offset = 0.0;

  /// \brief Lower bound of PWM input, has default (1000).
  ///
  /// The lower bound of PWM input should match SERVOX_MIN for this channel.

public:
  double servo_min = 1000.0;

  /// \brief Upper limit of PWM input, has default (2000).
  ///
  /// The upper limit of PWM input should match SERVOX_MAX for this channel.

public:
  double servo_max = 2000.0;

  /// \brief Publisher for sending commands

public:
  gz::transport::Node::Publisher pub;

  /// \brief unused coefficients

public:
  double rotorVelocitySlowdownSim;

public:
  double frequencyCutoff;

public:
  double samplingRate;

public:
  gz::math::OnePole<double> filter;

public:
  static double kDefaultRotorVelocitySlowdownSim;

public:
  static double kDefaultFrequencyCutoff;

public:
  static double kDefaultSamplingRate;
};

double Control::kDefaultRotorVelocitySlowdownSim = 10.0;
double Control::kDefaultFrequencyCutoff = 5.0;
double Control::kDefaultSamplingRate = 0.2;

/////////////////////////////////////////////////
// Private data class
class betaflight_gazebo::BetaFlightPluginPrivate
{
  /// \brief The model

public:
  gz::sim::Model model{gz::sim::kNullEntity};
  //
  /// \brief String of the model name;

public:
  std::string modelName;

  /// \brief The world

public:
  gz::sim::World world{gz::sim::kNullEntity};

  /// \brief The world name;

public:
  std::string worldName;

  /// \brief The name of the IMU sensor

public:
  std::string imuName;

  /// \brief Have we initialized subscription to the IMU data yet?

public:
  bool imuInitialized{false};

  /// \brief Transform from model orientation to x-forward and z-up

public:
  gz::math::Pose3d modelXYZToAirplaneXForwardZDown;

  /// \brief Transform from world frame to NEED frame

public:
  gz::math::Pose3d gazeboXYZToNED;

  /// \brief array of propellers

public:
  std::vector<Control> controls;

  /// \brief We need an gz-transport Node to subscribe to IMU data

public:
  gz::transport::Node node;

  /// \brief The address for the flight dynamics model (i.e. this plugin)

public:
  std::string fdm_addr{"127.0.0.1"};

  /// \brief The address for the SITL flight controller - auto detected

public:
  std::string listen_addr{"127.0.0.1"};

  /// \brief The port for the flight dynamics model

public:
  uint16_t fdm_port_in{9002};

  /// \brief The port for the SITL flight controller - auto detected

public:
  uint16_t fcu_port_out{9003};

  /// \brief Controller update mutex.

public:
  std::mutex mutex;
  //
  /// \brief Betaflight Socket for receive motor command on gazebo

public:
  BetaflightSocket socket_in;

  /// \brief Betaflight Socket to send state to Betaflight

public:
  BetaflightSocket socket_out;

  /// \brief The entity representing the link containing the imu sensor.

public:
  gz::sim::Entity imuLink{gz::sim::kNullEntity};

  /// \brief A copy of the most recently received IMU data message

public:
  gz::msgs::IMU imuMsg;

  /// \brief Have we received at least one IMU data message?

public:
  bool imuMsgValid{false};

  /// \brief This mutex should be used when accessing imuMsg or imuMsgValid

public:
  std::mutex imuMsgMutex;

  /// \brief This subscriber callback latches the most recently received
  ///        IMU data message for later use.

public:
  void ImuCb(const gz::msgs::IMU & _msg)
  {
    std::lock_guard<std::mutex> lock(this->imuMsgMutex);
    imuMsg = _msg;
    imuMsgValid = true;
  }

  float pressure = 101325;    // pressure in Pa (0m MSL);
  std::mutex airPressureMsgMutex;

  void onAirPressureMessageReceived(const gz::msgs::FluidPressure & _msg)
  {
    std::lock_guard<std::mutex> lock(this->airPressureMsgMutex);
    this->pressure = _msg.pressure();
  }

  /// \brief keep track of controller update sim-time.

public:
  std::chrono::steady_clock::duration lastControllerUpdateTime{0};

  /// \brief Keep track of the time the last servo packet was received.

public:
  std::chrono::steady_clock::duration lastServoPacketRecvTime{0};

  /// \brief Set true to enforce lock-step simulation

public:
  bool isLockStep{false};

  /// \brief false before Betaflight controller is online
  /// to allow gazebo to continue without waiting

public:
  bool betaflightOnline;

  /// \brief number of times ArduCotper skips update

public:
  int connectionTimeoutCount;

  /// \brief number of times ArduCotper skips update
  /// before marking Betaflight offline

public:
  int connectionTimeoutMaxCount;
};

/////////////////////////////////////////////////
betaflight_gazebo::BetaFlightPlugin::BetaFlightPlugin()
: dataPtr(new BetaFlightPluginPrivate())
{
}

/////////////////////////////////////////////////
betaflight_gazebo::BetaFlightPlugin::~BetaFlightPlugin()
{
}

/////////////////////////////////////////////////
void betaflight_gazebo::BetaFlightPlugin::Reset(
  const gz::sim::UpdateInfo & _info,
  gz::sim::EntityComponentManager & _ecm)
{
  // if (!_ecm.EntityHasComponentType(this->dataPtr->imuLink,
  //     gz::sim::components::WorldPose::typeId))
  // {
  //     _ecm.CreateComponent(this->dataPtr->imuLink,
  //         gz::sim::components::WorldPose());
  // }
  // if (!_ecm.EntityHasComponentType(this->dataPtr->imuLink,
  //     gz::sim::components::WorldLinearVelocity::typeId))
  // {
  //     _ecm.CreateComponent(this->dataPtr->imuLink,
  //     gz::sim::components::WorldLinearVelocity());
  // }
  //
  // // update velocity PID for controls and apply force to joint
  // for (size_t i = 0; i < this->dataPtr->controls.size(); ++i)
  // {
  //   gz::sim::components::JointForceCmd* jfcComp = nullptr;
  //   gz::sim::components::JointVelocityCmd* jvcComp = nullptr;
  //   if (this->dataPtr->controls[i].useForce ||
  //       this->dataPtr->controls[i].type == "EFFORT")
  //   {
  //     jfcComp = _ecm.Component<gz::sim::components::JointForceCmd>(
  //         this->dataPtr->controls[i].joint);
  //     if (jfcComp == nullptr)
  //     {
  //       jfcComp = _ecm.CreateComponent(this->dataPtr->controls[i].joint,
  //           gz::sim::components::JointForceCmd({0}));
  //     }
  //   }
  //   else if (this->dataPtr->controls[i].type == "VELOCITY")
  //   {
  //     jvcComp = _ecm.Component<gz::sim::components::JointVelocityCmd>(
  //         this->dataPtr->controls[i].joint);
  //     if (jvcComp == nullptr)
  //     {
  //       jvcComp = _ecm.CreateComponent(this->dataPtr->controls[i].joint,
  //           gz::sim::components::JointVelocityCmd({0}));
  //     }
  //   }
  // }
}

/////////////////////////////////////////////////
void betaflight_gazebo::BetaFlightPlugin::Configure(
  const gz::sim::Entity & _entity,
  const std::shared_ptr<const sdf::Element> & _sdf,
  gz::sim::EntityComponentManager & _ecm,
  gz::sim::EventManager & /*&_eventMgr*/)
{
  // Make a clone so that we can call non-const methods
  sdf::ElementPtr sdfClone = _sdf->Clone();

  this->dataPtr->model = gz::sim::Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm)) {
    gzerr << "BetaFlightPlugin should be attached to a model "
          << "entity. Failed to initialize." << "\n";
    return;
  }
  this->dataPtr->modelName = this->dataPtr->model.Name(_ecm);

  this->dataPtr->world = gz::sim::World(
    _ecm.EntityByComponents(gz::sim::components::World()));
  if (!this->dataPtr->world.Valid(_ecm)) {
    gzerr << "World entity not found" << std::endl;
    return;
  }
  if (this->dataPtr->world.Name(_ecm).has_value()) {
    this->dataPtr->worldName = this->dataPtr->world.Name(_ecm).value();
  }

  // modelXYZToAirplaneXForwardZDown brings us from gazebo model frame:
  // x-forward, y-right, z-down
  // to the aerospace convention: x-forward, y-left, z-up
  this->dataPtr->modelXYZToAirplaneXForwardZDown =
    gz::math::Pose3d(0, 0, 0, GZ_PI, 0, 0);
  if (sdfClone->HasElement("modelXYZToAirplaneXForwardZDown")) {
    this->dataPtr->modelXYZToAirplaneXForwardZDown =
      sdfClone->Get<gz::math::Pose3d>("modelXYZToAirplaneXForwardZDown");
  }

  // gazeboXYZToNED: from gazebo model frame: x-forward, y-right, z-down
  // to the aerospace convention: x-forward, y-left, z-up
  this->dataPtr->gazeboXYZToNED = gz::math::Pose3d(0, 0, 0, GZ_PI, 0, 0);
  if (sdfClone->HasElement("gazeboXYZToNED")) {
    this->dataPtr->gazeboXYZToNED =
      sdfClone->Get<gz::math::Pose3d>("gazeboXYZToNED");
  }

  // TODO(ahcorde): fix this topic name
  this->dataPtr->node.Subscribe(
    "/world/empty_betaflight_world/model/iris_with_Betaflight/model/iris_with_standoffs/"
    "link/imu_link/sensor/air_pressure_sensor/air_pressure",
    &BetaFlightPluginPrivate::onAirPressureMessageReceived, this->dataPtr.get());

  // Load control channel params
  this->LoadControlChannels(sdfClone, _ecm);

  // Load sensor params
  this->LoadImuSensors(sdfClone, _ecm);

  // Initialise sockets
  if (!InitSockets(sdfClone)) {
    return;
  }

  // Missed update count before we declare betaflightOnline status false
  this->dataPtr->connectionTimeoutMaxCount =
    sdfClone->Get("connectionTimeoutMaxCount", 10).first;

  // Enforce lock-step simulation (has default: false)
  this->dataPtr->isLockStep =
    sdfClone->Get("lock_step", this->dataPtr->isLockStep).first;
  //
  // // Add the signal handler
  // this->dataPtr->sigHandler.AddCallback(
  //     std::bind(
  //       &betaflight_gazebo::BetaFlightPluginPrivate::OnSignal,
  //       this->dataPtr.get(),
  //       std::placeholders::_1));
  //
  gzlog << "[" << this->dataPtr->modelName << "] "
        << "Betaflight ready to fly. The force will be with you" << "\n";
}

/////////////////////////////////////////////////
void betaflight_gazebo::BetaFlightPlugin::LoadControlChannels(
  sdf::ElementPtr _sdf,
  gz::sim::EntityComponentManager & _ecm)
{
  // per control channel
  sdf::ElementPtr controlSDF;
  if (_sdf->HasElement("control")) {
    controlSDF = _sdf->GetElement("control");
  } else if (_sdf->HasElement("rotor")) {
    gzwarn << "[" << this->dataPtr->modelName << "] "
           << "please deprecate <rotor> block, use <control> block instead.\n";
    controlSDF = _sdf->GetElement("rotor");
  }

  while (controlSDF) {
    Control control;

    if (controlSDF->HasAttribute("channel")) {
      control.channel =
        atoi(controlSDF->GetAttribute("channel")->GetAsString().c_str());
    } else if (controlSDF->HasAttribute("id")) {
      gzwarn << "[" << this->dataPtr->modelName << "] "
             << "please deprecate attribute id, use channel instead.\n";
      control.channel =
        atoi(controlSDF->GetAttribute("id")->GetAsString().c_str());
    } else {
      control.channel = this->dataPtr->controls.size();
      gzwarn << "[" << this->dataPtr->modelName << "] "
             << "id/channel attribute not specified, use order parsed ["
             << control.channel << "].\n";
    }

    if (controlSDF->HasElement("type")) {
      control.type = controlSDF->Get<std::string>("type");
    } else {
      gzerr << "[" << this->dataPtr->modelName << "] "
            << "Control type not specified,"
            << " using velocity control by default.\n";
      control.type = "VELOCITY";
    }

    if (control.type != "VELOCITY" &&
      control.type != "POSITION" &&
      control.type != "EFFORT" &&
      control.type != "COMMAND")
    {
      gzwarn << "[" << this->dataPtr->modelName << "] "
             << "Control type [" << control.type
             << "] not recognized, must be one of"
             << "VELOCITY, POSITION, EFFORT, COMMAND."
             << " default to VELOCITY.\n";
      control.type = "VELOCITY";
    }

    if (controlSDF->HasElement("useForce")) {
      control.useForce = controlSDF->Get<bool>("useForce");
    }

    if (controlSDF->HasElement("jointName")) {
      control.jointName = controlSDF->Get<std::string>("jointName");
    } else {
      gzerr << "[" << this->dataPtr->modelName << "] "
            << "Please specify a jointName,"
            << " where the control channel is attached.\n";
    }

    // Get the pointer to the joint.
    control.joint = JointByName(
      _ecm,
      this->dataPtr->model.Entity(), control.jointName);
    if (control.joint == gz::sim::kNullEntity) {
      gzerr << "[" << this->dataPtr->modelName << "] "
            << "Couldn't find specified joint ["
            << control.jointName << "]. This plugin will not run.\n";
      return;
    }

    // set up publisher if relaying the command
    if (control.type == "COMMAND") {
      if (controlSDF->HasElement("cmd_topic")) {
        control.cmdTopic = controlSDF->Get<std::string>("cmd_topic");
      } else {
        control.cmdTopic =
          "/world/" + this->dataPtr->worldName +
          "/model/" + this->dataPtr->modelName +
          "/joint/" + control.jointName + "/cmd";
        gzwarn << "[" << this->dataPtr->modelName << "] "
               << "Control type [" << control.type
               << "] requires a valid <cmd_topic>. Using default\n";
      }

      gzmsg << "[" << this->dataPtr->modelName << "] "
            << "Advertising on " << control.cmdTopic << ".\n";
      control.pub = this->dataPtr->
        node.Advertise<gz::msgs::Double>(control.cmdTopic);
    }

    if (controlSDF->HasElement("multiplier")) {
      // overwrite turningDirection, deprecated.
      control.multiplier = controlSDF->Get<double>("multiplier");
    } else if (controlSDF->HasElement("turningDirection")) {
      gzwarn << "[" << this->dataPtr->modelName << "] "
             << "<turningDirection> is deprecated. Please use"
             << " <multiplier>. Map 'cw' to '-1' and 'ccw' to '1'.\n";
      std::string turningDirection = controlSDF->Get<std::string>(
        "turningDirection");
      // special cases mimic from controls_gazebo_plugins
      if (turningDirection == "cw") {
        control.multiplier = -1;
      } else if (turningDirection == "ccw") {
        control.multiplier = 1;
      } else {
        gzdbg << "[" << this->dataPtr->modelName << "] "
              << "not string, check turningDirection as float\n";
        control.multiplier = controlSDF->Get<double>("turningDirection");
      }
    } else {
      gzdbg << "[" << this->dataPtr->modelName << "] "
            << "channel[" << control.channel
            << "]: <multiplier> (or deprecated <turningDirection>)"
            << " not specified, "
            << " default to " << control.multiplier
            << " (or deprecated <turningDirection> 'ccw').\n";
    }

    if (controlSDF->HasElement("offset")) {
      control.offset = controlSDF->Get<double>("offset");
    } else {
      gzdbg << "[" << this->dataPtr->modelName << "] "
            << "channel[" << control.channel
            << "]: <offset> not specified, default to "
            << control.offset << "\n";
    }

    if (controlSDF->HasElement("servo_min")) {
      control.servo_min = controlSDF->Get<double>("servo_min");
    } else {
      gzdbg << "[" << this->dataPtr->modelName << "] "
            << "channel[" << control.channel
            << "]: <servo_min> not specified, default to "
            << control.servo_min << "\n";
    }

    if (controlSDF->HasElement("servo_max")) {
      control.servo_max = controlSDF->Get<double>("servo_max");
    } else {
      gzdbg << "[" << this->dataPtr->modelName << "] "
            << "channel[" << control.channel
            << "]: <servo_max> not specified, default to "
            << control.servo_max << "\n";
    }

    control.rotorVelocitySlowdownSim =
      controlSDF->Get("rotorVelocitySlowdownSim", 1).first;

    if (gz::math::equal(control.rotorVelocitySlowdownSim, 0.0)) {
      gzwarn << "[" << this->dataPtr->modelName << "] "
             << "control for joint [" << control.jointName
             << "] rotorVelocitySlowdownSim is zero,"
             << " assume no slowdown.\n";
      control.rotorVelocitySlowdownSim = 1.0;
    }

    control.frequencyCutoff =
      controlSDF->Get("frequencyCutoff", control.frequencyCutoff).first;
    control.samplingRate =
      controlSDF->Get("samplingRate", control.samplingRate).first;

    // use gazebo::math::Filter
    control.filter.Fc(control.frequencyCutoff, control.samplingRate);

    // initialize filter to zero value
    control.filter.Set(0.0);

    // note to use this filter, do
    // stateFiltered = filter.Process(stateRaw);

    // Overload the PID parameters if they are available.
    double param;
    // carry over from ArduCopter plugin
    param = controlSDF->Get("vel_p_gain", control.pid.PGain()).first;
    control.pid.SetPGain(param);

    param = controlSDF->Get("vel_i_gain", control.pid.IGain()).first;
    control.pid.SetIGain(param);

    param = controlSDF->Get("vel_d_gain", control.pid.DGain()).first;
    control.pid.SetDGain(param);

    param = controlSDF->Get("vel_i_max", control.pid.IMax()).first;
    control.pid.SetIMax(param);

    param = controlSDF->Get("vel_i_min", control.pid.IMin()).first;
    control.pid.SetIMin(param);

    param = controlSDF->Get("vel_cmd_max", control.pid.CmdMax()).first;
    control.pid.SetCmdMax(param);

    param = controlSDF->Get("vel_cmd_min", control.pid.CmdMin()).first;
    control.pid.SetCmdMin(param);

    // new params, overwrite old params if exist
    param = controlSDF->Get("p_gain", control.pid.PGain()).first;
    control.pid.SetPGain(param);

    param = controlSDF->Get("i_gain", control.pid.IGain()).first;
    control.pid.SetIGain(param);

    param = controlSDF->Get("d_gain", control.pid.DGain()).first;
    control.pid.SetDGain(param);

    param = controlSDF->Get("i_max", control.pid.IMax()).first;
    control.pid.SetIMax(param);

    param = controlSDF->Get("i_min", control.pid.IMin()).first;
    control.pid.SetIMin(param);

    param = controlSDF->Get("cmd_max", control.pid.CmdMax()).first;
    control.pid.SetCmdMax(param);

    param = controlSDF->Get("cmd_min", control.pid.CmdMin()).first;
    control.pid.SetCmdMin(param);

    // set pid initial command
    control.pid.SetCmd(0.0);

    this->dataPtr->controls.push_back(control);
    controlSDF = controlSDF->GetNextElement("control");
  }
}

/////////////////////////////////////////////////
void betaflight_gazebo::BetaFlightPlugin::LoadImuSensors(
  sdf::ElementPtr _sdf,
  gz::sim::EntityComponentManager & /*_ecm*/)
{
  this->dataPtr->imuName =
    _sdf->Get("imuName", static_cast<std::string>("imu_sensor")).first;
}

/////////////////////////////////////////////////
void betaflight_gazebo::BetaFlightPlugin::PreUpdate(
  const gz::sim::UpdateInfo & _info,
  gz::sim::EntityComponentManager & _ecm)
{
  // This lookup is done in PreUpdate() because in Configure()
  // it's not possible to get the fully qualified topic name we want
  if (!this->dataPtr->imuInitialized) {
    // Set unconditionally because we're only going to try this once.
    this->dataPtr->imuInitialized = true;
    std::string imuTopicName;

    // The model must contain an imu sensor element:
    //  <sensor name="..." type="imu">
    //
    // Extract the following:
    //  - Sensor topic name: to subscribe to the imu data
    //  - Link containing the sensor: to get the pose to transform to
    //    the correct frame for Betaflight

    // try scoped names first
    auto entities = entitiesFromScopedName(
      this->dataPtr->imuName, _ecm, this->dataPtr->model.Entity());

    // fall-back to unscoped name
    if (entities.empty()) {
      entities = EntitiesFromUnscopedName(
        this->dataPtr->imuName, _ecm, this->dataPtr->model.Entity());
    }

    if (!entities.empty()) {
      if (entities.size() > 1) {
        gzwarn << "Multiple IMU sensors with name ["
               << this->dataPtr->imuName << "] found. "
               << "Using the first one.\n";
      }

      // select first entity
      gz::sim::Entity imuEntity = *entities.begin();

      // validate
      if (!_ecm.EntityHasComponentType(
          imuEntity,
          gz::sim::components::Imu::typeId))
      {
        gzerr << "Entity with name ["
              << this->dataPtr->imuName
              << "] is not an IMU sensor\n";
      } else {
        gzmsg << "Found IMU sensor with name ["
              << this->dataPtr->imuName
              << "]\n";

        // verify the parent of the imu sensor is a link.
        gz::sim::Entity parent = _ecm.ParentEntity(imuEntity);
        if (_ecm.EntityHasComponentType(
            parent,
            gz::sim::components::Link::typeId))
        {
          this->dataPtr->imuLink = parent;

          imuTopicName = gz::sim::scopedName(
            imuEntity, _ecm) + "/imu";

          gzdbg << "Computed IMU topic to be: "
                << imuTopicName << std::endl;
        } else {
          gzerr << "Parent of IMU sensor ["
                << this->dataPtr->imuName
                << "] is not a link\n";
        }
      }
    } else {
      gzerr << "[" << this->dataPtr->modelName << "] "
            << "imu_sensor [" << this->dataPtr->imuName
            << "] not found, abort Betaflight plugin." << "\n";
      return;
    }

    this->dataPtr->node.Subscribe(
      imuTopicName,
      &betaflight_gazebo::BetaFlightPluginPrivate::ImuCb,
      this->dataPtr.get());

    // Make sure that the 'imuLink' entity has WorldPose
    // and WorldLinearVelocity components, which we'll need later.
    if (!_ecm.EntityHasComponentType(
        this->dataPtr->imuLink,
        gz::sim::components::WorldPose::typeId))
    {
      _ecm.CreateComponent(
        this->dataPtr->imuLink,
        gz::sim::components::WorldPose());
    }
    if (!_ecm.EntityHasComponentType(
        this->dataPtr->imuLink,
        gz::sim::components::WorldLinearVelocity::typeId))
    {
      _ecm.CreateComponent(
        this->dataPtr->imuLink,
        gz::sim::components::WorldLinearVelocity());
    }
  } else {
    double t =
      std::chrono::duration_cast<std::chrono::duration<double>>(
      _info.simTime).count();
    // Update the control surfaces.
    if (!_info.paused && _info.simTime >
      this->dataPtr->lastControllerUpdateTime)
    {
      if (this->dataPtr->isLockStep) {
        while (!this->ReceiveServoPacket(t, _ecm) &&
          this->dataPtr->betaflightOnline)
        {
          // // SIGNINT should interrupt this loop.
          // if (this->dataPtr->signal != 0)
          // {
          //     break;
          // }
        }
        this->dataPtr->lastServoPacketRecvTime = _info.simTime;
      } else if (this->ReceiveServoPacket(t, _ecm)) {
        this->dataPtr->lastServoPacketRecvTime = _info.simTime;
      }

      if (this->dataPtr->betaflightOnline) {
        double dt =
          std::chrono::duration_cast<std::chrono::duration<double>>(
          _info.simTime - this->dataPtr->
          lastControllerUpdateTime).count();
        this->ApplyMotorForces(dt, _ecm);
      }
    }
  }
}

/////////////////////////////////////////////////
void betaflight_gazebo::BetaFlightPlugin::PostUpdate(
  const gz::sim::UpdateInfo & _info,
  const gz::sim::EntityComponentManager & _ecm)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // Publish the new state.
  if (!_info.paused && _info.simTime > this->dataPtr->lastControllerUpdateTime &&
    this->dataPtr->betaflightOnline)
  {
    double t =
      std::chrono::duration_cast<std::chrono::duration<double>>(
      _info.simTime).count();
    this->SendState(t, _ecm);
    this->dataPtr->lastControllerUpdateTime = _info.simTime;
  }
}

/////////////////////////////////////////////////
void betaflight_gazebo::BetaFlightPlugin::ResetPIDs()
{
  // Reset velocity PID for controls
  for (size_t i = 0; i < this->dataPtr->controls.size(); ++i) {
    this->dataPtr->controls[i].cmd = 0;
    // this->dataPtr->controls[i].pid.Reset();
  }
}

/////////////////////////////////////////////////
bool betaflight_gazebo::BetaFlightPlugin::InitSockets(sdf::ElementPtr _sdf) const
{
  this->dataPtr->fdm_addr =
    _sdf->Get("fdm_addr", static_cast<std::string>("127.0.0.1")).first;
  this->dataPtr->listen_addr =
    _sdf->Get("listen_addr", static_cast<std::string>("127.0.0.1")).first;
  this->dataPtr->fdm_port_in =
    _sdf->Get("fdm_port_in", static_cast<uint32_t>(9002)).first;
  this->dataPtr->fcu_port_out =
    _sdf->Get("fcu_port_out", static_cast<uint32_t>(9003)).first;

  if (!this->dataPtr->socket_in.Bind(
      this->dataPtr->listen_addr.c_str(),
      this->dataPtr->fdm_port_in))
  {
    gzerr << "[" << this->dataPtr->modelName << "] "
          << "failed to bind with " << this->dataPtr->listen_addr
          << ":" << this->dataPtr->fdm_port_in << " aborting plugin.\n";
    return false;
  }

  if (!this->dataPtr->socket_out.Connect(
      this->dataPtr->fdm_addr.c_str(),
      this->dataPtr->fcu_port_out))
  {
    gzerr << "[" << this->dataPtr->modelName << "] "
          << "failed to bind with " << this->dataPtr->fdm_addr
          << ":" << this->dataPtr->fcu_port_out << " aborting plugin.\n";
    return false;
  }

  return true;
}

/////////////////////////////////////////////////
void betaflight_gazebo::BetaFlightPlugin::ApplyMotorForces(
  const double _dt,
  gz::sim::EntityComponentManager & _ecm)
{
  // update velocity PID for controls and apply force to joint
  for (size_t i = 0; i < this->dataPtr->controls.size(); ++i) {
    // Publish commands to be relayed to other plugins
    if (this->dataPtr->controls[i].type == "COMMAND") {
      gz::msgs::Double cmd;
      cmd.set_data(this->dataPtr->controls[i].cmd);
      this->dataPtr->controls[i].pub.Publish(cmd);
      continue;
    }

    gz::sim::components::JointForceCmd * jfcComp = nullptr;
    gz::sim::components::JointVelocityCmd * jvcComp = nullptr;
    if (this->dataPtr->controls[i].useForce ||
      this->dataPtr->controls[i].type == "EFFORT")
    {
      jfcComp = _ecm.Component<gz::sim::components::JointForceCmd>(
        this->dataPtr->controls[i].joint);
      if (jfcComp == nullptr) {
        jfcComp = _ecm.CreateComponent(
          this->dataPtr->controls[i].joint,
          gz::sim::components::JointForceCmd({0}));
      }
    } else if (this->dataPtr->controls[i].type == "VELOCITY") {
      jvcComp = _ecm.Component<gz::sim::components::JointVelocityCmd>(
        this->dataPtr->controls[i].joint);
      if (jvcComp == nullptr) {
        jvcComp = _ecm.CreateComponent(
          this->dataPtr->controls[i].joint,
          gz::sim::components::JointVelocityCmd({0}));
      }
    }

    if (this->dataPtr->controls[i].useForce) {
      if (this->dataPtr->controls[i].type == "VELOCITY") {
        const double velTarget = this->dataPtr->controls[i].cmd /
          this->dataPtr->controls[i].rotorVelocitySlowdownSim;
        gz::sim::components::JointVelocity * vComp =
          _ecm.Component<gz::sim::components::JointVelocity>(
          this->dataPtr->controls[i].joint);
        if (vComp && !vComp->Data().empty()) {
          const double vel = vComp->Data()[0];
          const double error = vel - velTarget;
          const double force = this->dataPtr->controls[i].pid.Update(
            error, std::chrono::duration<double>(_dt));
          jfcComp->Data()[0] = force;
        }
      } else if (this->dataPtr->controls[i].type == "POSITION") {
        const double posTarget = this->dataPtr->controls[i].cmd;
        gz::sim::components::JointPosition * pComp =
          _ecm.Component<gz::sim::components::JointPosition>(
          this->dataPtr->controls[i].joint);
        if (pComp && !pComp->Data().empty()) {
          const double pos = pComp->Data()[0];
          const double error = pos - posTarget;
          const double force = this->dataPtr->controls[i].pid.Update(
            error, std::chrono::duration<double>(_dt));
          jfcComp->Data()[0] = force;
        }
      } else if (this->dataPtr->controls[i].type == "EFFORT") {
        const double force = this->dataPtr->controls[i].cmd;
        jfcComp->Data()[0] = force;
      } else {
        // do nothing
      }
    } else {
      if (this->dataPtr->controls[i].type == "VELOCITY") {
        jvcComp->Data()[0] = this->dataPtr->controls[i].cmd;
      } else if (this->dataPtr->controls[i].type == "POSITION") {
        /// \todo(anyone) figure out whether position control matters,
        /// and if so, how to use it.
        gzwarn << "Failed to do position control on joint " << i <<
          " because there's no JointPositionCmd component (yet?)" << "/n";
      } else if (this->dataPtr->controls[i].type == "EFFORT") {
        const double force = this->dataPtr->controls[i].cmd;
        jvcComp->Data()[0] = force;
      } else {
        // do nothing
      }
    }
  }
}

/////////////////////////////////////////////////
bool betaflight_gazebo::BetaFlightPlugin::ReceiveServoPacket(
  double _simTime,
  const gz::sim::EntityComponentManager & _ecm)
{
  // Added detection for whether Betaflight is online or not.
  // If Betaflight is detected (receive of fdm packet from someone),
  // then socket receive wait time is increased from 1ms to 1 sec
  // to accommodate network jitter.
  // If Betaflight is not detected, receive call blocks for 1ms
  // on each call.
  // Once Betaflight presence is detected, it takes this many
  // missed receives before declaring the FCS offline.

  uint32_t waitMs;
  if (this->dataPtr->betaflightOnline) {
    // Increase timeout for recv once we detect a packet from Betaflight FCS.
    // If this value is too high then it will block the main Gazebo
    // update loop and adversely affect the RTF.
    waitMs = 1;
  } else {
    // Otherwise skip quickly and do not set control force.
    waitMs = 1;
  }

  ServoPacket pkt;
  ssize_t recvSize =
    this->dataPtr->socket_in.Recv(&pkt, sizeof(ServoPacket), waitMs);

  // Drain the socket in the case we're backed up
  int counter = 0;
  ServoPacket last_pkt;
  while (true) {
    // last_pkt = pkt;
    const ssize_t recvSize_last =
      this->dataPtr->socket_in.Recv(&last_pkt, sizeof(ServoPacket), 0ul);
    if (recvSize_last == -1) {
      break;
    }
    counter++;
    pkt = last_pkt;
    recvSize = recvSize_last;
  }
  if (counter > 0) {
    gzdbg << "[" << this->dataPtr->modelName << "] "
          << "Drained n packets: " << counter << std::endl;
  }
  if (recvSize == -1) {
    // didn't receive a packet
    // gzdbg << "no packet\n";
    // gazebo::common::Time::NSleep(100);
    if (this->dataPtr->betaflightOnline) {
      // gzwarn << "[" << this->dataPtr->modelName << "] "
      //        << "Broken Betaflight connection, count ["
      //        << this->dataPtr->connectionTimeoutCount
      //        << "/" << this->dataPtr->connectionTimeoutMaxCount
      //        << "]\n";
      if (++this->dataPtr->connectionTimeoutCount >
        this->dataPtr->connectionTimeoutMaxCount)
      {
        this->dataPtr->connectionTimeoutCount = 0;
        // for lock-step resend last state rather than time out
        if (this->dataPtr->isLockStep) {
          this->SendState(_simTime, _ecm);
        } else {
          this->dataPtr->betaflightOnline = false;
          gzwarn << "[" << this->dataPtr->modelName << "] "
                 << "Broken Betaflight connection, resetting motor control.\n";
          this->ResetPIDs();
        }
      }
    } else {
      // Betaflight is not online yet, keep sending last state
      // until we get a servo packet from Betaflight
      this->SendState(_simTime, _ecm);

      gzdbg << "[" << this->dataPtr->modelName << "] "
            << "Waiting for Initial Betaflight connection, sending current state.\n";
    }
  } else {
    const ssize_t expectedPktSize =
      sizeof(pkt.motorSpeed[0]) * this->dataPtr->controls.size();
    if (recvSize < expectedPktSize) {
      gzerr << "[" << this->dataPtr->modelName << "] "
            << "got less than model needs. Got: " << recvSize
            << "commands, expected size: " << expectedPktSize << "\n";
    }
    const ssize_t recvChannels = recvSize / sizeof(pkt.motorSpeed[0]);

    if (!this->dataPtr->betaflightOnline) {
      gzdbg << "[" << this->dataPtr->modelName << "] "
            << "Betaflight controller online detected.\n";
      // made connection, set some flags
      this->dataPtr->connectionTimeoutCount = 0;
      this->dataPtr->betaflightOnline = true;
    }

    // compute command based on requested motorSpeed
    for (unsigned i = 0; i < this->dataPtr->controls.size(); ++i) {
      if (i < MAX_MOTORS) {
        if (this->dataPtr->controls[i].channel < recvChannels) {
          // bound incoming cmd between 0 and 1
          const double cmd = gz::math::clamp(
            pkt.motorSpeed[this->dataPtr->controls[i].channel],
            -1.0f, 1.0f);
          this->dataPtr->controls[i].cmd =
            this->dataPtr->controls[i].multiplier *
            (this->dataPtr->controls[i].offset + cmd);
          // gzdbg << "apply input chan[" << this->dataPtr->controls[i].channel
          //       << "] to control chan[" << i
          //       << "] with joint name ["
          //       << this->dataPtr->controls[i].jointName
          //       << "] raw cmd ["
          //       << pkt.motorSpeed[this->dataPtr->controls[i].channel]
          //       << "] adjusted cmd [" << this->dataPtr->controls[i].cmd
          //       << "].\n";
        } else {
          gzerr << "[" << this->dataPtr->modelName << "] "
                << "control[" << i << "] channel ["
                << this->dataPtr->controls[i].channel
                << "] is greater than incoming commands size["
                << recvChannels
                << "], control not applied.\n";
        }
      } else {
        gzerr << "[" << this->dataPtr->modelName << "] "
              << "too many motors, skipping [" << i
              << " > " << MAX_MOTORS << "].\n";
      }
    }
  }
  return true;
}

/////////////////////////////////////////////////
void betaflight_gazebo::BetaFlightPlugin::SendState(
  double _simTime,
  const gz::sim::EntityComponentManager & _ecm) const
{
  // send_fdm
  fdmPacket pkt;

  pkt.timestamp = _simTime;

  // Make a local copy of the latest IMU data (it's filled in
  // on receipt by ImuCb()).
  gz::msgs::IMU imuMsg;
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->imuMsgMutex);
    // Wait until we've received a valid message.
    if (!this->dataPtr->imuMsgValid) {
      return;
    }
    imuMsg = this->dataPtr->imuMsg;
  }

  {
    std::lock_guard<std::mutex> lock(this->dataPtr->airPressureMsgMutex);
    pkt.pressure = this->dataPtr->pressure;
  }

  // asssumed that the imu orientation is:
  //   x forward
  //   y right
  //   z down

  // get linear acceleration
  gz::math::Vector3d linearAccel{
    imuMsg.linear_acceleration().x(),
    imuMsg.linear_acceleration().y(),
    imuMsg.linear_acceleration().z()
  };

  // get angular velocity
  gz::math::Vector3d angularVel{
    imuMsg.angular_velocity().x(),
    imuMsg.angular_velocity().y(),
    imuMsg.angular_velocity().z(),
  };

  // copy to pkt
  pkt.imuLinearAccelerationXYZ[0] = linearAccel.X();
  pkt.imuLinearAccelerationXYZ[1] = linearAccel.Y();
  pkt.imuLinearAccelerationXYZ[2] = linearAccel.Z();
  // gzerr << "lin accel [" << linearAccel << "]\n";

  // copy to pkt
  pkt.imuAngularVelocityRPY[0] = angularVel.X();
  pkt.imuAngularVelocityRPY[1] = angularVel.Y();
  pkt.imuAngularVelocityRPY[2] = angularVel.Z();

  // get pose and velocity in Gazebo world frame
  const gz::sim::components::WorldPose * worldPose =
    _ecm.Component<gz::sim::components::WorldPose>(
    this->dataPtr->imuLink);

  const gz::sim::components::WorldLinearVelocity * worldLinearVel =
    _ecm.Component<gz::sim::components::WorldLinearVelocity>(
    this->dataPtr->imuLink);

  // get inertial pose and velocity
  // position of the uav in world frame
  // this position is used to calculate bearing and distance
  // from starting location, then use that to update gps position.
  // The algorithm looks something like below (from Betaflight helper
  // libraries):
  //   bearing = to_degrees(atan2(position.y, position.x));
  //   distance = math.sqrt(self.position.x**2 + self.position.y**2)
  //   (self.latitude, self.longitude) = util.gps_newpos(
  //    self.home_latitude, self.home_longitude, bearing, distance)
  // where xyz is in the NEED directions.
  // Gazebo world xyz is assumed to be N, -E, -D, so flip some stuff
  // around.
  // orientation of the uav in world NEED frame -
  // assuming the world NEED frame has xyz mapped to NEED,
  // imuLink is NEED - z down

  // model world pose brings us to model,
  // which for example zephyr has -y-forward, x-left, z-up
  // adding modelXYZToAirplaneXForwardZDown rotates
  //   from: model XYZ
  //   to: airplane x-forward, y-left, z-down
  const gz::math::Pose3d gazeboXYZToModelXForwardZDown =
    this->dataPtr->modelXYZToAirplaneXForwardZDown + worldPose->Data();

  // get transform from world NEED to Model frame
  const gz::math::Pose3d NEDToModelXForwardZUp =
    gazeboXYZToModelXForwardZDown - this->dataPtr->gazeboXYZToNED;

  // gzerr << "need to model [" << NEDToModelXForwardZUp << "]\n";

  // N
  pkt.positionXYZ[0] = NEDToModelXForwardZUp.Pos().X();

  // E
  pkt.positionXYZ[1] = NEDToModelXForwardZUp.Pos().Y();

  // D
  pkt.positionXYZ[2] = NEDToModelXForwardZUp.Pos().Z();

  // imuOrientationQuat is the rotation from world NEED frame
  // to the uav frame.
  pkt.imuOrientationQuat[0] = NEDToModelXForwardZUp.Rot().W();
  pkt.imuOrientationQuat[1] = NEDToModelXForwardZUp.Rot().X();
  pkt.imuOrientationQuat[2] = NEDToModelXForwardZUp.Rot().Y();
  pkt.imuOrientationQuat[3] = NEDToModelXForwardZUp.Rot().Z();

  // gzdbg << "imu [" << gazeboXYZToModelXForwardZDown.rot.GetAsEuler()
  //       << "]\n";
  // gzdbg << "need [" << this->gazeboXYZToNED.rot.GetAsEuler() << "]\n";
  // gzdbg << "rot [" << NEDToModelXForwardZUp.rot.GetAsEuler() << "]\n";

  // Get NEED velocity in body frame *
  // or...
  // Get model velocity in NEED frame
  const gz::math::Vector3d velGazeboWorldFrame = worldLinearVel->Data();
  const gz::math::Vector3d velNEDFrame =
    this->dataPtr->gazeboXYZToNED.Rot().RotateVectorReverse(velGazeboWorldFrame);
  pkt.velocityXYZ[0] = velNEDFrame.X();
  pkt.velocityXYZ[1] = velNEDFrame.Y();
  pkt.velocityXYZ[2] = velNEDFrame.Z();

  this->dataPtr->socket_out.Send(&pkt, sizeof(pkt));
}
