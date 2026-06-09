#include <cmath>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

// 引入 ROS 核心及消息机制
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <rsos_msgs/SetGimbalAngle.h>
#include <std_msgs/Float64.h>

using namespace gazebo;
using namespace std;

#include "GimbalSmall2dPlugin.hh"

GZ_REGISTER_MODEL_PLUGIN(GimbalSmall2dPlugin)

class gazebo::GimbalSmall2dPluginPrivate {
public:
  ~GimbalSmall2dPluginPrivate() {
    if (rosNode) {
      rosNode->shutdown();
    }
    if (rosQueueThread.joinable()) {
      rosQueueThread.join();
    }
  }

public:
  void QueueThread();

public:
  bool OnServiceCall(rsos_msgs::SetGimbalAngle::Request &req,
                     rsos_msgs::SetGimbalAngle::Response &res);

public:
  std::vector<event::ConnectionPtr> connections;

public:
  physics::ModelPtr model;

public:
  physics::JointPtr tiltJoint;

public:
  common::Time lastUpdateTime;

  // --- ROS 核心变量 ---
public:
  unique_ptr<ros::NodeHandle> rosNode;

public:
  ros::ServiceServer rosSrv;

public:
  ros::Publisher rosPub;

public:
  ros::CallbackQueue rosQueue;

public:
  std::thread rosQueueThread;

public:
  std::mutex mutex;

  // --- 配置参数与状态控制 ---
public:
  std::string rosServiceName = "set_gimbal_angle";

public:
  std::string rosTopicName = "gimbal_pitch";

public:
  std::string jointName = "tilt_joint";

public:
  std::string jointNameScoped = ""; // 🌟 存储带作用域的关节名
public:
  std::string vehicleModelName = "";

public:
  std::string currentMode = "body";

public:
  double targetAngleDeg = 0.0;

public:
  common::Time lastPubTime;
};

GimbalSmall2dPlugin::GimbalSmall2dPlugin()
    : dataPtr(new GimbalSmall2dPluginPrivate) {}

void GimbalSmall2dPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  this->dataPtr->model = _model;

  if (_sdf->HasElement("joint"))
    this->dataPtr->jointName = _sdf->Get<std::string>("joint");

  if (_sdf->HasElement("rosServiceName"))
    this->dataPtr->rosServiceName = _sdf->Get<std::string>("rosServiceName");

  if (_sdf->HasElement("rosTopicName"))
    this->dataPtr->rosTopicName = _sdf->Get<std::string>("rosTopicName");

  if (_sdf->HasElement("vehicleModelName"))
    this->dataPtr->vehicleModelName =
        _sdf->Get<std::string>("vehicleModelName");

  this->dataPtr->tiltJoint =
      this->dataPtr->model->GetJoint(this->dataPtr->jointName);
  if (!this->dataPtr->tiltJoint) {
    std::string scopedJointName =
        _model->GetScopedName() + "::" + this->dataPtr->jointName;
    this->dataPtr->tiltJoint = this->dataPtr->model->GetJoint(scopedJointName);
  }
  if (!this->dataPtr->tiltJoint) {
    gzerr << "GimbalSmall2dPlugin::Load ERROR! Can't find joint: "
          << this->dataPtr->jointName << endl;
    return;
  }

  // 🌟 获取带作用域的精确关节名称，用于 JointController
  this->dataPtr->jointNameScoped = this->dataPtr->tiltJoint->GetScopedName();

  if (!ros::isInitialized()) {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "gazebo_gimbal_plugin",
              ros::init_options::NoSigintHandler);
  }

  this->dataPtr->rosNode.reset(new ros::NodeHandle("~"));

  ros::AdvertiseServiceOptions aso =
      ros::AdvertiseServiceOptions::create<rsos_msgs::SetGimbalAngle>(
          this->dataPtr->rosServiceName,
          boost::bind(&GimbalSmall2dPluginPrivate::OnServiceCall,
                      this->dataPtr.get(), _1, _2),
          ros::VoidPtr(), &this->dataPtr->rosQueue);
  this->dataPtr->rosSrv = this->dataPtr->rosNode->advertiseService(aso);

  this->dataPtr->rosPub = this->dataPtr->rosNode->advertise<std_msgs::Float64>(
      this->dataPtr->rosTopicName, 1);
  this->dataPtr->rosQueueThread = std::thread(
      std::bind(&GimbalSmall2dPluginPrivate::QueueThread, this->dataPtr.get()));
}

void GimbalSmall2dPlugin::Init() {
  this->dataPtr->lastUpdateTime = this->dataPtr->model->GetWorld()->SimTime();
  this->dataPtr->lastPubTime = this->dataPtr->model->GetWorld()->SimTime();

  // 🌟 核心注入：在内置的 JointController 中注册该云台关节，并赋予一套合理的隐式
  // PID 参数
  // 这套参数既够硬（锁死不抖），又合乎物理规律（有最大力矩限制，不伤飞机）
  // this->dataPtr->model->GetJointController()->AddJoint(
  //     this->dataPtr->tiltJoint);
  // gazebo::common::PID safePid(25.0, 0.5, 1.2, 20.0, -20.0, 40.0, -40.0);
  // this->dataPtr->model->GetJointController()->SetPositionPID(
  //     this->dataPtr->jointNameScoped, safePid);

  this->dataPtr->connections.push_back(event::Events::ConnectWorldUpdateBegin(
      std::bind(&GimbalSmall2dPlugin::OnUpdate, this)));
}

void GimbalSmall2dPluginPrivate::QueueThread() {
  static const double timeout = 0.01;
  while (this->rosNode->ok()) {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}

bool GimbalSmall2dPluginPrivate::OnServiceCall(
    rsos_msgs::SetGimbalAngle::Request &req,
    rsos_msgs::SetGimbalAngle::Response &res) {
  if (req.mode != "abs" && req.mode != "body") {
    res.success = false;
    res.message = "Error: Invalid mode! Supported modes: 'abs' or 'body'.";
    return true;
  }

  std::lock_guard<std::mutex> lock(this->mutex);
  this->currentMode = req.mode;
  this->targetAngleDeg = req.angle;

  res.success = true;
  res.message = "Gimbal target updated to " + to_string(req.angle) + " deg (" +
                req.mode + ")";
  return true;
}

void GimbalSmall2dPlugin::OnUpdate()
{
  if (!this->dataPtr->tiltJoint) return;

  // 1. 获取机身纯净姿态
  physics::LinkPtr parentLink = this->dataPtr->tiltJoint->GetParent();
  if (!parentLink) return;
  ignition::math::Pose3d parentWorldPose = parentLink->WorldPose();

  // 2. 读取 ROS 目标角度（输入：0=水平向前，90=垂直向下）
  double targetAngleRad = 0.0;
  std::string mode;
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    targetAngleRad = this->dataPtr->targetAngleDeg * M_PI / 180.0;
    mode = this->dataPtr->currentMode;
  }

  double targetJointRad = 0.0;

  if (mode == "body") 
  {
    // 机身模式：由于你的关节天生就是 0度=向前，90度=向下，直接赋值
    targetJointRad = targetAngleRad;
  } 
  else if (mode == "abs") 
  {
    // 绝对大地模式：
    // Gazebo 标准的 parentWorldPose.Rot().Pitch() 定义是：抬头为正，低头为负
    // 物理定量分析：
    // 当输入绝对 90 度（向下）时：
    // - 如果飞机水平（Pitch=0），云台转动 90 度，垂直向下。
    // - 如果飞机抬头（Pitch=+10度），机头往上翘，云台相对机身必须【多向下转 10 度】才能保持垂直向下。
    // 综上所述，逻辑应该是非常纯粹的加法：
    double vehiclePitchRad = parentWorldPose.Rot().Pitch();
    targetJointRad = targetAngleRad + vehiclePitchRad;
  }

  // 3. 强行设定关节物理位置（无 PID，彻底消除突变和打架导致的微小抖动）
  this->dataPtr->tiltJoint->SetPosition(0, targetJointRad);

  // 4. 实时发布状态 (50Hz)
  common::Time time = this->dataPtr->model->GetWorld()->SimTime();
  if ((time - this->dataPtr->lastPubTime).Double() >= 0.02) {
    std_msgs::Float64 pitchMsg;
    double currentJointRad = this->dataPtr->tiltJoint->Position(0);
    double vehiclePitchRad = parentWorldPose.Rot().Pitch();

    if (mode == "body") {
      pitchMsg.data = -(currentJointRad * 180.0 / M_PI);
    } else if (mode == "abs") {
      // 绝对模式下，反馈当前相对于大地的绝对角度
      double currentAbsRad = currentJointRad - vehiclePitchRad;
      pitchMsg.data = -(currentAbsRad * 180.0 / M_PI);
    }

    this->dataPtr->rosPub.publish(pitchMsg);
    this->dataPtr->lastPubTime = time;
  }
}
