#include <string>
#include <vector>
#include <mutex>
#include <thread>
#include <cmath>

#include "gazebo/common/PID.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/gazebo.hh"

// 引入 ROS 核心及消息机制
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/Float64.h>
#include <rsos_msgs/SetGimbalAngle.h> 

using namespace gazebo;
using namespace std;

#include "GimbalSmall2dPlugin.hh"

GZ_REGISTER_MODEL_PLUGIN(GimbalSmall2dPlugin)

class gazebo::GimbalSmall2dPluginPrivate
{
  public: ~GimbalSmall2dPluginPrivate()
  {
    if (rosNode) {
      rosNode->shutdown();
    }
    if (rosQueueThread.joinable()) {
      rosQueueThread.join();
    }
  }

  public: void QueueThread();
  public: bool OnServiceCall(rsos_msgs::SetGimbalAngle::Request &req,
                              rsos_msgs::SetGimbalAngle::Response &res);

  public: std::vector<event::ConnectionPtr> connections;
  public: physics::ModelPtr model;
  public: physics::JointPtr tiltJoint;
  
  // 🌟 回归原版：使用原作者的独立 PID 控制器
  public: common::PID pid;
  public: common::Time lastUpdateTime;

  // --- ROS 核心变量 ---
  public: unique_ptr<ros::NodeHandle> rosNode;
  public: ros::ServiceServer rosSrv;
  public: ros::Publisher rosPub;
  public: ros::CallbackQueue rosQueue;
  public: std::thread rosQueueThread;
  public: std::mutex mutex;

  // --- 配置参数与状态控制 ---
  public: std::string rosServiceName = "set_gimbal_angle";
  public: std::string rosTopicName = "gimbal_pitch";
  public: std::string jointName = "tilt_joint";
  public: std::string vehicleModelName = ""; 
  
  public: std::string currentMode = "body"; 
  // 🌟 默认角度设为向前的 0 度
  public: double targetAngleDeg = 0.0;       
  public: common::Time lastPubTime;
};

GimbalSmall2dPlugin::GimbalSmall2dPlugin()
  : dataPtr(new GimbalSmall2dPluginPrivate)
{
  // 🌟 100% 还原原作者的 PID 参数和严格的 ±1.0 扭矩限幅
  // Init(P, I, D, Imax, Imin, CmdMax, CmdMin)
  this->dataPtr->pid.Init(1, 0, 0, 0, 0, 1.0, -1.0);
}

void GimbalSmall2dPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->dataPtr->model = _model;

  if (_sdf->HasElement("joint"))
    this->dataPtr->jointName = _sdf->Get<std::string>("joint");
    
  if (_sdf->HasElement("rosServiceName"))
    this->dataPtr->rosServiceName = _sdf->Get<std::string>("rosServiceName");

  if (_sdf->HasElement("rosTopicName"))
    this->dataPtr->rosTopicName = _sdf->Get<std::string>("rosTopicName");

  if (_sdf->HasElement("vehicleModelName"))
    this->dataPtr->vehicleModelName = _sdf->Get<std::string>("vehicleModelName");

  this->dataPtr->tiltJoint = this->dataPtr->model->GetJoint(this->dataPtr->jointName);
  if (!this->dataPtr->tiltJoint)
  {
    std::string scopedJointName = _model->GetScopedName() + "::" + this->dataPtr->jointName;
    this->dataPtr->tiltJoint = this->dataPtr->model->GetJoint(scopedJointName);
  }
  if (!this->dataPtr->tiltJoint)
  {
    gzerr << "GimbalSmall2dPlugin::Load ERROR! Can't find joint: " << this->dataPtr->jointName << endl;
    return;
  }

  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "gazebo_gimbal_plugin", ros::init_options::NoSigintHandler);
  }

  this->dataPtr->rosNode.reset(new ros::NodeHandle("~"));

  ros::AdvertiseServiceOptions aso =
    ros::AdvertiseServiceOptions::create<rsos_msgs::SetGimbalAngle>(
      this->dataPtr->rosServiceName,
      boost::bind(&GimbalSmall2dPluginPrivate::OnServiceCall, this->dataPtr.get(), _1, _2),
      ros::VoidPtr(),
      &this->dataPtr->rosQueue);
  this->dataPtr->rosSrv = this->dataPtr->rosNode->advertiseService(aso);

  this->dataPtr->rosPub = this->dataPtr->rosNode->advertise<std_msgs::Float64>(this->dataPtr->rosTopicName, 1);
  this->dataPtr->rosQueueThread = std::thread(std::bind(&GimbalSmall2dPluginPrivate::QueueThread, this->dataPtr.get()));
}

void GimbalSmall2dPlugin::Init()
{
  this->dataPtr->lastUpdateTime = this->dataPtr->model->GetWorld()->SimTime();
  this->dataPtr->lastPubTime = this->dataPtr->model->GetWorld()->SimTime();

  this->dataPtr->connections.push_back(event::Events::ConnectWorldUpdateBegin(
      std::bind(&GimbalSmall2dPlugin::OnUpdate, this)));
}

void GimbalSmall2dPluginPrivate::QueueThread()
{
  static const double timeout = 0.01;
  while (this->rosNode->ok())
  {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}

bool GimbalSmall2dPluginPrivate::OnServiceCall(rsos_msgs::SetGimbalAngle::Request &req,
                                               rsos_msgs::SetGimbalAngle::Response &res)
{
  if (req.mode != "abs" && req.mode != "body")
  {
    res.success = false;
    res.message = "Error: Invalid mode! Supported modes: 'abs' or 'body'.";
    return true;
  }

  std::lock_guard<std::mutex> lock(this->mutex);
  this->currentMode = req.mode;
  this->targetAngleDeg = req.angle;

  res.success = true;
  res.message = "Gimbal target updated to " + to_string(req.angle) + " deg (" + req.mode + ")";
  return true;
}

void GimbalSmall2dPlugin::OnUpdate()
{
  if (!this->dataPtr->tiltJoint) return;

  common::Time time = this->dataPtr->model->GetWorld()->SimTime();
  if (time <= this->dataPtr->lastUpdateTime) return;

  // 1. 获取目标飞机模型姿态 (为 abs 模式准备)
  physics::ModelPtr vehicleModel = this->dataPtr->model; 
  if (!this->dataPtr->vehicleModelName.empty())
  {
    auto foundModel = this->dataPtr->model->GetWorld()->ModelByName(this->dataPtr->vehicleModelName);
    if (foundModel) vehicleModel = foundModel;
  }

  // 计算飞机低头弧度（Nose Down 为正）
  ignition::math::Pose3d worldPose = vehicleModel->WorldPose();
  ignition::math::Vector3d forwardVec = worldPose.Rot().RotateVector(ignition::math::Vector3d::UnitX);
  double dronePitchDownRad = atan2(-forwardVec.Z(), sqrt(forwardVec.X()*forwardVec.X() + forwardVec.Y()*forwardVec.Y()));

  // 2. 获取当前状态与目标状态
  double currentJointRad = this->dataPtr->tiltJoint->Position(0);
  
  double targetAngleRad = 0.0;
  std::string mode;
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
    targetAngleRad = this->dataPtr->targetAngleDeg * M_PI / 180.0;
    mode = this->dataPtr->currentMode;
  }

  // 3. 计算物理关节应该达到的目标弧度 (0=向前, 1.57=向下)
  double targetJointRad = 0.0;
  if (mode == "body") {
    targetJointRad = targetAngleRad;
  } else if (mode == "abs") {
    targetJointRad = targetAngleRad - dronePitchDownRad;
  }

  // 🌟 4. 100% 还原原作者的底层力矩控制闭环算法
  double dt = (time - this->dataPtr->lastUpdateTime).Double();
  // 按照原作者公式：error = angle - command
  double error = currentJointRad - targetJointRad; 
  double force = this->dataPtr->pid.Update(error, dt);
  
  // 施加受限制的微小扭矩
  this->dataPtr->tiltJoint->SetForce(0, force);
  this->dataPtr->lastUpdateTime = time;

  // 5. 发布符合 FRD 的状态反馈 (50Hz，向下为负)
  if ((time - this->dataPtr->lastPubTime).Double() >= 0.02)
  {
    std_msgs::Float64 pitchMsg;
    
    if (mode == "body") {
      pitchMsg.data = - (currentJointRad * 180.0 / M_PI);
    } else if (mode == "abs") {
      double currentAbsDownwardRad = currentJointRad + dronePitchDownRad;
      pitchMsg.data = - (currentAbsDownwardRad * 180.0 / M_PI);
    }

    this->dataPtr->rosPub.publish(pitchMsg);
    this->dataPtr->lastPubTime = time;
  }
}