#include <string>
#include <vector>
#include <mutex>
#include <thread>

#include <gazebo/common/PID.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>

// 引入 ROS 核心及消息机制
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/Float64.h>
#include <rsos_msgs/SetGimbalAngle.h> // 🌟 自动匹配了你的 rsos_msgs 包

#include "GimbalSmall2dPlugin.hh"

using namespace gazebo;
using namespace std;

GZ_REGISTER_MODEL_PLUGIN(GimbalSmall2dPlugin)

class gazebo::GimbalSmall2dPluginPrivate
{
  // 🌟 将析构逻辑移入私有类，这样就不会和外部 .hh 冲突，且能安全释放线程
  public: ~GimbalSmall2dPluginPrivate()
  {
    if (rosNode) {
      rosNode->shutdown();
    }
    if (rosQueueThread.joinable()) {
      rosQueueThread.join();
    }
  }

  // 🌟 将这两个内部函数声明移入私有类
  public: void QueueThread();
  public: bool OnServiceCall(rsos_msgs::SetGimbalAngle::Request &req,
                              rsos_msgs::SetGimbalAngle::Response &res);

  public: std::vector<event::ConnectionPtr> connections;
  public: physics::ModelPtr model;
  public: physics::JointPtr tiltJoint;
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
  
  public: std::string currentMode = "body"; 
  public: double targetAngleDeg = 0.0;       
  public: common::Time lastPubTime;
};

GimbalSmall2dPlugin::GimbalSmall2dPlugin()
  : dataPtr(new GimbalSmall2dPluginPrivate)
{
  this->dataPtr->pid.Init(10.0, 0.1, 1.0, 10.0, -10.0, 10.0, -10.0);
}

// 🌟 删除了原本引起冲突的 GimbalSmall2dPlugin::~GimbalSmall2dPlugin() 外部实现

void GimbalSmall2dPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->dataPtr->model = _model;

  if (_sdf->HasElement("joint"))
    this->dataPtr->jointName = _sdf->Get<std::string>("joint");
    
  if (_sdf->HasElement("rosServiceName"))
    this->dataPtr->rosServiceName = _sdf->Get<std::string>("rosServiceName");

  if (_sdf->HasElement("rosTopicName"))
    this->dataPtr->rosTopicName = _sdf->Get<std::string>("rosTopicName");

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
    // 🌟 修复拼写错误：NoSigHandler -> NoSigintHandler
    ros::init(argc, argv, "gazebo_gimbal_plugin", ros::init_options::NoSigintHandler);
  }

  this->dataPtr->rosNode.reset(new ros::NodeHandle("~"));

  // 🌟 改变绑定目标：指向 GimbalSmall2dPluginPrivate::OnServiceCall
  ros::AdvertiseServiceOptions aso =
    ros::AdvertiseServiceOptions::create<rsos_msgs::SetGimbalAngle>(
      this->dataPtr->rosServiceName,
      boost::bind(&GimbalSmall2dPluginPrivate::OnServiceCall, this->dataPtr.get(), _1, _2),
      ros::VoidPtr(),
      &this->dataPtr->rosQueue);
  this->dataPtr->rosSrv = this->dataPtr->rosNode->advertiseService(aso);

  this->dataPtr->rosPub = this->dataPtr->rosNode->advertise<std_msgs::Float64>(this->dataPtr->rosTopicName, 1);

  // 🌟 改变绑定目标：指向 GimbalSmall2dPluginPrivate::QueueThread
  this->dataPtr->rosQueueThread = std::thread(std::bind(&GimbalSmall2dPluginPrivate::QueueThread, this->dataPtr.get()));
}

void GimbalSmall2dPlugin::Init()
{
  this->dataPtr->lastUpdateTime = this->dataPtr->model->GetWorld()->SimTime();
  this->dataPtr->lastPubTime = this->dataPtr->model->GetWorld()->SimTime();

  this->dataPtr->connections.push_back(event::Events::ConnectWorldUpdateBegin(
      std::bind(&GimbalSmall2dPlugin::OnUpdate, this)));
}

// 🌟 实现归属于 GimbalSmall2dPluginPrivate
void GimbalSmall2dPluginPrivate::QueueThread()
{
  static const double timeout = 0.01;
  while (this->rosNode->ok())
  {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}

// 🌟 实现归属于 GimbalSmall2dPluginPrivate
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
  res.message = "Gimbal command updated target to " + to_string(req.angle) + " (" + req.mode + ")";
  return true;
}

void GimbalSmall2dPlugin::OnUpdate()
{
  if (!this->dataPtr->tiltJoint) return;

  common::Time time = this->dataPtr->model->GetWorld()->SimTime();
  if (time <= this->dataPtr->lastUpdateTime)
  {
    this->dataPtr->lastUpdateTime = time;
    return;
  }
  double dt = (time - this->dataPtr->lastUpdateTime).Double();
  this->dataPtr->lastUpdateTime = time;

  ignition::math::Pose3d worldPose = this->dataPtr->model->WorldPose();
  double vehiclePitchRad = worldPose.Rot().Pitch(); 

  double currentJointRad = this->dataPtr->tiltJoint->Position(0);
  double targetJointRad = 0.0;
  
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  double targetAngleRad = this->dataPtr->targetAngleDeg * M_PI / 180.0;

  if (this->dataPtr->currentMode == "body")
  {
    targetJointRad = targetAngleRad;
  }
  else if (this->dataPtr->currentMode == "abs")
  {
    targetJointRad = targetAngleRad + vehiclePitchRad;
  }

  double error = currentJointRad - targetJointRad;
  double force = this->dataPtr->pid.Update(error, dt);
  this->dataPtr->tiltJoint->SetForce(0, force);

  if ((time - this->dataPtr->lastPubTime).Double() >= 0.02)
  {
    std_msgs::Float64 pitchMsg;
    
    if (this->dataPtr->currentMode == "body")
    {
      pitchMsg.data = - (currentJointRad * 180.0 / M_PI);
    }
    else if (this->dataPtr->currentMode == "abs")
    {
      double currentAbsDownwardRad = currentJointRad - vehiclePitchRad;
      pitchMsg.data = - (currentAbsDownwardRad * 180.0 / M_PI);
    }

    this->dataPtr->rosPub.publish(pitchMsg);
    this->dataPtr->lastPubTime = time;
  }
}