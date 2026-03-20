#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>
#include <functional>
#include <algorithm>

namespace gazebo
{
class MoveObstaclePlugin : public ModelPlugin
{
public:
  MoveObstaclePlugin()
      : speed_(0.2),
        xmin_(-9.0),
        xmax_(-5.0),
        direction_(1),
        x_current_(-9.0),
        y_fixed_(0.0),
        z_fixed_(1.25),
        initialized_(false)
  {
  }

  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    if (!_model)
    {
      gzerr << "[MoveObstaclePlugin] model pointer is null.\n";
      return;
    }

    model_ = _model;
    world_ = model_->GetWorld();

    if (!world_)
    {
      gzerr << "[MoveObstaclePlugin] world pointer is null.\n";
      return;
    }

    if (_sdf->HasElement("speed"))
      speed_ = _sdf->Get<double>("speed");

    if (_sdf->HasElement("xmin"))
      xmin_ = _sdf->Get<double>("xmin");

    if (_sdf->HasElement("xmax"))
      xmax_ = _sdf->Get<double>("xmax");

    if (_sdf->HasElement("y_fixed"))
      y_fixed_ = _sdf->Get<double>("y_fixed");

    if (_sdf->HasElement("z_fixed"))
      z_fixed_ = _sdf->Get<double>("z_fixed");

    if (xmin_ > xmax_)
      std::swap(xmin_, xmax_);

    ignition::math::Pose3d init_pose = model_->WorldPose();
    x_current_ = init_pose.Pos().X();

    if (x_current_ < xmin_)
      x_current_ = xmin_;
    if (x_current_ > xmax_)
      x_current_ = xmax_;

    // 初始化时直接把模型摆正并放到固定高度/固定y
    ignition::math::Pose3d target_pose(
        ignition::math::Vector3d(x_current_, y_fixed_, z_fixed_),
        ignition::math::Quaterniond(0.0, 0.0, 0.0));

    model_->SetLinearVel(ignition::math::Vector3d(0.0, 0.0, 0.0));
    model_->SetAngularVel(ignition::math::Vector3d(0.0, 0.0, 0.0));
    model_->SetWorldPose(target_pose);

    last_time_ = world_->SimTime();

    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&MoveObstaclePlugin::OnUpdate, this));

    initialized_ = true;

    gzmsg << "[MoveObstaclePlugin] Loaded for model: " << model_->GetName()
          << " | speed=" << speed_
          << " | xmin=" << xmin_
          << " | xmax=" << xmax_
          << " | y_fixed=" << y_fixed_
          << " | z_fixed=" << z_fixed_ << "\n";
  }

  void OnUpdate()
  {
    if (!initialized_ || !model_ || !world_)
      return;

    common::Time current_time = world_->SimTime();
    double dt = (current_time - last_time_).Double();
    last_time_ = current_time;

    if (dt <= 0.0)
      return;

    x_current_ += direction_ * speed_ * dt;

    if (x_current_ >= xmax_)
    {
      x_current_ = xmax_;
      direction_ = -1;
    }
    else if (x_current_ <= xmin_)
    {
      x_current_ = xmin_;
      direction_ = 1;
    }

    ignition::math::Pose3d target_pose(
        ignition::math::Vector3d(x_current_, y_fixed_, z_fixed_),
        ignition::math::Quaterniond(0.0, 0.0, 0.0));

    // 先清零线速度和角速度，再强制设置位姿，避免倾倒和旋转
    model_->SetLinearVel(ignition::math::Vector3d(0.0, 0.0, 0.0));
    model_->SetAngularVel(ignition::math::Vector3d(0.0, 0.0, 0.0));
    model_->SetWorldPose(target_pose);
  }

private:
  physics::ModelPtr model_;
  physics::WorldPtr world_;
  event::ConnectionPtr update_connection_;

  double speed_;
  double xmin_;
  double xmax_;
  int direction_;

  double x_current_;
  double y_fixed_;
  double z_fixed_;

  bool initialized_;
  common::Time last_time_;
};

GZ_REGISTER_MODEL_PLUGIN(MoveObstaclePlugin)
}