//
// Created by Aoalas on 2525/10/5.
//

#include "rm_manual/dart2_manual.h"

namespace rm_manual
{
Dart2Manual::Dart2Manual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee) : ManualBase(nh, nh_referee)
{
  XmlRpc::XmlRpcValue dart_list, targets, launch_id, steps_xml, trigger_status_rpc;
  nh.getParam("launch_id", launch_id);
  nh.getParam("dart_list", dart_list);
  nh.getParam("targets", targets);
  getList(dart_list, targets, launch_id);

  yaw_outpost_ = target_position_["outpost"][0];
  range_outpost_ = target_position_["outpost"][1];
  yaw_base_ = target_position_["base"][0];
  range_base_ = target_position_["base"][1];

  ros::NodeHandle nh_yaw = ros::NodeHandle(nh, "yaw");
  yaw_sender_ = new rm_common::JointPointCommandSender(nh_yaw, joint_state_);
  ros::NodeHandle nh_trigger = ros::NodeHandle(nh, "trigger");
  trigger_sender_ = new rm_common::JointPointCommandSender(nh_trigger, joint_state_);
  ros::NodeHandle nh_load = ros::NodeHandle(nh, "load");
  load_sender_ = new rm_common::JointPointCommandSender(nh_load, joint_state_);

  nh_load.getParam("use_load", use_load_);
  nh_load.getParam("load_init_position", load_init_position_);
  nh_load.getParam("load_left_position", load_left_position_);
  nh_load.getParam("load_mid_position", load_mid_position_);
  nh_load.getParam("load_right_position", load_right_position_);

  nh_trigger.getParam("trigger_home_command", trigger_home_command_);
  nh_trigger.getParam("trigger_work_command", trigger_work_command_);
  nh_trigger.getParam("trigger_confirm_home", trigger_confirm_home_);
  nh_trigger.getParam("trigger_confirm_work", trigger_confirm_work_);
  nh_trigger.getParam("false_engage_time", false_engage_time_);

  if (nh_trigger.getParam("trigger_status", trigger_status_rpc))
  {
    ROS_ASSERT(trigger_status_rpc.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int i = 0; i < trigger_status_rpc.size(); ++i)
    {
      trigger_status_.push_back(static_cast<bool>(trigger_status_rpc[i]));
    }
  }
  else
  {
    trigger_status_ = { true, true, true, true };
  }

  ros::NodeHandle nh_clamp = ros::NodeHandle(nh, "clamp_positions");
  nh_clamp.getParam("clamp_left_release_position", clamp_left_release_position_);
  nh_clamp.getParam("clamp_mid_release_position", clamp_mid_release_position_);
  nh_clamp.getParam("clamp_right_release_position", clamp_right_release_position_);
  nh_clamp.getParam("clamp_default_position", clamp_default_position_);

  ros::NodeHandle nh_belt_left = ros::NodeHandle(nh, "belt_left");
  ros::NodeHandle nh_belt_right = ros::NodeHandle(nh, "belt_right");
  belt_left_sender_ = new rm_common::JointPointCommandSender(nh_belt_left, joint_state_);
  belt_right_sender_ = new rm_common::JointPointCommandSender(nh_belt_right, joint_state_);
  nh_belt_left.getParam("belt_left_max", belt_left_max_);
  nh_belt_right.getParam("belt_right_max", belt_right_max_);
  nh_belt_left.getParam("belt_left_slow", belt_left_slow_);
  nh_belt_right.getParam("belt_right_slow", belt_right_slow_);
  nh_belt_left.getParam("belt_left_min", belt_left_min_);
  nh_belt_right.getParam("belt_right_min", belt_right_min_);
  nh_belt_left.getParam("belt_left_downward_vel", belt_left_downward_vel_);
  nh_belt_right.getParam("belt_right_downward_vel", belt_right_downward_vel_);
  nh_belt_left.getParam("belt_left_upward_vel", belt_left_upward_vel_);
  nh_belt_right.getParam("belt_right_upward_vel", belt_right_upward_vel_);
  nh_belt_left.getParam("belt_left_upward_slow_vel", belt_left_upward_slow_vel_);
  nh_belt_right.getParam("belt_right_upward_slow_vel", belt_right_upward_slow_vel_);

  ros::NodeHandle nh_range = ros::NodeHandle(nh, "range");
  range_sender_ = new rm_common::JointPointCommandSender(nh_range, joint_state_);

  ros::NodeHandle nh_clamp_left = ros::NodeHandle(nh, "clamp_left");
  ros::NodeHandle nh_clamp_mid = ros::NodeHandle(nh, "clamp_mid");
  ros::NodeHandle nh_clamp_right = ros::NodeHandle(nh, "clamp_right");
  clamp_left_sender_ = new rm_common::JointPointCommandSender(nh_clamp_left, joint_state_);
  clamp_mid_sender_ = new rm_common::JointPointCommandSender(nh_clamp_mid, joint_state_);
  clamp_right_sender_ = new rm_common::JointPointCommandSender(nh_clamp_right, joint_state_);

  XmlRpc::XmlRpcValue shooter_rpc_value, gimbal_rpc_value, clamp_rpc_value;
  nh.getParam("shooter_calibration", shooter_rpc_value);
  shooter_calibration_ = new rm_common::CalibrationQueue(shooter_rpc_value, nh, controller_manager_);
  nh.getParam("gimbal_calibration", gimbal_rpc_value);
  gimbal_calibration_ = new rm_common::CalibrationQueue(gimbal_rpc_value, nh, controller_manager_);
  nh.getParam("clamp_calibration", clamp_rpc_value);
  clamp_calibration_ = new rm_common::CalibrationQueue(clamp_rpc_value, nh, controller_manager_);

  ros::NodeHandle nh_camera = ros::NodeHandle(nh, "camera");
  nh_camera.getParam("camera_x_offset", camera_x_offset_);
  nh_camera.getParam("camera_y_offset", camera_y_offset_);
  // nh_camera.getParam("long_camera_p_x", long_camera_p_x_);
  // nh_camera.getParam("short_camera_p_x", short_camera_p_x_);

  nh_camera.getParam("camera_fast_p_x", camera_fast_p_x_);
  nh_camera.getParam("camera_normal_p_x", camera_normal_p_x_);
  nh_camera.getParam("camera_slow_p_x", camera_slow_p_x_);
  nh_camera.getParam("camera_retarget_slow_p_x", camera_retarget_slow_p_x_);
  nh_camera.getParam("camera_fast_x_threshold", camera_fast_x_threshold_);
  nh_camera.getParam("camera_normal_x_threshold", camera_normal_x_threshold_);
  nh_camera.getParam("camera_slow_x_threshold", camera_slow_x_threshold_);
  nh_camera.getParam("retarget_threshold", retarget_threshold_);

  nh_camera.getParam("long_camera_x_threshold", long_camera_x_threshold_);
  nh_camera.getParam("use_auto_aim", use_auto_aim_);

  nh_camera.getParam("random_fixed_target", random_fixed_target_);

  left_switch_up_event_.setActiveHigh(boost::bind(&Dart2Manual::leftSwitchUpOn, this));
  left_switch_mid_event_.setActiveHigh(boost::bind(&Dart2Manual::leftSwitchMidOn, this));
  left_switch_down_event_.setActiveHigh(boost::bind(&Dart2Manual::leftSwitchDownOn, this));
  right_switch_down_event_.setActiveHigh(boost::bind(&Dart2Manual::rightSwitchDownOn, this));
  right_switch_mid_event_.setRising(boost::bind(&Dart2Manual::rightSwitchMidRise, this));
  right_switch_up_event_.setRising(boost::bind(&Dart2Manual::rightSwitchUpRise, this));
  wheel_clockwise_event_.setRising(boost::bind(&Dart2Manual::wheelClockwise, this));
  wheel_anticlockwise_event_.setRising(boost::bind(&Dart2Manual::wheelAntiClockwise, this));
  dbus_sub_ = nh.subscribe<rm_msgs::DbusData>("/rm_ecat_hw/dbus", 10, &Dart2Manual::dbusDataCallback, this);
  dart_client_cmd_sub_ = nh_referee.subscribe<rm_msgs::DartClientCmd>("dart_client_cmd_data", 10,
                                                                      &Dart2Manual::dartClientCmdCallback, this);
  game_robot_hp_sub_ =
      nh_referee.subscribe<rm_msgs::GameRobotHp>("game_robot_hp", 10, &Dart2Manual::gameRobotHpCallback, this);
  game_status_sub_ =
      nh_referee.subscribe<rm_msgs::GameStatus>("game_status", 10, &Dart2Manual::gameStatusCallback, this);

  short_camera_data_sub_ =
      nh.subscribe<rm_msgs::Dart>("/short_dart_camera_distance", 10, &Dart2Manual::shortCameraDataCallback, this);
}

void Dart2Manual::getList(const XmlRpc::XmlRpcValue& darts, const XmlRpc::XmlRpcValue& targets,
                          const XmlRpc::XmlRpcValue& launch_id)
{
  for (const auto& dart : darts)
  {
    ROS_ASSERT(dart.second.hasMember("param") and dart.second.hasMember("id"));
    ROS_ASSERT(dart.second["param"].getType() == XmlRpc::XmlRpcValue::TypeArray and
               dart.second["id"].getType() == XmlRpc::XmlRpcValue::TypeInt);
    for (int i = 0; i < 4; ++i)
    {
      if (dart.second["id"] == launch_id[i])
      {
        Dart dart_info;
        dart_info.outpost_offset_ = static_cast<double>(dart.second["param"][0]);
        dart_info.outpost_range_ = static_cast<double>(dart.second["param"][1]);
        dart_info.base_offset_ = static_cast<double>(dart.second["param"][2]);
        dart_info.base_range_ = static_cast<double>(dart.second["param"][3]);
        dart_list_.insert(std::make_pair(i, dart_info));
      }
    }
  }

  for (const auto& target : targets)
  {
    ROS_ASSERT(target.second.hasMember("position"));
    ROS_ASSERT(target.second["position"].getType() == XmlRpc::XmlRpcValue::TypeArray);
    std::vector<double> position(2);
    position[0] = static_cast<double>(target.second["position"][0]);
    position[1] = static_cast<double>(target.second["position"][1]);
    target_position_.insert(std::make_pair(target.first, position));
  }
}

void Dart2Manual::gameRobotStatusCallback(const rm_msgs::GameRobotStatus::ConstPtr& data)
{
  ManualBase::gameRobotStatusCallback(data);
  robot_id_ = data->robot_id;
}

void Dart2Manual::remoteControlTurnOn()
{
  ManualBase::remoteControlTurnOn();
  gimbal_calibration_->stopController();
  gimbal_calibration_->reset();
  shooter_calibration_->stopController();
  shooter_calibration_->reset();
  clamp_calibration_->stopController();
  clamp_calibration_->reset();
}

void Dart2Manual::checkReferee()
{
  ManualBase::checkReferee();
}

void Dart2Manual::run()
{
  ManualBase::run();
  gimbal_calibration_->update(ros::Time::now());
  shooter_calibration_->update(ros::Time::now());
  clamp_calibration_->update(ros::Time::now());
  updateLaunchMode();
  updateAutoAimState();
}

void Dart2Manual::updatePc(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
}

void Dart2Manual::updateRc(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  ManualBase::updateRc(dbus_data);
  if (std::abs(dbus_data->ch_r_x) > std::abs(dbus_data->ch_r_y))
  {
    move(yaw_sender_, dbus_data->ch_r_x);
  }
  if (std::abs(dbus_data->ch_r_y) > std::abs(dbus_data->ch_r_x))
  {
    move(range_sender_, dbus_data->ch_r_y);
  }
  operateClamper(dbus_data);
}

void Dart2Manual::move(rm_common::JointPointCommandSender* joint, double ch)
{
  if (!joint_state_.position.empty())
  {
    double position = joint_state_.position[joint->getIndex()];
    if (ch != 0.)
    {
      joint->setPoint(position - ch * scale_);
      if_stop_ = true;
    }
    if (ch == 0. && if_stop_)
    {
      joint->setPoint(joint_state_.position[joint->getIndex()]);
      if_stop_ = false;
    }
  }
}

void Dart2Manual::updateLaunchMode()
{
  if (launch_mode_ != last_launch_mode_)
  {
    switch (launch_mode_)
    {
      case INIT:
        init();
        break;
      case PULLDOWN:
        pullDown();
        break;
      case ENGAGE:
        engage();
        break;
      case PULLUP:
        pullUp();
        break;
      case READY:
        ready();
        break;
      case PUSH:
        push();
        break;
      default:
        ROS_WARN("Invalid mode.");
        break;
    }
  }
  last_launch_mode_ = launch_mode_;
}

void Dart2Manual::init()
{
  ROS_INFO("Enter INIT");
  shooter_calibration_->reset();
  belt_left_sender_->setPoint(0.0);
  belt_right_sender_->setPoint(0.0);
  trigger_sender_->setPoint(trigger_home_command_);
  load_sender_->setPoint(load_init_position_);
  clamp_left_sender_->setPoint(0.0);
  clamp_mid_sender_->setPoint(0.0);
  clamp_right_sender_->setPoint(0.0);

  if (dart_fired_num_ > 3)
  {
    dart_fired_num_ = 0;
    ROS_INFO("Dart_fired_num reset to zero!");
  }
  last_init_time_ = ros::Time::now();
  auto_aim_state_ = NONE;

  if (dart_fired_num_ == 0)
  {
    camera_x_set_point_ = 0.0;
    current_x_offset_ = 0.0;
    camera_y_set_point_ = 0.0;
    current_y_offset_ = 0.0;
    camera_x_init_ = false;
  }

  start_aim_ = false;
  aim_failed_ = false;
  auto_aim_start_ = false;

  load_start_ = false;
  load_finished_ = false;
  belt_left_ready_ = false;
  belt_right_ready_ = false;
  dart_ready_ = false;
  vision_ready_ = false;
  retarget = false;
  clamp_finished_ = false;

  push_failed = false;
  push_succeeded = false;

  if (!camera_x_init_)
  {
    camera_x_after_push_ = 0;
    camera_x_init_ = true;
  }
}

void Dart2Manual::pullDown()
{
  ROS_INFO("Enter PULLDOWN");
  trigger_sender_->setPoint(trigger_work_command_);
  belt_left_sender_->setPoint(belt_left_downward_vel_);
  belt_right_sender_->setPoint(belt_right_downward_vel_);
}

void Dart2Manual::engage()
{
  ROS_INFO("Enter ENGAGE");

  size_t current_dart_idx = static_cast<size_t>(dart_fired_num_ % 4);
  bool is_dart_active = (trigger_status_.size() > current_dart_idx) ? trigger_status_[current_dart_idx] : true;

  if (is_dart_active)
  {
    trigger_sender_->setPoint(trigger_home_command_);
  }
  else
  {
    trigger_sender_->setPoint(trigger_work_command_);
    ROS_INFO("Dart %d is disabled, using work command.", dart_fired_num_);
  }

  belt_left_sender_->setPoint(0.25);
  belt_right_sender_->setPoint(0.25);
  last_engage_time_ = ros::Time::now();

  if (camera_x_init_ && dart_fired_num_ != 0)
  {
    camera_x_after_push_ = camera_x_;
  }
}

void Dart2Manual::pullUp()
{
  belt_left_sender_->setPoint(belt_right_upward_vel_);
  belt_right_sender_->setPoint(belt_left_upward_vel_);
}

void Dart2Manual::ready()
{
  ROS_INFO("Enter READY");
  belt_right_sender_->setPoint(0.0);
  belt_left_sender_->setPoint(0.0);
  trigger_sender_->setPoint(trigger_home_command_);
  last_ready_time_ = ros::Time::now();
  launch_result_ = false;
}

void Dart2Manual::push()
{
  ROS_INFO("Enter PUSH");
  trigger_sender_->setPoint(trigger_work_command_);
  dart_fired_num_++;
  ROS_INFO("Launch dart num:%d", dart_fired_num_);
  last_push_time_ = ros::Time::now();
  load_finished_ = false;
  if (triggerIsWorked())
  {
    launch_result_ = true;
    ROS_INFO_THROTTLE(2.0, "Launch Dart successed.");
  }
}

void Dart2Manual::readyLaunchDart(int dart_fired_num)
{
  switch (launch_mode_)
  {
    case INIT:
      if (shooter_calibration_->isCalibrated() && ros::Time::now() - last_init_time_ > ros::Duration(0.2) &&
          last_init_time_ > last_push_time_)
      {
        launch_mode_ = PULLDOWN;
        last_init_time_ = ros::Time::now();
      }
      break;

    case PULLDOWN:
      if (belt_left_position_ >= belt_left_max_ && belt_right_position_ >= belt_right_max_ &&
          std::abs(load_velocity_) < 0.01)
      {
        launch_mode_ = ENGAGE;
        last_engage_time_ = ros::Time::now();
      }
      break;

    case ENGAGE:
    {
      size_t current_dart_idx = static_cast<size_t>(dart_fired_num_ % 4);
      bool is_dart_active = (trigger_status_.size() > current_dart_idx) ? trigger_status_[current_dart_idx] : true;

      if (is_dart_active)
      {
        if (triggerIsHome() && ros::Time::now() - last_engage_time_ > ros::Duration(1.0))
        {
          launch_mode_ = PULLUP;
          load_start_ = true;
          auto_aim_start_ = true;
        }
      }
      else
      {
        if (ros::Time::now() - last_engage_time_ > ros::Duration(false_engage_time_))
        {
          launch_mode_ = PULLUP;
          load_start_ = true;
          auto_aim_start_ = true;
        }
      }
      break;
    }

    case PULLUP:
      load_work();
      if (belt_left_position_ <= belt_left_slow_ && belt_left_position_ > belt_right_min_)
      {
        belt_left_sender_->setPoint(belt_left_upward_slow_vel_);
      }
      if (belt_right_position_ <= belt_right_slow_ && belt_right_position_ > belt_right_min_)
      {
        belt_right_sender_->setPoint(belt_right_upward_slow_vel_);
      }

      if (belt_left_position_ <= belt_left_min_)
      {
        belt_left_sender_->setPoint(0.0);
        belt_left_ready_ = true;
      }
      if (belt_right_position_ <= belt_right_min_)
      {
        belt_right_sender_->setPoint(0.0);
        belt_right_ready_ = true;
      }

      if (belt_left_ready_ && belt_right_ready_ && dart_ready_ && vision_ready_)
      {
        launch_mode_ = READY;
        last_ready_time_ = ros::Time::now();
      }
      break;

    default:
      break;
  }
}

void Dart2Manual::leftSwitchDownOn()
{
  launch_mode_ = INIT;
}

void Dart2Manual::leftSwitchMidOn()
{
  switch (manual_state_)
  {
    case OUTPOST:
      yaw_sender_->setPoint(yaw_outpost_ + camera_x_set_point_);
      range_sender_->setPoint(range_outpost_ + camera_y_set_point_);
      break;
    case BASE:
      yaw_sender_->setPoint(yaw_base_ + camera_x_set_point_);
      range_sender_->setPoint(range_base_ + camera_y_set_point_);
      break;
  }
  readyLaunchDart(dart_fired_num_);
  if (auto_aim_start_)
  {
    autoAim();
  }
}

void Dart2Manual::leftSwitchUpOn()
{
  // if (launch_mode_ == READY) {
  if (launch_mode_ == READY && auto_aim_state_ == ADJUSTED && ros::Time::now() - vision_end_time_ > ros::Duration(0.5))
  {
    last_push_time_ = ros::Time::now();
    launch_mode_ = PUSH;
  }

  // if (launch_mode_ == PUSH && ros::Time::now() - last_push_time_ > ros::Duration(0.5) && !triggerIsWorked() && !push_failed
  // && !push_succeeded) {
  //   trigger_sender_->setPoint(trigger_home_command_);
  //   push_failed_time = ros::Time::now();
  //   push_failed = true;
  // }
  // if (ros::Time::now() - push_failed_time > ros::Duration(2.5) && push_failed && !triggerIsWorked() && !push_succeeded){
  //   trigger_sender_->setPoint(trigger_work_command_);
  //   last_push_time_ = ros::Time::now();
  //   push_failed = false;
  // }
  //
  // if (triggerIsWorked()) {
  //   push_succeeded = true;
  // }
}

void Dart2Manual::rightSwitchDownOn()
{
  recordPosition(dbus_data_);
  if (dbus_data_.ch_l_y == 1.)
  {
    if (manual_state_ == OUTPOST)
    {
      yaw_sender_->setPoint(yaw_outpost_);
      range_sender_->setPoint(range_outpost_);
    }
    else if (manual_state_ == BASE)
    {
      yaw_sender_->setPoint(yaw_base_);
      range_sender_->setPoint(range_base_);
    }
  }
  if (dbus_data_.ch_l_y == -1.)
  {
    if (manual_state_ == OUTPOST)
    {
      yaw_sender_->setPoint(target_position_["outpost"][0]);
      range_sender_->setPoint(target_position_["outpost"][1]);
    }
    else if (manual_state_ == BASE)
    {
      yaw_sender_->setPoint(target_position_["base"][0]);
      range_sender_->setPoint(target_position_["base"][1]);
    }
  }
}

void Dart2Manual::rightSwitchMidRise()
{
  ManualBase::rightSwitchMidRise();
  allow_dart_door_open_times_ = 0;
  move_state_ = NORMAL;
}

void Dart2Manual::rightSwitchUpRise()
{
  ManualBase::rightSwitchUpRise();
}

bool Dart2Manual::triggerIsHome() const
{
  return trigger_position_ <= trigger_confirm_home_;
}

bool Dart2Manual::triggerIsWorked() const
{
  return trigger_position_ >= trigger_confirm_work_;
}

void Dart2Manual::recordPosition(const rm_msgs::DbusData dbus_data)
{
  if (dbus_data.ch_r_y == 1.)
  {
    if (manual_state_ == OUTPOST)
    {
      yaw_outpost_ = joint_state_.position[yaw_sender_->getIndex()];
      range_outpost_ = joint_state_.position[range_sender_->getIndex()];
      ROS_INFO("Recorded outpost position.");
    }
    else if (manual_state_ == BASE)
    {
      yaw_base_ = joint_state_.position[yaw_sender_->getIndex()];
      range_base_ = joint_state_.position[range_sender_->getIndex()];
      ROS_INFO("Recorded base position.");
    }
  }
}

void Dart2Manual::sendCommand(const ros::Time& time)
{
  belt_left_sender_->sendCommand(time);
  belt_right_sender_->sendCommand(time);
  range_sender_->sendCommand(time);
  trigger_sender_->sendCommand(time);
  yaw_sender_->sendCommand(time);
  load_sender_->sendCommand(time);
  clamp_left_sender_->sendCommand(time);
  clamp_mid_sender_->sendCommand(time);
  clamp_right_sender_->sendCommand(time);
}

void Dart2Manual::dbusDataCallback(const rm_msgs::DbusData::ConstPtr& data)
{
  ManualBase::dbusDataCallback(data);
  if (!joint_state_.name.empty())
  {
    belt_left_position_ = std::abs(joint_state_.position[belt_left_sender_->getIndex()]);
    belt_right_position_ = std::abs(joint_state_.position[belt_right_sender_->getIndex()]);
    trigger_position_ = joint_state_.position[trigger_sender_->getIndex()];
    yaw_velocity_ = std::abs(joint_state_.velocity[yaw_sender_->getIndex()]);
    range_velocity_ = std::abs(joint_state_.velocity[range_sender_->getIndex()]);
    load_velocity_ = std::abs(joint_state_.velocity[load_sender_->getIndex()]);
  }
  wheel_clockwise_event_.update(data->wheel == 1.0);
  wheel_anticlockwise_event_.update(data->wheel == -1.0);
  dbus_data_ = *data;
}

void Dart2Manual::gameStatusCallback(const rm_msgs::GameStatus::ConstPtr& data)
{
  ManualBase::gameStatusCallback(data);
  game_progress_ = data->game_progress;
  if (game_progress_ == rm_msgs::GameStatus::IN_BATTLE)
    remain_time_ = data->stage_remain_time;
  else
    remain_time_ = 420;
}

void Dart2Manual::dartClientCmdCallback(const rm_msgs::DartClientCmd::ConstPtr& data)
{
  dart_launch_opening_status_ = data->dart_launch_opening_status;
}

void Dart2Manual::gameRobotHpCallback(const rm_msgs::GameRobotHp::ConstPtr& data)
{
}

void Dart2Manual::wheelClockwise()
{
  switch (clamp_num_)
  {
    case 0:
      clamp_num_ = 1;
      temp_load_position_ = load_init_position_;
      break;
    case 1:
      clamp_num_ = 2;
      temp_load_position_ = load_left_position_;
      break;
    case 2:
      clamp_num_ = 3;
      temp_load_position_ = load_mid_position_;
      break;
    case 3:
      temp_load_position_ = load_right_position_;
      clamp_num_ = 0;
      break;
  }
  ROS_INFO("Now temp_load_position_: %lf", temp_load_position_);

  switch (move_state_)
  {
    case NORMAL:
      scale_ = scale_micro_;
      move_state_ = MICRO;
      ROS_INFO("Pitch and yaw : MICRO_MOVE_MODE");
      temp_clamp_ = true;
      break;
    case MICRO:
      scale_ = 0.01;
      move_state_ = NORMAL;
      ROS_INFO("Pitch and yaw : NORMAL_MOVE_MODE");
      temp_clamp_ = false;
      break;
  }

  // dart_fired_num_ = dart_fired_num_ + 1;
  //
  // ROS_INFO("Changed launch dart num:%d", dart_fired_num_);
}

void Dart2Manual::wheelAntiClockwise()
{
  // if (temp_load_)
  // {
  //   load_sender_->setPoint(0.04);
  //   temp_load_ = false;
  // }
  // else {
  //   load_sender_->setPoint(0.04);
  //   temp_load_ = true;
  // }

  load_sender_->setPoint(temp_load_position_);

  // if (dart_fired_num_ != 0) {
  // dart_fired_num_ = dart_fired_num_ - 1;
  // }

  // ROS_INFO("Changed launch dart num:%d", dart_fired_num_);
}

/*
void Dart2Manual::longCameraDataCallback(const rm_msgs::Dart::ConstPtr& data)
{
  is_long_camera_found_ = data->is_found;
  long_camera_x_ = data->distance;
  long_camera_y_ = data->height;
  last_get_camera_data_time_ = data->stamp;
}
*/

void Dart2Manual::shortCameraDataCallback(const rm_msgs::Dart::ConstPtr& data)
{
  is_camera_found_ = data->is_found;
  camera_x_ = data->distance;
  camera_y_ = data->height;
  last_get_camera_data_time_ = data->stamp;
}

void Dart2Manual::updateAutoAimState()
{
  switch (auto_aim_state_)
  {
    case AIM:
      aim();
      break;
    case ADJUST:
      adjust();
      break;
  }
  last_auto_aim_state_ = auto_aim_state_;
}

void Dart2Manual::aim()
{
  if (last_auto_aim_state_ != auto_aim_state_)
  {
    ROS_INFO("ENTER AIM.");
    start_aim_ = true;
    start_aim_time_ = ros::Time::now();
  }
  if (is_camera_found_ && std::abs(camera_x_) >= camera_fast_x_threshold_)
  {
    camera_x_set_point_ += camera_x_ * camera_fast_p_x_;
  }
  if (is_camera_found_ && std::abs(camera_x_) < camera_fast_x_threshold_ &&
      std::abs(camera_x_) >= camera_normal_x_threshold_)
  {
    camera_x_set_point_ += camera_x_ * camera_normal_p_x_;
  }
  if (is_camera_found_ && std::abs(camera_x_) < camera_normal_x_threshold_ &&
      std::abs(camera_x_) >= camera_slow_x_threshold_)
  {
    if (retarget)
    {
      camera_x_set_point_ += camera_x_ * camera_retarget_slow_p_x_;
    }
    else
    {
      camera_x_set_point_ += camera_x_ * camera_slow_p_x_;
    }
  }

  if (auto_aim_state_ == AIM && std::abs(camera_x_) <= camera_slow_x_threshold_ && is_camera_found_ &&
      yaw_velocity_ < 0.01)
  {
    ROS_INFO("The %d dart had aimed.Now x error:%lf, yaw position: %lf ", dart_fired_num_, camera_x_,
             joint_state_.position[yaw_sender_->getIndex()]);
    ROS_INFO("aim time used: %lf s", (ros::Time::now() - start_aim_time_).toSec());
    start_aim_ = false;
    auto_aim_state_ = AIMED;
  }
}

void Dart2Manual::adjust()
{
  if (last_auto_aim_state_ != auto_aim_state_)
  {
    ROS_INFO("ENTER ADJUST.");
    last_adjust_time_ = ros::Time::now();
  }

  if (!had_adjust_)
  {
    target_x_offset = dart_list_[dart_fired_num_].base_offset_;
    target_y_offset_ = dart_list_[dart_fired_num_].base_range_;
    ROS_INFO("Adjusting Offset. Removing old: %lf, Adding new: %lf", current_x_offset_, target_x_offset);

    camera_x_set_point_ -= current_x_offset_;
    camera_x_set_point_ += target_x_offset;
    current_x_offset_ = target_x_offset;

    camera_y_set_point_ -= current_y_offset_;
    camera_y_set_point_ += target_y_offset_;
    current_y_offset_ = target_y_offset_;

    had_adjust_ = true;
  }

  if (yaw_velocity_ < 0.001 && ros::Time::now() - last_adjust_time_ > ros::Duration(0.1) && range_velocity_ < 0.001)
  {
    ROS_INFO("The %d dart had adjusted.Now x error:%lf, yaw position: %lf ,range position: %lf", dart_fired_num_,
             camera_x_, joint_state_.position[yaw_sender_->getIndex()],
             joint_state_.position[range_sender_->getIndex()]);
    camera_x_before_push_ = camera_x_;
    had_adjust_ = false;
    auto_aim_start_ = false;
    auto_aim_state_ = ADJUSTED;
    vision_ready_ = true;
    vision_end_time_ = ros::Time::now();
  }
}

void Dart2Manual::autoAim()
{
  camera_is_online_ = (ros::Time::now() - last_get_camera_data_time_ < ros::Duration(1.0));

  if (use_auto_aim_ && camera_is_online_)
  {
    if (start_aim_ && ros::Time::now() - start_aim_time_ > ros::Duration(5.0))
    {
      start_aim_ = false;
      ROS_INFO("Auto aim time out!!!");
      aim_failed_ = true;
      camera_x_set_point_ = 0;
      current_x_offset_ = 0.0;
      camera_y_set_point_ = 0;
      current_y_offset_ = 0.0;

      auto_aim_state_ = ADJUST;
    }

    if (!aim_failed_)
    {
      bool force_aim = (dart_fired_num_ == 0) || random_fixed_target_;
      if (random_fixed_target_)
      {
        ROS_INFO_THROTTLE(5.0, "Random fixed target is enabled, auto aim will be forced for the first dart.");
      }
      if (force_aim)
      {
        if (auto_aim_state_ != ADJUSTED && auto_aim_state_ != ADJUST && auto_aim_state_ != AIMED &&
            auto_aim_state_ != AIM)
        {
          current_x_offset_ = 0.0;
          current_y_offset_ = 0.0;
          auto_aim_state_ = AIM;
        }
      }
      else
      {
        if (auto_aim_state_ != ADJUSTED && auto_aim_state_ != ADJUST && auto_aim_state_ != AIMED &&
            auto_aim_state_ != AIM)
        {
          if (std::abs(camera_x_after_push_ - camera_x_before_push_) < retarget_threshold_)
          {
            auto_aim_state_ = ADJUST;
          }
          else
          {
            retarget = true;
            current_x_offset_ = 0.0;
            current_y_offset_ = 0.0;
            auto_aim_state_ = AIM;
          }
        }
      }
    }

    if (auto_aim_state_ == AIMED)
    {
      auto_aim_state_ = ADJUST;
    }
  }
  else if (auto_aim_state_ != ADJUST && auto_aim_state_ != ADJUSTED)
  {
    ROS_INFO("use_auto_aim False or Camera offline, aim disabled.");
    camera_x_set_point_ = 0;
    current_x_offset_ = 0.0;
    camera_y_set_point_ = 0;
    current_y_offset_ = 0.0;

    auto_aim_state_ = ADJUST;
  }
}

void Dart2Manual::load_dart(int step_idx)
{
  switch (step_idx)
  {
    case 0:
      current_load_target_ = load_init_position_;
      break;
    case 1:
      current_load_target_ = load_left_position_;
      break;
    case 2:
      clamp_left_sender_->setPoint(clamp_left_release_position_);
      current_load_target_ = load_mid_position_;
      break;
    case 3:
      clamp_left_sender_->setPoint(clamp_left_release_position_);
      clamp_mid_sender_->setPoint(clamp_mid_release_position_);
      current_load_target_ = load_right_position_;
      break;
    default:
      return;
  }

  load_sender_->setPoint(current_load_target_);
  ROS_INFO_THROTTLE(1.0, "Load Executing Step %d: Target Pos %.3f", step_idx, current_load_target_);
}

void Dart2Manual::load_work()
{
  if (use_load_)
  {
    cur_pos = joint_state_.position[load_sender_->getIndex()];

    if (!clamp_finished_ && !joint_state_.position.empty())
    {
      load_dart(dart_fired_num_ % 4);
    }

    if (dart_fired_num_ % 4 == 0 && std::abs(load_velocity_) < 0.001 && !joint_state_.position.empty())
    {
      load_start_ = false;
      clamp_finished_ = true;
      load_finished_ = true;
      dart_ready_ = true;
      load_sender_->setPoint(load_init_position_);
    }

    if (!joint_state_.position.empty())
    {
      if (std::abs(cur_pos - current_load_target_) < 0.03 && std::abs(load_velocity_) < 0.001 && !load_finished_)
      {
        load_finish_time_ = ros::Time::now();
        load_finished_ = true;
        load_start_ = false;
      }
    }
    else
    {
      ROS_WARN_THROTTLE(1.0, "joint_state_.position IS EMPTY!!");
      load_start_ = false;
      load_finished_ = true;
      clamp_finished_ = true;
      dart_ready_ = true;
    }

    if (!clamp_finished_)
    {
      last_clamp_finished_time_ = ros::Time::now();
    }

    if (!clamp_finished_ && load_finished_ && ros::Time::now() - load_finish_time_ > ros::Duration(1.0))
    {
      switch (dart_fired_num_ % 4)
      {
        case 1:
          clamp_left_sender_->setPoint(clamp_left_release_position_);
          break;
        case 2:
          clamp_mid_sender_->setPoint(clamp_mid_release_position_);
          break;
        case 3:
          clamp_right_sender_->setPoint(clamp_right_release_position_);
          break;
        case 0:
          break;
      }
      clamp_finished_ = true;
    }

    if (clamp_finished_ && load_finished_ && (ros::Time::now() - last_clamp_finished_time_) > ros::Duration(1.2))
    {
      load_sender_->setPoint(load_init_position_);
      if (std::abs(cur_pos - load_init_position_) < 0.03)
      {
        dart_ready_ = true;
      }
    }
  }
  else
  {
    ROS_INFO_THROTTLE(2.0, "use_load is false, skip load process.");
    dart_ready_ = true;
  }
}

void Dart2Manual::operateClamper(const rm_msgs::DbusData::ConstPtr& dbus_data)
{
  if (dbus_data->ch_l_y > 0.75)
  {
    clamp_left_sender_->setPoint(0.0);
    clamp_mid_sender_->setPoint(0.0);
    clamp_right_sender_->setPoint(0.0);
    return;
  }
  if (dbus_data->ch_l_x > 0.75)
  {
    clamp_right_sender_->setPoint(clamp_right_release_position_);
    return;
  }
  if (dbus_data->ch_l_y < -0.75)
  {
    clamp_mid_sender_->setPoint(clamp_mid_release_position_);
    return;
  }
  if (dbus_data->ch_l_x < -0.75)
  {
    clamp_left_sender_->setPoint(clamp_left_release_position_);
    return;
  }
}

}  // namespace rm_manual
