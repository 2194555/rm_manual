//
// Created by cilotta on 25-7-16.
//

#include "rm_manual/drone_manual.h"

namespace rm_manual
{
DroneManual::DroneManual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee) : ChassisGimbalShooterManual(nh, nh_referee)
{
  ros::NodeHandle detection_switch_left_nh(nh, "detection_switch_left");
  switch_detection_left_srv_ = new rm_common::SwitchDetectionCaller(detection_switch_left_nh);
  e_event_.setActive(boost::bind(&DroneManual::ePressing, this), boost::bind(&DroneManual::eReleasing, this));
  q_event_.setActive(boost::bind(&DroneManual::qPressing, this), boost::bind(&DroneManual::qReleasing, this));
  aim_priority_pub_ = nh.advertise<rm_msgs::PriorityArray>("/armor_processor_right/priority/priority_arr", 1);
}

void DroneManual::ePress()
{
  ePressing();
}
void DroneManual::eRelease()
{
  eReleasing();
}

void DroneManual::ePressing()
{
  // switch_armor_target_srv_->setArmorTargetType(rm_msgs::StatusChangeRequest::ARMOR_OUTPOST_BASE);
  // switch_armor_target_srv_->callService();
  // shooter_cmd_sender_->setArmorType(switch_armor_target_srv_->getArmorTarget());
  camera_switch_cmd_sender_->switchCameraLeft();
};

void DroneManual::eReleasing()
{
  // switch_armor_target_srv_->setArmorTargetType(rm_msgs::StatusChangeRequest::ARMOR_ALL);
  // switch_armor_target_srv_->callService();
  // shooter_cmd_sender_->setArmorType(switch_armor_target_srv_->getArmorTarget());
  camera_switch_cmd_sender_->switchCameraRight();
}

void DroneManual::qPressing()
{
  aim_priority_array_.rank_arr = { 0, 0, 0, 0, 0, 0, 0, 1 };
  pubAimPriority();
}

void DroneManual::qReleasing()
{
  aim_priority_array_.rank_arr = { 1, 1, 1, 1, 1, 1, 1, 1 };
  pubAimPriority();
}

void DroneManual::pubAimPriority()
{
  aim_priority_pub_.publish(aim_priority_array_);
}

void DroneManual::rPress()
{
  if (camera_switch_cmd_sender_)
    camera_switch_cmd_sender_->switchCameraLeft();
  ChassisGimbalShooterManual::rPress();
};

void DroneManual::ctrlBPress()
{
  ChassisGimbalShooterManual::ctrlBPress();
  switch_detection_left_srv_->switchEnemyColor();
  switch_detection_left_srv_->callService();
}

void DroneManual::gPress()
{
  ChassisGimbalShooterManual::gPress();
}

void DroneManual::vPress()
{
  ChassisGimbalShooterManual::vPress();
}

void DroneManual::remoteControlTurnOn()
{
  ChassisGimbalShooterManual::remoteControlTurnOn();
  std::string robot_color = robot_id_ >= 100 ? "blue" : "red";
  switch_detection_left_srv_->setEnemyColor(robot_id_, robot_color);
  aim_priority_array_.rank_arr = { 1, 1, 1, 1, 1, 1, 1, 1 };
}

}  // namespace rm_manual
