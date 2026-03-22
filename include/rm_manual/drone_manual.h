//
// Created by cilotta on 25-7-16.
//
#pragma once

#include "rm_manual/chassis_gimbal_shooter_manual.h"
#include "rm_msgs/PriorityArray.h"

namespace rm_manual
{
class DroneManual : public ChassisGimbalShooterManual
{
public:
  DroneManual(ros::NodeHandle& nh, ros::NodeHandle& nh_referee);

protected:
  void ePress() override;
  void eRelease() override;
  void rPress() override;
  void gPress() override;
  void vPress() override;
  void remoteControlTurnOn() override;

  void ePressing();
  void eReleasing();
  void qPressing();
  void qReleasing();

  void ctrlBPress() override;
  void pubAimPriority();

  rm_common::SwitchDetectionCaller* switch_detection_left_srv_{};
  ros::Publisher aim_priority_pub_;
  rm_msgs::PriorityArray aim_priority_array_;
};
}  // namespace rm_manual
