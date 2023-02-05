// Copyright (c) 2021 Ichiro ITS
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <stdio.h>
#include <eigen3/Eigen/Dense>  // use <Eigen/Dense> if this doesn't work, make sure to install Eigen3
#include <iostream>

#ifndef SUIRYOKU__NODE__FORWARD_KINEMATIC_HPP_
#define SUIRYOKU__NODE__FORWARD_KINEMATIC_HPP_

#define HEAD_PAN_TRANSLATION 0.0, 0.0, 0.0
#define HEAD_TILT_TRANSLATION 0.0, 50.7, 0.0
#define CAMERA_TRANSLATION 0.0, 38.465, 28.96
#define RIGHT_SHOULDER_PITCH_TRANSLATION 0.0, 0.0, 0.0
#define RIGHT_SHOULDER_ROLL_TRANSLATION -45.303, -14.75, 0.0
#define RIGHT_ELBOW_TRANSLATION 0.0, -99.291, 0.0
#define RIGHT_HAND_TRANSLATION 0.0, -107.163, 0.0
#define RIGHT_GRIPPER_TRANSLATION 0.0, 0.0, 0.0
#define LEFT_SHOULDER_PITCH_TRANSLATION 0.0, 0.0, 0.0
#define LEFT_SHOULDER_ROLL_TRANSLATION 45.303, -14.75, 0.0
#define LEFT_ELBOW_TRANSLATION 0.0, -99.291, 0.0
#define LEFT_HAND_TRANSLATION 0.0, -107.163, 0.0
#define LEFT_GRIPPER_TRANSLATION 0.0, 0.0, 0.0
#define RIGHT_HIP_YAW_TRANSLATION 0.0, 0.0, 0.0
#define RIGHT_HIP_ROLL_TRANSLATION 0.0, 0.0, 0.0
#define RIGHT_HIP_PITCH_TRANSLATION 0.0, 0.0, 0.0
#define RIGHT_KNEE_TRANSLATION 0.0, -134.0, 0.0
#define RIGHT_ANKLE_PITCH_TRANSLATION 0.0, -111.702, 0.0
#define RIGHT_ANKLE_ROLL_TRANSLATION 0.0, 0.0, 0.0
#define RIGHT_FOOT_TRANSLATION 0.0, -34.441, 0.0
#define LEFT_HIP_YAW_TRANSLATION 0.0, 0.0, 0.0
#define LEFT_HIP_ROLL_TRANSLATION 0.0, 0.0, 0.0
#define LEFT_HIP_PITCH_TRANSLATION 0.0, 0.0, 0.0
#define LEFT_KNEE_TRANSLATION 0.0, -134.0, 0.0
#define LEFT_ANKLE_PITCH_TRANSLATION 0.0, -111.702, 0.0
#define LEFT_ANKLE_ROLL_TRANSLATION 0.0, 0.0, 0.0
#define LEFT_FOOT_TRANSLATION 0.0, -34.441, 0.0

typedef Eigen::Matrix<double, 4, 4> Matrix4D;
typedef Eigen::Matrix<double, 4, 1> Vector4D;
typedef Eigen::Matrix<double, 3, 1> Vector3D;

class ForwardKinematic
{
private:
  int * arr;
  int size;
  class CoordinateFrame
  {
public:
    double theta;
    Vector3D translation;
    Matrix4D translation_matrix, rotation_matrix, transformation_matrix;
    int i;
    void convertDegreeToRadians();
    void calculateTransformationMatrix(int i);
  };

  class Head
  {
public:
    CoordinateFrame head_pan, head_tilt, camera;

    Matrix4D getHeadForwardKinematic(int *, int);
  };
  class RightArm
  {
public:
    CoordinateFrame right_shoulder_pitch, right_shoulder_roll, right_elbow, right_gripper,
      right_hand;
    Matrix4D getRightArmForwardKinematic(int *, int);
  };
  class LeftArm
  {
public:
    CoordinateFrame left_shoulder_pitch, left_shoulder_roll, left_elbow, left_gripper, left_hand;
    Matrix4D getLeftArmForwardKinematic(int *, int);
  };
  class RightLeg
  {
public:
    CoordinateFrame right_hip_yaw, right_hip_roll, right_hip_pitch, right_knee,
      right_ankle_pitch, right_ankle_roll, right_foot;
    Matrix4D getRightLegForwardKinematic(int *, int);
  };
  class LeftLeg
  {
public:
    CoordinateFrame left_hip_yaw, left_hip_roll, left_hip_pitch, left_knee,
      left_ankle_pitch, left_ankle_roll, left_foot;
    Matrix4D getLeftLegForwardKinematic(int *, int);
  };

private:
  ForwardKinematic::Head head;
  ForwardKinematic::RightArm rightarm;
  ForwardKinematic::LeftArm leftarm;
  ForwardKinematic::RightLeg rightleg;
  ForwardKinematic::LeftLeg leftleg;

public:
  class ForwardKinematicsNode : public rclcpp::Node
  {
public:
    ForwardKinematicsNode(int * arr, int size);

private:
    void timer_callback();
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    int * arr_;
    int size_;
  };

  int i;
  ForwardKinematic(int * arr, int size)
  {
    this->arr = arr;
    this->size = size;
  }

  Vector3D getCameraPosition();
  Vector3D getRightHandPosition();
  Vector3D getLeftHandPosition();
  Vector3D getRightLegPosition();
  Vector3D getLeftLegPosition();

  Vector3D getRightHandPositionFromCamera();
  Vector3D getLeftHandPositionFromCamera();
  Vector3D getRightLegPositionFromCamera();
  Vector3D getLeftLegPositionFromCamera();
};
#endif  // SUIRYOKU__NODE__FORWARD_KINEMATIC_HPP_
