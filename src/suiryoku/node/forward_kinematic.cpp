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

#include <std_msgs/msg/string.hpp>
#include <vector>
#include <memory>
#include <string>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"
#include "suiryoku/node/forward_kinematic.hpp"
#include "tachimawari/control/control.hpp"
#include "tachimawari/joint/joint.hpp"
#include "tachimawari/joint/protocol_1/mx28_address.hpp"

void ForwardKinematic::CoordinateFrame::convertDegreeToRadians()
{
  theta = theta * (M_PI / 180);
}

void ForwardKinematic::CoordinateFrame::calculateTransformationMatrix(int i)
{
  translation_matrix << 1.0, 0.0, 0.0, translation(0, 0),
    0.0, 1.0, 0.0, translation(1, 0),
    0.0, 0.0, 1.0, translation(2, 0),
    0.0, 0.0, 0.0, 1.0;
  switch (i) {
    case (0):
      rotation_matrix << 1.0, 0.0, 0.0, 0.0,
        0.0, cos(theta), -sin(theta), 0.0,
        0.0, sin(theta), cos(theta), 0.0,
        0.0, 0.0, 0.0, 1.0;
      break;
    case (1):
      rotation_matrix << cos(theta), 0.0, sin(theta), 0.0,
        0.0, 1.0, 0.0, 0.0,
        -sin(theta), 0.0, cos(theta), 0.0,
        0.0, 0.0, 0.0, 1.0;
      break;
    case (2):
      rotation_matrix << cos(theta), -sin(theta), 0.0, 0.0,
        sin(theta), cos(theta), 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0;
      break;
    default:
      rotation_matrix << 1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0;
      break;
  }
  transformation_matrix = translation_matrix * rotation_matrix;
}

Matrix4D ForwardKinematic::Head::getHeadForwardKinematic(int * arr, int size)
{
  Matrix4D result;
  ForwardKinematic angle(arr, size);
  head_pan.theta = (angle.arr[19] - 2048.0) * (360.0 / 4096.0);
  head_tilt.theta = -1.0 * (angle.arr[18] - 2048.0) * (360.0 / 4096.0);

  head_pan.translation << HEAD_PAN_TRANSLATION;
  head_tilt.translation << HEAD_TILT_TRANSLATION;
  camera.translation << CAMERA_TRANSLATION;

  head_pan.convertDegreeToRadians();
  head_tilt.convertDegreeToRadians();
  head_pan.calculateTransformationMatrix(0);
  head_tilt.calculateTransformationMatrix(1);
  camera.calculateTransformationMatrix(4);
  result = head_pan.transformation_matrix * head_tilt.transformation_matrix *
    camera.transformation_matrix;
  return result;
}

Matrix4D ForwardKinematic::RightArm::getRightArmForwardKinematic(int * arr, int size)
{
  Matrix4D result;
  ForwardKinematic angle(arr, size);
  right_shoulder_roll.theta = -1.0 * (angle.arr[2] - 2048.0) * (360.0 / 4096.0);
  right_shoulder_pitch.theta = -1.0 * (angle.arr[0] - 2048.0) * (360.0 / 4096.0);
  right_elbow.theta = -1.0 * (angle.arr[4] - 2048.0) * (360.0 / 4096.0);
  right_gripper.theta = 0;

  right_shoulder_roll.translation << RIGHT_SHOULDER_ROLL_TRANSLATION;
  right_shoulder_pitch.translation << RIGHT_SHOULDER_PITCH_TRANSLATION;
  right_elbow.translation << RIGHT_ELBOW_TRANSLATION;
  right_gripper.translation << RIGHT_ELBOW_TRANSLATION;
  right_hand.translation << RIGHT_HAND_TRANSLATION;

  right_shoulder_roll.convertDegreeToRadians();
  right_shoulder_pitch.convertDegreeToRadians();
  right_elbow.convertDegreeToRadians();
  right_gripper.convertDegreeToRadians();

  right_shoulder_roll.calculateTransformationMatrix(2);
  right_shoulder_pitch.calculateTransformationMatrix(0);
  right_elbow.calculateTransformationMatrix(0);
  right_gripper.calculateTransformationMatrix(4);
  right_hand.calculateTransformationMatrix(4);

  result = right_shoulder_roll.transformation_matrix * right_shoulder_pitch.transformation_matrix *
    right_elbow.transformation_matrix * right_gripper.transformation_matrix *
    right_hand.transformation_matrix;
  return result;
}

Matrix4D ForwardKinematic::LeftArm::getLeftArmForwardKinematic(int * arr, int size)
{
  Matrix4D result;
  ForwardKinematic angle(arr, size);
  left_shoulder_roll.theta = -1.0 * (angle.arr[3] - 2048.0) * (360.0 / 4096.0);
  left_shoulder_pitch.theta = (angle.arr[1] - 2048.0) * (360.0 / 4096.0);
  left_elbow.theta = (angle.arr[5] - 2048.0) * (360.0 / 4096.0);
  left_gripper.theta = 0;

  left_shoulder_roll.translation << LEFT_SHOULDER_ROLL_TRANSLATION;
  left_shoulder_pitch.translation << LEFT_SHOULDER_PITCH_TRANSLATION;
  left_elbow.translation << LEFT_ELBOW_TRANSLATION;
  left_gripper.translation << LEFT_GRIPPER_TRANSLATION;
  left_hand.translation << LEFT_HAND_TRANSLATION;

  left_shoulder_roll.convertDegreeToRadians();
  left_shoulder_pitch.convertDegreeToRadians();
  left_elbow.convertDegreeToRadians();
  left_gripper.convertDegreeToRadians();

  left_shoulder_roll.calculateTransformationMatrix(2);
  left_shoulder_pitch.calculateTransformationMatrix(0);
  left_elbow.calculateTransformationMatrix(0);
  left_gripper.calculateTransformationMatrix(4);
  left_hand.calculateTransformationMatrix(4);

  result = left_shoulder_roll.transformation_matrix * left_shoulder_pitch.transformation_matrix *
    left_elbow.transformation_matrix * left_gripper.transformation_matrix *
    left_hand.transformation_matrix;
  return result;
}

Matrix4D ForwardKinematic::RightLeg::getRightLegForwardKinematic(int * arr, int size)
{
  Matrix4D result;
  ForwardKinematic angle(arr, size);
  right_hip_yaw.theta = -1.0 * (angle.arr[6] - 2048.0) * (360.0 / 4096.0);
  right_hip_roll.theta = -1.0 * (angle.arr[8] - 2048.0) * (360.0 / 4096.0);
  right_hip_pitch.theta = (angle.arr[10] - 2048.0) * (360.0 / 4096.0);
  right_knee.theta = (angle.arr[12] - 2048.0) * (360.0 / 4096.0);
  right_ankle_pitch.theta = -1.0 * (angle.arr[14] - 2048.0) * (360.0 / 4096.0);
  right_ankle_roll.theta = (angle.arr[16] - 2048.0) * (360.0 / 4096.0);

  right_hip_yaw.translation << RIGHT_HIP_YAW_TRANSLATION;
  right_hip_roll.translation << RIGHT_HIP_ROLL_TRANSLATION;
  right_hip_pitch.translation << RIGHT_HIP_PITCH_TRANSLATION;
  right_knee.translation << RIGHT_KNEE_TRANSLATION;
  right_ankle_pitch.translation << RIGHT_ANKLE_PITCH_TRANSLATION;
  right_ankle_roll.translation << RIGHT_ANKLE_ROLL_TRANSLATION;
  right_foot.translation << RIGHT_FOOT_TRANSLATION;

  right_hip_yaw.convertDegreeToRadians();
  right_hip_roll.convertDegreeToRadians();
  right_hip_pitch.convertDegreeToRadians();
  right_knee.convertDegreeToRadians();
  right_ankle_pitch.convertDegreeToRadians();
  right_ankle_roll.convertDegreeToRadians();

  right_hip_yaw.calculateTransformationMatrix(1);
  right_hip_roll.calculateTransformationMatrix(2);
  right_hip_pitch.calculateTransformationMatrix(0);
  right_knee.calculateTransformationMatrix(0);
  right_ankle_pitch.calculateTransformationMatrix(0);
  right_ankle_roll.calculateTransformationMatrix(2);
  right_foot.calculateTransformationMatrix(4);

  result = right_hip_yaw.transformation_matrix * right_hip_roll.transformation_matrix *
    right_hip_pitch.transformation_matrix * right_knee.transformation_matrix *
    right_ankle_pitch.transformation_matrix * right_ankle_roll.transformation_matrix *
    right_foot.transformation_matrix;
  return result;
}

Matrix4D ForwardKinematic::LeftLeg::getLeftLegForwardKinematic(int * arr, int size)
{
  Matrix4D result;
  ForwardKinematic angle(arr, size);
  left_hip_yaw.theta = -1.0 * (angle.arr[7] - 2048.0) * (360.0 / 4096.0);
  left_hip_roll.theta = -1.0 * (angle.arr[9] - 2048.0) * (360.0 / 4096.0);
  left_hip_pitch.theta = -1.0 * (angle.arr[11] - 2048.0) * (360.0 / 4096.0);
  left_knee.theta = -1.0 * (angle.arr[13] - 2048.0) * (360.0 / 4096.0);
  left_ankle_pitch.theta = (angle.arr[15] - 2048.0) * (360.0 / 4096.0);
  left_ankle_roll.theta = (angle.arr[17] - 2048.0) * (360.0 / 4096.0);

  left_hip_yaw.translation << LEFT_HIP_YAW_TRANSLATION;
  left_hip_roll.translation << LEFT_HIP_ROLL_TRANSLATION;
  left_hip_pitch.translation << LEFT_HIP_PITCH_TRANSLATION;
  left_knee.translation << LEFT_KNEE_TRANSLATION;
  left_ankle_pitch.translation << LEFT_ANKLE_PITCH_TRANSLATION;
  left_ankle_roll.translation << LEFT_ANKLE_ROLL_TRANSLATION;
  left_foot.translation << LEFT_FOOT_TRANSLATION;

  left_hip_yaw.convertDegreeToRadians();
  left_hip_roll.convertDegreeToRadians();
  left_hip_pitch.convertDegreeToRadians();
  left_knee.convertDegreeToRadians();
  left_ankle_pitch.convertDegreeToRadians();
  left_ankle_roll.convertDegreeToRadians();

  left_hip_yaw.calculateTransformationMatrix(1);
  left_hip_roll.calculateTransformationMatrix(2);
  left_hip_pitch.calculateTransformationMatrix(0);
  left_knee.calculateTransformationMatrix(0);
  left_ankle_pitch.calculateTransformationMatrix(0);
  left_ankle_roll.calculateTransformationMatrix(2);
  left_foot.calculateTransformationMatrix(4);

  result = left_hip_yaw.transformation_matrix * left_hip_roll.transformation_matrix *
    left_hip_pitch.transformation_matrix * left_knee.transformation_matrix *
    left_ankle_pitch.transformation_matrix * left_ankle_roll.transformation_matrix *
    left_foot.transformation_matrix;
  return result;
}

Vector3D ForwardKinematic::getCameraPosition()
{
  Vector4D pos, base;
  Vector3D pos3;
  base << 0.0, 0.0, 0.0, 1.0;
  pos = head.getHeadForwardKinematic(arr, size) * base;
  pos3 << pos(0, 0), pos(1, 0), pos(2, 0);
  return pos3;
}
Vector3D ForwardKinematic::getRightHandPosition()
{
  Vector4D pos, base;
  Vector3D pos3;
  base << 0.0, 0.0, 0.0, 1.0;
  pos = rightarm.getRightArmForwardKinematic(arr, size) * base;
  pos3 << pos(0, 0), pos(1, 0), pos(2, 0);
  return pos3;
}
Vector3D ForwardKinematic::getLeftHandPosition()
{
  Vector4D pos, base;
  Vector3D pos3;
  base << 0.0, 0.0, 0.0, 1.0;
  pos = leftarm.getLeftArmForwardKinematic(arr, size) * base;
  pos3 << pos(0, 0), pos(1, 0), pos(2, 0);
  return pos3;
}
Vector3D ForwardKinematic::getRightLegPosition()
{
  Vector4D pos, base;
  Vector3D pos3;
  base << 0.0, 0.0, 0.0, 1.0;
  pos = rightleg.getRightLegForwardKinematic(arr, size) * base;
  pos3 << pos(0, 0), pos(1, 0), pos(2, 0);
  return pos3;
}
Vector3D ForwardKinematic::getLeftLegPosition()
{
  Vector4D pos, base;
  Vector3D pos3;
  base << 0.0, 0.0, 0.0, 1.0;
  pos = leftleg.getLeftLegForwardKinematic(arr, size) * base;
  pos3 << pos(0, 0), pos(1, 0), pos(2, 0);
  return pos3;
}

Vector3D ForwardKinematic::getRightHandPositionFromCamera()
{
  Vector3D arrow;
  arrow = (-1.0 * getCameraPosition()) + getRightHandPosition();
  return arrow;
}
Vector3D ForwardKinematic::getLeftHandPositionFromCamera()
{
  Vector3D arrow;
  arrow = (-1.0 * getCameraPosition()) + getLeftHandPosition();
  return arrow;
}
Vector3D ForwardKinematic::getRightLegPositionFromCamera()
{
  Vector3D arrow;
  arrow = (-1.0 * getCameraPosition()) + getRightLegPosition();
  return arrow;
}
Vector3D ForwardKinematic::getLeftLegPositionFromCamera()
{
  Vector3D arrow;
  arrow = (-1.0 * getCameraPosition()) + getLeftLegPosition();
  return arrow;
}

struct ForwardKinematicResult
{
  std::string camera_position;
  std::string right_hand_position;
  std::string left_hand_position;
  std::string right_leg_position;
  std::string left_leg_position;
  std::string right_hand_position_from_camera;
  std::string left_hand_position_from_camera;
  std::string right_leg_position_from_camera;
  std::string left_leg_position_from_camera;
  std::string value;
};

ForwardKinematic::ForwardKinematicsNode::ForwardKinematicsNode(int * arr, int size)
: Node("forward_kinematics"), arr_(arr), size_(size)
{
  // Set up a publisher to publish the forward kinematics results
  publisher_ = this->create_publisher<std_msgs::msg::String>("forward_kinematic_topic", 10);

  // Set up a timer to perform the forward kinematics calculations at a regular interval
  timer_ = this->create_wall_timer(
    std::chrono::seconds(1), std::bind(&ForwardKinematicsNode::timer_callback, this));
}

void ForwardKinematic::ForwardKinematicsNode::timer_callback()
{
  // Perform the forward kinematics calculations
  ForwardKinematic position(arr_, size_);
  ForwardKinematicResult result;
  result.value = "1:" + std::to_string(arr_[0]) + "\n" + "2:" + std::to_string(arr_[1]) + "\n" + "3:" + std::to_string(arr_[2])  + "\n" + "4:" + std::to_string(arr_[3]) + "\n"  + "5:" + std::to_string(arr_[4]) + "\n" + "6:" + std::to_string(arr_[5]) + "\n" + "7:" + std::to_string(arr_[6]) + "\n" + "8:" + std::to_string(arr_[7]) + "\n" + "9:" + std::to_string(arr_[8]) + "\n" + "10:" + std::to_string(arr_[9]) + "\n" + "11:" + std::to_string(arr_[10]) + "\n" + "12:" + std::to_string(arr_[11]) + "\n" + "13:" + std::to_string(arr_[12]) + "\n" + "14:" + std::to_string(arr_[13]) + "\n" + "15:" + std::to_string(arr_[14]) + "\n" + "16:" + std::to_string(arr_[15]) + "\n" + "17:" + std::to_string(arr_[16]) + "\n" + "18:" + std::to_string(arr_[17]) + "\n" + "19:" + std::to_string(arr_[18]) + "\n" + "20:" + std::to_string(arr_[19]);
  result.camera_position = "(CAMERA) X: " +
    std::to_string(position.ForwardKinematic::getCameraPosition()(0, 0)) +
    " Y: " + std::to_string(position.ForwardKinematic::getCameraPosition()(1, 0)) +
    " Z: " + std::to_string(position.ForwardKinematic::getCameraPosition()(2, 0));
  result.right_hand_position = "(RIGHT HAND) X: " + std::to_string(
    position.ForwardKinematic::getRightHandPosition()(0, 0)) +
    " Y: " + std::to_string(position.ForwardKinematic::getRightHandPosition()(1, 0)) +
    " Z: " + std::to_string(position.ForwardKinematic::getRightHandPosition()(2, 0));
  result.left_hand_position = "(LEFT HAND) X: " + std::to_string(
    position.ForwardKinematic::getLeftHandPosition()(0, 0)) +
    " Y: " + std::to_string(position.ForwardKinematic::getLeftHandPosition()(1, 0)) +
    " Z: " + std::to_string(position.ForwardKinematic::getLeftHandPosition()(2, 0));
  result.right_leg_position = "(RIGHT LEG) X: " + std::to_string(
    position.ForwardKinematic::getRightLegPosition()(0, 0)) +
    " Y: " + std::to_string(position.ForwardKinematic::getRightLegPosition()(1, 0)) +
    " Z: " + std::to_string(position.ForwardKinematic::getRightLegPosition()(2, 0));
  result.left_leg_position = "(LEFT LEG) X: " +
    std::to_string(position.ForwardKinematic::getLeftLegPosition()(0, 0)) +
    " Y: " + std::to_string(position.ForwardKinematic::getLeftLegPosition()(1, 0)) +
    " Z: " + std::to_string(position.ForwardKinematic::getLeftLegPosition()(2, 0));
  result.right_hand_position_from_camera = "(RIGHT HAND FROM CAMERA) X: " + std::to_string(
    position.ForwardKinematic::getRightHandPositionFromCamera()(0, 0)) +
    " Y: " + std::to_string(position.ForwardKinematic::getRightHandPositionFromCamera()(1, 0)) +
    " Z: " + std::to_string(position.ForwardKinematic::getRightHandPositionFromCamera()(2, 0));
  result.left_hand_position_from_camera = "(LEFT HAND FROM CAMERA) X: " + std::to_string(
    position.ForwardKinematic::getLeftHandPositionFromCamera()(0, 0)) +
    " Y: " + std::to_string(position.ForwardKinematic::getLeftHandPositionFromCamera()(1, 0)) +
    " Z: " + std::to_string(position.ForwardKinematic::getLeftHandPositionFromCamera()(2, 0));
  result.right_leg_position_from_camera = "(RIGHT LEG FROM CAMERA) X: " + std::to_string(
    position.ForwardKinematic::getRightLegPositionFromCamera()(0, 0)) +
    " Y: " + std::to_string(position.ForwardKinematic::getRightLegPositionFromCamera()(1, 0)) +
    " Z: " + std::to_string(position.ForwardKinematic::getRightLegPositionFromCamera()(2, 0));
  result.left_leg_position_from_camera = "(LEFT LEG FROM CAMERA) X: " + std::to_string(
    position.ForwardKinematic::getLeftLegPositionFromCamera()(0, 0)) +
    " Y: " + std::to_string(position.ForwardKinematic::getLeftLegPositionFromCamera()(1, 0)) +
    " Z: " + std::to_string(position.ForwardKinematic::getLeftLegPositionFromCamera()(2, 0));
  std::string message = result.value + "\n" + result.camera_position + "\n" + result.right_hand_position + "\n" +
    result.left_hand_position + "\n" + result.right_leg_position + "\n" +
    result.right_hand_position_from_camera + "\n" + result.left_hand_position_from_camera + "\n" +
    result.right_leg_position_from_camera + "\n" + result.left_leg_position_from_camera + "\n";
  auto msg = std_msgs::msg::String();
  msg.data = message;
  publisher_->publish(msg);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  int arr[20];
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto read_node = std::make_shared<rclcpp::Node>("read_joint_node");
  auto publish_node = std::make_shared<ForwardKinematic::ForwardKinematicsNode>(arr, 20);
  if (argc < 2) {
    std::cerr << "Please specify the mode! [sdk / cm740]" << std::endl;
    return 0;
  }

  std::string mode = argv[1];
  std::shared_ptr<tachimawari::control::ControlManager> control_manager;

  if (mode == "sdk") {
    control_manager = std::make_shared<tachimawari::control::DynamixelSDK>("/dev/ttyUSB0");
  } else if (mode == "cm740") {
    control_manager = std::make_shared<tachimawari::control::CM740>("/dev/ttyUSB0");
  } else {
    std::cerr << "Mode doesn't exist, select the correct mode! [sdk / cm740]" << std::endl;
    return 0;
  }

  if (!control_manager->connect()) {
    control_manager->set_port("/dev/ttyUSB1");

    if (!control_manager->connect()) {
      std::cout << "failed to connect controller\n";
      return 1;
    }
  }
  auto joint_manager = std::make_shared<tachimawari::joint::JointManager>(control_manager);
  auto timer = read_node->create_wall_timer(
    std::chrono::milliseconds(100), [&]() {
      auto joints = joint_manager->get_current_joints();
      std::vector<tachimawari::joint::Joint> new_joints(joints);
      for (auto & joint : new_joints) {
        float value = tachimawari::joint::Joint::CENTER_VALUE;
        int current_value = control_manager->read_packet(
          joint.get_id(), tachimawari::joint::protocol_1::MX28Address::PRESENT_POSITION_L, 2
        );
        value = (current_value == -1) ? value : current_value;
        joint.set_position_value(value);
      }
      for (const auto & joint : new_joints) {
        for (auto & current_joint : joints) {
          if (current_joint.get_id() == joint.get_id()) {
            current_joint.set_position(joint.get_position());
            current_joint.set_pid_gain(
              joint.get_pid_gain()[0], joint.get_pid_gain()[1], joint.get_pid_gain()[2]);
            break;
          }
        }
      }

      int pos = 0;
      for (auto joint : joints) {
        arr[pos] = joint.get_position_value();
        pos++;
      }
      RCLCPP_INFO(read_node->get_logger(), "Reading Servos' Value --- Calculating --- Publishing");
    });
  ForwardKinematic joint_value(arr, 20);
  executor->add_node(read_node);
  executor->add_node(publish_node);
  executor->spin();
  rclcpp::shutdown();
  return 0;
}
