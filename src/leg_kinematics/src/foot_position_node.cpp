// Subscribes to /joint_states, computes each foot's position in the body frame
// with the closed-form leg FK, publishes them as a PoseArray, and broadcasts a
// TF frame per foot.
//
// All robot geometry comes from parameters (see config/*.yaml); nothing about a
// particular robot is compiled in.

#include <array>
#include <cstddef>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <leg_kinematics/leg_kinematics.hpp>

namespace leg_kinematics
{

class FootPositionNode : public rclcpp::Node
{
public:
  FootPositionNode()
  : Node("foot_position_node")
  {
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    leg_names_ = declare_parameter<std::vector<std::string>>(
      "legs", std::vector<std::string>{"FL", "FR", "RL", "RR"});

    // Geometry shared by all four legs.
    const double l_abad = declare_parameter<double>("l_abad", 0.0);
    const double l_thigh = declare_parameter<double>("l_thigh", 0.0);
    const double l_calf = declare_parameter<double>("l_calf", 0.0);
    const Eigen::Vector3d q_min = vec3Param("q_min");
    const Eigen::Vector3d q_max = vec3Param("q_max");
    const Eigen::Vector3d q_nominal = vec3Param("q_nominal");

    // Per-leg geometry, and the joint names its angles arrive under.
    for (std::size_t i = 0; i < leg_names_.size(); ++i) {
      const std::string & leg = leg_names_[i];

      LegParams p;
      p.l_abad = l_abad;
      p.l_thigh = l_thigh;
      p.l_calf = l_calf;
      p.abad_sign = declare_parameter<double>(leg + ".abad_sign", 1.0);
      p.hip = vec3Param(leg + ".hip");
      p.q_min = q_min;
      p.q_max = q_max;
      p.q_nominal = q_nominal;
      legs_.push_back(p);

      // Seed for the IK branch choice later; also what we hold when a message
      // arrives without this leg in it.
      q_last_.push_back(q_nominal);

      const auto joints = declare_parameter<std::vector<std::string>>(
        leg + ".joints", std::vector<std::string>{});
      if (joints.size() != 3) {
        throw std::runtime_error(leg + ".joints must list exactly 3 joint names");
      }
      for (std::size_t j = 0; j < 3; ++j) {
        joint_slot_[joints[j]] = JointSlot{i, j};
      }
    }

    // Reliable, so any subscriber can connect. Depth 10 = buffer 10 messages.
    publisher_ = create_publisher<geometry_msgs::msg::PoseArray>("foot_positions", 10);

    // Best-effort: joint states are a stream, a dropped one is replaced in
    // milliseconds and is not worth resending.
    subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {onJointState(*msg);});

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(
      get_logger(), "ready: %zu legs, l_thigh=%.4f l_calf=%.4f, base frame '%s'",
      legs_.size(), l_thigh, l_calf, base_frame_.c_str());
  }

private:
  struct JointSlot { std::size_t leg; std::size_t joint; };

  /// Read a 3-element list parameter as a vector.
  Eigen::Vector3d vec3Param(const std::string & name)
  {
    const auto v = declare_parameter<std::vector<double>>(name, std::vector<double>{});
    if (v.size() != 3) {
      throw std::runtime_error("parameter '" + name + "' must have exactly 3 elements");
    }
    return Eigen::Vector3d(v[0], v[1], v[2]);
  }

  void onJointState(const sensor_msgs::msg::JointState & msg)
  {
    // Match joints BY NAME. A real driver is free to reorder them, and matching
    // by array index would silently swap legs.
    std::vector<Eigen::Vector3d> q = q_last_;
    std::vector<std::array<bool, 3>> found(legs_.size(), {false, false, false});

    for (std::size_t k = 0; k < msg.name.size() && k < msg.position.size(); ++k) {
      const auto it = joint_slot_.find(msg.name[k]);
      if (it == joint_slot_.end()) {
        continue;  // a joint we don't model; not an error
      }
      q[it->second.leg](static_cast<int>(it->second.joint)) = msg.position[k];
      found[it->second.leg][it->second.joint] = true;
    }

    // Publish all four legs or none, so a pose's index always means the same leg.
    for (std::size_t i = 0; i < legs_.size(); ++i) {
      for (std::size_t j = 0; j < 3; ++j) {
        if (!found[i][j]) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "incomplete joint state: leg %s missing joint %zu", leg_names_[i].c_str(), j);
          return;
        }
      }
    }

    // Carry the sender's timestamp, do NOT stamp with now(): the difference
    // between when a measurement was taken and when we processed it is a real
    // quantity, and Day 6 measures it. Fall back only if the sender left it unset.
    auto stamp = msg.header.stamp;
    if (stamp.sec == 0 && stamp.nanosec == 0u) {
      stamp = now();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "joint state has no timestamp; substituting receive time");
    }

    geometry_msgs::msg::PoseArray poses;
    poses.header.stamp = stamp;
    poses.header.frame_id = base_frame_;
    poses.poses.resize(legs_.size());

    std::vector<geometry_msgs::msg::TransformStamped> transforms(legs_.size());

    for (std::size_t i = 0; i < legs_.size(); ++i) {
      // FK gives the foot in the hip frame; the hip mount is a pure translation
      // in the body frame, so one addition puts the foot in the body frame.
      const Eigen::Vector3d p_foot = legs_[i].hip + footPositionFromJoints(q[i], legs_[i]);

      poses.poses[i].position.x = p_foot.x();
      poses.poses[i].position.y = p_foot.y();
      poses.poses[i].position.z = p_foot.z();
      poses.poses[i].orientation.w = 1.0;  // identity: a foot point has no orientation

      transforms[i].header.stamp = stamp;
      transforms[i].header.frame_id = base_frame_;
      transforms[i].child_frame_id = leg_names_[i] + "_foot";
      transforms[i].transform.translation.x = p_foot.x();
      transforms[i].transform.translation.y = p_foot.y();
      transforms[i].transform.translation.z = p_foot.z();
      transforms[i].transform.rotation.w = 1.0;
    }

    publisher_->publish(poses);
    tf_broadcaster_->sendTransform(transforms);
    q_last_ = q;
  }

  std::string base_frame_;
  std::vector<std::string> leg_names_;
  std::vector<LegParams> legs_;
  std::vector<Eigen::Vector3d> q_last_;
  std::unordered_map<std::string, JointSlot> joint_slot_;

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace leg_kinematics

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<leg_kinematics::FootPositionNode>());
  rclcpp::shutdown();
  return 0;
}
