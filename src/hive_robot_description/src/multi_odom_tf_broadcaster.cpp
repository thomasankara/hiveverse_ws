/************************************************************
 *  multi_odom_tf_broadcaster.cpp  (ROS 2 Jazzy)
 *  - Publie pour chaque robotN :
 *      map            ──► robotN/odom           (dynamique, timestamp = odom)
 *      robotN/odom    ──► robotN/base_footprint (dynamique, plan XY)
 *  - Associe chaque robot à partir de /world/pose_info (sans child_frame_id)
 *  - Republie map→odom à CHAQUE message odom pour éviter toute erreur RViz
 ************************************************************/

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <unordered_map>
#include <vector>
#include <array>
#include <cmath>
#include <chrono>

using nav_msgs::msg::Odometry;
using tf2_msgs::msg::TFMessage;
using geometry_msgs::msg::TransformStamped;

struct Pose2D
{
  double x = 0.0, y = 0.0, yaw = 0.0;
  rclcpp::Time stamp;
  bool valid = false;
};

class OdomToTF : public rclcpp::Node
{
public:
  OdomToTF() : Node("odom_to_tf_broadcaster"), qos_(1)
  {
    std::string mode = declare_parameter<std::string>("mode", "sim");
    if (mode == "prod") {
      qos_ = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
      RCLCPP_INFO(get_logger(), "QoS=PROD (Reliable)");
    } else {
      qos_ = rclcpp::SensorDataQoS();
      RCLCPP_INFO(get_logger(), "QoS=SIM (SensorDataQoS)");
    }

    tf_dyn_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&OdomToTF::discover_odom_topics, this));

    world_sub_ = create_subscription<TFMessage>(
      "/world/pose_info", qos_,
      std::bind(&OdomToTF::on_world_pose, this, std::placeholders::_1));
  }

private:
  rclcpp::QoS qos_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_dyn_;
  std::unordered_map<std::string,
      rclcpp::Subscription<Odometry>::SharedPtr> odom_subs_;
  rclcpp::Subscription<TFMessage>::SharedPtr world_sub_;

  std::unordered_map<std::string, Pose2D> odom_last_;  // "robot1", "robot2"

  struct Track { Pose2D world; bool valid=false; } r1_, r2_;

  void discover_odom_topics()
  {
    for (const auto & kv : get_topic_names_and_types())
    {
      const std::string & topic = kv.first;
      if (odom_subs_.count(topic)) continue;
      if (topic.find("/robot") == std::string::npos ||
          topic.find("/odom") == std::string::npos)   continue;

      auto sub = create_subscription<Odometry>(
        topic, qos_,
        [this, topic](Odometry::SharedPtr msg)
        {
          std::string ns = topic.substr(1, topic.find('/',1)-1); // "robotX"
          handle_odom(ns, msg);
        });
      odom_subs_[topic] = sub;
      RCLCPP_INFO(get_logger(), "Subscribed to %s", topic.c_str());
    }
  }

  void handle_odom(const std::string & ns, const Odometry::SharedPtr & msg)
  {
    Pose2D & odom = odom_last_[ns];
    odom.x = msg->pose.pose.position.x;
    odom.y = msg->pose.pose.position.y;
    odom.yaw = tf2::getYaw(msg->pose.pose.orientation);
    odom.stamp = msg->header.stamp;
    odom.valid = true;

    /* odom → base_footprint (planar) */
    TransformStamped t;
    t.header.stamp    = msg->header.stamp;
    t.header.frame_id = ns + "/odom";
    t.child_frame_id  = ns + "/base_footprint";
    t.transform.translation.x = odom.x;
    t.transform.translation.y = odom.y;
    t.transform.translation.z = 0.0;
    tf2::Quaternion q;  q.setRPY(0,0,odom.yaw);
    t.transform.rotation = tf2::toMsg(q);
    tf_dyn_->sendTransform(t);

    // -- republie map->odom à CHAQUE message odom
    if (r1_.valid && ns == "robot1") {
      publish_map_to_odom("robot1", r1_.world, odom_last_["robot1"]);
    }
    if (r2_.valid && ns == "robot2") {
      publish_map_to_odom("robot2", r2_.world, odom_last_["robot2"]);
    }
  }

  void on_world_pose(const TFMessage::SharedPtr msg)
  {
    std::vector<Pose2D> poses;
    for (const auto & tr : msg->transforms) {
      double x = tr.transform.translation.x;
      double y = tr.transform.translation.y;
      if (std::abs(x) < 1e-6 && std::abs(y) < 1e-6) continue;

      Pose2D p;
      p.x = x;
      p.y = y;
      p.yaw = tf2::getYaw(tr.transform.rotation);
      p.stamp = rclcpp::Time(tr.header.stamp);
      p.valid = true;
      poses.push_back(p);
    }
    if (poses.size() < 2) return;

    if (!r1_.valid || !r2_.valid) {
      size_t i_r1 = (hypot(poses[0].x, poses[0].y) <= hypot(poses[1].x, poses[1].y)) ? 0 : 1;
      size_t i_r2 = 1 - i_r1;
      r1_.world = poses[i_r1]; r1_.valid = true;
      r2_.world = poses[i_r2]; r2_.valid = true;
      return;
    }

    std::array<Pose2D*,2> prev = {&r1_.world, &r2_.world};
    std::array<size_t,2> idx   = {0,1};
    std::array<double,2> dist  = {1e9,1e9};

    for (size_t i=0;i<poses.size();++i)
      for (int r=0;r<2;++r) {
        double d = hypot(poses[i].x - prev[r]->x, poses[i].y - prev[r]->y);
        if (d < dist[r]) { dist[r] = d; idx[r] = i; }
      }
    if (idx[0]==idx[1]) idx[1]=1-idx[0];

    r1_.world = poses[idx[0]];
    r2_.world = poses[idx[1]];
    // -- Plus besoin de publier ici, tout se fait dans handle_odom
  }

  void publish_map_to_odom(const std::string & ns,
                           const Pose2D & world,
                           const Pose2D & odom)
  {
    if (!world.valid || !odom.valid) return;

    tf2::Transform T_map_base, T_odom_base;
    T_map_base.setOrigin({world.x, world.y, 0});
    tf2::Quaternion q_mb; q_mb.setRPY(0,0,world.yaw);
    T_map_base.setRotation(q_mb);

    T_odom_base.setOrigin({odom.x, odom.y, 0});
    tf2::Quaternion q_ob; q_ob.setRPY(0,0,odom.yaw);
    T_odom_base.setRotation(q_ob);

    tf2::Transform T_map_odom = T_map_base * T_odom_base.inverse();

    TransformStamped m2o;
    m2o.header.stamp    = odom.stamp;
    m2o.header.frame_id = "map";
    m2o.child_frame_id  = ns + "/odom";
    m2o.transform.translation.x = T_map_odom.getOrigin().x();
    m2o.transform.translation.y = T_map_odom.getOrigin().y();
    m2o.transform.translation.z = 0.0;
    m2o.transform.rotation = tf2::toMsg(T_map_odom.getRotation());
    tf_dyn_->sendTransform(m2o);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomToTF>());
  rclcpp::shutdown();
  return 0;
}
