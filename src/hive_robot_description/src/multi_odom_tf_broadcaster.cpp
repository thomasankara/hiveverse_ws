/*******************************************************
 *  multi_odom_tf_broadcaster.cpp
 *  - TF dynamique :   robotN/odom ➜ robotN/base_footprint
 *  - TF statique  :   map         ➜ robotN/odom
 *  QoS  sim (déf.) / prod  paramétrable
 *******************************************************/
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <regex>
#include <unordered_set>

using nav_msgs::msg::Odometry;
using geometry_msgs::msg::TransformStamped;

class OdomToTF : public rclcpp::Node
{
public:
  OdomToTF()
  : Node("odom_to_tf_broadcaster"),
    qos_(rclcpp::KeepLast(1))            // placeholder, sera écrasé
  {
    /* -------- QoS -------- */
    std::string mode = declare_parameter<std::string>("mode", "sim");
    if (mode == "prod") {
      qos_ = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
      RCLCPP_INFO(get_logger(),"QoS=PROD (Reliable)");
    } else {
      qos_ = rclcpp::SensorDataQoS();
      RCLCPP_INFO(get_logger(),"QoS=SIM  (SensorDataQoS)");
    }

    tf_dyn_   = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tf_static_= std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    /* découverte des /robotN/odom toutes les sec */
    timer_ = create_wall_timer(std::chrono::seconds(1),
                               std::bind(&OdomToTF::discover_topics, this));
  }

private:
  void discover_topics()
  {
    static const std::regex rgx("^/robot[0-9]+/odom$");

    for (const auto & kv : get_topic_names_and_types())
    {
      const std::string & topic = kv.first;
      if (!std::regex_match(topic, rgx) || subscribed_.count(topic)) continue;

      RCLCPP_INFO(get_logger(), "Subscribe %s", topic.c_str());
      auto sub = create_subscription<Odometry>(
        topic, qos_,
        [this, topic](Odometry::SharedPtr msg)
        {
          std::string ns = topic.substr(1, topic.find('/',1)-1);   // robotX

          /* ---- TF dynamique  robotX/odom ➜ robotX/base_footprint ---- */
          TransformStamped t;
          t.header.stamp = msg->header.stamp;
          t.header.frame_id  = ns + "/odom";
          t.child_frame_id   = ns + "/base_footprint";
          t.transform.translation.x = msg->pose.pose.position.x;
          t.transform.translation.y = msg->pose.pose.position.y;
          t.transform.translation.z = msg->pose.pose.position.z;
          t.transform.rotation      = msg->pose.pose.orientation;
          tf_dyn_->sendTransform(t);

          /* ---- TF statique  map ➜ robotX/odom  (envoyée 1×) ---- */
          if (published_static_.insert(ns).second) {
            TransformStamped s;
            s.header.stamp      = this->now();
            s.header.frame_id   = "map";
            s.child_frame_id    = ns + "/odom";
            s.transform.rotation.w = 1.0;          // identité
            tf_static_->sendTransform(s);
            std::cout << "\033[1;34m[TF STATIC] map → "
                      << s.child_frame_id << "\033[0m\n";
          }

          /* log vert une seule fois par robot pour le dynamique */
          std::string key = t.header.frame_id + "->" + t.child_frame_id;
          if (printed_.insert(key).second) {
            std::cout << "\033[1;32m[TF PUB] " << key << "\033[0m\n";
          }
        });

      subs_.push_back(sub);
      subscribed_.insert(topic);
    }
  }

  /* membres ------------------------------------------------ */
  rclcpp::QoS qos_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster>      tf_dyn_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster>tf_static_;
  std::vector<rclcpp::Subscription<Odometry>::SharedPtr> subs_;
  std::unordered_set<std::string> subscribed_, printed_, published_static_;
};

/* ---------------- main ---------------- */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomToTF>());
  rclcpp::shutdown();
  return 0;
}
