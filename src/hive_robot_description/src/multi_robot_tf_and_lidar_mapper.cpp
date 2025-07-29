/**************************************************************
 *  multi_robot_tf_and_lidar_mapper.cpp  (ROS 2 Jazzy)
 *
 *  ▶  TF dynamique :
 *       map          → robotN/odom           (timestamp = odom)
 *       robotN/odom  → robotN/base_footprint (plan XY)
 *
 *  ▶  Remap Lidar :
 *       /robotX/<name>_pc2_sim  → /robotX/<name>_pc2
 *       + correction header.frame_id  -> "robotX/<leaf>"
 *
 *  Paramètres ROS :
 *       mode   = sim   | prod    (QoS)
 *       topics = [...]           (liste de bases sans _sim ; vide = auto-discover)
 **************************************************************/

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>   // <-- pour getYaw

#include <nav_msgs/msg/odometry.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <unordered_map>
#include <regex>
#include <vector>
#include <array>
#include <cmath>
#include <chrono>

using nav_msgs::msg::Odometry;
using tf2_msgs::msg::TFMessage;
using sensor_msgs::msg::PointCloud2;
using geometry_msgs::msg::TransformStamped;

using namespace std::chrono_literals;

/* ---------------- Small helper ---------------- */
struct Pose2D { double x=0, y=0, yaw=0; rclcpp::Time stamp; bool valid=false; };

/* ---------------- Node class ------------------ */
class MultiRobotTFAndLidar : public rclcpp::Node
{
public:
  MultiRobotTFAndLidar()
  : Node("multi_robot_tf_and_lidar"),
    qos_(1), qos_pc2_(1)
  {
    /* ----------- Parameters ----------- */
    std::string mode = declare_parameter<std::string>("mode", "sim");
    topics_param_    = declare_parameter<std::vector<std::string>>("topics", std::vector<std::string>{});
    if (mode == "prod") {
      qos_ = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
      qos_pc2_ = rclcpp::SensorDataQoS();        // Best Effort pour prod
    } else {
      qos_ = rclcpp::SensorDataQoS();            // Best Effort, depth 0
      qos_pc2_ = rclcpp::QoS(rclcpp::KeepLast(1)).reliable(); // fiable pour RViz
    }

    tf_dyn_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    /* ---------- Timers ---------- */
    timer_discover_odom_ = create_wall_timer(
      1s, std::bind(&MultiRobotTFAndLidar::discover_odom_topics, this));

    timer_discover_pc2_  = create_wall_timer(
      2s, std::bind(&MultiRobotTFAndLidar::discover_pc2_topics, this));

    /* ---------- Ground-truth world pose ---------- */
    world_sub_ = create_subscription<TFMessage>(
      "/world/pose_info", qos_,
      std::bind(&MultiRobotTFAndLidar::on_world_pose, this, std::placeholders::_1));
  }

private:
  /* ---------- TF part ---------- */
  rclcpp::QoS qos_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_dyn_;
  rclcpp::Subscription<TFMessage>::SharedPtr world_sub_;
  rclcpp::TimerBase::SharedPtr timer_discover_odom_;
  std::unordered_map<std::string,
      rclcpp::Subscription<Odometry>::SharedPtr> odom_subs_;
  std::unordered_map<std::string, Pose2D> odom_last_;   // "robot1", "robot2"
  struct Track { Pose2D world; bool valid=false; } r1_, r2_;

  /* ---------- PointCloud mapper ---------- */
  rclcpp::QoS qos_pc2_;
  std::vector<std::string> topics_param_;
  rclcpp::TimerBase::SharedPtr timer_discover_pc2_;
  std::vector<rclcpp::Subscription<PointCloud2>::SharedPtr> pc2_subs_;
  std::vector<rclcpp::Publisher<PointCloud2>::SharedPtr>    pc2_pubs_;

  /* === Discover /robotN/odom === */
  void discover_odom_topics()
  {
    for (const auto & kv : get_topic_names_and_types())
    {
      const auto & topic = kv.first;
      if (odom_subs_.count(topic)) continue;
      if (topic.find("/robot") == std::string::npos || topic.find("/odom") == std::string::npos) continue;

      auto sub = create_subscription<Odometry>(
        topic, qos_,
        [this, topic](Odometry::SharedPtr msg)
        {
          std::string ns = topic.substr(1, topic.find('/',1)-1); // "robotX"
          handle_odom(ns, msg);
        });
      odom_subs_[topic] = sub;
      RCLCPP_INFO(get_logger(), "Subscribed odom %s", topic.c_str());
    }
  }

  /* === Handle Odometry === */
  void handle_odom(const std::string & ns, const Odometry::SharedPtr & msg)
  {
    Pose2D & o = odom_last_[ns];
    o.x = msg->pose.pose.position.x;
    o.y = msg->pose.pose.position.y;
    o.yaw = tf2::getYaw(msg->pose.pose.orientation);
    o.stamp = msg->header.stamp;
    o.valid = true;

    /* odom → base_footprint */
    TransformStamped t;
    t.header.stamp    = msg->header.stamp;
    t.header.frame_id = ns + "/odom";
    t.child_frame_id  = ns + "/base_footprint";
    t.transform.translation.x = o.x;
    t.transform.translation.y = o.y;
    t.transform.translation.z = 0.0;
    tf2::Quaternion q; q.setRPY(0,0,o.yaw);
    t.transform.rotation = tf2::toMsg(q);
    tf_dyn_->sendTransform(t);
  }

  /* === Ground-truth pose === */
  void on_world_pose(const TFMessage::SharedPtr msg)
  {
    std::vector<Pose2D> poses;
    for (const auto & tr : msg->transforms) {
      double x=tr.transform.translation.x, y=tr.transform.translation.y;
      if (fabs(x)<1e-6 && fabs(y)<1e-6) continue;
      Pose2D p; p.x=x; p.y=y; p.yaw=tf2::getYaw(tr.transform.rotation);
      p.stamp = rclcpp::Time(tr.header.stamp); p.valid=true;
      poses.push_back(p);
    }
    if (poses.size()<2) return;

    /* init : plus proche (0,0) = robot1 */
    if (!r1_.valid || !r2_.valid) {
      size_t i_r1 = hypot(poses[0].x,poses[0].y) <= hypot(poses[1].x,poses[1].y) ? 0 : 1;
      size_t i_r2 = 1-i_r1;
      r1_.world=poses[i_r1]; r1_.valid=true; r2_.world=poses[i_r2]; r2_.valid=true;
      return;
    }

    /* association par plus proche */
    std::array<Pose2D*,2> prev{&r1_.world,&r2_.world};
    std::array<size_t,2> idx{0,1}; std::array<double,2> dist{1e9,1e9};
    for (size_t i=0;i<poses.size();++i)
      for(int r=0;r<2;++r) {
        double d=hypot(poses[i].x-prev[r]->x,poses[i].y-prev[r]->y);
        if(d<dist[r]){dist[r]=d;idx[r]=i;}
      }
    if(idx[0]==idx[1]) idx[1]=1-idx[0];
    r1_.world=poses[idx[0]]; r2_.world=poses[idx[1]];

    publish_map_to_odom("robot1", r1_.world, odom_last_["robot1"]);
    publish_map_to_odom("robot2", r2_.world, odom_last_["robot2"]);
  }

  void publish_map_to_odom(const std::string & ns,const Pose2D & w,const Pose2D & o)
  {
    if(!w.valid||!o.valid) return;
    tf2::Transform T_map_base, T_odom_base;
    T_map_base.setOrigin({w.x,w.y,0}); tf2::Quaternion qmb;qmb.setRPY(0,0,w.yaw);T_map_base.setRotation(qmb);
    T_odom_base.setOrigin({o.x,o.y,0});tf2::Quaternion qob;qob.setRPY(0,0,o.yaw);T_odom_base.setRotation(qob);
    tf2::Transform T_map_odom = T_map_base*T_odom_base.inverse();

    TransformStamped m2o;
    m2o.header.stamp    = o.stamp;          // clé : même temps qu’odom
    m2o.header.frame_id = "map";
    m2o.child_frame_id  = ns + "/odom";
    m2o.transform.translation.x = T_map_odom.getOrigin().x();
    m2o.transform.translation.y = T_map_odom.getOrigin().y();
    m2o.transform.translation.z = 0.0;
    m2o.transform.rotation      = tf2::toMsg(T_map_odom.getRotation());
    tf_dyn_->sendTransform(m2o);
  }

  /* === Discover & remap PointCloud2 === */
  void discover_pc2_topics()
  {
    std::vector<std::string> bases = topics_param_;
    if (bases.empty()) {
      std::regex rgx(".*_sim$");
      for (const auto & kv : get_topic_names_and_types()) {
        if (!std::regex_match(kv.first,rgx)) continue;
        if (std::find(kv.second.begin(),kv.second.end(),"sensor_msgs/msg/PointCloud2")==kv.second.end()) continue;
        bases.push_back(kv.first.substr(0, kv.first.size()-4)); // enlève _sim
      }
    }
    for (const auto & base : bases) {
      std::string in_t = base+"_sim";
      std::string out_t= base;
      /* évite doublons */
      bool already=false;
      for(auto & p:pc2_pubs_) if(p->get_topic_name()==out_t) {already=true; break;}
      if(already) continue;

      auto pub = create_publisher<PointCloud2>(out_t, qos_pc2_);
      auto sub = create_subscription<PointCloud2>(
        in_t, qos_pc2_,
        [this,pub,base](PointCloud2::SharedPtr msg)
        {
          auto out = std::make_shared<PointCloud2>(*msg);
          /* corrige frame_id : garde le feuillet (leaf) et ajoute namespace */
          std::string leaf = out->header.frame_id;
          auto pos = leaf.rfind('/');
          leaf = (pos==std::string::npos)? leaf : leaf.substr(pos+1);
          std::string ns = base.substr(0, base.find('/',1)+1); // "/robot1/"
          out->header.frame_id = ns + leaf;
          pub->publish(*out);
        });
      pc2_pubs_.push_back(pub); pc2_subs_.push_back(sub);
      RCLCPP_INFO(get_logger(),"Remap %s → %s (+frame_id fix)", in_t.c_str(), out_t.c_str());
    }
  }
};

/* ---------------- main ---------------- */
int main(int argc,char**argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<MultiRobotTFAndLidar>());
  rclcpp::shutdown();
  return 0;
}
