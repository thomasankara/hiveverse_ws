#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <regex>
#include <iostream>

using sensor_msgs::msg::PointCloud2;

class LidarFrameMapper : public rclcpp::Node
{
public:
  LidarFrameMapper()
  : Node("lidar_frame_mapper")
  {
    // Paramètres optionnels : liste de topics (sans _sim), mode (simu/prod)
    this->declare_parameter<std::vector<std::string>>("topics", std::vector<std::string>({}));
    this->declare_parameter<std::string>("mode", "simu");

    // Détecte le mode QoS
    std::string mode = this->get_parameter("mode").as_string();
    rclcpp::QoS qos = rclcpp::QoS(1); // Default: depth=1
    if (mode == "prod") {
      qos = rclcpp::SensorDataQoS(); // BestEffort, Volatile, KeepLast(1)
      RCLCPP_INFO(this->get_logger(), "\033[1;33mMode PROD : SensorDataQoS (BestEffort, low delay, pertes possibles)\033[0m");
    } else {
      qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
      qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
      RCLCPP_INFO(this->get_logger(), "\033[1;36mMode SIMU/RViz : QoS RELIABLE (pour visu RViz2)\033[0m");
    }

    // Charge ou découvre les topics à mapper
    auto base_topics = this->get_parameter("topics").as_string_array();
    if (base_topics.empty())
    {
      RCLCPP_INFO(this->get_logger(), "No 'topics' param, auto-discovering *_sim PointCloud2 topics...");
      auto topic_names = this->get_topic_names_and_types();
      std::regex sim_rgx(".*_sim$");
      for (const auto & kv : topic_names)
      {
        const auto & name = kv.first;
        if (std::regex_match(name, sim_rgx) &&
            std::find(kv.second.begin(), kv.second.end(), "sensor_msgs/msg/PointCloud2") != kv.second.end())
        {
          std::string base = name.substr(0, name.size() - 4); // retire _sim
          base_topics.push_back(base);
        }
      }
    }
    if (base_topics.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "No PointCloud2 *_sim topics found. Exiting.");
      rclcpp::shutdown();
      return;
    }

    // Pour chaque topic, crée publisher et subscriber
    for (const auto & base : base_topics)
    {
      std::string in_topic  = base + "_sim";
      std::string out_topic = base;

      auto pub = this->create_publisher<PointCloud2>(out_topic, qos);
      auto sub = this->create_subscription<PointCloud2>(
        in_topic, qos,
        [this, pub, base](PointCloud2::SharedPtr msg)
        {
          auto out_msg = std::make_shared<PointCloud2>(*msg);

          // Correction du frame_id :
          std::string old_id = out_msg->header.frame_id;
          auto pos = old_id.rfind('/');
          std::string leaf = (pos == std::string::npos) ? old_id : old_id.substr(pos+1);

          auto ns_pos = base.find('/', 1);
          std::string ns = (ns_pos == std::string::npos) ? "" : base.substr(0, ns_pos+1);

          out_msg->header.frame_id = ns + leaf;

          pub->publish(*out_msg);
        });

      subs_.push_back(sub);
      pubs_.push_back(pub);

      // Affiche une fois en vert le mapping
      std::cout << "\033[1;32m[frame_id] mapping: "
                << in_topic << " --> " << out_topic
                << "  (QoS=" << (mode=="prod" ? "SensorData" : "RELIABLE") << ", frame fix active)\033[0m" << std::endl;
      RCLCPP_INFO(this->get_logger(), "Remapping %s -> %s (QoS %s, frame fix)", in_topic.c_str(), out_topic.c_str(), mode.c_str());
    }
  }

private:
  std::vector<rclcpp::Subscription<PointCloud2>::SharedPtr> subs_;
  std::vector<rclcpp::Publisher<PointCloud2>::SharedPtr> pubs_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarFrameMapper>());
  rclcpp::shutdown();
  return 0;
}
