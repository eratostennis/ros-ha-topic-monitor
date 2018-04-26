#include "ros/ros.h"
#include <map>
#include <string>
#include <chrono>
#define BOOST_CHRONO_VERSION 2
#include <boost/chrono.hpp>
#include <std_msgs/String.h>
#include <yaml-cpp/yaml.h>

#include "monitor.h"

Monitor::Monitor() {}

int Monitor::Init() {
  Monitor::StartSubscribersAndPublishers();
  return 0;
};

void Monitor::StartSubscribersAndPublishers() {
  ros::NodeHandle node_handle;
  ros::NodeHandle pnode_handle("~");

  std::string monitor_topic_name, pair_ping_topic_name, ping_topic_name;
  int monitor_acceptable_delay_ms;

  pnode_handle.param<std::string>("monitor_topic_name", monitor_topic_name, "/ha_topic_monitor");
  pnode_handle.param<std::string>("ping_topic_name", ping_topic_name, "/ping_topic");
  pnode_handle.param<std::string>("pair_ping_topic_name", pair_ping_topic_name, "/pair_ping_topic");
  pnode_handle.param<int>("monitor_acceptable_delay_ms", monitor_acceptable_delay_ms, 500);
  pnode_handle.param<int>("monitor_interval_ms", Monitor::interval_ms, 100);

  // Target Topic Subscribers
  YAML::Node lconf = YAML::LoadFile("/home/autoware/test-50.yaml");
  YAML::Node topics = lconf["topics"];
  for (std::size_t i=0;i<topics.size();i++) {
    YAML::Node topic = topics[i];
    std::string topic_name = topic["topic_name"].as<std::string>();

    MonitorSet monitor_set = MonitorSet({
        topic_name,
        topic["acceptable_delay_ms"].as<int>(),
        -1
    });
    Monitor::monitor_set_map.insert(std::make_pair(topic_name, monitor_set));
    Monitor::monitor_sub_map.insert(std::make_pair(topic_name, node_handle.subscribe(topic_name, 1, &Monitor::MonitorCallback, this))); 
  }

  // Monitor Result Publisher
  Monitor::monitor_pub = node_handle.advertise<std_msgs::String>(monitor_topic_name, 10, 1);
  Monitor::ping_pub = node_handle.advertise<std_msgs::String>(ping_topic_name, 10, 1);

  // Pair subscriber of HA Monitor
  MonitorSet monitor_set = MonitorSet({pair_ping_topic_name, Monitor::monitor_acceptable_delay_ms, -1});
  Monitor::monitor_set_map.insert(std::make_pair(pair_ping_topic_name, monitor_set));
  Monitor::monitor_sub_map.insert(std::make_pair(pair_ping_topic_name, node_handle.subscribe(pair_ping_topic_name, 1, &Monitor::MonitorCallback, this)));

  Monitor::watchdog_timer_thread = std::thread(&Monitor::WatchdogTimer, this);
  Monitor::watchdog_timer_thread.detach();

  ros::spin();
}

void Monitor::WatchdogTimer() {
  while (1) {
    long now_time = Monitor::TimestampMilliseconds();

    for(auto itr = Monitor::monitor_set_map.begin(); itr != Monitor::monitor_set_map.end(); ++itr) {
      if ((now_time - itr->second.prev_subscribe_time) > long(itr->second.acceptable_delay_ms)) {
        std_msgs::String delay_msg;
        delay_msg.data = itr->second.topic_name;
        Monitor::monitor_pub.publish(delay_msg);
      }
    }

    std_msgs::String ping_msg;
    ping_msg.data = "ping";
    Monitor::ping_pub.publish(ping_msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(Monitor::interval_ms));
  }
}

void Monitor::MonitorCallback(const ros::MessageEvent<std_msgs::String const>& event) {
  const ros::M_string& header = event.getConnectionHeader();
  std::string topic = header.at("topic");
  const std_msgs::StringConstPtr& msg = event.getMessage();

  Monitor::monitor_set_map[topic].prev_subscribe_time = Monitor::TimestampMilliseconds();
}


int main(int args, char **argv) {
  ros::init(args, argv, "monitor_topic");
  Monitor monitor;
  monitor.Init();

  return 0;
}
