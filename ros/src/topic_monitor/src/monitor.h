#include <string>
#include <thread>
#include <std_msgs/String.h>
#include <time.h>

typedef struct MonitorSet{
  std::string topic_name;
  int acceptable_delay_ms;
  long prev_subscribe_time;
};


class Monitor {
  public:
    Monitor();
    int Init();

  private:
    std::map<std::string, ros::Subscriber> monitor_sub_map;
    std::map<std::string, MonitorSet> monitor_set_map;
    std::thread watchdog_timer_thread;
    int monitor_acceptable_delay_ms, interval_ms;
    ros::Publisher monitor_pub;
    ros::Publisher ping_pub;

    void MonitorCallback(const ros::MessageEvent<std_msgs::String const>& event);
    void StartSubscribersAndPublishers();
    void WatchdogTimer();
    long TimestampMilliseconds() {
      using namespace boost::chrono;
      return duration_cast<milliseconds>(
                system_clock::now().time_since_epoch()).count();
    };
};
