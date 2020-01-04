/** Copyright 2020 Lucas Walter
 *
 */

#include <algorithm>
#include <deque>
#include <map>
#include <numeric>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

// https://stackoverflow.com/questions/49992249/how-to-calculate-the-sample-mean-standard-deviation-and-variance-in-c-from-r
template<typename T>
float mean(T it, T end, const size_t sz) {
  if (sz == 0) {
    return 0.0;
  }
  return std::accumulate(it, end, 0.0) / sz;
}

// TODO(lucasw) make way to get mean and standard deviation in one call,
// don't recompute mean.
// (though maybe optimizer does that effectively?)
template<typename T>
float variance(T it, T end, const size_t sz) {
  if (sz == 0) {
    return 0.0;
  }
  float x_bar = mean(it, end, sz);
  float sq_sum = std::inner_product(it, end, it, 0.0);
  return sq_sum / sz - x_bar * x_bar;
}

template<typename T>
float standard_deviation(T it, T end, const size_t sz) {
  return std::sqrt(variance(it, end, sz));
}

class FloatStats
{
public:
  FloatStats()
  {
    for (const auto& key : {"average", "stddev"}) {
      pubs_[key] = nh_.advertise<std_msgs::Float32>(key, 10);
    }

    int tmp_window;
    ros::param::get("~window", tmp_window);
    window_ = tmp_window;
    sub_ = nh_.subscribe("topic", 3, &FloatStats::callback, this);

    double period = 0.1;
    ros::param::get("~period", period);
    timer_ = nh_.createTimer(ros::Duration(period), &FloatStats::update, this);
  }
private:
  void callback(const std_msgs::Float32& msg)
  {
    vals_.push_back(msg.data);
    if (vals_.size() >= window_) {
      vals_.pop_front();
    }
  }

  void update(const ros::TimerEvent& te)
  {
    if (vals_.size() == 0) {
      return;
    }
    const float average = mean(vals_.begin(), vals_.end(), vals_.size());
    std_msgs::Float32 msg;
    msg.data = average;
    pubs_["average"].publish(msg);
    const float stddev = standard_deviation(vals_.begin(), vals_.end(), vals_.size());
    msg.data = stddev;
    pubs_["stddev"].publish(msg);

    // TODO(lucasw) optional histogram
  }

  ros::NodeHandle nh_;
  ros::Timer timer_;
  std::map<std::string, ros::Publisher> pubs_;
  ros::Subscriber sub_;

  size_t window_ = 200;
  std::deque<float> vals_;
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "float_stats");
  FloatStats float_stats;
  ros::spin();
  return 0;
}
