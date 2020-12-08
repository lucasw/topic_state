/** Copyright 2020 Lucas Walter
 *
 */

#include <algorithm>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <deque>
#include <map>
#include <memory>
#include <numeric>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <string>
#include <vector>

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

    ddr_ = std::make_unique<ddynamic_reconfigure::DDynamicReconfigure>();
    ddr_->registerVariable<int>("window", &num_bins_, "number of samples to keep", 1, 8000);
    ddr_->registerVariable<bool>("enable_histogram", &enable_histogram_, "Output a histogram", false, true);
    ddr_->registerVariable<int>("num_bins", &window_, "number of histogram bins", 1, 200);
    ddr_->registerVariable<double>("bin_width", &bin_width_, "bin width", 0.000001, 100.0);
    ddr_->publishServicesTopics();

    sub_ = nh_.subscribe("topic", 3, &FloatStats::callback, this);

    double period = 0.1;
    ros::param::get("~period", period);
    timer_ = nh_.createTimer(ros::Duration(period), &FloatStats::update, this);
  }
private:
  void callback(const std_msgs::Float32& msg)
  {
    vals_.push_back(msg.data);
    while (vals_.size() >= static_cast<size_t>(window_)) {
      vals_.pop_front();
    }
  }

  void update(const ros::TimerEvent& te)
  {
    if (vals_.size() == 0) {
      return;
    }
    const int num_bins = num_bins_;
    const double bin_width = bin_width_;

    const float average = mean(vals_.begin(), vals_.end(), vals_.size());
    std_msgs::Float32 msg;
    msg.data = average;
    pubs_["average"].publish(msg);
    const float stddev = standard_deviation(vals_.begin(), vals_.end(), vals_.size());
    msg.data = stddev;
    pubs_["stddev"].publish(msg);

    // TODO(lucasw) optional histogram
    if (enable_histogram_) {
      std::vector<double> bounds;
      std::map<double, int> bounds_map;
      // TODO(lucasw) Stick all values < the lowest upper bound into the bottom bin,
      // and likewise with the upper bound?
      for (size_t i = 0; i <= num_bins; ++i) {
        const double bound = average + (static_cast<double>(i) - static_cast<double>(num_bins) / 2.0) * bin_width;
        bounds.push_back(bound);
        bounds_map[bound] = i;
      }

      std::vector<int> buckets;
      buckets.resize(bounds.size() + 1);

      for (const auto& val : vals_) {
        const auto lower_bound = std::find_if(bounds.begin(), bounds.end(),
            [&val](const double& bound) {return val < bound;});
        ++buckets[bounds_map[*lower_bound]];
      }

      std::cout << "histogram: ";
      for (const auto& num : bounds) {
        std::cout << num << " ";
      }
      std::cout << "\n";
      for (const auto& num : buckets) {
        std::cout << num << " ";
      }
      std::cout << "\n";
    }
  }

  ros::NodeHandle nh_;
  ros::Timer timer_;
  std::map<std::string, ros::Publisher> pubs_;
  ros::Subscriber sub_;

  int window_ = 200;
  std::deque<float> vals_;

  bool enable_histogram_ = true;
  int num_bins_ = 10;
  double bin_width_ = 0.001;
  std::unique_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddr_;
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "float_stats");
  FloatStats float_stats;
  ros::spin();
  return 0;
}
