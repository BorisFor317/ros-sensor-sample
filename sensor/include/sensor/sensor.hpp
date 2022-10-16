#include <string>

#include "sensor_msgs/msg/temperature.hpp"

struct Temperature {
  using Msg = sensor_msgs::msg::Temperature;

  const static std::string TopicName;
};

const std::string Temperature::TopicName = "temperature";

template <typename T> struct ISensor {
  using SharedPtr = std::shared_ptr<ISensor<T>>;

  virtual T read() = 0;
};
