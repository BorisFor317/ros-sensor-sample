#include <string>

#include "sensor_msgs/msg/temperature.hpp"

// using TemperatureMsg = sensor_msgs::msg::Temperature;
// const std::string TemperatureTopic = "temperature";
// const std::string Temperature

struct Temperature {
    using Msg = sensor_msgs::msg::Temperature;

    const static std::string TopicName;
};

const std::string Temperature::TopicName = "temperature";
