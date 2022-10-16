#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor/sensor.hpp"
#include "sensor_msgs/msg/temperature.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

template <typename TMessage> class ConstantSensor : public ISensor<TMessage> {
public:
  ConstantSensor(TMessage value) : value_(value) {}

  TMessage read() { return value_; }

private:
  TMessage value_;
};

class TemperaturePublisher : public rclcpp::Node {
public:
  explicit TemperaturePublisher(ISensor<Temperature::Msg>::SharedPtr sensor)
      : Node("temperature_sensor"), sensor_(sensor) {
    publisher_ = create_publisher<Temperature::Msg>(Temperature::TopicName, 10);
    timer_ = create_wall_timer(
        500ms, std::bind(&TemperaturePublisher::timer_callback, this));
  }

private:
  void timer_callback() const {
    auto reading = sensor_->read();
    publisher_->publish(reading);
    RCLCPP_INFO(get_logger(), "read from %s and published to %s", get_name(),
                publisher_->get_topic_name());
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<Temperature::Msg>::SharedPtr publisher_;
  ISensor<Temperature::Msg>::SharedPtr sensor_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto temperature = Temperature::Msg();
  temperature.temperature = 5;
  auto temperature_sensor =
      std::make_shared<ConstantSensor<Temperature::Msg>>(temperature);
  auto temperature_publisher =
      std::make_shared<TemperaturePublisher>(temperature_sensor);
  rclcpp::spin(temperature_publisher);
  rclcpp::shutdown();
  return 0;
}
