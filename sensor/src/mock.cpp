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

template <typename T>
struct ISensor {
  using SharedPtr = std::shared_ptr<ISensor<T>>;

  virtual T read() = 0;
};

template <typename T>
class ConstantSensor : public ISensor<T> {
public:
  ConstantSensor(T value) : value_(value) {}
  ConstantSensor() : value_(T()) {}

  T read() { return value_; }

private:
  T value_;
};

class TemperaturePublisher : public rclcpp::Node {
public:
  TemperaturePublisher(ISensor<Temperature::Msg>::SharedPtr sensor)
      : Node("temperature_sensor"), sensor_(sensor) {
    publisher_ = this->create_publisher<Temperature::Msg>(Temperature::TopicName, 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&TemperaturePublisher::timer_callback, this));
  }

private:
  void timer_callback() {
    auto reading = sensor_->read();
    publisher_->publish(reading);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<Temperature::Msg>::SharedPtr publisher_;
  ISensor<Temperature::Msg>::SharedPtr sensor_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto temperature_sensor = std::make_shared<ConstantSensor<Temperature::Msg>>();
  auto temperature_publisher =
      std::make_shared<TemperaturePublisher>(temperature_sensor);
  rclcpp::spin(temperature_publisher);
  rclcpp::shutdown();
  return 0;
}
