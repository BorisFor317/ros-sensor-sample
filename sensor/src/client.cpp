#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor/sensor.hpp"
#include "sensor_msgs/msg/temperature.hpp"

using std::placeholders::_1;

// class SensorSubscriber : public rclcpp::Node
// {
//   public:
//     SensorSubscriber()
//     : Node("minimal_subscriber")
//     {
//       subscription_ = create_subscription<Temperature::Msg>(
//       Temperature::TopicName, 10,
//       std::bind(&SensorSubscriber::topic_callback, this, _1));
//     }

//   private:
//     rclcpp::Subscription<Temperature::Msg>::SharedPtr subscription_;

//     void topic_callback(const Temperature::Msg::SharedPtr msg) const
//     {
//       RCLCPP_INFO(get_logger(), "I heard temperature: %f", msg->temperature);
//     }
// };

template <typename TMessage>
class SensorRosReader : public rclcpp::Node, public ISensor<TMessage> {
public:
  explicit SensorRosReader(std::string node_name) : Node(node_name) {
    subscription_ = create_subscription<TMessage>(
        Temperature::TopicName, 10,
        std::bind(&SensorRosReader::reading_callback, this, _1));
  }

  TMessage read() override { return latest_; }

private:
  typename rclcpp::Subscription<TMessage>::SharedPtr subscription_;
  TMessage latest_;

  void reading_callback(const std::shared_ptr<TMessage> msg) { latest_ = *msg; }
};

using TemperatureRosReader = SensorRosReader<Temperature::Msg>;

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TemperatureRosReader>("temperature_reader"));
  rclcpp::shutdown();
  return 0;
}
