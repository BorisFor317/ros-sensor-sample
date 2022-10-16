#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor/sensor.hpp"

using std::placeholders::_1;

class SensorSubscriber : public rclcpp::Node
{
  public:
    SensorSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<Temperature::Msg>(
      Temperature::TopicName, 10, std::bind(&SensorSubscriber::topic_callback, this, _1));
    }

  private:
    rclcpp::Subscription<Temperature::Msg>::SharedPtr subscription_;

    void topic_callback(const Temperature::Msg::SharedPtr msg) const
    {
      RCLCPP_INFO(get_logger(), "I heard temperature: %f", msg->temperature);
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorSubscriber>());
  rclcpp::shutdown();
  return 0;
}
