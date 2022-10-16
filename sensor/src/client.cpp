// #include <memory>

// #include "rclcpp/rclcpp.hpp"
// #include "sensor_msgs/msg/temperature.hpp"
// #include "sensor.hpp"

// using std::placeholders::_1;

// class SensorSubscriber : public rclcpp::Node
// {
//   public:
//     SensorSubscriber()
//     : Node("minimal_subscriber")
//     {
//       subscription_ = this->create_subscription<TemperatureMsg>(
//       "topic", 10, std::bind(&SensorSubscriber::topic_callback, this, _1));
//     }

//   private:
//     using TemperatureMsg = sensor_msgs::msg::Temperature;
//     rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

//     void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
//     {
//       RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
//     }
// };

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<SensorSubscriber>());
//   rclcpp::shutdown();
//   return 0;
// }
