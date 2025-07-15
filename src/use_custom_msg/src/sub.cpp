#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "custom_msg/msg/age.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("msg_sub")
    {
      subscription_ = this->create_subscription<custom_msg::msg::Age>(
      "new_msg", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:

    void topic_callback(const custom_msg::msg::Age & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%d', '%d','%d'", msg.day, msg.month, msg.year);
    }
    rclcpp::Subscription<custom_msg::msg::Age>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}