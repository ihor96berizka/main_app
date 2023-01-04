#include "main_node.hpp"

int main(int argc, char * argv[])
{
  std::cout << "Waiting for data......\n";
  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr main_node = std::make_shared<MainSwcNode>();
  rclcpp::spin(main_node);
  rclcpp::shutdown();
  return 0;
}