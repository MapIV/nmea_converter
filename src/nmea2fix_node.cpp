
#include <nmea2fix/nmea2fix.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nmea2Fix>());
  rclcpp::shutdown();

  return 0;
}
