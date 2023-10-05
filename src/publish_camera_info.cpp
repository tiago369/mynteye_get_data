#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

class PublishCameraInfo : public rclcpp::Node
{
    public:
    PublishCameraInfo();

    private:
    void camera_publisher();
    
    void camera_callback();
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PublishCameraInfo>());
  rclcpp::shutdown();
  return 0;
}