#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

using std::placeholders::_1;

class CamInfoPub : public rclcpp::Node
{
    public:
    CamInfoPub()
    : Node("minimal_publisher")
    {
        std::vector<double> camera_matrix = {438.783367, 0.000000, 305.593336, 0.000000, 437.302876, 243.738352, 0.000000, 0.000000, 1.000000};
        std::vector<double> distortion_coefficients = {-0.361976, 0.110510, 0.001014, 0.000505, 0.000000};
        std::vector<double> rectification_matrix = {0.999978, 0.002789, -0.006046, -0.002816, 0.999986, -0.004401, 0.006034, 0.004417, 0.999972};
        std::vector<double> projection_matrix = {393.653800, 0.000000, 322.797939, 0.000000, 0.000000, 393.653800, 241.090902, 0.000000, 0.000000, 0.000000, 1.000000, 0.00000};

        this->declare_parameter("frame_id", "camera_link");
        this->declare_parameter("image_height", 480);
        this->declare_parameter("image_width", 640);
        this->declare_parameter("camera_matrix", camera_matrix);
        this->declare_parameter("distortion_model", "plumb_bob");
        this->declare_parameter("distortion_coefficients", distortion_coefficients);
        this->declare_parameter("rectification_matrix", rectification_matrix);
        this->declare_parameter("projection_matrix", projection_matrix);
        this->declare_parameter("x_offset", 0);

        frame_id_ =  this->get_parameter("frame_id").as_string();
        image_height =  this->get_parameter("image_height").as_int();
        image_width =  this->get_parameter("image_width").as_int();
        // this->declare_parameter("");

        info_subscription_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
                             "camera_info_sub", 10, std::bind(&CamInfoPub::camera_info_callback,
                             this, std::placeholders::_1));

        info_publisher_ =  this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info_pub", 10);
    }

    private:
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_publisher_;
    sensor_msgs::msg::CameraInfo::UniquePtr info_publish;
    std::string frame_id_;
    int image_height;
    int image_width;

    void camera_info_callback(sensor_msgs::msg::CameraInfo::SharedPtr msg) const {
        info_publish.reset(new sensor_msgs::msg::CameraInfo);
        
        // msg.get()->header.frame_id = this->get_parameter("frame_id")
        info_publish.header.frame_id = frame_id_;
        
        info_publisher_->publish(info_publish);
    }

};