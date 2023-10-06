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
        // std::vector<double> camera_matrix = {438.783367, 0.000000, 305.593336, 0.000000, 437.302876, 243.738352, 0.000000, 0.000000, 1.000000};
        // std::vector<double> distortion_coefficients = {-0.361976, 0.110510, 0.001014, 0.000505, 0.000000};
        // std::vector<double> rectification_matrix = {0.999978, 0.002789, -0.006046, -0.002816, 0.999986, -0.004401, 0.006034, 0.004417, 0.999972};
        // std::vector<double> projection_matrix = {393.653800, 0.000000, 322.797939, 0.000000, 0.000000, 393.653800, 241.090902, 0.000000, 0.000000, 0.000000, 1.000000, 0.00000};
        std::vector<double> d = {-0.3038463869640932, 
                                 0.07994522697622304, 
                                 0.0006346541981213411, 
                                 0.0006469544912017434, 
                                 0.0};

        std::vector<double> k = {373.15589595715227, 
                                 0.0, 
                                 378.71496990841257, 
                                 0.0, 
                                 373.4987811023343, 
                                 226.4124041469278, 
                                 0.0, 
                                 0.0, 
                                 1.0};

        std::vector<double> r = {0.9893448929616738, 
                                 0.0006427585452755401, 
                                 0.14558938708610178, 
                                 0.0004763014478010809, 
                                 0.9999706139956793, 
                                 -0.007651423529955466, 
                                 -0.14559002681360123, 
                                 0.007639241229101802, 
                                 0.9893155139215489};

        std::vector<double> p = {367.7567866068178,
                                 0.0, 
                                 196.61796951293945, 
                                 -136.85175603319885, 
                                 0.0, 
                                 367.7567866068178, 
                                 231.72658729553223, 
                                 0.0, 
                                 0.0, 
                                 0.0, 
                                 1.0, 
                                 0.0};

        this->declare_parameter("frame_id", "camera_link");
        this->declare_parameter("image_height", 480);
        this->declare_parameter("image_width", 640);
        this->declare_parameter("distortion_model", "plumb_bob");
        this->declare_parameter("D", d);
        this->declare_parameter("K", k);
        this->declare_parameter("R", r);
        this->declare_parameter("P", p);
        // this->declare_parameter("camera_matrix", camera_matrix);
        // this->declare_parameter("distortion_coefficients", distortion_coefficients);
        // this->declare_parameter("rectification_matrix", rectification_matrix);
        // this->declare_parameter("projection_matrix", projection_matrix);
        this->declare_parameter("x_offset", 0);

        frame_id_ =  this->get_parameter("frame_id").as_string();
        image_height_ =  this->get_parameter("image_height").as_int();
        image_width_ =  this->get_parameter("image_width").as_int();
        distortion_model_ = this->get_parameter("distortion_model").as_string();
        d = this->get_parameter("D").as_double_array();
        k = this->get_parameter("K").as_double_array();
        r = this->get_parameter("R").as_double_array();
        p = this->get_parameter("P").as_double_array();
        std::copy_n(k.begin(), 9UL, k_.begin());
        std::copy_n(r.begin(), 9UL, r_.begin());
        std::copy_n(p.begin(), 12UL, p_.begin());
        // camera_matrix_ = this->get_parameter("camera_matrix").as_double_array();
        // distortion_coefficients_ = this->get_parameter("distortion_coefficients").as_double_array();
        // rectification_matrix_ = this->get_parameter("rectification_matrix").as_double_array();
        // projection_matrix_ = this->get_parameter("projection_matrix").as_double_array();

        info_subscription_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
                             "camera_info_sub", 10, std::bind(&CamInfoPub::camera_info_callback,
                             this, std::placeholders::_1));

        info_publisher_ =  this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info_pub", 10);
    }

    private:
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_publisher_;
    sensor_msgs::msg::CameraInfo info_publish;
    std::string frame_id_;
    std::string distortion_model_;
    int image_height_;
    int image_width_;
    std::vector<double> d_;
    std::array<double, 9UL> k_;
    std::array<double, 9UL> r_;
    std::array<double, 12UL> p_;
    // std::vector<double> camera_matrix_;
    // std::vector<double> distortion_coefficients_;
    // std::vector<double> rectification_matrix_;
    // std::vector<double> projection_matrix_;

    void camera_info_callback(sensor_msgs::msg::CameraInfo::SharedPtr msg) const {
        sensor_msgs::msg::CameraInfo info_publish = sensor_msgs::msg::CameraInfo();
        
        info_publish.header.frame_id = frame_id_;
        info_publish.height = image_height_;
        info_publish.width = image_width_;
        info_publish.distortion_model = distortion_model_;
        info_publish.d = d_;
        info_publish.k = k_;
        info_publish.r = r_;
        info_publish.p = p_;
        
        info_publisher_->publish(info_publish);
    }

};