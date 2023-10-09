#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "geometry_msgs/msg/pose.hpp"

class ArucoMean : public rclcpp::Node
{
    public:
    ArucoMean()
    : Node("aruco_mean")
    {
        std::vector<long int> ids = {0,1,2,3};
        std::vector<double> left_offset = {0.0,0.0,0.0};
        std::vector<double> right_offset = {0.0,0.0,0.0};

        this->declare_parameter("yaml_name", "yaml.yaml");
        this->declare_parameter("left_offset", left_offset);
        this->declare_parameter("right_offset", right_offset);
        this->declare_parameter("aruco_ids", ids);

        yaml_name_ = this->get_parameter("yaml_name").as_string();
        left_offset_ = this->get_parameter("left_offset").as_double_array();
        right_offset_ = this->get_parameter("right_offset").as_double_array();
        ids_ = this->get_parameter("aruco_ids").as_integer_array();

        left_subscription_ = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
            "left_topic", 10,std::bind(&ArucoMean::left_aruco_subscriber,
            this, std::placeholders::_1));
        right_subscription_ = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
            "right_topic", 10,std::bind(&ArucoMean::right_aruco_subscriber,
            this, std::placeholders::_1));
        pose_publisher_ = this->create_publisher<ros2_aruco_interfaces::msg::ArucoMarkers>("pose", 10);
    }

    private:

    bool areVectorsEqual(const std::vector<long int> vec1, const std::vector<long int> vec2) {
        // Check if the sizes are different
        if (vec1.size() != vec2.size()) {
            return false;
        }

        // Sort them
        std::vector<long int> sortedVec1 = vec1;
        std::vector<long int> sortedVec2 = vec2;

        std::sort(sortedVec1.begin(), sortedVec1.end());
        std::sort(sortedVec2.begin(), sortedVec2.end());

        // Compare the sorted vectors element-wise
        return sortedVec1 == sortedVec2;
    }

    void left_aruco_subscriber(const ros2_aruco_interfaces::msg::ArucoMarkers & msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if(areVectorsEqual(ids_, msg.marker_ids)){
            if (static_cast<int>(left_aruco_.size()) <= buffer_size_) {
                left_aruco_.push_back(msg);
            } else {
                left_aruco_.erase(left_aruco_.begin());
                left_aruco_.push_back(msg);
            }
        }
    }

    void right_aruco_subscriber(const ros2_aruco_interfaces::msg::ArucoMarkers & msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if(areVectorsEqual(ids_, msg.marker_ids)){
            if (static_cast<int>(right_aruco_.size()) <= buffer_size_) {
                right_aruco_.push_back(msg);
            } else {
                right_aruco_.erase(right_aruco_.begin());
                right_aruco_.push_back(msg);
            }
        }
    }

    ros2_aruco_interfaces::msg::ArucoMarkers vector_mean(std::vector<ros2_aruco_interfaces::msg::ArucoMarkers> aruco){
        ros2_aruco_interfaces::msg::ArucoMarkers result = ros2_aruco_interfaces::msg::ArucoMarkers();

        for(int j=0; j < aruco.at(0).marker_ids.size(); j++){
            result.marker_ids = aruco.at(j).marker_ids;
            for(int i=0; i < aruco.size(); i++){            
                result.poses.at(j).position.x += aruco.at(i).poses.at(j).position.x;
                result.poses.at(j).position.y += aruco.at(i).poses.at(j).position.y;
                result.poses.at(j).position.z += aruco.at(i).poses.at(j).position.z;
            }
        result.poses.at(j).position.x = result.poses.at(j).position.x / aruco.size();
        result.poses.at(j).position.y = result.poses.at(j).position.y / aruco.size();
        result.poses.at(j).position.z = result.poses.at(j).position.z / aruco.size();
        }

        return result;
    }

    ros2_aruco_interfaces::msg::ArucoMarkers aruco_mean(ros2_aruco_interfaces::msg::ArucoMarkers one,
                                  ros2_aruco_interfaces::msg::ArucoMarkers two) {
        ros2_aruco_interfaces::msg::ArucoMarkers result = ros2_aruco_interfaces::msg::ArucoMarkers();
        for(int i=0; i < static_cast<int>(one.marker_ids.size()); i++){
            result.poses.at(i).position.x = (one.poses.at(i).position.x + two.poses.at(i).position.x)/2;
            result.poses.at(i).position.y = (one.poses.at(i).position.y + two.poses.at(i).position.y)/2;
            result.poses.at(i).position.z = (one.poses.at(i).position.z + two.poses.at(i).position.z)/2;
        }
        return result;
    }

    ros2_aruco_interfaces::msg::ArucoMarkers add_offset(ros2_aruco_interfaces::msg::ArucoMarkers aruco, std::vector<double> offset){
        ros2_aruco_interfaces::msg::ArucoMarkers result = ros2_aruco_interfaces::msg::ArucoMarkers();
        // result.marker_ids = aruco.at(0).marker_ids;
        for(int j=0; j < static_cast<int>(aruco.marker_ids.size()); j++){
            result.poses.at(j).position.x += offset.at(0);
            result.poses.at(j).position.y += offset.at(1);
            result.poses.at(j).position.z += offset.at(2);
            }

        return result;
    }

    void mean_pose() {
        
        //Check if list is full
        while (static_cast<int>(left_aruco_.size()) < buffer_size_ &&
               static_cast<int>(right_aruco_.size()) < buffer_size_) {
            RCLCPP_INFO_ONCE(this->get_logger(), "Collecting aruco data...");
            return;
        }

        // Check if timestamp is correct
        mutex_.lock();
        if(left_aruco_.at(0).header.stamp.sec != right_aruco_.at(0).header.stamp.sec){
            left_aruco_.clear();
            right_aruco_.clear();
            mutex_.unlock();
            return;
        }
        // Calculate the mean off the points
        ros2_aruco_interfaces::msg::ArucoMarkers pose;
        pose = aruco_mean(add_offset(vector_mean(left_aruco_), left_offset_),
                          add_offset(vector_mean(right_aruco_), right_offset_));
        mutex_.unlock();
        pose_publisher_->publish(pose);

        return;
    }

    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr left_subscription_;
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr right_subscription_;
    rclcpp::Publisher<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr pose_publisher_;
    std::vector<ros2_aruco_interfaces::msg::ArucoMarkers> left_aruco_{};
    std::vector<ros2_aruco_interfaces::msg::ArucoMarkers> right_aruco_{};
    int buffer_size_{50};
    std::string yaml_name_;
    std::mutex mutex_;
    std::vector<long int> ids_;
    std::vector<double> left_offset_;
    std::vector<double> right_offset_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoMean>());
  rclcpp::shutdown();
  return 0;
}