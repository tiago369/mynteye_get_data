#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>
#include <iostream>
#include <vector>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "rclcpp/rclcpp.hpp"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <geometry_msgs/msg/quaternion.hpp>

class ArucoMean : public rclcpp::Node
{
    public:
    ArucoMean()
    : Node("aruco_mean")
    {
        std::vector<long int> ids = {0, 1, 2, 3};
        std::vector<double> left_offset = {0.0, 0.0, 0.0};
        std::vector<double> right_offset = {0.0, 0.0, 0.0};

        this->declare_parameter("filename", "file.csv");
        this->declare_parameter("left_offset", left_offset);
        this->declare_parameter("right_offset", right_offset);
        this->declare_parameter("aruco_ids", ids);
        this->declare_parameter("buffer", 10);

        filename_ = this->get_parameter("filename").as_string();
        left_offset_ = this->get_parameter("left_offset").as_double_array();
        right_offset_ = this->get_parameter("right_offset").as_double_array();
        ids_ = this->get_parameter("aruco_ids").as_integer_array();
        buffer_size_ = this->get_parameter("buffer").as_int();

        left_subscription_ = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
            "left_topic", 10, std::bind(&ArucoMean::left_aruco_subscriber,
            this, std::placeholders::_1));
        right_subscription_ = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
            "right_topic", 10, std::bind(&ArucoMean::right_aruco_subscriber,
            this, std::placeholders::_1));
        pose_publisher_ = this->create_publisher<ros2_aruco_interfaces::msg::ArucoMarkers>("pose", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
            std::bind(&ArucoMean::mean_pose, this));

    }

    private:

    void quaternionToRPY(geometry_msgs::msg::Pose pose, double& roll, double& pitch, double& yaw) {
        // Extract quaternion components from the Pose
        double qx = pose.orientation.x;
        double qy = pose.orientation.y;
        double qz = pose.orientation.z;
        double qw = pose.orientation.w;

        // Roll (x-axis rotation)
        double sinr_cosp = 2.0 * (qw * qx + qy * qz);
        double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
        roll = std::atan2(sinr_cosp, cosr_cosp);

        // Pitch (y-axis rotation)
        double sinp = 2.0 * (qw * qy - qz * qx);
        if (std::abs(sinp) >= 1)
            pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        else
            pitch = std::asin(sinp);

        // Yaw (z-axis rotation)
        double siny_cosp = 2.0 * (qw * qz + qx * qy);
        double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
        yaw = std::atan2(siny_cosp, cosy_cosp);
}

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

    ros2_aruco_interfaces::msg::ArucoMarkers organize_markers(ros2_aruco_interfaces::msg::ArucoMarkers arucoMarkersMsg) {
        // Estrutura auxiliar para manter o índice original e o identificador do marcador
        std::vector<std::pair<int, int>> indexIdPairs;

        for (int i = 0; i < static_cast<int>(arucoMarkersMsg.marker_ids.size()); i++) {
            indexIdPairs.push_back(std::make_pair(i, arucoMarkersMsg.marker_ids[i]));
        }

        // Ordena com base no identificador do marcador
        std::sort(indexIdPairs.begin(), indexIdPairs.end(), 
                [](const std::pair<int, int> &a, const std::pair<int, int> &b) {
                    return a.second < b.second;
                });

        // Cria cópias temporárias das listas originais
        auto markersOriginal = arucoMarkersMsg.marker_ids;
        auto posesOriginal = arucoMarkersMsg.poses;

        // Reorganiza as listas com base na nova ordem
        for (int i = 0; i < static_cast<int>(indexIdPairs.size()); i++) {
            arucoMarkersMsg.marker_ids[i] = markersOriginal[indexIdPairs[i].first];
            arucoMarkersMsg.poses[i] = posesOriginal[indexIdPairs[i].first];
        }

        return arucoMarkersMsg;
    }

    void left_aruco_subscriber(const ros2_aruco_interfaces::msg::ArucoMarkers & msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (areVectorsEqual(ids_, msg.marker_ids)) {
            RCLCPP_INFO(this->get_logger(), "GOT LEFT");
            ros2_aruco_interfaces::msg::ArucoMarkers
            new_msg = organize_markers(msg);
            if (static_cast<int>(left_aruco_.size()) <= buffer_size_) {
                left_aruco_.push_back(new_msg);
            } else {
                left_aruco_.erase(left_aruco_.begin());
                left_aruco_.push_back(new_msg);
            }
        }
    }

    void right_aruco_subscriber(const ros2_aruco_interfaces::msg::ArucoMarkers & msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (areVectorsEqual(ids_, msg.marker_ids)) {
            RCLCPP_INFO(this->get_logger(), "GOT RIGHT");
            ros2_aruco_interfaces::msg::ArucoMarkers
            new_msg = organize_markers(new_msg);
            if (static_cast<int>(right_aruco_.size()) <= buffer_size_) {
                right_aruco_.push_back(new_msg);
            } else {
                right_aruco_.erase(right_aruco_.begin());
                right_aruco_.push_back(new_msg);
            }
        }
    }

    ros2_aruco_interfaces::msg::ArucoMarkers vector_mean(const std::vector<ros2_aruco_interfaces::msg::ArucoMarkers> aruco) {
        // Assuming all ArucoMarkers in the input vector have the same size
        size_t numMarkers = aruco[0].marker_ids.size();

        ros2_aruco_interfaces::msg::ArucoMarkers result;
        result.marker_ids.resize(numMarkers);
        result.poses.resize(numMarkers);

        // Iterate over each marker
        for (size_t j = 0; j < numMarkers; ++j) {
            // Initialize the sum for each component of the position and orientation
            double sumX = 0.0;
            double sumY = 0.0;
            double sumZ = 0.0;
            double sumRoll = 0.0;
            double sumPitch = 0.0;
            double sumYaw = 0.0;

            // Iterate over each ArucoMarkers message
            for (const auto& arucoMsg : aruco) {
                // Accumulate the position components
                sumX += arucoMsg.poses[j].position.x;
                sumY += arucoMsg.poses[j].position.y;
                sumZ += arucoMsg.poses[j].position.z;

                // Convert quaternion to RPY
                tf2::Quaternion quat;
                tf2::fromMsg(arucoMsg.poses[j].orientation, quat);
                double roll, pitch, yaw;
                tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

                // Accumulate the RPY components
                sumRoll += roll;
                sumPitch += pitch;
                sumYaw += yaw;
            }

            // Calculate the mean position components
            result.poses[j].position.x = sumX / aruco.size();
            result.poses[j].position.y = sumY / aruco.size();
            result.poses[j].position.z = sumZ / aruco.size();

            // Calculate the mean RPY components
            double meanRoll = sumRoll / aruco.size();
            double meanPitch = sumPitch / aruco.size();
            double meanYaw = sumYaw / aruco.size();

            // Convert mean RPY back to quaternion
            tf2::Quaternion meanQuat;
            meanQuat.setRPY(meanRoll, meanPitch, meanYaw);

            // Convert quaternion to geometry_msgs::msg::Quaternion
            result.poses[j].orientation = tf2::toMsg(meanQuat);

            // Assuming marker_ids are the same for all ArucoMarkers messages
            result.marker_ids[j] = aruco[0].marker_ids[j];
        }

        return result;
    }

    ros2_aruco_interfaces::msg::ArucoMarkers aruco_mean(const ros2_aruco_interfaces::msg::ArucoMarkers one,
                                                        const ros2_aruco_interfaces::msg::ArucoMarkers two) {
    ros2_aruco_interfaces::msg::ArucoMarkers result;

    for (size_t i = 0; i < one.marker_ids.size(); ++i) {
        result.marker_ids.push_back(one.marker_ids[i]);

        result.poses.push_back(geometry_msgs::msg::Pose());
        result.poses[i].position.x = (one.poses[i].position.x + two.poses[i].position.x) / 2;
        result.poses[i].position.y = (one.poses[i].position.y + two.poses[i].position.y) / 2;
        result.poses[i].position.z = (one.poses[i].position.z + two.poses[i].position.z) / 2;

        // Convert quaternions to tf2::Quaternion for mean calculation
        tf2::Quaternion one_quat, two_quat, mean_quat;
        tf2::fromMsg(one.poses[i].orientation, one_quat);
        tf2::fromMsg(two.poses[i].orientation, two_quat);

        // Calculate the mean quaternion
        mean_quat = tf2::slerp(one_quat, two_quat, 0.5);

        // Convert mean quaternion to geometry_msgs::msg::Quaternion
        result.poses[i].orientation = tf2::toMsg(mean_quat);
    }

    return result;
}

    ros2_aruco_interfaces::msg::ArucoMarkers add_offset(ros2_aruco_interfaces::msg::ArucoMarkers aruco, std::vector<double> offset){

        for(int j=0; j < static_cast<int>(aruco.marker_ids.size()); j++){
            aruco.poses.at(j).position.x += offset.at(0);
            aruco.poses.at(j).position.y += offset.at(1);
            aruco.poses.at(j).position.z += offset.at(2);
            }

        return aruco;
    }

    void mean_pose() {
        //Check if list is full
        while (static_cast<int>(left_aruco_.size()) <= buffer_size_ &&
               static_cast<int>(right_aruco_.size()) <= buffer_size_) {
            RCLCPP_INFO_ONCE(this->get_logger(), "Collecting aruco data...");
            return;
        }
        //Check if list are the same size
        if (left_aruco_.size() != right_aruco_.size()){
            RCLCPP_WARN(this->get_logger(), "DIFERENT SIZES");
            return;
        }

        // Check if timestamp is correct
        mutex_.lock();
        if(left_aruco_.at(0).header.stamp.sec != right_aruco_.at(0).header.stamp.sec){
            RCLCPP_WARN(this->get_logger(), "Images are not synchronized");
            left_aruco_.clear();
            right_aruco_.clear();
            
            mutex_.unlock();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Calculating pose");
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
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<ros2_aruco_interfaces::msg::ArucoMarkers> left_aruco_{};
    std::vector<ros2_aruco_interfaces::msg::ArucoMarkers> right_aruco_{};
    int buffer_size_;
    std::string filename_;
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