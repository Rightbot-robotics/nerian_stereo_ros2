#include <iostream>
#include <exception>
#include <stdio.h>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/int32.hpp"

#include <visiontransfer/deviceenumeration.h>
#include <visiontransfer/imagetransfer.h>
#include <visiontransfer/imageset.h>
#include <visiontransfer/deviceparameters.h>


float mapAngleToHeight(float angle, float angleMin, float angleMax, float heightMin, float heightMax) {
            // Check for invalid input (min > max)
            if (angleMin >= angleMax || heightMin >= heightMax) {
                throw std::invalid_argument("Invalid min/max values. Min must be less than Max.");
            }

            // Normalize the angle value between 0 and 1
            float normalizedAngle = (angle - angleMin) / (angleMax - angleMin);

            // Map the normalized angle to the desired height range
            float mappedHeight = normalizedAngle * (heightMax - heightMin) + heightMin;

            return mappedHeight;
            }

class JointStateSubscriber : public rclcpp::Node
{
    public:
        JointStateSubscriber() : Node("joint_state_subscriber")
        {
            // Search for Nerian stereo devices
            visiontransfer::DeviceEnumeration deviceEnum;
            visiontransfer::DeviceEnumeration::DeviceList devices = deviceEnum.discoverDevices();
            if(devices.size() == 0) {
                RCLCPP_ERROR(this->get_logger(), "No devices discovered!");
                return;
            }

            // Print discovered devices
            RCLCPP_INFO(this->get_logger(), "Discovered devices:");
            for(unsigned int i = 0; i < devices.size(); i++) {
                RCLCPP_INFO(this->get_logger(), "%s", devices[i].toString().c_str());
            }

            // Initialize the stereo camera parameters directly
            parameters_ = std::make_shared<visiontransfer::DeviceParameters>(devices[0]);
            // Subscribe to joint states topic
            subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "joint_states", 10, std::bind(&JointStateSubscriber::jointStateCallback, this, std::placeholders::_1));
            publisher_ = this->create_publisher<std_msgs::msg::Int32>("roi", 10);
        }

        void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
        {
            for (size_t i = 0; i < msg->name.size(); ++i)
            {
                if (msg->name[i] == "DC_hinge_joint")
                {
                    try {
                        // Set auto-exposure ROI
                        // max: -400, min: 150 | h_min: 0 h_max: 550 and y = 150-h
                        int h = mapAngleToHeight(msg->position[i], -0.23, 0.23, 0, 550);
                        std_msgs::msg::Int32 message;
                        if (abs(prev_angle - msg->position[i]) < 0.05) {
                            return;
                        }
                        prev_angle = msg->position[i];
                        parameters_->setAutoROI(50, 150-h, 650, 100);
                        message.data = 150-h;
                        RCLCPP_INFO(this->get_logger(), "Position of DC_hinge_joint: %f", msg->position[i]);
                        RCLCPP_INFO(this->get_logger(), "Setting auto-exposure ROI...%d", message.data);
                        publisher_->publish(message);
                    } catch(const std::exception& ex) {
                        RCLCPP_ERROR(this->get_logger(), "Exception occurred while setting auto-exposure ROI: %s", ex.what());
                    }
                    break;
                }
            }
        }

        
    private:
        double prev_angle;
        std::shared_ptr<visiontransfer::DeviceParameters> parameters_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
        
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointStateSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
