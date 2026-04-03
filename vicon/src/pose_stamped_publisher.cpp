#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"  // <-- CAMBIO

#include "../libs/Vicon_DataStreamSDK_Linux64/DataStreamClient.h"

using namespace std::chrono_literals;
using namespace ViconDataStreamSDK::CPP;

class PosePublisher : public rclcpp::Node
{

    Client vicon_client;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher; // <-- CAMBIO
    rclcpp::TimerBase::SharedPtr timer;

public:
    PosePublisher() : Node("pose_publisher")
    {   
        // Node Config
        publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("robot1/pose", 10); // <-- CAMBIO
        timer = this->create_wall_timer(10ms, std::bind(&PosePublisher::timer_callback, this));

        // Vicon DataStream Version
        Output_GetVersion output = vicon_client.GetVersion();
        RCLCPP_INFO(this->get_logger(), ("Vicon DataStream SDK: " + std::to_string(output.Major) + "." + std::to_string(output.Minor) + "." + std::to_string(output.Point)).c_str());
        
        // Connect to Vicon Host
        vicon_client.Connect("192.168.10.2:801");
        Output_IsConnected is_connected = vicon_client.IsConnected();

        if (is_connected.Connected)
        {
            RCLCPP_INFO(this->get_logger(), "The Client is Connected");
        }
        else 
        {
            RCLCPP_INFO(this->get_logger(), "Could not Connect the Client");
            return;
        }

        vicon_client.SetStreamMode(StreamMode::ClientPull);
        vicon_client.EnableSegmentData();
    }

    ~PosePublisher()
    {
        this->vicon_client.Disconnect();
        RCLCPP_INFO(this->get_logger(), "Client Disconnected");
    }

private:
    void timer_callback()
    {
        vicon_client.GetFrame();

        Output_GetSubjectName object_name = this->vicon_client.GetSubjectName(0);
        Output_GetSubjectRootSegmentName object_root_segment = this->vicon_client.GetSubjectRootSegmentName(object_name.SubjectName);

        Output_GetSegmentGlobalTranslation object_translation =
            this->vicon_client.GetSegmentGlobalTranslation(object_name.SubjectName, object_root_segment.SegmentName);

        Output_GetSegmentGlobalRotationQuaternion object_rotation =
            this->vicon_client.GetSegmentGlobalRotationQuaternion(object_name.SubjectName, object_root_segment.SegmentName);

        // Set PoseStamped Message
        geometry_msgs::msg::PoseStamped pose_msg;

        // HEADER (esto es lo que te faltaba)
        pose_msg.header.stamp = this->get_clock()->now();
        pose_msg.header.frame_id = "vicon_world";

        // POSE
        pose_msg.pose.position.x = object_translation.Translation[0];
        pose_msg.pose.position.y = object_translation.Translation[1];
        pose_msg.pose.position.z = object_translation.Translation[2];

        pose_msg.pose.orientation.x = object_rotation.Rotation[0];
        pose_msg.pose.orientation.y = object_rotation.Rotation[1];
        pose_msg.pose.orientation.z = object_rotation.Rotation[2];
        pose_msg.pose.orientation.w = object_rotation.Rotation[3];

        publisher->publish(pose_msg);
    }
};

int main(int argc, char * argv[])
{   
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PosePublisher>());
    rclcpp::shutdown();
    return 0;
}