#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "../libs/Vicon_DataStreamSDK_Linux64/DataStreamClient.h"

using namespace std::chrono_literals;
using namespace ViconDataStreamSDK::CPP;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class MultiRobotPosePublisher : public rclcpp::Node
{

    Client vicon_client;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr robot1_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr robot2_publisher;
    rclcpp::TimerBase::SharedPtr timer;

    public:MultiRobotPosePublisher() : Node("multi_robot_pose_publisher")
    {   

        // Node Config
        robot1_publisher = this->create_publisher<geometry_msgs::msg::Pose>("robot1/pose", 10);
        robot2_publisher = this->create_publisher<geometry_msgs::msg::Pose>("robot2/pose", 10);
        timer = this->create_wall_timer(100ms, std::bind(&MultiRobotPosePublisher::timer_callback, this));

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

    // Destructor
    public:~MultiRobotPosePublisher()
    {
        this->vicon_client.Disconnect();
        RCLCPP_INFO(this->get_logger(), "Client Disconnected");
    }

    private: void timer_callback()
    {
        vicon_client.GetFrame();
        // Output_GetSubjectCount object_count = this->vicon_client.GetSubjectCount();
        // RCLCPP_INFO(this->get_logger(), "Number of objects: %i", object_count.SubjectCount);

        // Get Objects Name and Root Segment Name
        Output_GetSubjectName object1_name = this->vicon_client.GetSubjectName(0);
        Output_GetSubjectRootSegmentName object1_root_segment = this->vicon_client.GetSubjectRootSegmentName(object1_name.SubjectName);

        Output_GetSubjectName object2_name = this->vicon_client.GetSubjectName(1);
        Output_GetSubjectRootSegmentName object2_root_segment = this->vicon_client.GetSubjectRootSegmentName(object2_name.SubjectName);

        // RCLCPP_INFO(this->get_logger(), "Number of segments: %i", this->vicon_client.GetSegmentCount(object_name.SubjectName).SegmentCount);

        // Get objects Translation and Rotation
        Output_GetSegmentGlobalTranslation object1_translation = this->vicon_client.GetSegmentGlobalTranslation(object1_name.SubjectName, object1_root_segment.SegmentName);
        Output_GetSegmentGlobalRotationQuaternion object1_rotation = this->vicon_client.GetSegmentGlobalRotationQuaternion(object1_name.SubjectName, object1_root_segment.SegmentName);

        Output_GetSegmentGlobalTranslation object2_translation = this->vicon_client.GetSegmentGlobalTranslation(object2_name.SubjectName, object2_root_segment.SegmentName);
        Output_GetSegmentGlobalRotationQuaternion object2_rotation = this->vicon_client.GetSegmentGlobalRotationQuaternion(object2_name.SubjectName, object2_root_segment.SegmentName);


        // Set Pose Message
        geometry_msgs::msg::Pose pose_msg;

        pose_msg.position.set__x(object1_translation.Translation[0]/1000);
        pose_msg.position.set__y(object1_translation.Translation[1]/1000);
        pose_msg.position.set__z(object1_translation.Translation[2]/1000);

        pose_msg.orientation.set__x(object1_rotation.Rotation[0]);
        pose_msg.orientation.set__y(object1_rotation.Rotation[1]);
        pose_msg.orientation.set__z(object1_rotation.Rotation[2]);
        pose_msg.orientation.set__w(object1_rotation.Rotation[3]);

        robot1_publisher->publish(pose_msg);
        //////////////////////

        pose_msg.position.set__x(object2_translation.Translation[0]/1000);
        pose_msg.position.set__y(object2_translation.Translation[1]/1000);
        pose_msg.position.set__z(object2_translation.Translation[2]/1000);

        pose_msg.orientation.set__x(object2_rotation.Rotation[0]);
        pose_msg.orientation.set__y(object2_rotation.Rotation[1]);
        pose_msg.orientation.set__z(object2_rotation.Rotation[2]);
        pose_msg.orientation.set__w(object2_rotation.Rotation[3]);

        robot2_publisher->publish(pose_msg);

        // RCLCPP_INFO(this->get_logger(), "x: %f", object_translation.Translation[0]);
        // RCLCPP_INFO(this->get_logger(), "y: %f", object_translation.Translation[1]);
        // RCLCPP_INFO(this->get_logger(), "z: %f", object_translation.Translation[2]);
    }

};

int main(int argc, char * argv[])
{   

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MultiRobotPosePublisher>());
    rclcpp::shutdown();
    return 0;
    
}