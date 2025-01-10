#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include <cmath>
#include <chrono>


using namespace std::chrono_literals;

class JointPublisher : public rclcpp::Node
{
public:
    JointPublisher()
        : Node("joint_publisher"), count_(0) // Initialize count_
    {
        // Create the publisher
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/pan_tilt_joint_trajectory_controller/joint_trajectory", 10);

        // Create the timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&JointPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = trajectory_msgs::msg::JointTrajectory();
        // Add names of the joints you want to control
        message.joint_names.push_back("pan_platform_joint");
        message.joint_names.push_back("tilt_platform_joint");

        auto point = trajectory_msgs::msg::JointTrajectoryPoint();
        double position1 = 1.5 * (1 - cos(count_ * 0.1));
        double position2 = position1 - 0.78;
        point.positions.push_back(position1);
        point.positions.push_back(position2);
        point.time_from_start = rclcpp::Duration::from_seconds(1.0);
        message.points.push_back(point);

        // Publish the trajectory message
        publisher_->publish(message);

        RCLCPP_INFO(this->get_logger(), "Publishing positions: '%f', '%f'", position1, position2);

        count_ += 1; // Increment count_
    }

    rclcpp::TimerBase::SharedPtr timer_; // Timer declaration
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_; // Publisher declaration
    size_t count_; // Declare count_ as a private member variable
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointPublisher>());
    rclcpp::shutdown();
    return 0;
}





// Different way to do it below

// #include <rclcpp/rclcpp.hpp>
// #include <trajectory_msgs/msg/joint_trajectory.hpp>
// #include <trajectory_msgs/msg/joint_trajectory_point.hpp>
// #include <string>
// #include <vector>
// #include <chrono>

// using namespace std::chrono_literals;

// class JointPublisher : public rclcpp::Node
// {
// public:
//     JointPublisher(const std::string &node_name)
//         : Node(node_name)
//     {
//         // Declare and get parameters for joint names
//         this->declare_parameter<std::vector<std::string>>("joint_names", {"pan_platform_joint", "tilt_platform_joint"});
//         joint_names_ = this->get_parameter("joint_names").as_string_array();

//         // Validate joint names
//         if (joint_names_.empty())
//         {
//             RCLCPP_ERROR(this->get_logger(), "No joint names provided. Exiting...");
//             rclcpp::shutdown();
//             return;
//         }

//         // Create the publisher for the trajectory
//         trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
//             "/joint_trajectory", 10);

//         // Set a timer to send trajectories
//         timer_ = this->create_wall_timer(1s, std::bind(&JointPublisher::sendTrajectory, this));

//         RCLCPP_INFO(this->get_logger(), "JointPublisher node initialized.");
//     }

// private:
//     void sendTrajectory()
//     {
//         if (joint_names_.empty())
//         {
//             RCLCPP_ERROR(this->get_logger(), "No joint names specified. Cannot send trajectory.");
//             return;
//         }

//         // Create the JointTrajectory message
//         trajectory_msgs::msg::JointTrajectory msg;
//         msg.header.stamp = this->now();
//         msg.joint_names = joint_names_;

//         // Define a trajectory point
//         trajectory_msgs::msg::JointTrajectoryPoint point;
//         point.positions = {1.0, -0.5};  // Desired joint positions
//         point.velocities = {0.5, 0.5}; // Desired joint velocities (optional)
//         point.time_from_start = rclcpp::Duration(2s); // Time to reach this point

//         // Add the point to the trajectory
//         msg.points.push_back(point);

//         // Publish the message
//         RCLCPP_INFO(this->get_logger(), "Publishing trajectory for joints: [%s, %s]",
//                     joint_names_[0].c_str(), joint_names_[1].c_str());
//         trajectory_pub_->publish(msg);
//     }

//     std::vector<std::string> joint_names_; // Names of the joints
//     rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
//     rclcpp::TimerBase::SharedPtr timer_;
// };

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);

//     // Create the node
//     auto node = std::make_shared<JointPublisher>("joint_publisher");

//     // Spin the node
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }
