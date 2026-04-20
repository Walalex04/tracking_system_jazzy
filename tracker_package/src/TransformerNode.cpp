#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <map>
#include <sstream>


#include <tracker_package/msg/robot_info.hpp>

using std::placeholders::_1;

class Transformer : public rclcpp::Node
{
public:
    struct TransformerData
    {
        std::map<int, rclcpp::Publisher<tracker_package::msg::RobotInfo>::SharedPtr> info_publishers;

        double x_offset;
        double y_offset;
        double angular_offset;
        double scale_factor;
    } transformer;

    Transformer() : Node("transformer_node")
    {
        // Subscriber
        sub_tracker_ = this->create_subscription<std_msgs::msg::UInt32MultiArray>(
            "tracker/positions_stamped",
            20,
            std::bind(&Transformer::transformerCallback, this, _1));

        // Parámetros
        this->declare_parameter("x_offset", 0.0);
        this->declare_parameter("y_offset", 0.0);
        this->declare_parameter("angular_offset", 0.0);
        this->declare_parameter("scale_factor", 1.0);

        transformer.x_offset = this->get_parameter("x_offset").as_double();
        transformer.y_offset = this->get_parameter("y_offset").as_double();
        transformer.angular_offset = this->get_parameter("angular_offset").as_double();
        transformer.scale_factor = this->get_parameter("scale_factor").as_double();

        // TF broadcaster
        br_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

private:
    void transformerCallback(const std_msgs::msg::UInt32MultiArray::SharedPtr msg)
    {
        int size = msg->data.size();

        if (size > 2)
        {
            for (int i = 2; i < size; i += 9)
            {
                int robot_id = msg->data.at(i);

                if (robot_id < 0)
                {
                    RCLCPP_WARN(this->get_logger(), "ID invalido: %d", robot_id);
                    continue;
                }

                // 🔹 Publisher dinámico
                if (transformer.info_publishers.count(robot_id) == 0)
                {
                    std::stringstream topic_info;
                    topic_info << "/epuck_" << robot_id << "/odom";

                    transformer.info_publishers[robot_id] =
                        this->create_publisher<tracker_package::msg::RobotInfo>(topic_info.str(), 10);

                    RCLCPP_INFO(this->get_logger(), "Creado publisher para robot %d", robot_id);
                }

                // =========================
                // 🔹 ODOMETRÍA
                // =========================
                nav_msgs::msg::Odometry robot_pose_msg;

                robot_pose_msg.header.frame_id = "camera_optical_frame";
                robot_pose_msg.header.stamp = this->now();

                std::stringstream child;
                child << "base_link_" << robot_id;
                robot_pose_msg.child_frame_id = child.str();

                std::vector<double> x(4), y(4);

                for (int j = 0; j < 4; ++j)
                {
                    int x_tracker = -msg->data.at(2 * j + 1 + i);
                    int y_tracker = msg->data.at(2 * j + 2 + i);

                    double x_tmp = (x_tracker - transformer.x_offset) * transformer.scale_factor;
                    double y_tmp = (y_tracker - transformer.y_offset) * transformer.scale_factor;

                    x[j] = x_tmp * cos(-transformer.angular_offset) - y_tmp * sin(-transformer.angular_offset);
                    y[j] = x_tmp * sin(-transformer.angular_offset) + y_tmp * cos(-transformer.angular_offset);
                }

                double pose_x = 0.0, pose_y = 0.0;
                for (int j = 0; j < 4; ++j)
                {
                    pose_x += x[j] / 4.0;
                    pose_y += y[j] / 4.0;
                }

                robot_pose_msg.pose.pose.position.x = pose_x;
                robot_pose_msg.pose.pose.position.y = pose_y;
                robot_pose_msg.pose.pose.position.z = 0.0;

                double dx = x[1] - x[0];
                double dy = y[1] - y[0];

                if (std::fabs(dx) < 1e-6 && std::fabs(dy) < 1e-6)
                {
                    RCLCPP_WARN(this->get_logger(), "Orientacion invalida en ID %d", robot_id);
                    continue;
                }

                double yaw = atan2(dy, dx);

                tf2::Quaternion q;
                q.setRPY(0, 0, yaw);
                robot_pose_msg.pose.pose.orientation = tf2::toMsg(q);

                // 🔹 TF
                geometry_msgs::msg::TransformStamped t;
                t.header.stamp = robot_pose_msg.header.stamp;
                t.header.frame_id = "camera_optical_frame";
                t.child_frame_id = robot_pose_msg.child_frame_id;

                t.transform.translation.x = pose_x;
                t.transform.translation.y = pose_y;
                t.transform.translation.z = 0.0;
                t.transform.rotation = robot_pose_msg.pose.pose.orientation;

                br_->sendTransform(t);

                // 🔹 RobotInfo
                tracker_package::msg::RobotInfo robot_info_msg;
                robot_info_msg.odom = robot_pose_msg;

                robot_info_msg.robotstate = tracker_package::msg::RobotInfo::WORKING;
                robot_info_msg.typework = tracker_package::msg::RobotInfo::NON_SPECIALIST;
                robot_info_msg.timework = 0.0;

                transformer.info_publishers[robot_id]->publish(robot_info_msg);
            }
        }
    }

    rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr sub_tracker_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> br_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Transformer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}