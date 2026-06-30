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
#include <visualization_msgs/msg/marker.hpp>

using std::placeholders::_1;

class Transformer : public rclcpp::Node
{
public:
    struct TransformerData
    {
        std::map<int, rclcpp::Publisher<tracker_package::msg::RobotInfo>::SharedPtr> info_publishers;
        double x_offset, y_offset, angular_offset, scale_factor;
    } transformer;

    Transformer() : Node("transformer_node")
    {

        this->declare_parameter("x_offset", 0.0);
        this->declare_parameter("y_offset", 0.0);
        this->declare_parameter("angular_offset", 0.0);
        this->declare_parameter("scale_factor", 1.0);

        this->get_parameter("x_offset", transformer.x_offset);
        this->get_parameter("y_offset", transformer.y_offset);
        this->get_parameter("angular_offset", transformer.angular_offset);
        this->get_parameter("scale_factor", transformer.scale_factor);

        this->declare_parameter<std::vector<std::string>>("robots", std::vector<std::string>());
        this->declare_parameter<std::vector<int64_t>>("id_robots", std::vector<int64_t>());

        robots_ = this->get_parameter("robots").as_string_array();
        id_robots_ = this->get_parameter("id_robots").as_integer_array();

        RCLCPP_INFO(this->get_logger(), "Parametros cargados: X:%f, Y:%f, Scale:%f",
                    transformer.x_offset, transformer.y_offset, transformer.scale_factor);

        // En el constructor:
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("tracker/robot_markers", 10);

        sub_tracker_ = this->create_subscription<std_msgs::msg::UInt32MultiArray>(
            "tracker/positions_stamped", 20, std::bind(&Transformer::transformerCallback, this, _1));

        br_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

private:
    void transformerCallback(const std_msgs::msg::UInt32MultiArray::SharedPtr msg)
    {
        // Header: 2 elementos (sec, nsec).
        // Cada robot: 1 (ID) + 8 (4 corners x,y) + 1 (ColorCode) = 10 elementos.
        int size = static_cast<int>(msg->data.size());

        // Validar: Header (2) + al menos un bloque de robot completo (10)
        if (size < 12)
            return;

        for (int i = 2; i < size; i += 10)
        {
            // Seguridad: verificar que el bloque completo de 10 elementos exista
            if (i + 9 >= size)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "Datos recibidos incompletos. Tamaño esperado vs real: %d", size);
                break;
            }

            int robot_id = static_cast<int64_t>(msg->data.at(i));
            int color_code = static_cast<int>(msg->data.at(i + 9));

            auto it = std::find(id_robots_.begin(), id_robots_.end(), robot_id);

            if (it == id_robots_.end())
            {
                continue;
            }

            if (transformer.info_publishers.count(robot_id) == 0)
            {
                int indice = std::distance(id_robots_.begin(), it);

                std::stringstream topic_info;
                topic_info << "/qupa_" << robots_[indice] << "/odom";
                transformer.info_publishers[robot_id] = this->create_publisher<tracker_package::msg::RobotInfo>(topic_info.str(), 10);
                RCLCPP_INFO(this->get_logger(), "Publicador creado para ID: %d", robot_id);
            }

            // ODOMETRÍA Y TF
            nav_msgs::msg::Odometry robot_pose_msg;
            robot_pose_msg.header.frame_id = "camera_optical_frame";
            robot_pose_msg.header.stamp = this->now();
            std::stringstream child;
            child << "base_link_" << robot_id;
            robot_pose_msg.child_frame_id = child.str();

            std::vector<double> x(4), y(4);
            for (int j = 0; j < 4; ++j)
            {
                // Seguridad adicional: cada coordenada debe estar dentro del bloque
                int x_tracker = -static_cast<int>(msg->data.at(i + 1 + 2 * j));
                int y_tracker = static_cast<int>(msg->data.at(i + 2 + 2 * j));

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

            double yaw = atan2(y[1] - y[0], x[1] - x[0]);
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);
            robot_pose_msg.pose.pose.orientation = tf2::toMsg(q);

            // TF ---------------------------------------------------
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = robot_pose_msg.header.stamp;
            t.header.frame_id = "camera_optical_frame";
            t.child_frame_id = robot_pose_msg.child_frame_id;
            t.transform.translation.x = pose_x;
            t.transform.translation.y = pose_y;
            t.transform.rotation = robot_pose_msg.pose.pose.orientation;
            br_->sendTransform(t);

            // MARKER ------------------------------------------------------------------
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "camera_optical_frame";
            marker.header.stamp = this->now();
            marker.ns = "robot_colors";
            marker.id = robot_id; // Cada robot tiene su propio marcador
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;

            // Posición igual a la del robot
            marker.pose.position.x = pose_x;
            marker.pose.position.y = pose_y;
            marker.pose.position.z = 0.0;
            marker.scale.x = 0.1; // Diámetro del punto
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;

            // Lógica de color dinámica
            marker.color.a = 1.0; // Opacidad total
            if (color_code == 1)
            { // Azul
                marker.color.r = 0.0;
                marker.color.g = 0.0;
                marker.color.b = 1.0;
            }
            else if (color_code == 2)
            { // Verde
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
            }
            else if (color_code == 3)
            { // Rojo
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
            }
            else
            { // Desconocido/Default (Gris)
                marker.color.r = 0.5;
                marker.color.g = 0.5;
                marker.color.b = 0.5;
            }

            marker_pub_->publish(marker);

            // RobotInfo
            tracker_package::msg::RobotInfo robot_info_msg;
            robot_info_msg.odom = robot_pose_msg;
            robot_info_msg.robotstate = tracker_package::msg::RobotInfo::SEARCHING;
            robot_info_msg.typework = tracker_package::msg::RobotInfo::UNDEFINED;

            if (color_code == 1)
            {
                robot_info_msg.robotstate = tracker_package::msg::RobotInfo::WORKING;
                robot_info_msg.typework = tracker_package::msg::RobotInfo::WORK_BLUE;
            }
            else if (color_code == 2)
            {
                robot_info_msg.typework = tracker_package::msg::RobotInfo::WORK_GREEN;
                robot_info_msg.robotstate = tracker_package::msg::RobotInfo::WORKING;
            }
            else if (color_code == 3)
            {
                robot_info_msg.robotstate = tracker_package::msg::RobotInfo::REJECTED;
                robot_info_msg.typework = tracker_package::msg::RobotInfo::UNDEFINED;
            }
            robot_info_msg.timework = 0.0;
            transformer.info_publishers[robot_id]->publish(robot_info_msg);
        }
    }

    rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr sub_tracker_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> br_;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    std::vector<std::string> robots_;
    std::vector<int64_t> id_robots_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Transformer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}