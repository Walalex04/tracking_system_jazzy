#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <opencv2/core.hpp>

#include <map>
#include <string>
#include <vector>

class SupervisorRobot : public rclcpp::Node
{
public:
    SupervisorRobot() : Node("supervisor_node"),
                        arena_status(18, 0) // inicializa vector con 18 ceros
    {
        pubStatusArena_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("arena_status", 10);

        loadPoints();
        discoverRobots();
    }

private:
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pubStatusArena_;
    std::map<std::string, nav_msgs::msg::Odometry> robot_states;
    std::map<std::string, rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> robot_subs;
    std::vector<cv::Point> points_tam;
    std::vector<int> arena_status;

    // =============================
    // 🔹 Cargar parámetros
    // =============================
    void loadPoints()
    {
        int i = 0;

        while (true)
        {
            std::string param_name = "point_" + std::to_string(i);

            // declarar parámetro si no existe
            this->declare_parameter<std::vector<int64_t>>(param_name, std::vector<int64_t>());

            std::vector<int64_t> point;

            if (!this->get_parameter(param_name, point) || point.empty())
            {
                RCLCPP_WARN(this->get_logger(), "No hay más puntos o parámetro vacío: %s", param_name.c_str());
                break;
            }

            points_tam.emplace_back(point[0], point[1]);

            RCLCPP_INFO(this->get_logger(), "Punto cargado: (%ld, %ld)", point[0], point[1]);

            i++;
        }
    }

    // =============================
    // 🔹 Detectar robots dinámicamente
    // =============================
    void discoverRobots()
    {
        auto topics = this->get_topic_names_and_types();

        for (const auto &topic : topics)
        {
            std::string topic_name = topic.first;

            if (topic_name.find("odometry/filtered") != std::string::npos)
            {
                std::size_t first_slash = topic_name.find('/');
                std::size_t second_slash = topic_name.find('/', first_slash + 1);

                std::string robot_name = topic_name.substr(first_slash + 1,
                                                           second_slash - first_slash - 1);

                if (robot_subs.find(robot_name) == robot_subs.end())
                {
                    auto callback = [this, robot_name](nav_msgs::msg::Odometry::SharedPtr msg)
                    {
                        this->supervisorCallback(msg, robot_name);
                    };

                    robot_subs[robot_name] =
                        this->create_subscription<nav_msgs::msg::Odometry>(
                            topic_name,
                            10,
                            callback);

                    RCLCPP_INFO(this->get_logger(),
                                "Subscriber creado para %s (%s)",
                                robot_name.c_str(),
                                topic_name.c_str());
                }
            }
        }
    }

    // =============================
    // 🔹 Callback por robot
    // =============================
    void supervisorCallback(
        const nav_msgs::msg::Odometry::SharedPtr msg,
        const std::string &robot_name)
    {
        double x = -1 * msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;

        robot_states[robot_name] = *msg;

        double ratio = 5.0;

        for (size_t i = 0; i < points_tam.size(); ++i)
        {
            const auto &p = points_tam[i];

            if ((x <= p.x + ratio && x >= p.x - ratio) &&
                (y <= p.y + ratio && y >= p.y - ratio))
            {
                RCLCPP_INFO(this->get_logger(),
                            "Robot %s llegó a zona %ld",
                            robot_name.c_str(),
                            i);

                arena_status[i] = 1; // ejemplo: marcar zona ocupada
            }
        }

        publishArenaStatus();
    }

    // =============================
    // 🔹 Publicar estado del arena
    // =============================
    void publishArenaStatus()
    {
        std_msgs::msg::UInt8MultiArray msg;

        msg.data.clear();

        for (auto val : arena_status)
        {
            msg.data.push_back(static_cast<uint8_t>(val));
        }

        pubStatusArena_->publish(msg);
    }
};

// =============================
// 🔹 MAIN
// =============================
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<SupervisorRobot>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}