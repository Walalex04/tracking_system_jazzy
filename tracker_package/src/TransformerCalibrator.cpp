#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <iostream>
#include <fstream>
#include <cmath>
#include <map>

using std::placeholders::_1;

class TransformerCalibrator : public rclcpp::Node
{
public:
    struct Point
    {
        double x;
        double y;
    };

    TransformerCalibrator() : Node("transformer_calibrator"), done(false)
    {
        // Subscriber
        sub_tracker_ = this->create_subscription<std_msgs::msg::UInt32MultiArray>(
            "tracker/positions_stamped",
            10,
            std::bind(&TransformerCalibrator::calibratorCallback, this, _1));

        // Publisher
        pub_alive_ = this->create_publisher<std_msgs::msg::Bool>("alive", 10);

        // Ruta del paquete (ROS2 way)
        std::string pkg_path = ament_index_cpp::get_package_share_directory("tracker_package");
        std::string filename = pkg_path + "/config/transformer.yaml";

        yaml_.open(filename, std::fstream::out | std::fstream::trunc);

        if (yaml_.is_open())
        {
            RCLCPP_INFO(this->get_logger(), "Writing configuration to %s", filename.c_str());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Could not open %s", filename.c_str());
        }
    }

    ~TransformerCalibrator()
    {
        yaml_.close();
    }

private:
    bool done;

    // TAGS
    const int center_tag = 13;
    const int tag_tr = 14;
    const int tag_tl = 17;
    const int tag_bl = 18;
    const int tag_br = 19;

    void calibratorCallback(const std_msgs::msg::UInt32MultiArray::SharedPtr msg)
    {
        int size = msg->data.size();

        if (size >= 47)
        {
            std::map<int, Point> tags;

            for (int i = 2; i < size; i += 9)
            {
                Point tag_centre{0.0, 0.0};

                for (int j = 0; j < 4; ++j)
                {
                    tag_centre.x -= static_cast<double>(msg->data.at(2 * j + 1 + i)) / 4.0;
                    tag_centre.y += static_cast<double>(msg->data.at(2 * j + 2 + i)) / 4.0;
                }

                tags[msg->data.at(i)] = tag_centre;
            }

            // Validación mínima
            if (!(tags.count(center_tag) &&
                  tags.count(tag_tr) &&
                  tags.count(tag_tl) &&
                  tags.count(tag_bl)))
            {
                return;
            }

            double angular_offset = 0.0;
            double scale_factor = 0.0;

            // Horizontal
            {
                double x1 = tags[tag_tr].x;
                double y1 = tags[tag_tr].y;
                double x2 = tags[tag_tl].x;
                double y2 = tags[tag_tl].y;

                double orientation = std::atan2(y2 - y1, x2 - x1);
                angular_offset += (orientation - 0.0) / 2.0;

                double size = std::sqrt(std::pow(y2 - y1, 2) + std::pow(x2 - x1, 2));
                scale_factor += (1.75 / size) / 2.0;
            }

            // Vertical
            {
                double x1 = tags[tag_tl].x;
                double y1 = tags[tag_tl].y;
                double x2 = tags[tag_bl].x;
                double y2 = tags[tag_bl].y;

                double orientation = std::atan2(y2 - y1, x2 - x1);
                angular_offset += (orientation - M_PI_2) / 2.0;

                double size = std::sqrt(std::pow(y2 - y1, 2) + std::pow(x2 - x1, 2));
                scale_factor += (1.37 / size) / 2.0;
            }

            // Guardar YAML
            yaml_ << "x_offset : " << tags[center_tag].x << "\n";
            yaml_ << "y_offset : " << tags[center_tag].y << "\n";
            yaml_ << "angular_offset : " << angular_offset << "\n";
            yaml_ << "scale_factor : " << scale_factor << "\n";

            done = true;
        }

        std_msgs::msg::Bool alive_msg;
        alive_msg.data = !done;
        pub_alive_->publish(alive_msg);
    }

    rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr sub_tracker_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_alive_;
    std::ofstream yaml_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TransformerCalibrator>();

    rclcpp::Rate rate(30);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);

        if (!rclcpp::ok())
            break;

        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}