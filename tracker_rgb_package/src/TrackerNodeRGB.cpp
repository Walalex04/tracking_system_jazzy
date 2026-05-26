#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

using std::placeholders::_1;

class Tracker : public rclcpp::Node
{
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<std_msgs::msg::UInt32MultiArray>::SharedPtr pub_;

    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Ptr<cv::aruco::DetectorParameters> parameters;

public:
    Tracker() : Node("tracker")
    {
        dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
        parameters = cv::aruco::DetectorParameters::create();

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_cropped", 20, std::bind(&Tracker::imageCb, this, _1));
        pub_ = this->create_publisher<std_msgs::msg::UInt32MultiArray>("tracker/positions_stamped", 20);
    }

    int getColorForCenter(const cv::Mat &hsv, cv::Point2f center)
    {
        // Radio de búsqueda alrededor del centro del ArUco (ajustar según tamaño del robot)
        int radius = 25;

        // Crear máscara circular para restringir la zona de búsqueda
        cv::Mat mask_roi = cv::Mat::zeros(hsv.size(), CV_8UC1);
        cv::circle(mask_roi, center, radius, cv::Scalar(255), -1);

        // Definir rangos HSV [Azul, Verde, Rojo]
        // NOTA: Ajusta estos valores según la iluminación real de tu entorno
        std::vector<cv::Scalar> lower = {{109, 68, 213}, {56, 68, 213}, {168, 68, 213}};
        std::vector<cv::Scalar> upper = {{119, 255, 255}, {75, 255, 255}, {225, 255, 255}};

        int best_color = 0;
        int max_pixels = 0;

        for (int i = 0; i < 3; ++i)
        {
            cv::Mat color_mask;
            cv::inRange(hsv, lower[i], upper[i], color_mask);

            // Cruzar máscara de color con el área circular definida
            cv::Mat masked;
            cv::bitwise_and(color_mask, mask_roi, masked);

            int count = cv::countNonZero(masked);

            // Si el color es Rojo, considera también el rango superior (0-10 y 160-180)
            if (i == 2)
            {
                cv::Mat m2;
                cv::inRange(hsv, cv::Scalar(160, 50, 50), cv::Scalar(180, 255, 255), m2);
                cv::bitwise_and(m2, mask_roi, m2);
                count += cv::countNonZero(m2);
            }

            if (count > max_pixels)
            {
                max_pixels = count;
                best_color = i + 1; // 1: Azul, 2: Verde, 3: Rojo
            }
        }

        // Umbral de píxeles mínimos para evitar detecciones falsas por ruido
        return (max_pixels > 150) ? best_color : 0;
    }

    void imageCb(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat hsv;
        cv::cvtColor(cv_ptr->image, hsv, cv::COLOR_BGR2HSV);

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(cv_ptr->image, dictionary, corners, ids, parameters);

        std_msgs::msg::UInt32MultiArray msg_out;
        msg_out.data.push_back(msg->header.stamp.sec);
        msg_out.data.push_back(msg->header.stamp.nanosec);

        for (size_t i = 0; i < ids.size(); i++)
        {
            cv::Point2f center = (corners[i][0] + corners[i][1] + corners[i][2] + corners[i][3]) * 0.25f;
            int color_code = getColorForCenter(hsv, center);

            msg_out.data.push_back(static_cast<uint32_t>(ids[i]));
            for (auto &p : corners[i])
            {
                msg_out.data.push_back(static_cast<uint32_t>(p.x));
                msg_out.data.push_back(static_cast<uint32_t>(p.y));
            }
            msg_out.data.push_back(static_cast<uint32_t>(color_code));
        }
        pub_->publish(msg_out);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Tracker>());
    rclcpp::shutdown();
    return 0;
}