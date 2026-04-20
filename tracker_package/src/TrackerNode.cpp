#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>

using std::placeholders::_1;

class Tracker : public rclcpp::Node
{
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<std_msgs::msg::UInt32MultiArray>::SharedPtr pub_;

    cv::Mat camera_matrix, dist_coeff;
    cv::Ptr<cv::aruco::DetectorParameters> parameters;
    cv::Ptr<cv::aruco::Dictionary> dictionary;

public:
    Tracker() : Node("tracker")
    {
        // Parámetro
        this->declare_parameter<bool>("tracker.crop", false);
        bool crop = this->get_parameter("tracker.crop").as_bool();

        std::string topic_name = crop ? "image_cropped" : "/image";

        // Subscriber
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            topic_name,
            20,
            std::bind(&Tracker::imageCb, this, _1));

        // Publisher
        pub_ = this->create_publisher<std_msgs::msg::UInt32MultiArray>(
            "tracker/positions_stamped", 20);

        // Diccionario ArUco
        dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
        parameters = cv::aruco::DetectorParameters::create();

        // Ajuste de parámetros
        parameters->adaptiveThreshWinSizeMin = 3;
        parameters->adaptiveThreshWinSizeMax = 53;
        parameters->adaptiveThreshWinSizeStep = 2;
        parameters->minDistanceToBorder = 1;
        parameters->minMarkerPerimeterRate = 0.01;
        parameters->perspectiveRemovePixelPerCell = 4;
        parameters->errorCorrectionRate = 0.9;
        parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    }

    void imageCb(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;

        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Timestamp
        rclcpp::Time stamp = msg->header.stamp;

        // Detección ArUco
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;

        cv::aruco::detectMarkers(cv_ptr->image, dictionary, corners, ids, parameters);

        // Mensaje
        std_msgs::msg::UInt32MultiArray positions_stamped;

        positions_stamped.data.push_back(stamp.seconds());
        positions_stamped.data.push_back(stamp.nanoseconds() % 1000000000);

        for (size_t i = 0; i < corners.size(); i++)
        {
            positions_stamped.data.push_back(ids.at(i));
            for (size_t j = 0; j < corners.at(i).size(); j++)
            {
                positions_stamped.data.push_back(static_cast<uint32_t>(corners.at(i).at(j).x));
                positions_stamped.data.push_back(static_cast<uint32_t>(corners.at(i).at(j).y));
            }
        }

        pub_->publish(positions_stamped);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Tracker>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}