#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using std::placeholders::_1;

class CropperNode : public rclcpp::Node
{
public:
    CropperNode() : Node("cropper_node"), mask_defined(false)
    {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_rect",
            rclcpp::SensorDataQoS(),
            std::bind(&CropperNode::imageCallback, this, _1));

        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "image_cropped", 10);

        // Leer puntos desde parámetros
        int i = 0;
        while (true)
        {
            std::string param_name = "point_" + std::to_string(i);
            this->declare_parameter<std::vector<int64_t>>(param_name, {});

            auto point64 = this->get_parameter(param_name).as_integer_array();

            if (point64.empty())
            {
                RCLCPP_WARN(this->get_logger(), "No more points found");
                break;
            }

            vertices_.emplace_back(point64[0], point64[1]);
            i++;
        }
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;

        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
            return;
        }

        img_ = cv_ptr->image;

        if (!mask_defined)
        {
            mask_ = cv::Mat::zeros(img_.rows, img_.cols, CV_8UC1);
            std::vector<std::vector<cv::Point>> pts{vertices_};
            cv::fillPoly(mask_, pts, cv::Scalar(255));
            mask_defined = true;
        }

        img_.copyTo(roi_, mask_);

        auto out_msg = cv_bridge::CvImage(msg->header, "bgr8", roi_).toImageMsg();
        image_pub_->publish(*out_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

    cv::Mat img_, roi_, mask_;
    std::vector<cv::Point> vertices_;
    bool mask_defined;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CropperNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}