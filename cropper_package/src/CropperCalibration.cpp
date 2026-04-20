#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <fstream>

using std::placeholders::_1;

class CropperCalibrator : public rclcpp::Node
{
public:
    CropperCalibrator()
        : Node("calibrator"), cropped(false), n_points(0), ratio_(0.8)
    {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_rect",
            rclcpp::SensorDataQoS(),
            std::bind(&CropperCalibrator::imageCallback, this, _1));

        pub_alive_ = this->create_publisher<std_msgs::msg::Bool>("alive", 10);

        yaml_.open("config_cropper.yaml", std::fstream::out | std::fstream::trunc);

        if (!yaml_.is_open())
            RCLCPP_ERROR(this->get_logger(), "Could not open YAML file");
    }

    ~CropperCalibrator()
    {
        yaml_.close();
    }

    bool cropped;

private:
    static void mouseCallbackStatic(int event, int x, int y, int, void *userdata)
    {
        auto *self = reinterpret_cast<CropperCalibrator *>(userdata);
        self->mouseCallback(event, x, y);
    }

    void mouseCallback(int event, int x, int y)
    {
        if (event == cv::EVENT_LBUTTONDOWN)
        {
            vertices_.emplace_back(x, y);

            int x_img = x / ratio_;
            int y_img = y / ratio_;

            yaml_ << "point_" << n_points << ": [" << x_img << ", " << y_img << "]\n";
            n_points++;
        }

        if (event == cv::EVENT_RBUTTONDOWN && vertices_.size() >= 3)
        {
            cv::line(img_resize_, vertices_.back(), vertices_[0], cv::Scalar(0,0,0));
            cropped = true;
        }
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (cropped)
            return;

        auto cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");

        img_ = cv_ptr->image;
        cv::resize(img_, img_resize_, cv::Size(), ratio_, ratio_);

        cv::namedWindow("Calibration");
        cv::setMouseCallback("Calibration", mouseCallbackStatic, this);

        cv::imshow("Calibration", img_resize_);
        cv::waitKey(1);

        std_msgs::msg::Bool alive_msg;
        alive_msg.data = !cropped;
        pub_alive_->publish(alive_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_alive_;

    std::ofstream yaml_;
    cv::Mat img_, img_resize_;
    std::vector<cv::Point> vertices_;

    unsigned int n_points;
    double ratio_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CropperCalibrator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}