#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <fstream>
#include <vector>

class ArenaCalibration : public rclcpp::Node
{
public:
    bool is_finished;
    unsigned int n_points;

    ArenaCalibration()
        : Node("arena_calibrator"),
          is_finished(false),
          n_points(0),
          window_initialized_(false)
    {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_rect",
            10,
            std::bind(&ArenaCalibration::imageCallback, this, std::placeholders::_1));

        std::string path = ament_index_cpp::get_package_share_directory("arena_package") + "/config";
        std::string fileName = path + "/tam_points.yaml";

        yaml_.open(fileName, std::fstream::out | std::fstream::trunc);

        if (yaml_.is_open())
        {
            RCLCPP_INFO(this->get_logger(), "Writing points to %s", fileName.c_str());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Cannot open file");
        }
    }

    ~ArenaCalibration()
    {
        yaml_.close();
        cv::destroyAllWindows();
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    std::ofstream yaml_;
    std::vector<cv::Point> tam_points_;

    cv::Mat img_, img_resize_;
    bool window_initialized_;


    static void mouseCallbackStatic(int event, int x, int y, int flags, void *userdata)
    {
        auto *self = reinterpret_cast<ArenaCalibration *>(userdata);
        self->mouseCallback(event, x, y);
    }


    void mouseCallback(int event, int x, int y)
    {
        if (event == cv::EVENT_RBUTTONDOWN)
        {
            std::cout << "Right click (" << x << ", " << y << ")" << std::endl;

            if (tam_points_.empty())
            {
                std::cout << "Need at least 1 point!" << std::endl;
                return;
            }

            is_finished = true;
            RCLCPP_INFO(this->get_logger(), "Calibration finished");
            return;
        }

        if (event == cv::EVENT_LBUTTONDOWN)
        {
            std::cout << "Left click (" << x << ", " << y << ")" << std::endl;

            tam_points_.push_back(cv::Point(x, y));

            // Guardar en YAML
            yaml_ << "point_" << n_points << " : [" << x << ", " << y << "]\n";
            n_points++;
        }
    }


    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (is_finished)
            return;

        cv_bridge::CvImagePtr cv_ptr;

        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        img_ = cv_ptr->image;

        cv::resize(img_, img_resize_, cv::Size(), 0.8, 0.8);


        if (!window_initialized_)
        {
            cv::namedWindow("ImageDisplay", cv::WINDOW_AUTOSIZE);
            cv::setMouseCallback("ImageDisplay", mouseCallbackStatic, this);
            window_initialized_ = true;
        }


        for (const auto &p : tam_points_)
        {
            cv::drawMarker(img_resize_, p, cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 10, 2);
        }

        cv::imshow("ImageDisplay", img_resize_);
        cv::waitKey(1); 
    }
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ArenaCalibration>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}