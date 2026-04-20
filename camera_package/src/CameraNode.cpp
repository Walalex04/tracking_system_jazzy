#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

class CameraNode : public rclcpp::Node {

public:
    CameraNode() : Node("camera_node"),
                   caminfo(this, "camera")
                   
    {
        this->declare_parameter<std::string>("camera_info_url", "");

        std::string camera_info_url;
        this->get_parameter("camera_info_url", camera_info_url);

            // Load calibration
        if (!camera_info_url.empty()) {
            if (caminfo.validateURL(camera_info_url)) {
                caminfo.loadCameraInfo(camera_info_url);
                RCLCPP_INFO(this->get_logger(), "Loaded camera calibration");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Invalid camera_info_url");
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "No camera_info_url provided");
        }

        cap = cv::VideoCapture(0, cv::CAP_V4L2);
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/image", 10);
        camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera_info", 10);

        width = 1280;
        height = 720;

        initilizeCamera();

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(66),
            std::bind(&CameraNode::publisherImage, this)
        );

        RCLCPP_INFO(this->get_logger(), "Camera Node initialized");
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    camera_info_manager::CameraInfoManager caminfo;
    cv::VideoCapture cap;
    int width, height;
    rclcpp::TimerBase::SharedPtr timer_;

    void publisherImage(){
        cv::Mat frame;
        cap >> frame;

        if (frame.empty()) return;

        std_msgs::msg::Header header;
        header.stamp = this->get_clock()->now();
        header.frame_id = "camera";

        auto img_msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();

        auto cam_info = caminfo.getCameraInfo();
        cam_info.header = header;

        image_pub_->publish(*img_msg);
        camera_info_pub_->publish(cam_info);
    }

    void initilizeCamera(){
        cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);

        if(!cap.isOpened()){
            RCLCPP_ERROR(this->get_logger(), "Camera cannot open");
        }
    }
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}