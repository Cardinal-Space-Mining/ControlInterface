#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

class CamPub : public rclcpp::Node {
    public:
        CamPub() : Node("cam_pub"), cap_("/dev/video0") {
            if (!cap_.isOpened()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open camera");
                return;
            }

            pub = this->create_publisher<sensor_msgs::msg::CompressedImage>("camera/image_compressed", 10);

            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(30),
                [this](){this->publish();});
        }

    private:
        void publish() {
            cv::Mat frame;
            cap_ >> frame;

            if (frame.empty()) {
                RCLCPP_WARN(this->get_logger(), "Capture empty frame");
                return;
            }

            auto msg_com = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", frame).toCompressedImageMsg();
            msg_com->header.stamp = this->get_clock()->now();
            pub->publish(*msg_com);
        }

    private:
        rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub;
        rclcpp::TimerBase::SharedPtr timer_;
        cv::VideoCapture cap_;
};

int main(int argc, char* argv[]) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CamPub>());
    rclcpp::shutdown();

    return 0;
}
