#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

class CameraPublisherNode : public rclcpp::Node {
public:
    CameraPublisherNode()
    : Node("camera_publisher_node"), cap(0) {
        if (!cap.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera!");
            rclcpp::shutdown();
        }

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&CameraPublisherNode::timer_callback, this)
        );
    }

    void set_publisher(const image_transport::Publisher& pub) {
        pub_ = pub;
    }

private:
    void timer_callback() {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) return;

        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = "camera_frame";

        // Publish as compressed image
        auto cv_ptr = cv_bridge::CvImage(header, "bgr8", frame);
        sensor_msgs::msg::Image::SharedPtr img_msg = cv_ptr.toImageMsg();
        pub_.publish(img_msg);
    }

    image_transport::Publisher pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<CameraPublisherNode>();

    // ImageTransport expects a Node::SharedPtr
    image_transport::ImageTransport it(node);
        // Advertise compressed image topic
        node->set_publisher(it.advertise("camera/image/compressed", 1));

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}