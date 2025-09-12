#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

class CameraPublisherNode : public rclcpp::Node {
public:
    CameraPublisherNode()
    : Node("camera_publisher_node"), cap(0) {
        if (!cap.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera!");
            rclcpp::shutdown();
            return;
        }

        // Declare parameters
        this->declare_parameter("fps", 30.0);
        this->declare_parameter("brightness", 0.5);
        this->declare_parameter("contrast", 0.5);
        this->declare_parameter("saturation", 0.5);
        this->declare_parameter("hue", 0.5);
        this->declare_parameter("gain", 0.5);
        this->declare_parameter("exposure", 0.5);

        // Get initial parameter values and apply to camera
        apply_camera_parameters();

        // Register parameter callback
        parameter_event_sub_ = this->create_subscription<rcl_interfaces::msg::ParameterEvent>(
            "/parameter_events", 10,
            std::bind(&CameraPublisherNode::on_parameter_event, this, std::placeholders::_1));

        // Set up timer based on initial FPS
        double initial_fps = this->get_parameter("fps").as_double();
        setup_timer(initial_fps);
    }

    void set_publisher(const image_transport::Publisher& pub) {
        pub_ = pub;
    }

private:
    void apply_camera_parameters() {
        bool success;
        success = cap.set(cv::CAP_PROP_FPS, this->get_parameter("fps").as_double());
        RCLCPP_INFO(this->get_logger(), "Setting FPS to %f: %s", this->get_parameter("fps").as_double(), success ? "SUCCESS" : "FAILED");
        success = cap.set(cv::CAP_PROP_BRIGHTNESS, this->get_parameter("brightness").as_double());
        RCLCPP_INFO(this->get_logger(), "Setting BRIGHTNESS to %f: %s", this->get_parameter("brightness").as_double(), success ? "SUCCESS" : "FAILED");
        success = cap.set(cv::CAP_PROP_CONTRAST, this->get_parameter("contrast").as_double());
        RCLCPP_INFO(this->get_logger(), "Setting CONTRAST to %f: %s", this->get_parameter("contrast").as_double(), success ? "SUCCESS" : "FAILED");
        success = cap.set(cv::CAP_PROP_SATURATION, this->get_parameter("saturation").as_double());
        RCLCPP_INFO(this->get_logger(), "Setting SATURATION to %f: %s", this->get_parameter("saturation").as_double(), success ? "SUCCESS" : "FAILED");
        success = cap.set(cv::CAP_PROP_HUE, this->get_parameter("hue").as_double());
        RCLCPP_INFO(this->get_logger(), "Setting HUE to %f: %s", this->get_parameter("hue").as_double(), success ? "SUCCESS" : "FAILED");
        success = cap.set(cv::CAP_PROP_GAIN, this->get_parameter("gain").as_double());
        RCLCPP_INFO(this->get_logger(), "Setting GAIN to %f: %s", this->get_parameter("gain").as_double(), success ? "SUCCESS" : "FAILED");
        success = cap.set(cv::CAP_PROP_EXPOSURE, this->get_parameter("exposure").as_double());
        RCLCPP_INFO(this->get_logger(), "Setting EXPOSURE to %f: %s", this->get_parameter("exposure").as_double(), success ? "SUCCESS" : "FAILED");
        RCLCPP_INFO(this->get_logger(), "Applied initial camera parameters.");
    }

    void setup_timer(double fps) {
        if (timer_) {
            timer_->cancel(); // Cancel existing timer if any
        }
        if (fps > 0) {
            long long period_ms = static_cast<long long>(1000.0 / fps);
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(period_ms),
                std::bind(&CameraPublisherNode::timer_callback, this)
            );
            RCLCPP_INFO(this->get_logger(), "Timer set to %f FPS (period: %lld ms).", fps, period_ms);
        } else {
            RCLCPP_WARN(this->get_logger(), "FPS is zero or negative, timer not started.");
        }
    }

    void timer_callback() {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Failed to capture frame.");
            return;
        }

        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = "camera_frame";

        // Publish as compressed image
        auto cv_ptr = cv_bridge::CvImage(header, "bgr8", frame);
        sensor_msgs::msg::Image::SharedPtr img_msg = cv_ptr.toImageMsg();
        pub_.publish(img_msg);
    }

    void on_parameter_event(const rcl_interfaces::msg::ParameterEvent::SharedPtr event) {
        // Filter for events from this node
        if (event->node == this->get_name()) {
            for (const auto& changed_parameter : event->changed_parameters) {
                bool success;
                if (changed_parameter.name == "fps") {
                    double new_fps = changed_parameter.value.double_value;
                    success = cap.set(cv::CAP_PROP_FPS, new_fps);
                    setup_timer(new_fps); // Recreate timer with new period
                    RCLCPP_INFO(this->get_logger(), "FPS changed to %f: %s", new_fps, success ? "SUCCESS" : "FAILED");
                } else if (changed_parameter.name == "brightness") {
                    success = cap.set(cv::CAP_PROP_BRIGHTNESS, changed_parameter.value.double_value);
                    RCLCPP_INFO(this->get_logger(), "Brightness changed to %f: %s", changed_parameter.value.double_value, success ? "SUCCESS" : "FAILED");
                } else if (changed_parameter.name == "contrast") {
                    success = cap.set(cv::CAP_PROP_CONTRAST, changed_parameter.value.double_value);
                    RCLCPP_INFO(this->get_logger(), "Contrast changed to %f: %s", changed_parameter.value.double_value, success ? "SUCCESS" : "FAILED");
                } else if (changed_parameter.name == "saturation") {
                    success = cap.set(cv::CAP_PROP_SATURATION, changed_parameter.value.double_value);
                    RCLCPP_INFO(this->get_logger(), "Saturation changed to %f: %s", changed_parameter.value.double_value, success ? "SUCCESS" : "FAILED");
                } else if (changed_parameter.name == "hue") {
                    success = cap.set(cv::CAP_PROP_HUE, changed_parameter.value.double_value);
                    RCLCPP_INFO(this->get_logger(), "Hue changed to %f: %s", changed_parameter.value.double_value, success ? "SUCCESS" : "FAILED");
                } else if (changed_parameter.name == "gain") {
                    success = cap.set(cv::CAP_PROP_GAIN, changed_parameter.value.double_value);
                    RCLCPP_INFO(this->get_logger(), "Gain changed to %f: %s", changed_parameter.value.double_value, success ? "SUCCESS" : "FAILED");
                } else if (changed_parameter.name == "exposure") {
                    success = cap.set(cv::CAP_PROP_EXPOSURE, changed_parameter.value.double_value);
                    RCLCPP_INFO(this->get_logger(), "Exposure changed to %f: %s", changed_parameter.value.double_value, success ? "SUCCESS" : "FAILED");
                }
            }
        }
    }

    image_transport::Publisher pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<CameraPublisherNode>();

    image_transport::ImageTransport it(node);
    node->set_publisher(it.advertise("camera/image/compressed", 1));

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
