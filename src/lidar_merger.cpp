#include <localization_msgs/msg/point_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

class LidarMerger : public rclcpp::Node {
public:
    LidarMerger() : Node("lidar_merger") {
        // runtime parameters
        this->declare_parameter<std::string>("lidar_topic1", "scan1");
        this->declare_parameter<std::string>("lidar_topic2", "scan2");
        this->declare_parameter<std::string>("output_topic", "merged_scan");

        std::string lidar_topic1, lidar_topic2, output_topic;
        this->get_parameter("lidar_topic1", lidar_topic1);
        this->get_parameter("lidar_topic2", lidar_topic2);
        this->get_parameter("output_topic", output_topic);

        lidar_topic1_ = this->create_subscription<localization_msgs::msg::PointArray>(
            lidar_topic1, 10,
            [this](const localization_msgs::msg::PointArray::SharedPtr msg) { lidarCallback(msg, 0); });
        lidar_topic2_ = this->create_subscription<localization_msgs::msg::PointArray>(
            lidar_topic2, 10,
            [this](const localization_msgs::msg::PointArray::SharedPtr msg) { lidarCallback(msg, 1); });
        output_topic_ = this->create_publisher<localization_msgs::msg::PointArray>(output_topic, 10);
    }

private:
    rclcpp::Subscription<localization_msgs::msg::PointArray>::SharedPtr lidar_topic1_, lidar_topic2_;
    rclcpp::Publisher<localization_msgs::msg::PointArray>::SharedPtr output_topic_;
    localization_msgs::msg::PointArray buffer_[2];

    void lidarCallback(const localization_msgs::msg::PointArray::SharedPtr msg, const int lidar_id) {
        buffer_[lidar_id] = *msg;

        // Publish message
        msg->points.clear();
        msg->points.reserve(buffer_[0].points.size() + buffer_[1].points.size());
        std::copy(buffer_[0].points.begin(), buffer_[0].points.end(), std::back_inserter(msg->points));
        std::copy(buffer_[1].points.begin(), buffer_[1].points.end(), std::back_inserter(msg->points));
        output_topic_->publish(*msg);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarMerger>());
    rclcpp::shutdown();
    return 0;
}